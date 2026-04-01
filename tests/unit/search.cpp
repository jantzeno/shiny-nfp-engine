#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

#include "packing/decoder.hpp"
#include "runtime/execution.hpp"
#include "search/config.hpp"
#include "search/detail/run_event_sink.hpp"
#include "search/jostle.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using Catch::Approx;
using shiny::nfp::AlgorithmKind;
using shiny::nfp::pack::BinPrototype;
using shiny::nfp::pack::PackingConfig;
using shiny::nfp::pack::PieceInput;
using shiny::nfp::pack::SharedCutOptimizationMode;
using shiny::nfp::place::PlacementPolicy;
using shiny::nfp::runtime::ExecutionControlConfig;
using shiny::nfp::search::GeneticSearchConfig;
using shiny::nfp::search::JostleSearch;
using shiny::nfp::search::LocalSearchConfig;
using shiny::nfp::search::SearchCancellationAcknowledgedEvent;
using shiny::nfp::search::SearchEvent;
using shiny::nfp::search::SearchEventKind;
using shiny::nfp::search::SearchImprovementFoundEvent;
using shiny::nfp::search::SearchMoveKind;
using shiny::nfp::search::SearchProgressEntry;
using shiny::nfp::search::SearchRequest;
using shiny::nfp::search::SearchRunCompletedEvent;
using shiny::nfp::search::SearchRunStartedEvent;
using shiny::nfp::search::SearchRunStatus;
using shiny::nfp::search::SearchRunSummary;
using shiny::nfp::search::SearchStepProgressEvent;
using shiny::nfp::search::SearchTimeoutReachedEvent;
using shiny::nfp::test::load_fixture_file;
using shiny::nfp::test::parse_polygon;
using shiny::nfp::test::require_fixture_metadata;

auto parse_rotations(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<double> {
  std::vector<double> rotations;
  for (const auto &child : node) {
    rotations.push_back(child.second.get_value<double>());
  }
  return rotations;
}

auto parse_bed_grain_direction(std::string_view value)
    -> shiny::nfp::place::BedGrainDirection {
  if (value == "unrestricted") {
    return shiny::nfp::place::BedGrainDirection::unrestricted;
  }
  if (value == "along_x") {
    return shiny::nfp::place::BedGrainDirection::along_x;
  }
  if (value == "along_y") {
    return shiny::nfp::place::BedGrainDirection::along_y;
  }
  throw std::runtime_error("unknown search fixture bed grain direction");
}

auto parse_part_grain_compatibility(std::string_view value)
    -> shiny::nfp::place::PartGrainCompatibility {
  if (value == "unrestricted") {
    return shiny::nfp::place::PartGrainCompatibility::unrestricted;
  }
  if (value == "parallel_to_bed") {
    return shiny::nfp::place::PartGrainCompatibility::parallel_to_bed;
  }
  if (value == "perpendicular_to_bed") {
    return shiny::nfp::place::PartGrainCompatibility::perpendicular_to_bed;
  }
  throw std::runtime_error("unknown search fixture grain compatibility");
}

auto parse_exclusion_zones(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<shiny::nfp::place::BedExclusionZone> {
  std::vector<shiny::nfp::place::BedExclusionZone> zones;
  for (const auto &child : node) {
    zones.push_back({
        .zone_id = child.second.get<std::uint32_t>("zone_id", 0),
        .region = {.outer = shiny::nfp::test::parse_ring(
                       child.second.get_child("region"))},
    });
  }
  return zones;
}

auto parse_ids(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> values;
  for (const auto &child : node) {
    values.push_back(child.second.get_value<std::uint32_t>());
  }
  return values;
}

auto parse_policy(std::string_view value) -> PlacementPolicy {
  if (value == "bottom_left") {
    return PlacementPolicy::bottom_left;
  }
  if (value == "minimum_length") {
    return PlacementPolicy::minimum_length;
  }
  if (value == "maximum_utilization") {
    return PlacementPolicy::maximum_utilization;
  }
  throw std::runtime_error("unknown search fixture policy");
}

auto parse_shared_cut_mode(std::string_view value)
    -> SharedCutOptimizationMode {
  if (value == "off") {
    return SharedCutOptimizationMode::off;
  }
  if (value == "remove_fully_covered_coincident_segments") {
    return SharedCutOptimizationMode::remove_fully_covered_coincident_segments;
  }
  throw std::runtime_error("unknown search fixture shared-cut mode");
}

auto parse_move_kind(std::string_view value) -> SearchMoveKind {
  if (value == "none") {
    return SearchMoveKind::none;
  }
  if (value == "jostle_oscillation") {
    return SearchMoveKind::jostle_oscillation;
  }
  if (value == "one_piece_insert") {
    return SearchMoveKind::one_piece_insert;
  }
  if (value == "genetic_generation") {
    return SearchMoveKind::genetic_generation;
  }
  throw std::runtime_error("unknown search fixture move kind");
}

auto parse_local_search_config(const shiny::nfp::test::pt::ptree &node)
    -> LocalSearchConfig {
  LocalSearchConfig config{};
  if (const auto max_iterations =
          node.get_optional<std::uint32_t>("max_iterations")) {
    config.max_iterations = *max_iterations;
  }
  if (const auto deterministic_seed =
          node.get_optional<std::uint32_t>("deterministic_seed")) {
    config.deterministic_seed = *deterministic_seed;
  }
  if (const auto plateau_budget =
          node.get_optional<std::uint32_t>("plateau_budget")) {
    config.plateau_budget = *plateau_budget;
  }
  return config;
}

auto parse_genetic_search_config(const shiny::nfp::test::pt::ptree &node)
    -> GeneticSearchConfig {
  GeneticSearchConfig config{};
  if (const auto max_generations =
          node.get_optional<std::uint32_t>("max_generations")) {
    config.max_generations = *max_generations;
  }
  if (const auto population_size =
          node.get_optional<std::uint32_t>("population_size")) {
    config.population_size = *population_size;
  }
  if (const auto deterministic_seed =
          node.get_optional<std::uint32_t>("deterministic_seed")) {
    config.deterministic_seed = *deterministic_seed;
  }
  if (const auto mutation_rate =
          node.get_optional<std::uint8_t>("mutation_rate_percent")) {
    config.mutation_rate_percent = *mutation_rate;
  }
  if (const auto elite_count =
          node.get_optional<std::uint32_t>("elite_count")) {
    config.elite_count = *elite_count;
  }
  if (const auto tournament_size =
          node.get_optional<std::uint32_t>("tournament_size")) {
    config.tournament_size = *tournament_size;
  }
  if (const auto plateau_generations =
          node.get_optional<std::uint32_t>("plateau_generations")) {
    config.plateau_generations = *plateau_generations;
  }
  if (const auto enable_local_search_polish =
          node.get_optional<bool>("enable_local_search_polish")) {
    config.enable_local_search_polish = *enable_local_search_polish;
  }
  if (const auto enabled = node.get_optional<bool>("enabled")) {
    config.enabled = *enabled;
  }
  return config;
}

auto parse_packing_config(const shiny::nfp::test::pt::ptree &node)
    -> PackingConfig {
  PackingConfig config{};

  if (const auto placement = node.get_child_optional("placement")) {
    if (const auto part_clearance =
            placement->get_optional<double>("part_clearance")) {
      config.placement.part_clearance = *part_clearance;
    }
    if (const auto rotations =
            placement->get_child_optional("allowed_rotations")) {
      config.placement.allowed_rotations.angles_degrees =
          parse_rotations(*rotations);
    }
    if (const auto enable_part_in_part =
            placement->get_optional<bool>("enable_part_in_part_placement")) {
      config.placement.enable_part_in_part_placement = *enable_part_in_part;
    }
    if (const auto explore_concave =
            placement->get_optional<bool>("explore_concave_candidates")) {
      config.placement.explore_concave_candidates = *explore_concave;
    }
    if (const auto grain_direction =
            placement->get_optional<std::string>("bed_grain_direction")) {
      config.placement.bed_grain_direction =
          parse_bed_grain_direction(*grain_direction);
    }
    if (const auto exclusion_zones =
            placement->get_child_optional("exclusion_zones")) {
      config.placement.exclusion_zones =
          parse_exclusion_zones(*exclusion_zones);
    }
  }

  if (const auto hole_first =
          node.get_optional<bool>("enable_hole_first_placement")) {
    config.enable_hole_first_placement = *hole_first;
  }

  if (const auto laser = node.get_child_optional("laser_cut_optimization")) {
    if (const auto mode = laser->get_optional<std::string>("mode")) {
      config.laser_cut_optimization.mode = parse_shared_cut_mode(*mode);
    }
    if (const auto exact =
            laser->get_optional<bool>("require_exact_collinearity")) {
      config.laser_cut_optimization.require_exact_collinearity = *exact;
    }
    if (const auto preserve =
            laser->get_optional<bool>("preserve_visible_notches")) {
      config.laser_cut_optimization.preserve_visible_notches = *preserve;
    }
  }

  return config;
}

auto parse_piece_inputs(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<PieceInput> {
  std::vector<PieceInput> pieces;
  for (const auto &child : node) {
    pieces.push_back({
        .piece_id = child.second.get<std::uint32_t>("piece_id"),
        .polygon = parse_polygon(child.second.get_child("polygon")),
        .geometry_revision =
            child.second.get<std::uint64_t>("geometry_revision", 0),
        .grain_compatibility =
            parse_part_grain_compatibility(child.second.get<std::string>(
                "grain_compatibility", "unrestricted")),
    });
  }
  return pieces;
}

auto parse_bin_prototype(const shiny::nfp::test::pt::ptree &node)
    -> BinPrototype {
  return {
      .base_bin_id = node.get<std::uint32_t>("base_bin_id", 0),
      .polygon = parse_polygon(node.get_child("polygon")),
      .geometry_revision = node.get<std::uint64_t>("geometry_revision", 0),
  };
}

auto parse_search_request(const shiny::nfp::test::pt::ptree &node)
    -> SearchRequest {
  SearchRequest request{};

  if (const auto local_search = node.get_child_optional("local_search")) {
    request.local_search = parse_local_search_config(*local_search);
  }
  if (const auto genetic_search = node.get_child_optional("genetic_search")) {
    request.genetic_search = parse_genetic_search_config(*genetic_search);
  }

  const auto &decoder_request = node.get_child("decoder_request");
  request.decoder_request = {
      .bin = parse_bin_prototype(decoder_request.get_child("bin")),
      .pieces = parse_piece_inputs(decoder_request.get_child("pieces")),
      .policy = parse_policy(
          decoder_request.get<std::string>("policy", "bottom_left")),
  };

  if (const auto max_bins =
          decoder_request.get_optional<std::size_t>("max_bin_count")) {
    request.decoder_request.max_bin_count = *max_bins;
  }
  if (const auto config = decoder_request.get_child_optional("config")) {
    request.decoder_request.config = parse_packing_config(*config);
  }

  return request;
}

auto find_fixture(const shiny::nfp::test::pt::ptree &root, std::string_view id)
    -> const shiny::nfp::test::pt::ptree & {
  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    if (fixture.get<std::string>("id") == id) {
      return fixture;
    }
  }
  throw std::runtime_error("search fixture id not found");
}

void require_progress_matches(const SearchProgressEntry &actual,
                              const shiny::nfp::test::pt::ptree &expected) {
  REQUIRE(actual.iteration == expected.get<std::uint32_t>("iteration"));
  REQUIRE(actual.move_kind ==
          parse_move_kind(expected.get<std::string>("move_kind")));
  REQUIRE(actual.improved == expected.get<bool>("improved"));
  if (const auto best_piece_order =
          expected.get_child_optional("best_piece_order")) {
    REQUIRE(actual.best_piece_order == parse_ids(*best_piece_order));
  }
}

void require_same_progress_entry(const SearchProgressEntry &actual,
                                 const SearchProgressEntry &expected) {
  REQUIRE(actual.algorithm_kind == expected.algorithm_kind);
  REQUIRE(actual.iteration == expected.iteration);
  REQUIRE(actual.iteration_budget == expected.iteration_budget);
  REQUIRE(actual.move_kind == expected.move_kind);
  REQUIRE(actual.improved == expected.improved);
  REQUIRE(actual.timestamp_unix_ms == expected.timestamp_unix_ms);
  REQUIRE(actual.elapsed_ms == expected.elapsed_ms);
  REQUIRE(actual.evaluated_layout_count == expected.evaluated_layout_count);
  REQUIRE(actual.reevaluation_cache_hits == expected.reevaluation_cache_hits);
  REQUIRE(actual.best_bin_count == expected.best_bin_count);
  REQUIRE(actual.best_placed_piece_count == expected.best_placed_piece_count);
  REQUIRE(actual.best_unplaced_piece_count ==
          expected.best_unplaced_piece_count);
  REQUIRE(actual.best_total_utilization ==
          Approx(expected.best_total_utilization));
  REQUIRE(actual.best_piece_order == expected.best_piece_order);
}

void require_same_run_summary(const SearchRunSummary &actual,
                              const SearchRunSummary &expected) {
  REQUIRE(actual.algorithm_kind == expected.algorithm_kind);
  REQUIRE(actual.iterations_completed == expected.iterations_completed);
  REQUIRE(actual.iteration_budget == expected.iteration_budget);
  REQUIRE(actual.timestamp_unix_ms == expected.timestamp_unix_ms);
  REQUIRE(actual.elapsed_ms == expected.elapsed_ms);
  REQUIRE(actual.evaluated_layout_count == expected.evaluated_layout_count);
  REQUIRE(actual.reevaluation_cache_hits == expected.reevaluation_cache_hits);
  REQUIRE(actual.best_bin_count == expected.best_bin_count);
  REQUIRE(actual.best_placed_piece_count == expected.best_placed_piece_count);
  REQUIRE(actual.best_unplaced_piece_count ==
          expected.best_unplaced_piece_count);
  REQUIRE(actual.best_total_utilization ==
          Approx(expected.best_total_utilization));
  REQUIRE(actual.best_piece_order == expected.best_piece_order);
}

void require_same_event(const SearchEvent &actual,
                        const SearchEvent &expected) {
  REQUIRE(shiny::nfp::search::search_event_kind(actual) ==
          shiny::nfp::search::search_event_kind(expected));

  std::visit(
      [&](const auto &actual_payload) {
        using Payload = std::decay_t<decltype(actual_payload)>;
        REQUIRE(std::holds_alternative<Payload>(expected));
        const auto &expected_payload = std::get<Payload>(expected);

        if constexpr (std::is_same_v<Payload, SearchRunStartedEvent>) {
          REQUIRE(actual_payload.algorithm_kind ==
                  expected_payload.algorithm_kind);
          REQUIRE(actual_payload.deterministic_seed ==
                  expected_payload.deterministic_seed);
          REQUIRE(actual_payload.iteration_budget ==
                  expected_payload.iteration_budget);
          REQUIRE(actual_payload.piece_count == expected_payload.piece_count);
          REQUIRE(actual_payload.timestamp_unix_ms ==
                  expected_payload.timestamp_unix_ms);
          REQUIRE(actual_payload.elapsed_ms == expected_payload.elapsed_ms);
        } else if constexpr (std::is_same_v<Payload, SearchStepProgressEvent> ||
                             std::is_same_v<Payload,
                                            SearchImprovementFoundEvent>) {
          require_same_progress_entry(actual_payload.progress,
                                      expected_payload.progress);
        } else if constexpr (std::is_same_v<Payload, SearchRunCompletedEvent> ||
                             std::is_same_v<Payload,
                                            SearchTimeoutReachedEvent> ||
                             std::is_same_v<
                                 Payload,
                                 SearchCancellationAcknowledgedEvent>) {
          require_same_run_summary(actual_payload.summary,
                                   expected_payload.summary);
        }
      },
      actual);
}

} // namespace

TEST_CASE("algorithm kind serialization uses canonical vocabulary",
          "[search][algorithm]") {
  REQUIRE(shiny::nfp::to_string(AlgorithmKind::constructive_decoder) ==
          "constructive_decoder");
  REQUIRE(shiny::nfp::to_string(AlgorithmKind::jostle_search) ==
          "jostle_search");
  REQUIRE(shiny::nfp::to_string(AlgorithmKind::genetic_search) ==
          "genetic_search");
  REQUIRE(shiny::nfp::to_string(AlgorithmKind::masonry_builder) ==
          "masonry_builder");

  const auto parsed = shiny::nfp::parse_algorithm_kind("jostle_search");
  REQUIRE(parsed.has_value());
  REQUIRE(*parsed == AlgorithmKind::jostle_search);
  const auto genetic_parsed =
      shiny::nfp::parse_algorithm_kind("genetic_search");
  REQUIRE(genetic_parsed.has_value());
  REQUIRE(*genetic_parsed == AlgorithmKind::genetic_search);
  REQUIRE_FALSE(shiny::nfp::parse_algorithm_kind("local_search").has_value());
}

TEST_CASE("search config fixtures", "[search][config][fixtures]") {
  const auto root = load_fixture_file("search/config_surface.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "search_config");
      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");

      const auto local_search =
          parse_local_search_config(inputs.get_child("local_search"));
      const auto genetic_search =
          parse_genetic_search_config(inputs.get_child("genetic_search"));

      REQUIRE(local_search.is_valid() == expected.get<bool>("local_valid"));
      REQUIRE(genetic_search.is_valid() == expected.get<bool>("genetic_valid"));
    }
  }
}

TEST_CASE("execution control validates bounded worker counts",
          "[search][execution]") {
  ExecutionControlConfig control{};
  REQUIRE(control.is_valid());

  control.worker_count = 0U;
  REQUIRE_FALSE(control.is_valid());

  control.worker_count = 257U;
  REQUIRE_FALSE(control.is_valid());

  control.worker_count = 8U;
  REQUIRE(control.is_valid());
}

TEST_CASE("run event sink serializes concurrent observer delivery per run",
          "[search][observer][thread-safety]") {
  shiny::nfp::search::SearchExecutionConfig execution{};
  execution.control.capture_timestamps = false;
  execution.control.max_retained_events = 64U;

  std::vector<SearchEvent> live_events;
  execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
  };

  std::vector<SearchEvent> retained_events;
  shiny::nfp::search::detail::RunEventSink sink{execution, retained_events};

  constexpr std::size_t thread_count = 4U;
  constexpr std::size_t events_per_thread = 16U;
  std::atomic<std::uint32_t> next_iteration{0U};

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (std::size_t thread_index = 0U; thread_index < thread_count;
       ++thread_index) {
    threads.emplace_back([&]() {
      for (std::size_t event_index = 0U; event_index < events_per_thread;
           ++event_index) {
        const auto iteration = next_iteration.fetch_add(1U);
        sink.emit(SearchStepProgressEvent{
            .progress = SearchProgressEntry{
                .algorithm_kind = AlgorithmKind::jostle_search,
                .iteration = iteration,
                .iteration_budget = 64U,
            }});
      }
    });
  }

  for (auto &thread : threads) {
    thread.join();
  }

  REQUIRE(live_events.size() == thread_count * events_per_thread);
  REQUIRE(retained_events.size() == thread_count * events_per_thread);
  for (std::size_t index = 0; index < live_events.size(); ++index) {
    require_same_event(live_events[index], retained_events[index]);
  }
}

TEST_CASE("search scenario fixtures", "[search][scenario][fixtures]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  REQUIRE(root.get<std::string>("algorithm") == "jostle_search");

  const auto parsed_algorithm =
      shiny::nfp::parse_algorithm_kind(root.get<std::string>("algorithm"));
  REQUIRE(parsed_algorithm.has_value());
  REQUIRE(*parsed_algorithm == AlgorithmKind::jostle_search);

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "search_scenario");

      JostleSearch search;
      const auto request = parse_search_request(
          fixture.get_child("inputs").get_child("request"));
      const auto result = search.improve(request);
      const auto expected = fixture.get_child("expected");

      REQUIRE(result.algorithm == AlgorithmKind::jostle_search);
      REQUIRE(shiny::nfp::to_string(result.algorithm) == "jostle_search");
      REQUIRE(result.status == SearchRunStatus::completed);
      REQUIRE(result.improved() == expected.get<bool>("improved"));
      REQUIRE(result.baseline.unplaced_piece_count ==
              expected.get<std::size_t>("baseline_unplaced_piece_count"));
      if (const auto baseline_bin_count =
              expected.get_optional<std::size_t>("baseline_bin_count")) {
        REQUIRE(result.baseline.bin_count == *baseline_bin_count);
      }
      REQUIRE(result.best.unplaced_piece_count ==
              expected.get<std::size_t>("best_unplaced_piece_count"));
      REQUIRE(result.best.bin_count ==
              expected.get<std::size_t>("best_bin_count"));
      REQUIRE(result.best.piece_order ==
              parse_ids(expected.get_child("best_piece_order")));
      REQUIRE(result.iterations_completed ==
              expected.get<std::uint32_t>("iterations_completed"));
      REQUIRE(result.progress.size() ==
              expected.get<std::size_t>("progress_count"));

      if (const auto best_total_utilization =
              expected.get_optional<double>("best_total_utilization")) {
        REQUIRE(result.best.total_utilization ==
                Approx(*best_total_utilization));
      }

      if (const auto progress = expected.get_child_optional("progress")) {
        REQUIRE(result.progress.size() == progress->size());
        auto progress_it = progress->begin();
        for (std::size_t index = 0; index < progress->size(); ++index) {
          require_progress_matches(result.progress[index], progress_it->second);
          ++progress_it;
        }
      }
    }
  }
}

TEST_CASE("search is deterministic for a fixed scenario seed",
          "[search][determinism]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.worker_count = 4U;

  JostleSearch first_search;
  JostleSearch second_search;

  const auto first = first_search.improve(request);
  const auto second = second_search.improve(request);

  REQUIRE(first.best.piece_order == second.best.piece_order);
  REQUIRE(first.best.unplaced_piece_count == second.best.unplaced_piece_count);
  REQUIRE(first.best.bin_count == second.best.bin_count);
  REQUIRE(first.best.total_utilization ==
          Approx(second.best.total_utilization));
  REQUIRE(first.algorithm == second.algorithm);
  REQUIRE(first.status == second.status);
  REQUIRE(first.progress.size() == second.progress.size());
}

TEST_CASE("search preserves the single-worker baseline across worker counts",
          "[search][parallel][baseline]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto serial_request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  auto parallel_request = serial_request;
  serial_request.execution.control.worker_count = 1U;
  parallel_request.execution.control.worker_count = 4U;

  JostleSearch serial_search;
  JostleSearch parallel_search;

  const auto serial = serial_search.improve(serial_request);
  const auto parallel = parallel_search.improve(parallel_request);

  REQUIRE(parallel.status == serial.status);
  REQUIRE(parallel.best.piece_order == serial.best.piece_order);
  REQUIRE(parallel.best.unplaced_piece_count ==
          serial.best.unplaced_piece_count);
  REQUIRE(parallel.best.bin_count == serial.best.bin_count);
  REQUIRE(parallel.best.total_utilization ==
          Approx(serial.best.total_utilization));
  REQUIRE(parallel.iterations_completed == serial.iterations_completed);
  REQUIRE(parallel.progress.size() == serial.progress.size());
}

TEST_CASE("search repeated parallel runs remain stable",
          "[search][parallel][repeat]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.worker_count = 3U;

  JostleSearch search;

  const auto first = search.improve(request);
  const auto second = search.improve(request);
  const auto third = search.improve(request);

  REQUIRE(second.status == first.status);
  REQUIRE(third.status == first.status);
  REQUIRE(second.best.piece_order == first.best.piece_order);
  REQUIRE(third.best.piece_order == first.best.piece_order);
  REQUIRE(second.best.unplaced_piece_count == first.best.unplaced_piece_count);
  REQUIRE(third.best.unplaced_piece_count == first.best.unplaced_piece_count);
  REQUIRE(second.best.bin_count == first.best.bin_count);
  REQUIRE(third.best.bin_count == first.best.bin_count);
  REQUIRE(second.best.total_utilization ==
          Approx(first.best.total_utilization));
  REQUIRE(third.best.total_utilization == Approx(first.best.total_utilization));
  REQUIRE(second.iterations_completed == first.iterations_completed);
  REQUIRE(third.iterations_completed == first.iterations_completed);
}

TEST_CASE("search observer replay matches retained history and final progress",
          "[search][observer]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.control.worker_count = 4U;

  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
  };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::completed);
  REQUIRE_FALSE(result.events.empty());
  REQUIRE(live_events.size() == result.events.size());
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.front()) ==
          SearchEventKind::run_started);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.back()) ==
          SearchEventKind::run_completed);

  for (std::size_t index = 0; index < live_events.size(); ++index) {
    require_same_event(live_events[index], result.events[index]);
  }

  std::vector<SearchProgressEntry> observed_progress;
  std::size_t observed_improvements = 0U;
  for (const auto &event : live_events) {
    std::visit(
        [&](const auto &payload) {
          using Payload = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<Payload, SearchStepProgressEvent>) {
            observed_progress.push_back(payload.progress);
          } else if constexpr (std::is_same_v<Payload,
                                              SearchImprovementFoundEvent>) {
            ++observed_improvements;
          }
        },
        event);
  }

  REQUIRE(observed_progress.size() == result.progress.size());
  for (std::size_t index = 0; index < result.progress.size(); ++index) {
    require_same_progress_entry(observed_progress[index],
                                result.progress[index]);
  }

  const auto expected_improvements = static_cast<std::size_t>(std::count_if(
      result.progress.begin(), result.progress.end(),
      [](const SearchProgressEntry &entry) { return entry.improved; }));
  REQUIRE(observed_improvements == expected_improvements);
}

TEST_CASE("search acknowledges cancellation at a safe observer boundary",
          "[search][observer][cancellation]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;

  std::vector<SearchEvent> live_events;
  std::size_t progress_count = 0U;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    if (shiny::nfp::search::search_event_kind(event) ==
        SearchEventKind::step_progress) {
      ++progress_count;
    }
    live_events.push_back(event);
  };
  request.execution.cancellation_requested = [&]() {
    return progress_count >= 1U;
  };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::cancelled);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
  REQUIRE(live_events.size() == 3U);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[0]) ==
          SearchEventKind::run_started);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[1]) ==
          SearchEventKind::step_progress);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[2]) ==
          SearchEventKind::cancellation_acknowledged);
}

TEST_CASE("search discards a cancelled in-flight iteration",
          "[search][observer][cancellation][batch]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.control.worker_count = 4U;

  std::atomic<bool> arm_cancellation{false};
  std::atomic<std::uint32_t> cancellation_polls{0U};
  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
    if (shiny::nfp::search::search_event_kind(event) ==
        SearchEventKind::step_progress) {
      const auto &progress = std::get<SearchStepProgressEvent>(event).progress;
      if (progress.iteration == 0U) {
        arm_cancellation.store(true);
      }
    }
  };
  request.execution.cancellation_requested = [&]() {
    if (!arm_cancellation.load()) {
      return false;
    }
    return cancellation_polls.fetch_add(1U) >= 2U;
  };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::cancelled);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
  REQUIRE(result.best.piece_order == result.baseline.piece_order);
  REQUIRE(result.best.unplaced_piece_count ==
          result.baseline.unplaced_piece_count);
  REQUIRE(live_events.size() == 3U);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.front()) ==
          SearchEventKind::run_started);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[1]) ==
          SearchEventKind::step_progress);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.back()) ==
          SearchEventKind::cancellation_acknowledged);
}

TEST_CASE("search discards a timed-out in-flight iteration",
          "[search][observer][timeout][batch]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.control.worker_count = 4U;
  request.execution.control.time_budget_ms = 1U;

  std::atomic<bool> arm_timeout{false};
  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
    if (shiny::nfp::search::search_event_kind(event) ==
        SearchEventKind::step_progress) {
      const auto &progress = std::get<SearchStepProgressEvent>(event).progress;
      if (progress.iteration == 0U) {
        arm_timeout.store(true);
      }
    }
  };
  request.execution.cancellation_requested = [&]() {
    if (!arm_timeout.load()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return false;
  };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::timed_out);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
  REQUIRE(result.best.piece_order == result.baseline.piece_order);
  REQUIRE(result.best.unplaced_piece_count ==
          result.baseline.unplaced_piece_count);
  REQUIRE(live_events.size() == 3U);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.front()) ==
          SearchEventKind::run_started);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[1]) ==
          SearchEventKind::step_progress);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events.back()) ==
          SearchEventKind::timeout_reached);
}

TEST_CASE("search emits timeout when the time budget is exceeded",
          "[search][observer][timeout]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.control.time_budget_ms = 1U;

  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
    if (shiny::nfp::search::search_event_kind(event) ==
        SearchEventKind::step_progress) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::timed_out);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
  REQUIRE(live_events.size() == 3U);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[0]) ==
          SearchEventKind::run_started);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[1]) ==
          SearchEventKind::step_progress);
  REQUIRE(shiny::nfp::search::search_event_kind(live_events[2]) ==
          SearchEventKind::timeout_reached);
}

TEST_CASE("search does not cache an interrupted baseline decode",
          "[search][cache][cancellation]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.cancellation_requested = []() { return true; };

  JostleSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::cancelled);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
  REQUIRE(result.baseline.decode.interrupted);
  REQUIRE(result.best.decode.interrupted);
  REQUIRE(result.best.piece_order == result.baseline.piece_order);
  REQUIRE(search.reevaluation_cache_size() == 0U);
  REQUIRE(result.reevaluation_cache_hits == 0U);
  REQUIRE(result.evaluated_layout_count == 1U);
}

TEST_CASE("search reuses reevaluation cache across repeated runs",
          "[search][cache]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  const auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));

  JostleSearch search;

  const auto first = search.improve(request);
  const auto second = search.improve(request);

  REQUIRE(first.evaluated_layout_count > 0U);
  REQUIRE(search.reevaluation_cache_size() > 0U);
  REQUIRE(second.reevaluation_cache_hits > 0U);
  REQUIRE(second.evaluated_layout_count <= first.evaluated_layout_count);
}

TEST_CASE("search uses the primary cache path for tiny parallel batches",
          "[search][cache][parallel-cutoff]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");

  auto serial_request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  auto parallel_request = serial_request;
  serial_request.execution.control.worker_count = 1U;
  parallel_request.execution.control.worker_count = 4U;

  JostleSearch serial_search;
  JostleSearch parallel_search;

  const auto serial_first = serial_search.improve(serial_request);
  const auto parallel_first = parallel_search.improve(parallel_request);
  const auto serial_second = serial_search.improve(serial_request);
  const auto parallel_second = parallel_search.improve(parallel_request);

  REQUIRE(serial_first.status == SearchRunStatus::completed);
  REQUIRE(parallel_first.status == SearchRunStatus::completed);
  REQUIRE(serial_search.reevaluation_cache_size() > 0U);
  REQUIRE(parallel_search.reevaluation_cache_size() ==
          serial_search.reevaluation_cache_size());
  REQUIRE(serial_second.reevaluation_cache_hits > 0U);
  REQUIRE(parallel_second.reevaluation_cache_hits > 0U);
}

TEST_CASE("search cache identity changes when manufacturing constraints change",
          "[search][cache][constraints]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "search_jostle_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));

  JostleSearch search;
  const auto baseline = search.improve(request);
  const auto repeated = search.improve(request);
  const auto baseline_cache_size = search.reevaluation_cache_size();

  REQUIRE(baseline.evaluated_layout_count > 0U);
  REQUIRE(repeated.reevaluation_cache_hits > 0U);

  request.decoder_request.config.placement.bed_grain_direction =
      shiny::nfp::place::BedGrainDirection::along_x;
  request.decoder_request.pieces.front().grain_compatibility =
      shiny::nfp::place::PartGrainCompatibility::parallel_to_bed;
  request.decoder_request.config.placement.exclusion_zones = {
      {.zone_id = 41,
       .region = {.outer = {{4.0, 0.0}, {6.0, 0.0}, {6.0, 4.0}, {4.0, 4.0}}}}};

  const auto constrained = search.improve(request);
  REQUIRE(constrained.evaluated_layout_count > 0U);
  REQUIRE(search.reevaluation_cache_size() > baseline_cache_size);
}

TEST_CASE("search plateau budget delays termination after a stall",
          "[search][plateau]") {
  const auto root = load_fixture_file("search/scenario_cases.json");
  const auto &fixture = find_fixture(
      root, "search_preserves_full_layout_when_no_better_neighbor_exists");

  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.local_search.plateau_budget = 1;

  JostleSearch immediate_search;
  const auto immediate = immediate_search.improve(request);
  REQUIRE(immediate.iterations_completed == 1U);
  REQUIRE(immediate.progress.size() == 2U);
  REQUIRE_FALSE(immediate.improved());

  request.local_search.plateau_budget = 3;

  JostleSearch plateau_search;
  const auto plateau = plateau_search.improve(request);
  REQUIRE(plateau.iterations_completed == 3U);
  REQUIRE(plateau.progress.size() == 4U);
  REQUIRE_FALSE(plateau.improved());
  REQUIRE(plateau.best.piece_order == immediate.best.piece_order);
}
