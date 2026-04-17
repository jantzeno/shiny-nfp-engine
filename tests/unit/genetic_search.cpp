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
#include "search/genetic.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using Catch::Approx;
using shiny::nfp::AlgorithmKind;
using shiny::nfp::pack::BinInput;
using shiny::nfp::pack::PackingConfig;
using shiny::nfp::pack::PieceInput;
using shiny::nfp::place::PlacementPolicy;
using shiny::nfp::search::GeneticSearch;
using shiny::nfp::search::GeneticSearchConfig;
using shiny::nfp::search::LocalSearchConfig;
using shiny::nfp::search::SearchEvent;
using shiny::nfp::search::SearchEventKind;
using shiny::nfp::search::SearchImprovementFoundEvent;
using shiny::nfp::search::SearchRequest;
using shiny::nfp::search::SearchRunStatus;
using shiny::nfp::search::SearchStepProgressEvent;
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
  throw std::runtime_error("unknown GA fixture bed grain direction");
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
  throw std::runtime_error("unknown GA fixture grain compatibility");
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
  throw std::runtime_error("unknown GA fixture policy");
}

auto parse_local_search_config(const shiny::nfp::test::pt::ptree &node)
    -> LocalSearchConfig {
  LocalSearchConfig config{};
  config.max_iterations = node.get<std::uint32_t>("max_iterations", 250);
  config.deterministic_seed = node.get<std::uint32_t>("deterministic_seed", 1);
  config.plateau_budget = node.get<std::uint32_t>("plateau_budget", 3);
  return config;
}

auto parse_genetic_search_config(const shiny::nfp::test::pt::ptree &node)
    -> GeneticSearchConfig {
  GeneticSearchConfig config{};
  config.max_generations = node.get<std::uint32_t>("max_generations", 40);
  config.population_size = node.get<std::uint32_t>("population_size", 30);
  config.deterministic_seed = node.get<std::uint32_t>("deterministic_seed", 1);
  config.mutation_rate_percent =
      node.get<std::uint8_t>("mutation_rate_percent", 10);
  config.elite_count = node.get<std::uint32_t>("elite_count", 2);
  config.tournament_size = node.get<std::uint32_t>("tournament_size", 3);
  config.plateau_generations =
      node.get<std::uint32_t>("plateau_generations", 6);
  config.enable_local_search_polish =
      node.get<bool>("enable_local_search_polish", true);
  config.enabled = node.get<bool>("enabled", true);
  return config;
}

auto parse_packing_config(const shiny::nfp::test::pt::ptree &node)
    -> PackingConfig {
  PackingConfig config{};
  if (const auto placement = node.get_child_optional("placement")) {
    if (const auto rotations =
            placement->get_child_optional("allowed_rotations")) {
      config.placement.allowed_rotations.angles_degrees =
          parse_rotations(*rotations);
    }
    if (const auto grain_direction =
            placement->get_optional<std::string>("bed_grain_direction")) {
      config.placement.bed_grain_direction =
          parse_bed_grain_direction(*grain_direction);
    }
  }
  if (const auto hole_first =
          node.get_optional<bool>("enable_hole_first_placement")) {
    config.enable_hole_first_placement = *hole_first;
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
        .allowed_bin_ids = [&]() {
          if (const auto ids = child.second.get_child_optional("allowed_bin_ids")) {
            return parse_ids(*ids);
          }
          return std::vector<std::uint32_t>{};
        }(),
    });
  }
  return pieces;
}

auto parse_bin_input(const shiny::nfp::test::pt::ptree &node) -> BinInput {
  return {
      .bin_id = node.get<std::uint32_t>("base_bin_id", 0),
      .polygon = parse_polygon(node.get_child("polygon")),
      .geometry_revision = node.get<std::uint64_t>("geometry_revision", 0),
  };
}

auto resolve_fixture_bin_count(const std::size_t max_bin_count,
                               const std::size_t piece_count)
    -> std::size_t {
  if (max_bin_count != 0U) {
    return max_bin_count;
  }
  return std::max<std::size_t>(piece_count, 1U);
}

auto expand_bins(BinInput base_bin, const std::size_t count)
    -> std::vector<BinInput> {
  std::vector<BinInput> bins;
  bins.reserve(std::max<std::size_t>(count, 1));
  for (std::size_t index = 0; index < std::max<std::size_t>(count, 1); ++index) {
    BinInput bin = base_bin;
    bin.bin_id = base_bin.bin_id + static_cast<std::uint32_t>(index);
    bins.push_back(std::move(bin));
  }
  return bins;
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
  const auto max_bins = decoder_request.get<std::size_t>("max_bin_count", 1);
  const auto pieces = parse_piece_inputs(decoder_request.get_child("pieces"));
  request.decoder_request = {
      .bins = expand_bins(parse_bin_input(decoder_request.get_child("bin")),
                          resolve_fixture_bin_count(max_bins, pieces.size())),
      .pieces = pieces,
      .policy = parse_policy(
          decoder_request.get<std::string>("policy", "bottom_left")),
  };
  if (const auto config = decoder_request.get_child_optional("config")) {
    request.decoder_request.config = parse_packing_config(*config);
  }

  request.execution.control.capture_timestamps = false;
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
  throw std::runtime_error("GA fixture id not found");
}

} // namespace

TEST_CASE("genetic search scenario fixtures", "[search][genetic][fixtures]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  REQUIRE(root.get<std::string>("algorithm") == "genetic_search");

  const auto parsed_algorithm =
      shiny::nfp::parse_algorithm_kind(root.get<std::string>("algorithm"));
  REQUIRE(parsed_algorithm.has_value());
  REQUIRE(*parsed_algorithm == AlgorithmKind::genetic_search);

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    DYNAMIC_SECTION(fixture.get<std::string>("id")) {
      require_fixture_metadata(fixture, "search_scenario");

      GeneticSearch search;
      const auto request = parse_search_request(
          fixture.get_child("inputs").get_child("request"));
      const auto result = search.improve(request);
      const auto expected = fixture.get_child("expected");

      REQUIRE(result.algorithm == AlgorithmKind::genetic_search);
      REQUIRE(result.status == SearchRunStatus::completed);
      REQUIRE(result.improved() == expected.get<bool>("improved"));
      REQUIRE(result.baseline.unplaced_piece_count ==
              expected.get<std::size_t>("baseline_unplaced_piece_count"));
      REQUIRE(result.best.unplaced_piece_count ==
              expected.get<std::size_t>("best_unplaced_piece_count"));
      REQUIRE(result.best.bin_count ==
              expected.get<std::size_t>("best_bin_count"));
      REQUIRE(result.best.piece_order ==
              parse_ids(expected.get_child("best_piece_order")));
      if (const auto best_total_utilization =
              expected.get_optional<double>("best_total_utilization")) {
        REQUIRE(result.best.total_utilization ==
                Approx(*best_total_utilization));
      }
      if (const auto iterations_completed =
              expected.get_optional<std::uint32_t>("iterations_completed")) {
        REQUIRE(result.iterations_completed == *iterations_completed);
      }
      if (const auto progress_count =
              expected.get_optional<std::size_t>("progress_count")) {
        REQUIRE(result.progress.size() == *progress_count);
      }
    }
  }
}

TEST_CASE("genetic search is deterministic for a fixed scenario seed",
          "[search][genetic][determinism]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.worker_count = 4U;

  GeneticSearch first_search;
  GeneticSearch second_search;

  const auto first = first_search.improve(request);
  const auto second = second_search.improve(request);

  REQUIRE(first.best.piece_order == second.best.piece_order);
  REQUIRE(first.best.unplaced_piece_count == second.best.unplaced_piece_count);
  REQUIRE(first.best.bin_count == second.best.bin_count);
  REQUIRE(first.best.total_utilization ==
          Approx(second.best.total_utilization));
  REQUIRE(first.iterations_completed == second.iterations_completed);
}

TEST_CASE(
    "genetic search preserves the single-worker baseline across worker counts",
    "[search][genetic][parallel][baseline]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto serial_request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  auto parallel_request = serial_request;
  serial_request.execution.control.worker_count = 1U;
  parallel_request.execution.control.worker_count = 4U;

  GeneticSearch serial_search;
  GeneticSearch parallel_search;

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

TEST_CASE("genetic search repeated parallel runs remain stable",
          "[search][genetic][parallel][repeat]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.worker_count = 3U;

  GeneticSearch search;

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

TEST_CASE("genetic search observer replay matches retained history",
          "[search][genetic][observer]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.worker_count = 4U;

  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
  };

  GeneticSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::completed);
  REQUIRE(live_events.size() == result.events.size());

  std::vector<std::uint32_t> observed_progress_iterations;
  std::vector<std::uint32_t> observed_improvement_iterations;
  for (const auto &event : live_events) {
    std::visit(
        [&](const auto &payload) {
          using Payload = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<Payload, SearchStepProgressEvent>) {
            REQUIRE(payload.progress.algorithm_kind ==
                    AlgorithmKind::genetic_search);
            observed_progress_iterations.push_back(payload.progress.iteration);
          } else if constexpr (std::is_same_v<Payload,
                                              SearchImprovementFoundEvent>) {
            observed_improvement_iterations.push_back(
                payload.progress.iteration);
          }
        },
        event);
  }

  REQUIRE(observed_progress_iterations.size() == result.progress.size());
  for (std::size_t index = 0; index < result.progress.size(); ++index) {
    REQUIRE(observed_progress_iterations[index] ==
            result.progress[index].iteration);
  }
  REQUIRE(observed_improvement_iterations == std::vector<std::uint32_t>{1U});
}

TEST_CASE("genetic search acknowledges cancellation at a safe boundary",
          "[search][genetic][cancellation]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));

  std::size_t progress_count = 0U;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    if (shiny::nfp::search::search_event_kind(event) ==
        SearchEventKind::step_progress) {
      ++progress_count;
    }
  };
  request.execution.cancellation_requested = [&]() {
    return progress_count >= 1U;
  };

  GeneticSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::cancelled);
  REQUIRE(result.iterations_completed == 0U);
  REQUIRE(result.progress.size() == 1U);
}

TEST_CASE("genetic search discards a cancelled in-flight generation",
          "[search][genetic][cancellation][batch]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
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

  GeneticSearch search;
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

TEST_CASE("genetic search discards a timed-out in-flight generation",
          "[search][genetic][timeout][batch]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
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

  GeneticSearch search;
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

TEST_CASE("genetic search does not cache an interrupted baseline decode",
          "[search][genetic][cache][cancellation]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.execution.control.capture_timestamps = false;
  request.execution.cancellation_requested = []() { return true; };

  GeneticSearch search;
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

TEST_CASE("genetic search plateau budget bounds stalled generations",
          "[search][genetic][plateau]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture = find_fixture(
      root, "genetic_search_preserves_full_layout_when_baseline_is_optimal");

  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.genetic_search.plateau_generations = 1;

  GeneticSearch immediate_search;
  const auto immediate = immediate_search.improve(request);
  REQUIRE(immediate.iterations_completed == 1U);
  REQUIRE_FALSE(immediate.improved());

  request.genetic_search.plateau_generations = 3;

  GeneticSearch plateau_search;
  const auto plateau = plateau_search.improve(request);
  REQUIRE(plateau.iterations_completed == 3U);
  REQUIRE_FALSE(plateau.improved());
  REQUIRE(plateau.best.piece_order == immediate.best.piece_order);
}

TEST_CASE("genetic search requires the GA config to be enabled",
          "[search][genetic][config]") {
  const auto root = load_fixture_file("search/genetic_scenario_cases.json");
  const auto &fixture =
      find_fixture(root, "genetic_search_reorders_tall_piece_forward");
  auto request =
      parse_search_request(fixture.get_child("inputs").get_child("request"));
  request.genetic_search.enabled = false;

  GeneticSearch search;
  const auto result = search.improve(request);

  REQUIRE(result.status == SearchRunStatus::invalid_request);
  REQUIRE(result.progress.empty());
  REQUIRE(result.events.empty());
}
