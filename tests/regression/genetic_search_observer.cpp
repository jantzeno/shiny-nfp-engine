#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "packing/decoder.hpp"
#include "search/genetic.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nfp::AlgorithmKind;
using shiny::nfp::pack::BinInput;
using shiny::nfp::pack::PackingConfig;
using shiny::nfp::pack::PieceInput;
using shiny::nfp::place::PlacementPolicy;
using shiny::nfp::search::SearchEvent;
using shiny::nfp::search::SearchEventKind;
using shiny::nfp::search::SearchRequest;
using shiny::nfp::search::SearchRunStatus;
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
  throw std::runtime_error("unknown GA observer grain compatibility");
}

auto parse_ids(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> values;
  for (const auto &child : node) {
    values.push_back(child.second.get_value<std::uint32_t>());
  }
  return values;
}

auto parse_event_kinds(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<SearchEventKind> {
  std::vector<SearchEventKind> values;
  for (const auto &child : node) {
    const auto parsed = shiny::nfp::search::parse_search_event_kind(
        child.second.get_value<std::string>());
    if (!parsed.has_value()) {
      throw std::runtime_error("unknown GA observer fixture event kind");
    }
    values.push_back(*parsed);
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
  throw std::runtime_error("unknown GA observer policy");
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

auto parse_packing_config(const shiny::nfp::test::pt::ptree &node)
    -> PackingConfig {
  PackingConfig config{};
  if (const auto placement = node.get_child_optional("placement")) {
    if (const auto rotations =
            placement->get_child_optional("allowed_rotations")) {
      config.placement.allowed_rotations.angles_degrees =
          parse_rotations(*rotations);
    }
  }
  if (const auto hole_first =
          node.get_optional<bool>("enable_hole_first_placement")) {
    config.enable_hole_first_placement = *hole_first;
  }
  return config;
}

auto parse_search_request(const shiny::nfp::test::pt::ptree &node)
    -> SearchRequest {
  SearchRequest request{};
  if (const auto local_search = node.get_child_optional("local_search")) {
    request.local_search.max_iterations =
        local_search->get<std::uint32_t>("max_iterations", 250);
    request.local_search.deterministic_seed =
        local_search->get<std::uint32_t>("deterministic_seed", 1);
    request.local_search.plateau_budget =
        local_search->get<std::uint32_t>("plateau_budget", 3);
  }
  if (const auto genetic_search = node.get_child_optional("genetic_search")) {
    request.genetic_search.max_generations =
        genetic_search->get<std::uint32_t>("max_generations", 40);
    request.genetic_search.population_size =
        genetic_search->get<std::uint32_t>("population_size", 30);
    request.genetic_search.deterministic_seed =
        genetic_search->get<std::uint32_t>("deterministic_seed", 1);
    request.genetic_search.mutation_rate_percent =
        genetic_search->get<std::uint8_t>("mutation_rate_percent", 10);
    request.genetic_search.elite_count =
        genetic_search->get<std::uint32_t>("elite_count", 2);
    request.genetic_search.tournament_size =
        genetic_search->get<std::uint32_t>("tournament_size", 3);
    request.genetic_search.plateau_generations =
        genetic_search->get<std::uint32_t>("plateau_generations", 6);
    request.genetic_search.enable_local_search_polish =
        genetic_search->get<bool>("enable_local_search_polish", true);
    request.genetic_search.enabled = genetic_search->get<bool>("enabled", true);
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

auto parse_run_status(std::string_view value) -> SearchRunStatus {
  if (value == "completed") {
    return SearchRunStatus::completed;
  }
  if (value == "cancelled") {
    return SearchRunStatus::cancelled;
  }
  if (value == "timed_out") {
    return SearchRunStatus::timed_out;
  }
  throw std::runtime_error("unknown GA observer run status");
}

} // namespace

TEST_CASE("genetic search observer regression fixtures",
          "[regression][search][genetic][observer]") {
  const auto root = load_fixture_file("search/genetic_observer_cases.json");
  REQUIRE(root.get<std::string>("algorithm") == "genetic_search");

  const auto parsed_algorithm =
      shiny::nfp::parse_algorithm_kind(root.get<std::string>("algorithm"));
  REQUIRE(parsed_algorithm.has_value());
  REQUIRE(*parsed_algorithm == AlgorithmKind::genetic_search);

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    DYNAMIC_SECTION(fixture.get<std::string>("id")) {
      require_fixture_metadata(fixture, "search_observer");

      auto request = parse_search_request(
          fixture.get_child("inputs").get_child("request"));
      std::vector<SearchEvent> live_events;
      std::size_t progress_count = 0U;

      request.execution.observer.on_event = [&](const SearchEvent &event) {
        if (shiny::nfp::search::search_event_kind(event) ==
            SearchEventKind::step_progress) {
          ++progress_count;
        }
        live_events.push_back(event);
      };

      if (fixture.get_child("inputs").get<std::string>("mode") ==
          "cancel_after_first_progress") {
        request.execution.cancellation_requested = [&]() {
          return progress_count >= 1U;
        };
      }

      shiny::nfp::search::GeneticSearch search;
      const auto result = search.improve(request);
      const auto &expected = fixture.get_child("expected");

      REQUIRE(result.algorithm == AlgorithmKind::genetic_search);
      REQUIRE(result.status ==
              parse_run_status(expected.get<std::string>("status")));

      const auto expected_event_kinds =
          parse_event_kinds(expected.get_child("event_kinds"));
      REQUIRE(live_events.size() == expected_event_kinds.size());

      for (std::size_t index = 0; index < live_events.size(); ++index) {
        REQUIRE(shiny::nfp::search::search_event_kind(live_events[index]) ==
                expected_event_kinds[index]);
        REQUIRE(shiny::nfp::search::search_event_algorithm_kind(
                    live_events[index]) == AlgorithmKind::genetic_search);
      }

      std::vector<std::uint32_t> progress_iterations;
      std::vector<std::uint32_t> improvement_iterations;
      for (const auto &event : live_events) {
        std::visit(
            [&](const auto &payload) {
              using Payload = std::decay_t<decltype(payload)>;
              if constexpr (std::is_same_v<
                                Payload,
                                shiny::nfp::search::SearchStepProgressEvent>) {
                progress_iterations.push_back(payload.progress.iteration);
              } else if constexpr (std::is_same_v<
                                       Payload,
                                       shiny::nfp::search::
                                           SearchImprovementFoundEvent>) {
                improvement_iterations.push_back(payload.progress.iteration);
              }
            },
            event);
      }

      REQUIRE(progress_iterations ==
              parse_ids(expected.get_child("progress_iterations")));
      REQUIRE(improvement_iterations ==
              parse_ids(expected.get_child("improvement_iterations")));
      REQUIRE(result.best.piece_order ==
              parse_ids(expected.get_child("best_piece_order")));
    }
  }
}
