#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "packing/decoder.hpp"
#include "search/jostle.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::AlgorithmKind;
using shiny::nesting::pack::BinInput;
using shiny::nesting::pack::PackingConfig;
using shiny::nesting::pack::PieceInput;
using shiny::nesting::place::PlacementPolicy;
using shiny::nesting::search::SearchEvent;
using shiny::nesting::search::SearchEventKind;
using shiny::nesting::search::SearchRequest;
using shiny::nesting::search::SearchRunStatus;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::require_fixture_metadata;

auto parse_rotations(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<double> {
  std::vector<double> rotations;
  for (const auto &child : node) {
    rotations.push_back(child.second.get_value<double>());
  }
  return rotations;
}

auto parse_bed_grain_direction(std::string_view value)
    -> shiny::nesting::place::BedGrainDirection {
  if (value == "unrestricted") {
    return shiny::nesting::place::BedGrainDirection::unrestricted;
  }
  if (value == "along_x") {
    return shiny::nesting::place::BedGrainDirection::along_x;
  }
  if (value == "along_y") {
    return shiny::nesting::place::BedGrainDirection::along_y;
  }
  throw std::runtime_error("unknown observer fixture bed grain direction");
}

auto parse_part_grain_compatibility(std::string_view value)
    -> shiny::nesting::place::PartGrainCompatibility {
  if (value == "unrestricted") {
    return shiny::nesting::place::PartGrainCompatibility::unrestricted;
  }
  if (value == "parallel_to_bed") {
    return shiny::nesting::place::PartGrainCompatibility::parallel_to_bed;
  }
  if (value == "perpendicular_to_bed") {
    return shiny::nesting::place::PartGrainCompatibility::perpendicular_to_bed;
  }
  throw std::runtime_error("unknown observer fixture grain compatibility");
}

auto parse_exclusion_zones(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::place::BedExclusionZone> {
  std::vector<shiny::nesting::place::BedExclusionZone> zones;
  for (const auto &child : node) {
    zones.push_back({
        .zone_id = child.second.get<std::uint32_t>("zone_id", 0),
        .region = {.outer = shiny::nesting::test::parse_ring(
                       child.second.get_child("region"))},
    });
  }
  return zones;
}

auto parse_ids(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> values;
  for (const auto &child : node) {
    values.push_back(child.second.get_value<std::uint32_t>());
  }
  return values;
}

auto parse_event_kinds(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<SearchEventKind> {
  std::vector<SearchEventKind> values;
  values.reserve(node.size());
  for (const auto &child : node) {
    const auto parsed = shiny::nesting::search::parse_search_event_kind(
        child.second.get_value<std::string>());
    if (!parsed.has_value()) {
      throw std::runtime_error("unknown observer fixture event kind");
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
  throw std::runtime_error("unknown observer fixture policy");
}

auto parse_bin_input(const shiny::nesting::test::pt::ptree &node) -> BinInput {
  return {
      .bin_id = node.get<std::uint32_t>("base_bin_id", 0),
      .polygon = parse_polygon(node.get_child("polygon")),
      .geometry_revision = node.get<std::uint64_t>("geometry_revision", 0),
  };
}

auto resolve_fixture_bin_count(const std::size_t max_bin_count,
                               const std::size_t piece_count) -> std::size_t {
  if (max_bin_count != 0U) {
    return max_bin_count;
  }
  return std::max<std::size_t>(piece_count, 1U);
}

auto expand_bins(BinInput base_bin, const std::size_t count)
    -> std::vector<BinInput> {
  std::vector<BinInput> bins;
  bins.reserve(std::max<std::size_t>(count, 1));
  for (std::size_t index = 0; index < std::max<std::size_t>(count, 1);
       ++index) {
    BinInput bin = base_bin;
    bin.bin_id = base_bin.bin_id + static_cast<std::uint32_t>(index);
    bins.push_back(std::move(bin));
  }
  return bins;
}

auto parse_piece_inputs(const shiny::nesting::test::pt::ptree &node)
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
        .allowed_bin_ids =
            [&]() {
              if (const auto ids =
                      child.second.get_child_optional("allowed_bin_ids")) {
                return parse_ids(*ids);
              }
              return std::vector<std::uint32_t>{};
            }(),
    });
  }
  return pieces;
}

auto parse_packing_config(const shiny::nesting::test::pt::ptree &node)
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
  return config;
}

auto parse_search_request(const shiny::nesting::test::pt::ptree &node)
    -> SearchRequest {
  SearchRequest request{};
  if (const auto local_search = node.get_child_optional("local_search")) {
    request.local_search.max_refinements =
        local_search->get<std::uint32_t>("max_iterations", 250);
    request.local_search.deterministic_seed =
        local_search->get<std::uint32_t>("deterministic_seed", 1);
    request.local_search.plateau_budget =
        local_search->get<std::uint32_t>("plateau_budget", 3);
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
  throw std::runtime_error("unknown observer fixture run status");
}

} // namespace

TEST_CASE("search observer regression fixtures",
          "[regression][search][observer]") {
  const auto root = load_fixture_file("search/observer_cases.json");
  REQUIRE(root.get<std::string>("algorithm") == "jostle_search");

  const auto parsed_algorithm =
      shiny::nesting::parse_algorithm_kind(root.get<std::string>("algorithm"));
  REQUIRE(parsed_algorithm.has_value());
  REQUIRE(*parsed_algorithm == AlgorithmKind::jostle_search);

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "search_observer");

      auto request = parse_search_request(
          fixture.get_child("inputs").get_child("request"));
      std::vector<SearchEvent> live_events;
      std::size_t progress_count = 0U;

      request.execution.observer.on_event = [&](const SearchEvent &event) {
        if (shiny::nesting::search::search_event_kind(event) ==
            SearchEventKind::step_progress) {
          ++progress_count;
        }
        live_events.push_back(event);
      };

      const auto mode = fixture.get_child("inputs").get<std::string>("mode");
      if (mode == "cancel_after_first_progress") {
        request.execution.cancellation_requested = [&]() {
          return progress_count >= 1U;
        };
      }

      shiny::nesting::search::JostleSearch search;
      const auto result = search.improve(request);
      const auto &expected = fixture.get_child("expected");

      REQUIRE(result.algorithm == AlgorithmKind::jostle_search);
      REQUIRE(result.status ==
              parse_run_status(expected.get<std::string>("status")));
      REQUIRE(live_events.size() == result.events.size());

      const auto expected_event_kinds =
          parse_event_kinds(expected.get_child("event_kinds"));
      REQUIRE(live_events.size() == expected_event_kinds.size());
      for (std::size_t index = 0; index < live_events.size(); ++index) {
        REQUIRE(shiny::nesting::search::search_event_kind(live_events[index]) ==
                expected_event_kinds[index]);
        REQUIRE(shiny::nesting::search::search_event_kind(
                    result.events[index]) == expected_event_kinds[index]);
        REQUIRE(shiny::nesting::search::search_event_algorithm_kind(
                    live_events[index]) == AlgorithmKind::jostle_search);
      }

      std::vector<std::uint32_t> progress_iterations;
      std::vector<std::uint32_t> improvement_iterations;
      for (const auto &event : live_events) {
        std::visit(
            [&](const auto &payload) {
              using Payload = std::decay_t<decltype(payload)>;
              if constexpr (std::is_same_v<Payload,
                                           shiny::nesting::search::
                                               SearchStepProgressEvent>) {
                progress_iterations.push_back(payload.progress.iteration);
              } else if constexpr (std::is_same_v<
                                       Payload,
                                       shiny::nesting::search::
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
