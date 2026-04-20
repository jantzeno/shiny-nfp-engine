#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "geometry/transform.hpp"
#include "placement/engine.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using Catch::Approx;
using shiny::nesting::NfpFeatureKind;
using shiny::nesting::NfpLoop;
using shiny::nesting::NfpResult;
using shiny::nesting::place::BedGrainDirection;
using shiny::nesting::place::grain_compatibility_allows_rotation;
using shiny::nesting::place::PartGrainCompatibility;
using shiny::nesting::place::PlacementConfig;
using shiny::nesting::place::PlacementEngine;
using shiny::nesting::place::PlacementPolicy;
using shiny::nesting::place::PlacementRequest;
using shiny::nesting::place::RankedPlacementSet;
using shiny::nesting::place::rotation_is_allowed;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::parse_ring;
using shiny::nesting::test::parse_segment;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_point_equal;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> shiny::nesting::geom::PolygonWithHoles {
  return {
      .outer =
          {
              {.x = min_x, .y = min_y},
              {.x = max_x, .y = min_y},
              {.x = max_x, .y = max_y},
              {.x = min_x, .y = max_y},
          },
  };
}

auto parse_points(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Point2> {
  std::vector<shiny::nesting::geom::Point2> points;
  for (const auto &child : node) {
    points.push_back(parse_point(child.second));
  }
  return points;
}

auto parse_segments(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Segment2> {
  std::vector<shiny::nesting::geom::Segment2> segments;
  for (const auto &child : node) {
    segments.push_back(parse_segment(child.second));
  }
  return segments;
}

auto parse_polygons(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::PolygonWithHoles> {
  std::vector<shiny::nesting::geom::PolygonWithHoles> polygons;
  for (const auto &child : node) {
    polygons.push_back(parse_polygon(child.second));
  }
  return polygons;
}

auto parse_rotations(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<double> {
  std::vector<double> rotations;
  for (const auto &child : node) {
    rotations.push_back(child.second.get_value<double>());
  }
  return rotations;
}

auto parse_bed_grain_direction(std::string_view value) -> BedGrainDirection {
  if (value == "unrestricted") {
    return BedGrainDirection::unrestricted;
  }
  if (value == "along_x") {
    return BedGrainDirection::along_x;
  }
  if (value == "along_y") {
    return BedGrainDirection::along_y;
  }
  throw std::runtime_error("unknown placement bed grain direction");
}

auto parse_part_grain_compatibility(std::string_view value)
    -> PartGrainCompatibility {
  if (value == "unrestricted") {
    return PartGrainCompatibility::unrestricted;
  }
  if (value == "parallel_to_bed") {
    return PartGrainCompatibility::parallel_to_bed;
  }
  if (value == "perpendicular_to_bed") {
    return PartGrainCompatibility::perpendicular_to_bed;
  }
  throw std::runtime_error("unknown placement part grain compatibility");
}

auto parse_exclusion_zones(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::place::BedExclusionZone> {
  std::vector<shiny::nesting::place::BedExclusionZone> zones;
  for (const auto &child : node) {
    zones.push_back({
        .zone_id = child.second.get<std::uint32_t>("zone_id", 0),
        .region = {.outer = parse_ring(child.second.get_child("region"))},
    });
  }
  return zones;
}

auto parse_rotation_index_value(const shiny::nesting::test::pt::ptree &node,
                                std::string_view key)
    -> shiny::nesting::geom::RotationIndex {
  return {.value = node.get<std::uint16_t>(std::string{key})};
}

auto parse_bools(const shiny::nesting::test::pt::ptree &node) -> std::vector<bool> {
  std::vector<bool> values;
  for (const auto &child : node) {
    values.push_back(child.second.get_value<bool>());
  }
  return values;
}

auto parse_indices(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<std::int32_t> {
  std::vector<std::int32_t> values;
  for (const auto &child : node) {
    values.push_back(child.second.get_value<std::int32_t>());
  }
  return values;
}

auto parse_config(const shiny::nesting::test::pt::ptree &node) -> PlacementConfig {
  PlacementConfig config{};
  if (const auto part_clearance = node.get_optional<double>("part_clearance")) {
    config.part_clearance = *part_clearance;
  }
  if (const auto rotations = node.get_child_optional("allowed_rotations")) {
    config.allowed_rotations.angles_degrees = parse_rotations(*rotations);
  }
  if (const auto grain_direction =
          node.get_optional<std::string>("bed_grain_direction")) {
    config.bed_grain_direction = parse_bed_grain_direction(*grain_direction);
  }
  if (const auto enable_part_in_part =
          node.get_optional<bool>("enable_part_in_part_placement")) {
    config.enable_part_in_part_placement = *enable_part_in_part;
  }
  if (const auto explore_concave =
          node.get_optional<bool>("explore_concave_candidates")) {
    config.explore_concave_candidates = *explore_concave;
  }
  if (const auto exclusion_zones = node.get_child_optional("exclusion_zones")) {
    config.exclusion_zones = parse_exclusion_zones(*exclusion_zones);
  }
  return config;
}

auto parse_nfp_result(const shiny::nesting::test::pt::ptree &node) -> NfpResult {
  NfpResult result{};

  if (const auto outer_loops = node.get_child_optional("outer_loops")) {
    for (const auto &child : *outer_loops) {
      result.loops.push_back(NfpLoop{.vertices = parse_ring(child.second),
                                     .kind = NfpFeatureKind::outer_loop});
    }
  }
  if (const auto holes = node.get_child_optional("holes")) {
    for (const auto &child : *holes) {
      result.loops.push_back(NfpLoop{.vertices = parse_ring(child.second),
                                     .kind = NfpFeatureKind::hole});
    }
  }
  if (const auto points = node.get_child_optional("perfect_fit_points")) {
    result.perfect_fit_points = parse_points(*points);
  }
  if (const auto segments =
          node.get_child_optional("perfect_sliding_segments")) {
    result.perfect_sliding_segments = parse_segments(*segments);
  }
  return result;
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
  throw std::runtime_error("unknown placement policy fixture value");
}

auto parse_request(const shiny::nesting::test::pt::ptree &node)
    -> PlacementRequest {
  PlacementRequest request{
      .bin_id = node.get<std::uint32_t>("bin_id"),
      .piece_id = node.get<std::uint32_t>("piece_id"),
      .piece = parse_polygon(node.get_child("piece")),
      .bin = parse_polygon(node.get_child("bin")),
      .merged_region = parse_polygon(node.get_child("merged_region")),
      .rotation_index = parse_rotation_index_value(node, "rotation_index"),
      .piece_geometry_revision =
          node.get<std::uint64_t>("piece_geometry_revision", 0),
      .bin_geometry_revision =
          node.get<std::uint64_t>("bin_geometry_revision", 0),
      .merged_region_revision =
          node.get<std::uint64_t>("merged_region_revision"),
      .hole_set_revision = node.get<std::uint64_t>("hole_set_revision"),
  };

  if (const auto holes = node.get_child_optional("holes")) {
    request.holes = parse_polygons(*holes);
  }
  if (const auto compatibility =
          node.get_optional<std::string>("part_grain_compatibility")) {
    request.part_grain_compatibility =
        parse_part_grain_compatibility(*compatibility);
  }
  if (const auto config = node.get_child_optional("config")) {
    request.config = parse_config(*config);
  }
  return request;
}

void require_ordered_translations(
    const std::vector<shiny::nesting::place::PlacementCandidate> &actual,
    const std::vector<shiny::nesting::geom::Point2> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    require_point_equal(actual[index].translation, expected[index]);
  }
}

void require_ranked_result_matches(
    const RankedPlacementSet &result,
    const shiny::nesting::test::pt::ptree &expected) {
  REQUIRE(result.candidates.size() ==
          expected.get<std::size_t>("candidate_count"));
  if (const auto ordered_translations =
          expected.get_child_optional("ordered_translations")) {
    require_ordered_translations(result.candidates,
                                 parse_points(*ordered_translations));
  }

  const auto *best = result.best();
  if (const auto best_translation =
          expected.get_child_optional("best_translation")) {
    REQUIRE(best != nullptr);
    require_point_equal(best->translation, parse_point(*best_translation));
  } else {
    REQUIRE(best == nullptr);
  }

  if (const auto best_score = expected.get_optional<double>("best_score")) {
    REQUIRE(best != nullptr);
    REQUIRE(best->score == Approx(*best_score));
  }
  if (const auto best_inside_hole =
          expected.get_optional<bool>("best_inside_hole")) {
    REQUIRE(best != nullptr);
    REQUIRE(best->inside_hole == *best_inside_hole);
  }
  if (const auto best_hole_index =
          expected.get_optional<std::int32_t>("best_hole_index")) {
    REQUIRE(best != nullptr);
    REQUIRE(best->hole_index == *best_hole_index);
  }
  if (const auto ordered_inside_hole =
          expected.get_child_optional("ordered_inside_hole")) {
    const auto flags = parse_bools(*ordered_inside_hole);
    REQUIRE(result.candidates.size() == flags.size());
    for (std::size_t index = 0; index < flags.size(); ++index) {
      REQUIRE(result.candidates[index].inside_hole == flags[index]);
    }
  }
  if (const auto ordered_hole_indices =
          expected.get_child_optional("ordered_hole_indices")) {
    const auto values = parse_indices(*ordered_hole_indices);
    REQUIRE(result.candidates.size() == values.size());
    for (std::size_t index = 0; index < values.size(); ++index) {
      REQUIRE(result.candidates[index].hole_index == values[index]);
    }
  }
}

} // namespace

TEST_CASE("placement config fixtures", "[placement][config][fixtures]") {
  const auto root = load_fixture_file("placement/config_surface.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "placement_config");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto config = parse_config(inputs.get_child("config"));
      const auto rotation_index =
          parse_rotation_index_value(inputs, "rotation_index");

      REQUIRE(config.is_valid() == expected.get<bool>("valid"));
      REQUIRE(rotation_is_allowed(rotation_index, config) ==
              expected.get<bool>("rotation_allowed"));
    }
  }
}

TEST_CASE("placement grain compatibility respects the bed grain axis",
          "[placement][config][grain]") {
  REQUIRE(grain_compatibility_allows_rotation(
      {.degrees = 0.0}, BedGrainDirection::along_x,
      PartGrainCompatibility::parallel_to_bed));
  REQUIRE_FALSE(grain_compatibility_allows_rotation(
      {.degrees = 90.0}, BedGrainDirection::along_x,
      PartGrainCompatibility::parallel_to_bed));
  REQUIRE(grain_compatibility_allows_rotation(
      {.degrees = 90.0}, BedGrainDirection::along_x,
      PartGrainCompatibility::perpendicular_to_bed));
  REQUIRE(grain_compatibility_allows_rotation(
      {.degrees = 270.0}, BedGrainDirection::along_y,
      PartGrainCompatibility::parallel_to_bed));
  REQUIRE_FALSE(grain_compatibility_allows_rotation(
      {.degrees = 45.0}, BedGrainDirection::along_y,
      PartGrainCompatibility::parallel_to_bed));
}

TEST_CASE("placement config rejects invalid exclusion zones",
          "[placement][config][exclusion]") {
  PlacementConfig config{};
  config.exclusion_zones.push_back({
      .zone_id = 3,
      .region = {.outer = {{0.0, 0.0}, {1.0, 0.0}}},
  });

  REQUIRE_FALSE(config.is_valid());
}

TEST_CASE("placement candidate extraction fixtures",
          "[placement][extract][fixtures]") {
  const auto root = load_fixture_file("placement/candidate_extraction.json");
  PlacementEngine engine;

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "placement_candidate_extraction");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = parse_nfp_result(inputs.get_child("result"));
      const auto config = parse_config(inputs.get_child("config"));
      const auto rotation_index =
          parse_rotation_index_value(inputs, "rotation_index");

      const auto candidates =
          engine.extract_candidates(result, rotation_index, config);

      REQUIRE(candidates.size() ==
              expected.get<std::size_t>("candidate_count"));
      require_ordered_translations(
          candidates, parse_points(expected.get_child("ordered_translations")));
    }
  }
}

TEST_CASE("placement query fixtures", "[placement][query][fixtures]") {
  const auto root = load_fixture_file("placement/query_cases.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "placement_query");

      PlacementEngine engine;
      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");

      const auto request = parse_request(inputs.get_child("request"));
      const auto policy = parse_policy(inputs.get<std::string>("policy"));

      std::optional<NfpResult> occupied_region_nfp;
      if (const auto node = inputs.get_child_optional("occupied_region_nfp")) {
        occupied_region_nfp = parse_nfp_result(*node);
      }

      std::optional<NfpResult> bin_ifp;
      if (const auto node = inputs.get_child_optional("bin_ifp")) {
        bin_ifp = parse_nfp_result(*node);
      }

      std::vector<NfpResult> hole_ifps;
      if (const auto node = inputs.get_child_optional("hole_ifps")) {
        for (const auto &child : *node) {
          hole_ifps.push_back(parse_nfp_result(child.second));
        }
      }

      const auto result = engine.query_candidates(
          request, occupied_region_nfp ? &*occupied_region_nfp : nullptr,
          bin_ifp ? &*bin_ifp : nullptr, std::span<const NfpResult>(hole_ifps),
          policy);

      require_ranked_result_matches(result, expected);
      if (const auto cache_size =
              expected.get_optional<std::size_t>("cache_size_after_first")) {
        REQUIRE(engine.cache_size() == *cache_size);
      }

      if (inputs.get("repeat_same_query", false)) {
        const auto repeated = engine.query_candidates(
            request, occupied_region_nfp ? &*occupied_region_nfp : nullptr,
            bin_ifp ? &*bin_ifp : nullptr,
            std::span<const NfpResult>(hole_ifps), policy);
        require_ranked_result_matches(repeated, expected);
        if (const auto cache_size =
                expected.get_optional<std::size_t>("cache_size_after_second")) {
          REQUIRE(engine.cache_size() == *cache_size);
        }
      } else if (const auto second_request =
                     inputs.get_child_optional("second_request")) {
        const auto repeated = engine.query_candidates(
            parse_request(*second_request),
            occupied_region_nfp ? &*occupied_region_nfp : nullptr,
            bin_ifp ? &*bin_ifp : nullptr,
            std::span<const NfpResult>(hole_ifps), policy);
        const auto expected_second = expected.get_child("second");
        require_ranked_result_matches(repeated, expected_second);
        if (const auto cache_size = expected_second.get_optional<std::size_t>(
                "cache_size_after_second")) {
          REQUIRE(engine.cache_size() == *cache_size);
        }
      }
    }
  }
}

TEST_CASE("placement filtering rejects candidates that overlap keep-outs",
          "[placement][filter][exclusion]") {
  PlacementEngine engine;
  PlacementRequest request{
      .bin_id = 1,
      .piece_id = 2,
      .piece = make_rectangle(0.0, 0.0, 2.0, 2.0),
      .bin = make_rectangle(0.0, 0.0, 6.0, 6.0),
      .merged_region = {},
      .rotation_index = {.value = 0},
      .piece_geometry_revision = 1,
      .bin_geometry_revision = 2,
      .merged_region_revision = 0,
      .hole_set_revision = 0,
      .config =
          PlacementConfig{
              .allowed_rotations = {.angles_degrees = {0.0}},
              .exclusion_zones = {{
                  .zone_id = 9,
                  .region = {.outer = {{0.0, 0.0},
                                       {3.0, 0.0},
                                       {3.0, 3.0},
                                       {0.0, 3.0}}},
              }},
          },
  };

  const std::vector<shiny::nesting::place::PlacementCandidate> candidates{
      {.translation = {0.0, 0.0}, .rotation_index = {.value = 0}},
      {.translation = {3.0, 0.0}, .rotation_index = {.value = 0}},
  };

  const auto filtered = engine.filter_candidates(request, candidates);
  REQUIRE(filtered.size() == 1U);
  require_point_equal(filtered.front().translation, {3.0, 0.0});
}