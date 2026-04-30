#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "algorithm_kind.hpp"
#include "packing/masonry.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::AlgorithmKind;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::MasonryBuilder;
using shiny::nesting::pack::MasonryRequest;
using shiny::nesting::pack::PackingConfig;
using shiny::nesting::pack::PieceInput;
using shiny::nesting::place::BedExclusionZone;
using shiny::nesting::place::BedGrainDirection;
using shiny::nesting::place::PartGrainCompatibility;
using shiny::nesting::place::PlacementPolicy;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::parse_ring;
using shiny::nesting::test::require_fixture_metadata;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
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

auto make_donut() -> PolygonWithHoles {
  return {
      .outer =
          {
              {.x = 0.0, .y = 0.0},
              {.x = 6.0, .y = 0.0},
              {.x = 6.0, .y = 6.0},
              {.x = 0.0, .y = 6.0},
          },
      .holes = {{
          {.x = 2.0, .y = 2.0},
          {.x = 4.0, .y = 2.0},
          {.x = 4.0, .y = 4.0},
          {.x = 2.0, .y = 4.0},
      }},
  };
}

auto make_piece(std::uint32_t piece_id, PolygonWithHoles polygon,
                std::uint64_t geometry_revision,
                PartGrainCompatibility grain_compatibility =
                    PartGrainCompatibility::unrestricted) -> PieceInput {
  return {
      .piece_id = piece_id,
      .polygon = std::move(polygon),
      .geometry_revision = geometry_revision,
      .grain_compatibility = grain_compatibility,
  };
}

auto compute_bounds(const PolygonWithHoles &polygon)
    -> std::pair<Point2, Point2> {
  Point2 minimum{.x = std::numeric_limits<double>::infinity(),
                 .y = std::numeric_limits<double>::infinity()};
  Point2 maximum{.x = -std::numeric_limits<double>::infinity(),
                 .y = -std::numeric_limits<double>::infinity()};

  auto accumulate = [&](const auto &ring) {
    for (const auto &point : ring) {
      minimum.x = std::min(minimum.x, point.x);
      minimum.y = std::min(minimum.y, point.y);
      maximum.x = std::max(maximum.x, point.x);
      maximum.y = std::max(maximum.y, point.y);
    }
  };

  accumulate(polygon.outer);
  for (const auto &hole : polygon.holes) {
    accumulate(hole);
  }

  return {minimum, maximum};
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
  throw std::runtime_error("unknown masonry bed grain direction");
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
  throw std::runtime_error("unknown masonry part grain compatibility");
}

auto parse_exclusion_zones(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<BedExclusionZone> {
  std::vector<BedExclusionZone> zones;
  for (const auto &child : node) {
    zones.push_back({
        .zone_id = child.second.get<std::uint32_t>("zone_id", 0),
        .region = {.outer = parse_ring(child.second.get_child("region"))},
    });
  }
  return zones;
}

auto parse_ids(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> ids;
  for (const auto &child : node) {
    ids.push_back(child.second.get_value<std::uint32_t>());
  }
  return ids;
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
  throw std::runtime_error("unknown masonry fixture policy");
}

auto parse_masonry_request(const shiny::nesting::test::pt::ptree &node)
    -> MasonryRequest {
  MasonryRequest request{};
  const auto &decoder = node;
  request.decoder_request.policy =
      parse_policy(decoder.get<std::string>("policy", "bottom_left"));
  const auto max_bins = decoder.get<std::size_t>("max_bin_count", 1);
  const auto base_bin_id =
      decoder.get_child("bin").get<std::uint32_t>("base_bin_id", 0);
  const auto base_polygon =
      parse_polygon(decoder.get_child("bin").get_child("polygon"));
  const auto base_geometry_revision =
      decoder.get_child("bin").get<std::uint64_t>("geometry_revision", 0);

  for (const auto &child : decoder.get_child("pieces")) {
    request.decoder_request.pieces.push_back({
        .piece_id = child.second.get<std::uint32_t>("piece_id"),
        .polygon = parse_polygon(child.second.get_child("polygon")),
        .geometry_revision =
            child.second.get<std::uint64_t>("geometry_revision", 0),
        .grain_compatibility =
            parse_part_grain_compatibility(child.second.get<std::string>(
                "grain_compatibility", "unrestricted")),
    });
  }

  const auto bin_count =
      max_bins == 0U
          ? std::max<std::size_t>(request.decoder_request.pieces.size(), 1U)
          : max_bins;
  request.decoder_request.bins.reserve(bin_count);
  for (std::size_t index = 0; index < bin_count; ++index) {
    request.decoder_request.bins.push_back({
        .bin_id = base_bin_id + static_cast<std::uint32_t>(index),
        .polygon = base_polygon,
        .geometry_revision = base_geometry_revision,
    });
  }

  if (const auto config = decoder.get_child_optional("config")) {
    if (const auto placement = config->get_child_optional("placement")) {
      if (const auto rotations =
              placement->get_child_optional("allowed_rotations")) {
        request.decoder_request.config.placement.allowed_rotations
            .angles_degrees = parse_rotations(*rotations);
      }
      if (const auto grain =
              placement->get_optional<std::string>("bed_grain_direction")) {
        request.decoder_request.config.placement.bed_grain_direction =
            parse_bed_grain_direction(*grain);
      }
      if (const auto exclusion =
              placement->get_child_optional("exclusion_zones")) {
        request.decoder_request.config.placement.exclusion_zones =
            parse_exclusion_zones(*exclusion);
      }
      if (const auto enable_part_in_part =
              placement->get_optional<bool>("enable_part_in_part_placement")) {
        request.decoder_request.config.placement.enable_part_in_part_placement =
            *enable_part_in_part;
      }
    }
    if (const auto hole_first =
            config->get_optional<bool>("enable_hole_first_placement")) {
      request.decoder_request.config.enable_hole_first_placement = *hole_first;
    }
  }

  if (const auto masonry = decoder.get_child_optional("masonry")) {
    request.masonry.fill_existing_shelves_first =
        masonry->get<bool>("fill_existing_shelves_first", true);
    request.masonry.shelf_alignment_tolerance =
        masonry->get<double>("shelf_alignment_tolerance", 1e-6);
  }

  return request;
}

void require_trace_matches(
    const shiny::nesting::test::pt::ptree &expected_trace,
    const shiny::nesting::pack::MasonryResult &result) {
  REQUIRE(result.trace.size() == expected_trace.size());
  std::size_t index = 0;
  for (const auto &child : expected_trace) {
    const auto &expected = child.second;
    const auto &actual = result.trace[index++];
    REQUIRE(actual.piece_id == expected.get<std::uint32_t>("piece_id"));
    REQUIRE(actual.bin_id == expected.get<std::uint32_t>("bin_id"));
    REQUIRE(actual.shelf_index == expected.get<std::size_t>("shelf_index"));
    REQUIRE(actual.translation ==
            parse_point(expected.get_child("translation")));
    REQUIRE(actual.resolved_rotation.degrees ==
            expected.get<double>("resolved_rotation_degrees"));
    REQUIRE(actual.opened_new_bin == expected.get<bool>("opened_new_bin"));
    REQUIRE(actual.started_new_shelf ==
            expected.get<bool>("started_new_shelf"));
    REQUIRE(actual.inside_hole == expected.get<bool>("inside_hole", false));
    REQUIRE(actual.hole_index == expected.get<std::int32_t>("hole_index", -1));
  }
}

void require_progress_matches(
    const shiny::nesting::test::pt::ptree &expected_progress,
    const shiny::nesting::pack::MasonryResult &result) {
  REQUIRE(result.progress.size() == expected_progress.size());
  std::size_t index = 0;
  for (const auto &child : expected_progress) {
    const auto &expected = child.second;
    const auto &actual = result.progress[index++];
    REQUIRE(actual.current_piece_id ==
            expected.get<std::uint32_t>("current_piece_id"));
    REQUIRE(actual.processed_piece_count ==
            expected.get<std::size_t>("processed_piece_count"));
    REQUIRE(actual.placed_piece_count ==
            expected.get<std::size_t>("placed_piece_count"));
    REQUIRE(actual.unplaced_piece_count ==
            expected.get<std::size_t>("unplaced_piece_count"));
    REQUIRE(actual.bin_count == expected.get<std::size_t>("bin_count"));
  }
}

void require_same_trace(const shiny::nesting::pack::MasonryResult &lhs,
                        const shiny::nesting::pack::MasonryResult &rhs) {
  REQUIRE(lhs.trace.size() == rhs.trace.size());
  for (std::size_t index = 0; index < lhs.trace.size(); ++index) {
    const auto &left = lhs.trace[index];
    const auto &right = rhs.trace[index];
    REQUIRE(left.piece_id == right.piece_id);
    REQUIRE(left.bin_id == right.bin_id);
    REQUIRE(left.shelf_index == right.shelf_index);
    REQUIRE(left.rotation_index == right.rotation_index);
    REQUIRE(left.resolved_rotation.degrees == right.resolved_rotation.degrees);
    REQUIRE(left.translation == right.translation);
    REQUIRE(left.opened_new_bin == right.opened_new_bin);
    REQUIRE(left.started_new_shelf == right.started_new_shelf);
    REQUIRE(left.inside_hole == right.inside_hole);
    REQUIRE(left.hole_index == right.hole_index);
  }
}

} // namespace

TEST_CASE("masonry builder fills shelves deterministically",
          "[packing][masonry][determinism]") {
  MasonryBuilder builder;

  MasonryRequest request{};
  request.decoder_request = {
      .bins = {{
          .bin_id = 20,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 100,
      }},
      .pieces =
          {
              make_piece(1, make_rectangle(0.0, 0.0, 6.0, 4.0), 1),
              make_piece(2, make_rectangle(0.0, 0.0, 4.0, 4.0), 2),
              make_piece(3, make_rectangle(0.0, 0.0, 5.0, 3.0), 3),
          },
      .policy = PlacementPolicy::bottom_left,
      .config = PackingConfig{},
  };

  const auto first = builder.build(request);
  const auto second = builder.build(request);

  REQUIRE(first.algorithm == AlgorithmKind::masonry_builder);
  REQUIRE(first.layout.unplaced_piece_ids.empty());
  REQUIRE(first.bins.size() == 1);
  REQUIRE(first.trace.size() == 3);
  REQUIRE(first.progress.size() == 3);
  REQUIRE(first.progress[0].processed_piece_count == 1);
  REQUIRE(first.progress[0].placed_piece_count == 1);
  REQUIRE(first.progress[0].unplaced_piece_count == 0);
  REQUIRE(first.progress[1].processed_piece_count == 2);
  REQUIRE(first.progress[1].placed_piece_count == 2);
  REQUIRE(first.progress[2].processed_piece_count == 3);
  REQUIRE(first.progress[2].placed_piece_count == 3);
  REQUIRE(first.progress[2].bin_count == 1);

  REQUIRE(first.trace[0].opened_new_bin);
  REQUIRE(first.trace[0].started_new_shelf);
  REQUIRE(first.trace[0].shelf_index == 0);
  REQUIRE(first.trace[0].translation == Point2{.x = 0.0, .y = 0.0});

  REQUIRE_FALSE(first.trace[1].opened_new_bin);
  REQUIRE_FALSE(first.trace[1].started_new_shelf);
  REQUIRE(first.trace[1].shelf_index == 0);
  REQUIRE(first.trace[1].translation == Point2{.x = 6.0, .y = 0.0});

  REQUIRE_FALSE(first.trace[2].opened_new_bin);
  REQUIRE(first.trace[2].started_new_shelf);
  REQUIRE(first.trace[2].shelf_index == 1);
  REQUIRE(first.trace[2].resolved_rotation.degrees == 90.0);
  REQUIRE(first.trace[2].translation == Point2{.x = 3.0, .y = 5.0});

  require_same_trace(first, second);
}

TEST_CASE("masonry builder uses hole-aware placement when enabled",
          "[packing][masonry][holes]") {
  MasonryBuilder builder;

  PackingConfig config{};
  config.enable_hole_first_placement = true;
  config.placement.enable_part_in_part_placement = true;

  MasonryRequest request{};
  request.decoder_request = {
      .bins = {{
          .bin_id = 30,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 200,
      }},
      .pieces =
          {
              make_piece(10, make_donut(), 10),
              make_piece(11, make_rectangle(0.0, 0.0, 2.0, 2.0), 11),
          },
      .policy = PlacementPolicy::bottom_left,
      .config = config,
  };

  const auto result = builder.build(request);

  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.trace.size() == 2);
  REQUIRE(result.progress.size() == 2);
  REQUIRE(result.trace[1].inside_hole);
  REQUIRE(result.trace[1].hole_index == 0);
  REQUIRE(result.trace[1].translation == Point2{.x = 2.0, .y = 2.0});
  REQUIRE(result.bins.front().placements.size() == 2);
  REQUIRE(result.bins.front().placements[1].inside_hole);
}

TEST_CASE("masonry builder respects grain and exclusion constraints",
          "[packing][masonry][constraints]") {
  MasonryBuilder builder;

  PackingConfig config{};
  config.placement.allowed_rotations.angles_degrees = {0.0, 90.0};
  config.placement.bed_grain_direction = BedGrainDirection::along_y;
  config.placement.exclusion_zones = {
      BedExclusionZone{
          .zone_id = 1,
          .region = {.outer = make_rectangle(0.0, 0.0, 4.0, 2.0).outer},
      },
  };

  MasonryRequest request{};
  request.decoder_request = {
      .bins = {{
          .bin_id = 40,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 8.0),
          .geometry_revision = 300,
      }},
      .pieces =
          {
              make_piece(21, make_rectangle(0.0, 0.0, 4.0, 2.0), 21,
                         PartGrainCompatibility::parallel_to_bed),
          },
      .policy = PlacementPolicy::bottom_left,
      .config = config,
  };

  const auto result = builder.build(request);

  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.trace.size() == 1);
  REQUIRE(result.progress.size() == 1);
  REQUIRE(result.trace.front().resolved_rotation.degrees == 90.0);
  REQUIRE(result.trace.front().translation == Point2{.x = 10.0, .y = 0.0});

  const auto &placed_piece = result.bins.front().placements.front().polygon;
  const auto [minimum, maximum] = compute_bounds(placed_piece);
  REQUIRE(minimum == Point2{.x = 8.0, .y = 0.0});
  REQUIRE(maximum == Point2{.x = 10.0, .y = 4.0});
}

TEST_CASE("masonry builder honors top-right start corner",
          "[packing][masonry][start-corner]") {
  MasonryBuilder builder;

  MasonryRequest request{};
  request.decoder_request = {
      .bins = {{
          .bin_id = 50,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 400,
          .start_corner =
              shiny::nesting::place::PlacementStartCorner::top_right,
      }},
      .pieces =
          {
              make_piece(31, make_rectangle(0.0, 0.0, 6.0, 4.0), 31),
              make_piece(32, make_rectangle(0.0, 0.0, 4.0, 4.0), 32),
          },
      .policy = PlacementPolicy::bottom_left,
      .config = PackingConfig{},
  };

  const auto result = builder.build(request);

  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.trace.size() == 2);
  REQUIRE(result.trace[0].opened_new_bin);
  REQUIRE(result.trace[0].started_new_shelf);
  REQUIRE(result.trace[0].translation == Point2{.x = 4.0, .y = 6.0});
  REQUIRE_FALSE(result.trace[1].opened_new_bin);
  REQUIRE_FALSE(result.trace[1].started_new_shelf);
  REQUIRE(result.trace[1].translation == Point2{.x = 0.0, .y = 6.0});
}

TEST_CASE("masonry fixture cases", "[packing][masonry][fixtures]") {
  const auto root = load_fixture_file("packing/masonry_cases.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "packing_masonry");
      const auto request = parse_masonry_request(
          fixture.get_child("inputs").get_child("request"));
      MasonryBuilder builder;
      const auto result = builder.build(request);
      const auto &expected = fixture.get_child("expected");

      REQUIRE(result.algorithm == AlgorithmKind::masonry_builder);
      REQUIRE(result.bins.size() == expected.get<std::size_t>("bin_count"));
      REQUIRE(result.trace.size() == expected.get<std::size_t>("trace_count"));
      REQUIRE(result.progress.size() ==
              expected.get<std::size_t>("progress_count"));
      REQUIRE(result.layout.placement_trace.size() ==
              expected.get<std::size_t>("placed_piece_count"));
      REQUIRE(result.layout.unplaced_piece_ids ==
              parse_ids(expected.get_child("unplaced_piece_ids")));

      require_trace_matches(expected.get_child("trace"), result);
      require_progress_matches(expected.get_child("progress"), result);
    }
  }
}
