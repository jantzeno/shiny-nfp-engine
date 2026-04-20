#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "packing/bounding_box_packer.hpp"
#include "packing/decoder.hpp"
#include "polygon_ops/merge_region.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using Catch::Approx;
using shiny::nesting::pack::BinInput;
using shiny::nesting::pack::ConstructiveDecoder;
using shiny::nesting::pack::CutPlan;
using shiny::nesting::pack::DecoderRequest;
using shiny::nesting::pack::Layout;
using shiny::nesting::pack::LayoutBin;
using shiny::nesting::pack::PackingConfig;
using shiny::nesting::pack::PieceInput;
using shiny::nesting::pack::SharedCutOptimizationMode;
using shiny::nesting::place::PlacementPolicy;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::parse_segment;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_point_equal;

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
  throw std::runtime_error("unknown packing bed grain direction");
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
  throw std::runtime_error("unknown packing part grain compatibility");
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
  throw std::runtime_error("unknown packing fixture policy");
}

auto parse_shared_cut_mode(std::string_view value)
    -> SharedCutOptimizationMode {
  if (value == "off") {
    return SharedCutOptimizationMode::off;
  }
  if (value == "remove_fully_covered_coincident_segments") {
    return SharedCutOptimizationMode::remove_fully_covered_coincident_segments;
  }
  throw std::runtime_error("unknown shared cut mode fixture value");
}

auto parse_packing_config(const shiny::nesting::test::pt::ptree &node)
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

auto parse_bin_input(const shiny::nesting::test::pt::ptree &node) -> BinInput {
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

auto parse_decoder_request(const shiny::nesting::test::pt::ptree &node)
    -> DecoderRequest {
  const auto max_bins = node.get<std::size_t>("max_bin_count", 0);
  const auto pieces = parse_piece_inputs(node.get_child("pieces"));
  DecoderRequest request{
      .bins = expand_bins(parse_bin_input(node.get_child("bin")),
                          resolve_fixture_bin_count(max_bins, pieces.size())),
      .pieces = pieces,
      .policy = parse_policy(node.get<std::string>("policy", "bottom_left")),
  };

  if (const auto config = node.get_child_optional("config")) {
    request.config = parse_packing_config(*config);
  }
  return request;
}

auto parse_layout_bin(const shiny::nesting::test::pt::ptree &node) -> LayoutBin {
  LayoutBin bin{.bin_id = node.get<std::uint32_t>("bin_id")};
  if (const auto placements = node.get_child_optional("placements")) {
    for (const auto &child : *placements) {
      bin.placements.push_back({
          .placement = {.piece_id = child.second.get<std::uint32_t>("piece_id"),
                        .bin_id = bin.bin_id},
          .polygon = parse_polygon(child.second.get_child("polygon")),
      });
    }
  }

  for (const auto &piece : bin.placements) {
    if (bin.occupied.regions.empty()) {
      bin.occupied = shiny::nesting::poly::make_merged_region(piece.polygon);
    } else {
      bin.occupied = shiny::nesting::poly::merge_polygon_into_region(bin.occupied,
                                                                 piece.polygon);
    }
  }

  return bin;
}

auto parse_layout(const shiny::nesting::test::pt::ptree &node) -> Layout {
  Layout layout{};
  if (const auto bins = node.get_child_optional("bins")) {
    for (const auto &child : *bins) {
      layout.bins.push_back(parse_layout_bin(child.second));
    }
  }
  return layout;
}

void require_trace_matches(const shiny::nesting::test::pt::ptree &expected_trace,
                           const Layout &layout) {
  REQUIRE(layout.placement_trace.size() == expected_trace.size());
  std::size_t index = 0;
  for (const auto &entry_node : expected_trace) {
    const auto &expected = entry_node.second;
    const auto &actual = layout.placement_trace[index++];

    REQUIRE(actual.piece_id == expected.get<std::uint32_t>("piece_id"));
    REQUIRE(actual.bin_id == expected.get<std::uint32_t>("bin_id"));
    require_point_equal(actual.translation,
                        parse_point(expected.get_child("translation")));
    REQUIRE(actual.opened_new_bin == expected.get<bool>("opened_new_bin"));
    REQUIRE(actual.inside_hole == expected.get<bool>("inside_hole", false));
    REQUIRE(actual.hole_index == expected.get<std::int32_t>("hole_index", -1));
  }
}

void require_bin_matches(const shiny::nesting::test::pt::ptree &expected_bin,
                         const shiny::nesting::pack::BinState &actual_bin,
                         const LayoutBin &layout_bin) {
  REQUIRE(actual_bin.bin_id == expected_bin.get<std::uint32_t>("bin_id"));
  REQUIRE(actual_bin.placements.size() ==
          expected_bin.get<std::size_t>("placement_count"));
  REQUIRE(layout_bin.placements.size() == actual_bin.placements.size());

  if (const auto occupied_area =
          expected_bin.get_optional<double>("occupied_area")) {
    REQUIRE(actual_bin.utilization.occupied_area == Approx(*occupied_area));
  }
  if (const auto utilization =
          expected_bin.get_optional<double>("utilization")) {
    REQUIRE(actual_bin.utilization.utilization == Approx(*utilization));
  }
  if (const auto hole_count =
          expected_bin.get_optional<std::size_t>("hole_count")) {
    REQUIRE(actual_bin.holes.size() == *hole_count);
  }

  if (const auto placements = expected_bin.get_child_optional("placements")) {
    REQUIRE(actual_bin.placements.size() == placements->size());
    std::size_t index = 0;
    for (const auto &placement_node : *placements) {
      const auto &expected = placement_node.second;
      const auto &actual = actual_bin.placements[index++];
      REQUIRE(actual.placement.piece_id ==
              expected.get<std::uint32_t>("piece_id"));
      require_point_equal(actual.placement.translation,
                          parse_point(expected.get_child("translation")));
      REQUIRE(actual.inside_hole == expected.get<bool>("inside_hole", false));
      REQUIRE(actual.hole_index ==
              expected.get<std::int32_t>("hole_index", -1));
    }
  }
}

auto has_segment(const CutPlan &plan, std::uint32_t piece_id,
                 const shiny::nesting::geom::Segment2 &expected) -> bool {
  return std::any_of(plan.segments.begin(), plan.segments.end(),
                     [&](const auto &segment) {
                       const auto same_direction =
                           segment.segment.start.x == expected.start.x &&
                           segment.segment.start.y == expected.start.y &&
                           segment.segment.end.x == expected.end.x &&
                           segment.segment.end.y == expected.end.y;
                       const auto reverse_direction =
                           segment.segment.start.x == expected.end.x &&
                           segment.segment.start.y == expected.end.y &&
                           segment.segment.end.x == expected.start.x &&
                           segment.segment.end.y == expected.start.y;
                       return segment.piece_id == piece_id &&
                              (same_direction || reverse_direction);
                     });
}

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

} // namespace

TEST_CASE("packing config fixtures", "[packing][config][fixtures]") {
  const auto root = load_fixture_file("packing/config_surface.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "packing_config");
      const auto config =
          parse_packing_config(fixture.get_child("inputs").get_child("config"));
      REQUIRE(config.is_valid() ==
              fixture.get_child("expected").get<bool>("valid"));
    }
  }
}

TEST_CASE("packing decoder fixtures", "[packing][decoder][fixtures]") {
  const auto root = load_fixture_file("packing/decoder_cases.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "packing_decode");

      ConstructiveDecoder decoder;
      const auto request = parse_decoder_request(
          fixture.get_child("inputs").get_child("request"));
      const auto result = decoder.decode(request);
      const auto expected = fixture.get_child("expected");

      REQUIRE(result.bins.size() == expected.get<std::size_t>("bin_count"));
      REQUIRE(result.layout.bins.size() == result.bins.size());
      REQUIRE(result.layout.placement_trace.size() ==
              expected.get<std::size_t>("placement_trace_count"));

      if (const auto unplaced =
              expected.get_child_optional("unplaced_piece_ids")) {
        REQUIRE(result.layout.unplaced_piece_ids == parse_ids(*unplaced));
      } else {
        REQUIRE(result.layout.unplaced_piece_ids.empty());
      }

      if (const auto bins = expected.get_child_optional("bins")) {
        REQUIRE(result.bins.size() == bins->size());
        std::size_t index = 0;
        for (const auto &expected_bin : *bins) {
          require_bin_matches(expected_bin.second, result.bins[index],
                              result.layout.bins[index]);
          ++index;
        }
      }

      if (const auto trace = expected.get_child_optional("placement_trace")) {
        require_trace_matches(*trace, result.layout);
      }
    }
  }
}

TEST_CASE("packing cut plan fixtures", "[packing][cut-plan][fixtures]") {
  const auto root = load_fixture_file("packing/cut_plan_cases.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "packing_cut_plan");

      const auto layout =
          parse_layout(fixture.get_child("inputs").get_child("layout"));
      const auto config =
          parse_packing_config(fixture.get_child("inputs").get_child("config"));
      const auto plan = shiny::nesting::pack::build_cut_plan(
          layout, config.laser_cut_optimization);
      const auto expected = fixture.get_child("expected");

      REQUIRE(plan.segments.size() ==
              expected.get<std::size_t>("segment_count"));
      REQUIRE(plan.raw_cut_length ==
              Approx(expected.get<double>("raw_cut_length")));
      REQUIRE(plan.total_cut_length ==
              Approx(expected.get<double>("total_cut_length")));
      REQUIRE(plan.removed_cut_length ==
              Approx(expected.get<double>("removed_cut_length")));

      if (const auto present =
              expected.get_child_optional("present_segments")) {
        for (const auto &segment_node : *present) {
          const auto piece_id =
              segment_node.second.get<std::uint32_t>("piece_id");
          const auto segment =
              parse_segment(segment_node.second.get_child("segment"));
          REQUIRE(has_segment(plan, piece_id, segment));
        }
      }

      if (const auto absent = expected.get_child_optional("absent_segments")) {
        for (const auto &segment_node : *absent) {
          const auto piece_id =
              segment_node.second.get<std::uint32_t>("piece_id");
          const auto segment =
              parse_segment(segment_node.second.get_child("segment"));
          REQUIRE_FALSE(has_segment(plan, piece_id, segment));
        }
      }
    }
  }
}

TEST_CASE("packing decoder routes convex bins through IFP queries",
          "[packing][decoder][ifp]") {
  ConstructiveDecoder decoder;
  DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 7,
                  .polygon = make_rectangle(0.0, 0.0, 4.0, 4.0),
                  .geometry_revision = 2,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request);
  REQUIRE(result.layout.placement_trace.size() == 1U);
  REQUIRE(decoder.convex_cache_size() == 1U);
}

TEST_CASE("packing decoder isolates transient occupied-region cache keys",
          "[packing][decoder][cache]") {
  const auto make_request = [](double first_piece_width,
                               std::uint64_t first_piece_revision) {
    return DecoderRequest{
        .bins = {{
            .bin_id = 0,
            .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
            .geometry_revision = 50,
        }},
        .pieces =
            {
                PieceInput{
                    .piece_id = 1,
                    .polygon = make_rectangle(0.0, 0.0, first_piece_width, 4.0),
                    .geometry_revision = first_piece_revision,
                },
                PieceInput{
                    .piece_id = 2,
                    .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
                    .geometry_revision = 77,
                },
            },
        .policy = PlacementPolicy::bottom_left,
        .config =
            {
                .placement =
                    {
                        .allowed_rotations = {.angles_degrees = {0.0}},
                    },
                .enable_hole_first_placement = false,
            },
    };
  };

  ConstructiveDecoder decoder;

  const auto first_result = decoder.decode(make_request(4.0, 11));
  REQUIRE(first_result.bins.size() == 1U);
  REQUIRE(first_result.bins.front().placements.size() == 2U);
  const auto first_cache_size = decoder.nonconvex_cache_size();
  REQUIRE(first_cache_size > 0U);

  const auto second_result = decoder.decode(make_request(6.0, 12));
  REQUIRE(second_result.bins.size() == 1U);
  REQUIRE(second_result.bins.front().placements.size() == 2U);
  REQUIRE(decoder.nonconvex_cache_size() > first_cache_size);
}

TEST_CASE("packing decoder rejects grain-incompatible piece rotations",
          "[packing][decoder][grain]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 8.0, 8.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 7,
                  .polygon = make_rectangle(0.0, 0.0, 4.0, 2.0),
                  .geometry_revision = 2,
                  .grain_compatibility = shiny::nesting::place::
                      PartGrainCompatibility::parallel_to_bed,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {90.0}},
                      .bed_grain_direction =
                          shiny::nesting::place::BedGrainDirection::along_x,
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request);
  REQUIRE(result.bins.empty());
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{7U});
}

TEST_CASE("packing decoder treats exclusion zones as keep-outs",
          "[packing][decoder][exclusion]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 6.0, 6.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 3,
                  .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
                  .geometry_revision = 4,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                      .enable_part_in_part_placement = true,
                      .exclusion_zones = {{
                          .zone_id = 11,
                          .region = {.outer = {{0.0, 0.0},
                                               {3.0, 0.0},
                                               {3.0, 3.0},
                                               {0.0, 3.0}}},
                      }},
                  },
              .enable_hole_first_placement = true,
          },
  };

  const auto result = decoder.decode(request);
  REQUIRE(result.layout.placement_trace.size() == 1U);
  require_point_equal(result.layout.placement_trace.front().translation,
                      {4.0, 0.0});
  REQUIRE_FALSE(result.layout.placement_trace.front().inside_hole);
}

TEST_CASE("packing decoder allows pieces to touch on occupied boundaries",
          "[packing][decoder][boundary]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 6.0, 3.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 1,
                  .polygon = make_rectangle(0.0, 0.0, 3.0, 3.0),
                  .geometry_revision = 2,
              },
              PieceInput{
                  .piece_id = 2,
                  .polygon = make_rectangle(0.0, 0.0, 3.0, 3.0),
                  .geometry_revision = 3,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request);

  REQUIRE_FALSE(result.interrupted);
  REQUIRE(result.layout.placement_trace.size() == 2U);
  require_point_equal(result.layout.placement_trace[0].translation, {0.0, 0.0});
  require_point_equal(result.layout.placement_trace[1].translation, {3.0, 0.0});
  REQUIRE(result.layout.unplaced_piece_ids.empty());
}

TEST_CASE("packing decoder allows placements tangent to exclusion zones",
          "[packing][decoder][exclusion][boundary]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 6.0, 6.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 9,
                  .polygon = make_rectangle(0.0, 0.0, 3.0, 3.0),
                  .geometry_revision = 2,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                      .exclusion_zones = {{
                          .zone_id = 21,
                          .region = {.outer = {{0.0, 0.0},
                                               {3.0, 0.0},
                                               {3.0, 3.0},
                                               {0.0, 3.0}}},
                      }},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request);

  REQUIRE_FALSE(result.interrupted);
  REQUIRE(result.layout.placement_trace.size() == 1U);
  require_point_equal(result.layout.placement_trace.front().translation,
                      {3.0, 0.0});
  REQUIRE(result.layout.unplaced_piece_ids.empty());
}

TEST_CASE("packing decoder reports interrupted partial runs",
          "[packing][decoder][interrupt]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 8.0, 8.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 1,
                  .polygon = make_rectangle(0.0, 0.0, 3.0, 3.0),
                  .geometry_revision = 2,
              },
              PieceInput{
                  .piece_id = 2,
                  .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
                  .geometry_revision = 3,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request, []() { return true; });

  REQUIRE(result.interrupted);
  REQUIRE(result.bins.empty());
  REQUIRE(result.layout.bins.empty());
  REQUIRE(result.layout.placement_trace.empty());
  REQUIRE(result.layout.unplaced_piece_ids ==
          std::vector<std::uint32_t>{1U, 2U});
  REQUIRE(decoder.convex_cache_size() == 0U);
  REQUIRE(decoder.nonconvex_cache_size() == 0U);
  REQUIRE(decoder.decomposition_cache_size() == 0U);
}

TEST_CASE("packing decoder preserves placed pieces on later interruption",
          "[packing][decoder][interrupt][partial]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 8.0, 4.0),
          .geometry_revision = 1,
      }},
      .pieces =
          {
              PieceInput{
                  .piece_id = 1,
                  .polygon = make_rectangle(0.0, 0.0, 4.0, 4.0),
                  .geometry_revision = 2,
              },
              PieceInput{
                  .piece_id = 2,
                  .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
                  .geometry_revision = 3,
              },
          },
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(
      request, [&decoder]() { return decoder.nonconvex_cache_size() > 0U; });

  REQUIRE(result.interrupted);
  REQUIRE(result.layout.bins.size() == 1U);
  REQUIRE(result.layout.placement_trace.size() == 1U);
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{2U});
  require_point_equal(result.layout.placement_trace.front().translation,
                      {0.0, 0.0});
  REQUIRE(decoder.nonconvex_cache_size() > 0U);
}

TEST_CASE("packing decoder honors top-right start corner",
          "[packing][decoder][start-corner]") {
  ConstructiveDecoder decoder;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 1,
          .start_corner = shiny::nesting::place::PlacementStartCorner::top_right,
      }},
      .pieces = {{
          .piece_id = 1,
          .polygon = make_rectangle(0.0, 0.0, 4.0, 3.0),
          .geometry_revision = 2,
      }},
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = decoder.decode(request);

  REQUIRE(result.layout.placement_trace.size() == 1U);
  require_point_equal(result.layout.placement_trace.front().translation,
                      {6.0, 7.0});
}

TEST_CASE("bounding box packer honors bottom-right start corner",
          "[packing][bbox][start-corner]") {
  shiny::nesting::pack::BoundingBoxPacker packer;
  const DecoderRequest request{
      .bins = {{
          .bin_id = 0,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 1,
          .start_corner = shiny::nesting::place::PlacementStartCorner::bottom_right,
      }},
      .pieces = {{
          .piece_id = 1,
          .polygon = make_rectangle(0.0, 0.0, 4.0, 3.0),
          .geometry_revision = 2,
      }},
      .policy = PlacementPolicy::bottom_left,
      .config =
          {
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees = {0.0}},
                  },
              .enable_hole_first_placement = false,
          },
  };

  const auto result = packer.decode(request);

  REQUIRE(result.layout.placement_trace.size() == 1U);
  require_point_equal(result.layout.placement_trace.front().translation,
                      {6.0, 0.0});
}
