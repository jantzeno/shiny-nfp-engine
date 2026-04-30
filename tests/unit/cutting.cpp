#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>
#include <vector>

#include "geometry/normalize.hpp"
#include "packing/common_edge.hpp"
#include "packing/cutting_sequence.hpp"
#include "packing/layout.hpp"
#include "packing/pierce_point.hpp"
#include "predicates/point_location.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Segment2;
using shiny::nesting::pack::CutContour;
using shiny::nesting::pack::CutContourOrder;
using shiny::nesting::pack::CutPlan;
using shiny::nesting::pack::Layout;
using shiny::nesting::pack::PlacedPiece;
using shiny::nesting::pack::SharedCutOptimizationMode;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}},
  });
}

auto make_frame_piece() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}},
      .holes = {{{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}},
  });
}

auto make_piece(std::uint32_t piece_id, PolygonWithHoles polygon)
    -> PlacedPiece {
  return {
      .placement = {.piece_id = piece_id},
      .polygon = std::move(polygon),
  };
}

auto make_layout(std::vector<PlacedPiece> placements) -> Layout {
  return {
      .bins = {{
          .bin_id = 1,
          .placements = std::move(placements),
      }},
  };
}

auto segment_matches(const Segment2 &lhs, const Segment2 &rhs) -> bool {
  return (lhs.start.x == rhs.start.x && lhs.start.y == rhs.start.y &&
          lhs.end.x == rhs.end.x && lhs.end.y == rhs.end.y) ||
         (lhs.start.x == rhs.end.x && lhs.start.y == rhs.end.y &&
          lhs.end.x == rhs.start.x && lhs.end.y == rhs.start.y);
}

auto count_segment(const CutPlan &plan, const Segment2 &segment)
    -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      plan.segments.begin(), plan.segments.end(), [&](const auto &entry) {
        return segment_matches(entry.segment, segment);
      }));
}

auto make_contour(std::uint32_t piece_id, PolygonWithHoles polygon)
    -> CutContour {
  auto normalized = shiny::nesting::geom::normalize_polygon(std::move(polygon));
  return {
      .bin_id = 1,
      .piece_id = piece_id,
      .from_hole = false,
      .ring = std::move(normalized.outer),
  };
}

auto sample_arc_midpoint(const CutContourOrder::LeadArc &arc) -> Point2 {
  const auto start_angle =
      std::atan2(arc.start.y - arc.center.y, arc.start.x - arc.center.x);
  const auto end_angle =
      std::atan2(arc.end.y - arc.center.y, arc.end.x - arc.center.x);
  auto delta = end_angle - start_angle;
  if (arc.clockwise) {
    if (delta >= 0.0) {
      delta -= 2.0 * std::numbers::pi_v<double>;
    }
  } else if (delta <= 0.0) {
    delta += 2.0 * std::numbers::pi_v<double>;
  }

  const auto mid_angle = start_angle + delta * 0.5;
  const auto radius =
      std::hypot(arc.start.x - arc.center.x, arc.start.y - arc.center.y);
  return {
      .x = arc.center.x + std::cos(mid_angle) * radius,
      .y = arc.center.y + std::sin(mid_angle) * radius,
  };
}

} // namespace

TEST_CASE("cut plan sequences holes before exterior contours",
          "[packing][cut-plan][sequence]") {
  const auto layout = make_layout({
      make_piece(1, make_frame_piece()),
      make_piece(2, make_rectangle(6.0, 0.0, 7.0, 1.0)),
  });

  const auto plan = shiny::nesting::pack::build_cut_plan(
      layout, {.mode = SharedCutOptimizationMode::off});

  REQUIRE(plan.contour_order.size() == 3U);
  const auto hole_it =
      std::find_if(plan.contour_order.begin(), plan.contour_order.end(),
                   [](const auto &entry) {
                     return entry.piece_id == 1U && entry.from_hole;
                   });
  const auto exterior_it =
      std::find_if(plan.contour_order.begin(), plan.contour_order.end(),
                   [](const auto &entry) {
                     return entry.piece_id == 1U && !entry.from_hole;
                   });
  REQUIRE(hole_it != plan.contour_order.end());
  REQUIRE(exterior_it != plan.contour_order.end());
  REQUIRE(hole_it < exterior_it);
}

TEST_CASE("cutting sequence respects contained-piece before enclosing exterior",
          "[packing][cut-plan][dag]") {
  const auto layout = make_layout({
      make_piece(1, make_frame_piece()),
      make_piece(2, make_rectangle(1.5, 1.5, 2.5, 2.5)),
  });

  const auto sequence = shiny::nesting::pack::build_cutting_sequence(layout);
  REQUIRE(sequence.size() == 3U);
  REQUIRE(sequence[0].piece_id == 1U);
  REQUIRE(sequence[0].from_hole);
  REQUIRE(sequence[1].piece_id == 2U);
  REQUIRE_FALSE(sequence[1].from_hole);
  REQUIRE(sequence[2].piece_id == 1U);
  REQUIRE_FALSE(sequence[2].from_hole);
}

TEST_CASE("cut plan removes redundant common edges once",
          "[packing][cut-plan][common-edge]") {
  const auto layout = make_layout({
      make_piece(1, make_rectangle(0.0, 0.0, 1.0, 1.0)),
      make_piece(2, make_rectangle(1.0, 0.0, 2.0, 1.0)),
  });

  const auto plan = shiny::nesting::pack::build_cut_plan(
      layout, {.mode = SharedCutOptimizationMode::
                   remove_fully_covered_coincident_segments});

  REQUIRE(plan.raw_cut_length == 8.0);
  REQUIRE(plan.total_cut_length == 7.0);
  REQUIRE(plan.removed_cut_length == 1.0);
  REQUIRE(count_segment(plan, {{1.0, 0.0}, {1.0, 1.0}}) == 1U);
}

TEST_CASE(
    "cutting sequence prefers nearer contours and emits matching pierce points",
    "[packing][cut-plan][travel]") {
  const auto layout = make_layout({
      make_piece(1, make_rectangle(0.0, 0.0, 1.0, 1.0)),
      make_piece(2, make_rectangle(10.0, 0.0, 11.0, 1.0)),
      make_piece(3, make_rectangle(3.0, 0.0, 4.0, 1.0)),
  });

  const auto sequence = shiny::nesting::pack::build_cutting_sequence(layout);
  REQUIRE(sequence.size() == 3U);
  REQUIRE(sequence[0].piece_id == 1U);
  REQUIRE(sequence[1].piece_id == 3U);
  REQUIRE(sequence[2].piece_id == 2U);

  const auto plan = shiny::nesting::pack::build_cut_plan(
      layout, {.mode = SharedCutOptimizationMode::off});
  REQUIRE(plan.contour_order.size() == 3U);
  REQUIRE(plan.contour_order[0].pierce_point.x == 0.0);
  REQUIRE(plan.contour_order[0].pierce_point.y == 0.0);
  REQUIRE(plan.contour_order[0].lead_in.enabled);
  REQUIRE(plan.contour_order[0].lead_out.enabled);
  REQUIRE(plan.contour_order[0].lead_in.start.x == Catch::Approx(-0.25));
  REQUIRE(plan.contour_order[0].lead_in.start.y == Catch::Approx(-0.25));
  REQUIRE(plan.contour_order[0].lead_out.end.x == Catch::Approx(-0.25));
  REQUIRE(plan.contour_order[0].lead_out.end.y == Catch::Approx(-0.25));
  REQUIRE(plan.contour_order[1].pierce_point.x == 3.0);
  REQUIRE(plan.contour_order[1].pierce_point.y == 0.5);
  REQUIRE(plan.contour_order[1].lead_in.enabled);
  REQUIRE(plan.contour_order[1].lead_out.enabled);
  REQUIRE(plan.contour_order[2].pierce_point.x == 10.0);
  REQUIRE(plan.contour_order[2].pierce_point.y == 0.5);
  REQUIRE(plan.contour_order[2].lead_in.enabled);
  REQUIRE(plan.contour_order[2].lead_out.enabled);
}

TEST_CASE("pierce plan can select an edge midpoint and keep lead arcs outside "
          "material",
          "[packing][cut-plan][pierce]") {
  const auto polygon = make_rectangle(0.0, 0.0, 6.0, 1.0);
  const auto contour = make_contour(1U, polygon);

  const auto plan =
      shiny::nesting::pack::select_pierce_plan(contour, {.x = 3.05, .y = -0.2});

  REQUIRE(plan.pierce_point.x == Catch::Approx(3.0));
  REQUIRE(plan.pierce_point.y == Catch::Approx(0.0));
  REQUIRE(plan.lead_in.enabled);
  REQUIRE(plan.lead_out.enabled);

  const auto lead_in_midpoint = sample_arc_midpoint(plan.lead_in);
  const auto lead_out_midpoint = sample_arc_midpoint(plan.lead_out);
  REQUIRE(
      shiny::nesting::pred::locate_point_in_polygon(lead_in_midpoint, polygon)
          .location == shiny::nesting::pred::PointLocation::exterior);
  REQUIRE(
      shiny::nesting::pred::locate_point_in_polygon(lead_out_midpoint, polygon)
          .location == shiny::nesting::pred::PointLocation::exterior);
}

TEST_CASE("common-edge detection merges adjacent collinear coverage",
          "[packing][cut-plan][common-edge]") {
  const auto merged = shiny::nesting::pack::detect_common_edges({
      {.bin_id = 1, .piece_id = 1, .segment = {{0.0, 0.0}, {1.0, 0.0}}},
      {.bin_id = 1, .piece_id = 2, .segment = {{1.0, 0.0}, {2.0, 0.0}}},
  });

  REQUIRE(merged.size() == 1U);
  REQUIRE(segment_matches(merged.front().segment, {{0.0, 0.0}, {2.0, 0.0}}));
}

TEST_CASE("common-edge detection merges overlapping collinear coverage into "
          "one segment",
          "[packing][cut-plan][common-edge]") {
  const auto merged = shiny::nesting::pack::detect_common_edges({
      {.bin_id = 1, .piece_id = 1, .segment = {{0.0, 0.0}, {2.0, 0.0}}},
      {.bin_id = 1, .piece_id = 2, .segment = {{1.0, 0.0}, {3.0, 0.0}}},
  });

  REQUIRE(merged.size() == 1U);
  REQUIRE(segment_matches(merged.front().segment, {{0.0, 0.0}, {3.0, 0.0}}));
}

TEST_CASE("common-edge detection preserves disjoint collinear segments",
          "[packing][cut-plan][common-edge]") {
  const auto merged = shiny::nesting::pack::detect_common_edges({
      {.bin_id = 1, .piece_id = 1, .segment = {{0.0, 0.0}, {1.0, 0.0}}},
      {.bin_id = 1, .piece_id = 2, .segment = {{2.0, 0.0}, {3.0, 0.0}}},
  });

  REQUIRE(merged.size() == 2U);
  REQUIRE(segment_matches(merged[0].segment, {{0.0, 0.0}, {1.0, 0.0}}));
  REQUIRE(segment_matches(merged[1].segment, {{2.0, 0.0}, {3.0, 0.0}}));
}

TEST_CASE("pierce plan emits non-collapsed lead arcs for a degenerate "
          "(zero-area) ring",
          "[packing][cut-plan][pierce][pierce-degenerate]") {
  // Three collinear points along the x-axis: ring_signed_area == 0,
  // so outward_sign falls back to +1 (review Phase 12 §1). The lead
  // arcs must remain non-degenerate (non-zero radius, distinct
  // start/end), not collapse onto the pierce point.
  shiny::nesting::pack::CutContour contour{
      .bin_id = 1U,
      .piece_id = 1U,
      .from_hole = false,
      .ring = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
  };

  const auto plan =
      shiny::nesting::pack::select_pierce_plan(contour, {.x = -1.0, .y = 0.0});

  REQUIRE(std::isfinite(plan.pierce_point.x));
  REQUIRE(std::isfinite(plan.pierce_point.y));
  if (plan.lead_in.enabled) {
    const auto dx = plan.lead_in.start.x - plan.lead_in.end.x;
    const auto dy = plan.lead_in.start.y - plan.lead_in.end.y;
    REQUIRE(std::sqrt(dx * dx + dy * dy) > 1e-6);
  }
  if (plan.lead_out.enabled) {
    const auto dx = plan.lead_out.start.x - plan.lead_out.end.x;
    const auto dy = plan.lead_out.start.y - plan.lead_out.end.y;
    REQUIRE(std::sqrt(dx * dx + dy * dy) > 1e-6);
  }
}
