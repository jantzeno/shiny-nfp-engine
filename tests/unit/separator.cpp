#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "packing/item_mover.hpp"
#include "packing/separator.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "runtime/deterministic_rng.hpp"

namespace {

using shiny::nesting::geom::PolygonWithHoles;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {
          {.x = min_x, .y = min_y},
          {.x = max_x, .y = min_y},
          {.x = max_x, .y = max_y},
          {.x = min_x, .y = max_y},
      },
  });
}

auto overlap_area(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs) -> double {
  double area = 0.0;
  for (const auto &polygon : shiny::nesting::poly::intersection_polygons(lhs, rhs)) {
    area += shiny::nesting::geom::polygon_area(polygon);
  }
  return area;
}

auto rotated(const PolygonWithHoles &polygon, const double degrees)
    -> PolygonWithHoles {
  return shiny::nesting::geom::rotate(
      polygon, shiny::nesting::geom::ResolvedRotation{.degrees = degrees});
}

auto bounds_center(const PolygonWithHoles &polygon) -> shiny::nesting::geom::Point2 {
  const auto bounds = shiny::nesting::geom::compute_bounds(polygon);
  return {
      .x = (bounds.min.x + bounds.max.x) / 2.0,
      .y = (bounds.min.y + bounds.max.y) / 2.0,
  };
}

auto polygon_matches(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs) -> bool {
  if (lhs.outer.size() != rhs.outer.size() || lhs.holes.size() != rhs.holes.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.outer.size(); ++index) {
    if (std::fabs(lhs.outer[index].x - rhs.outer[index].x) > 1e-9 ||
        std::fabs(lhs.outer[index].y - rhs.outer[index].y) > 1e-9) {
      return false;
    }
  }
  for (std::size_t hole_index = 0; hole_index < lhs.holes.size(); ++hole_index) {
    if (lhs.holes[hole_index].size() != rhs.holes[hole_index].size()) {
      return false;
    }
    for (std::size_t point_index = 0; point_index < lhs.holes[hole_index].size();
         ++point_index) {
      if (std::fabs(lhs.holes[hole_index][point_index].x -
                    rhs.holes[hole_index][point_index].x) > 1e-9 ||
          std::fabs(lhs.holes[hole_index][point_index].y -
                    rhs.holes[hole_index][point_index].y) > 1e-9) {
        return false;
      }
    }
  }
  return true;
}

} // namespace

TEST_CASE("separator resolves a small overlapping layout", "[packing][separator]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 4.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.item_id = 2, .geometry_revision = 2,
       .polygon = rectangle(2.0, 0.0, 6.0, 4.0)},
  };

  const auto result = shiny::nesting::pack::run_separator(
      container, items, shiny::nesting::pack::SeparatorConfig{});

  REQUIRE(result.total_loss == Catch::Approx(0.0).margin(1e-8));
  REQUIRE(result.polygons.size() == 2U);
  REQUIRE(overlap_area(result.polygons[0], result.polygons[1]) ==
          Catch::Approx(0.0).margin(1e-8));
}

TEST_CASE("multi-worker separation is no worse than single-worker separation",
          "[packing][separator][parallel]") {
  const auto container = rectangle(0.0, 0.0, 12.0, 4.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.item_id = 2, .geometry_revision = 2,
       .polygon = rectangle(2.0, 0.0, 6.0, 4.0)},
      {.item_id = 3, .geometry_revision = 3,
       .polygon = rectangle(4.0, 0.0, 8.0, 4.0)},
  };

  const auto single = shiny::nesting::pack::run_separator(
      container, items, shiny::nesting::pack::SeparatorConfig{.worker_count = 1},
      17U);
  const auto multi = shiny::nesting::pack::run_separator(
      container, items, shiny::nesting::pack::SeparatorConfig{.worker_count = 2},
      17U);

  REQUIRE(multi.total_loss <= single.total_loss + 1e-8);
}

TEST_CASE("item mover can improve overlap with rotation-only refinement",
          "[packing][separator][item-mover]") {
  const auto container = rectangle(-10.0, -10.0, 10.0, 10.0);
  const auto stationary = rectangle(-4.0, -0.3, 4.0, 0.3);
  const auto moving = rotated(stationary, 35.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1, .polygon = stationary},
      {.item_id = 2, .geometry_revision = 2, .polygon = moving},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  shiny::nesting::pack::ItemMoverConfig config{
      .global_samples = 0U,
      .focused_samples = 0U,
      .coordinate_descent_iterations = 12U,
      .coarse_step_ratio = 0.0,
      .coarse_min_step_ratio = 1.0,
      .fine_step_ratio = 0.0,
      .min_step_ratio = 1.0,
      .angle_step_degrees = 5.0,
      .min_angle_step_degrees = 0.1,
      .enable_rotation_axis = true,
  };
  shiny::nesting::runtime::DeterministicRng rng(7U);

  const auto move = shiny::nesting::pack::move_item(tracker, 1U, config, rng);

  REQUIRE(move.has_value());
  REQUIRE(overlap_area(stationary, move->polygon) + 1e-9 <
          overlap_area(stationary, moving));
  REQUIRE(bounds_center(move->polygon).x ==
          Catch::Approx(bounds_center(moving).x).margin(1e-9));
  REQUIRE(bounds_center(move->polygon).y ==
          Catch::Approx(bounds_center(moving).y).margin(1e-9));
}

TEST_CASE("item mover translates when rotation locked but rotation axis enabled",
          "[packing][separator][item-mover]") {
  const auto container = rectangle(-10.0, -10.0, 10.0, 10.0);
  const auto stationary = rectangle(0.0, 0.0, 4.0, 4.0);
  const auto moving = rectangle(2.0, 0.0, 6.0, 4.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1, .polygon = stationary},
      {.item_id = 2,
       .geometry_revision = 2,
       .polygon = moving,
       .rotation_locked = true},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  shiny::nesting::pack::ItemMoverConfig config{
      .global_samples = 0U,
      .focused_samples = 0U,
      .coordinate_descent_iterations = 12U,
      .coarse_step_ratio = 0.05,
      .coarse_min_step_ratio = 0.001,
      .fine_step_ratio = 0.02,
      .min_step_ratio = 0.0005,
      .angle_step_degrees = 5.0,
      .min_angle_step_degrees = 0.1,
      .enable_rotation_axis = true,
  };
  shiny::nesting::runtime::DeterministicRng rng(7U);

  const auto move = shiny::nesting::pack::move_item(tracker, 1U, config, rng);

  REQUIRE(move.has_value());
  REQUIRE(overlap_area(stationary, move->polygon) + 1e-9 <
          overlap_area(stationary, moving));
  // Rotation lock honoured: the moving piece's bbox dimensions are
  // preserved (a rotation would change them).
  const auto original_bounds = shiny::nesting::geom::compute_bounds(moving);
  const auto moved_bounds = shiny::nesting::geom::compute_bounds(move->polygon);
  REQUIRE(moved_bounds.max.x - moved_bounds.min.x ==
          Catch::Approx(original_bounds.max.x - original_bounds.min.x).margin(1e-9));
  REQUIRE(moved_bounds.max.y - moved_bounds.min.y ==
          Catch::Approx(original_bounds.max.y - original_bounds.min.y).margin(1e-9));
}

TEST_CASE("item mover respects rotation locks", "[packing][separator][item-mover]") {
  const auto container = rectangle(-10.0, -10.0, 10.0, 10.0);
  const auto stationary = rectangle(-4.0, -0.3, 4.0, 0.3);
  const auto moving = rotated(stationary, 35.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1, .polygon = stationary},
      {.item_id = 2,
       .geometry_revision = 2,
       .polygon = moving,
       .rotation_locked = true},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  shiny::nesting::pack::ItemMoverConfig config{
      .global_samples = 0U,
      .focused_samples = 0U,
      .coordinate_descent_iterations = 12U,
      .coarse_step_ratio = 0.0,
      .coarse_min_step_ratio = 1.0,
      .fine_step_ratio = 0.0,
      .min_step_ratio = 1.0,
      .angle_step_degrees = 5.0,
      .min_angle_step_degrees = 0.1,
      .enable_rotation_axis = true,
  };
  shiny::nesting::runtime::DeterministicRng rng(7U);

  const auto move = shiny::nesting::pack::move_item(tracker, 1U, config, rng);

  REQUIRE_FALSE(move.has_value());
}

TEST_CASE("multi-worker separator is deterministic for a fixed seed",
          "[packing][separator][parallel]") {
  const auto container = rectangle(0.0, 0.0, 12.0, 4.0);
  const std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.item_id = 2, .geometry_revision = 2,
       .polygon = rectangle(2.0, 0.0, 6.0, 4.0)},
      {.item_id = 3, .geometry_revision = 3,
       .polygon = rectangle(4.0, 0.0, 8.0, 4.0)},
  };

  const auto first = shiny::nesting::pack::run_separator(
      container, items, shiny::nesting::pack::SeparatorConfig{.worker_count = 4},
      17U);
  for (std::size_t attempt = 0; attempt < 5U; ++attempt) {
    const auto repeated = shiny::nesting::pack::run_separator(
        container, items, shiny::nesting::pack::SeparatorConfig{.worker_count = 4},
        17U);
    REQUIRE(repeated.total_loss == Catch::Approx(first.total_loss).margin(1e-8));
    REQUIRE(repeated.polygons.size() == first.polygons.size());
    for (std::size_t index = 0; index < first.polygons.size(); ++index) {
      REQUIRE(polygon_matches(repeated.polygons[index], first.polygons[index]));
    }
  }
}
