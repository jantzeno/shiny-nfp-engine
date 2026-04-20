#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/spatial_index.hpp"
#include "geometry/transform.hpp"
#include "geometry/validity.hpp"

namespace {

using shiny::nesting::geom::Box2;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonValidityIssue;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::ResolvedRotation;
using shiny::nesting::geom::RotationIndex;
using shiny::nesting::geom::SpatialIndex;
using shiny::nesting::geom::Transform2;
using shiny::nesting::geom::Vector2;

auto rectangle(const double width, const double height) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{.outer = {
                                                 {.x = 0.0, .y = 0.0},
                                                 {.x = width, .y = 0.0},
                                                 {.x = width, .y = height},
                                                 {.x = 0.0, .y = height},
                                             }});
}

} // namespace

TEST_CASE("geometry foundation handles convex, concave, and holed polygons",
          "[geometry][foundation]") {
  const auto convex = rectangle(4.0, 3.0);
  const auto concave = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {
          {.x = 0.0, .y = 0.0},
          {.x = 4.0, .y = 0.0},
          {.x = 4.0, .y = 4.0},
          {.x = 2.0, .y = 4.0},
          {.x = 2.0, .y = 2.0},
          {.x = 0.0, .y = 2.0},
      },
  });
  const auto holed = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = rectangle(5.0, 5.0).outer,
      .holes = {{{.x = 1.0, .y = 1.0},
                 {.x = 1.0, .y = 2.0},
                 {.x = 2.0, .y = 2.0},
                 {.x = 2.0, .y = 1.0}}},
  });

  REQUIRE(shiny::nesting::geom::polygon_area(convex) == 12.0);
  REQUIRE(shiny::nesting::geom::polygon_area(concave) == 12.0);
  REQUIRE(shiny::nesting::geom::polygon_area(holed) == 24.0);
  REQUIRE(shiny::nesting::geom::validate_polygon(convex).is_valid());
  REQUIRE(shiny::nesting::geom::validate_polygon(concave).is_valid());
  REQUIRE(shiny::nesting::geom::validate_polygon(holed).is_valid());
}

TEST_CASE("geometry foundation applies deterministic transforms",
          "[geometry][transform]") {
  const auto polygon = rectangle(2.0, 1.0);
  const auto rotated = shiny::nesting::geom::rotate_polygon(
      polygon, ResolvedRotation{.degrees = 90.0});
  const auto rotated_bounds = shiny::nesting::geom::compute_bounds(rotated);

  REQUIRE(rotated_bounds.min.x == -1.0);
  REQUIRE(rotated_bounds.max.x == 0.0);
  REQUIRE(rotated_bounds.min.y == 0.0);
  REQUIRE(rotated_bounds.max.y == 2.0);

  shiny::nesting::geom::DiscreteRotationSet rotations{{0.0, 90.0}};
  const auto transformed = shiny::nesting::geom::apply_transform(
      polygon, Transform2{.rotation_index = RotationIndex{1},
                          .translation = Vector2{.x = 10.0, .y = 20.0}},
      rotations);
  REQUIRE(transformed.has_value());
  const auto bounds = shiny::nesting::geom::compute_bounds(*transformed);
  REQUIRE(bounds.min.x == 9.0);
  REQUIRE(bounds.max.x == 10.0);
  REQUIRE(bounds.min.y == 20.0);
  REQUIRE(bounds.max.y == 22.0);
}

TEST_CASE("geometry validity rejects invalid input", "[geometry][validity]") {
  const PolygonWithHoles self_intersecting{.outer = {
      {.x = 0.0, .y = 0.0},
      {.x = 2.0, .y = 2.0},
      {.x = 0.0, .y = 2.0},
      {.x = 2.0, .y = 0.0},
  }};
  const auto self_intersection_validity =
      shiny::nesting::geom::validate_polygon(self_intersecting);
  REQUIRE_FALSE(self_intersection_validity.is_valid());
  REQUIRE(self_intersection_validity.issue ==
          PolygonValidityIssue::self_intersection);

  const PolygonWithHoles hole_outside{
      .outer = rectangle(4.0, 4.0).outer,
      .holes = {{{.x = 5.0, .y = 5.0},
                 {.x = 5.0, .y = 6.0},
                 {.x = 6.0, .y = 6.0},
                 {.x = 6.0, .y = 5.0}}},
  };
  const auto hole_validity = shiny::nesting::geom::validate_polygon(hole_outside);
  REQUIRE_FALSE(hole_validity.is_valid());
  REQUIRE(hole_validity.issue == PolygonValidityIssue::hole_outside_outer);
}

TEST_CASE("geometry spatial index queries overlapping boxes",
          "[geometry][spatial-index]") {
  SpatialIndex index(5.0);
  index.insert(1, Box2{.min = {.x = 0.0, .y = 0.0},
                       .max = {.x = 4.0, .y = 4.0}});
  index.insert(2, Box2{.min = {.x = 7.0, .y = 7.0},
                       .max = {.x = 9.0, .y = 9.0}});
  index.insert(3, Box2{.min = {.x = 3.0, .y = 3.0},
                       .max = {.x = 8.0, .y = 8.0}});

  const auto matches = index.query(
      Box2{.min = {.x = 2.0, .y = 2.0}, .max = {.x = 6.0, .y = 6.0}});

  REQUIRE(matches == std::vector<std::uint32_t>{1U, 3U});
}
