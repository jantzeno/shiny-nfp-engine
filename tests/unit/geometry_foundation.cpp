#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "geometry/concepts.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "geometry/validity.hpp"

namespace {

using shiny::nesting::geom::Box2;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonValidityIssue;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::ResolvedRotation;
using shiny::nesting::geom::Ring;
using shiny::nesting::geom::RotationIndex;
using shiny::nesting::geom::Transform2;
using shiny::nesting::geom::Vector2;

auto rectangle(const double width, const double height) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(width, 0.0),
          shiny::nesting::geom::Point2(width, height),
          shiny::nesting::geom::Point2(0.0, height),
      }));
}

} // namespace

static_assert(shiny::nesting::geom::PointGeometry<Point2>);
static_assert(shiny::nesting::geom::RingGeometry<Ring>);
static_assert(shiny::nesting::geom::PolygonGeometry<Polygon>);
static_assert(shiny::nesting::geom::PolygonWithHolesGeometry<PolygonWithHoles>);
static_assert(shiny::nesting::geom::TransformGeometry<Point2>);
static_assert(shiny::nesting::geom::TransformGeometry<Ring>);
static_assert(shiny::nesting::geom::TransformGeometry<Polygon>);
static_assert(shiny::nesting::geom::TransformGeometry<PolygonWithHoles>);

TEST_CASE("geometry foundation handles convex, concave, and holed polygons",
          "[geometry][foundation]") {
  const auto convex = rectangle(4.0, 3.0);
  const auto concave = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 4.0),
          shiny::nesting::geom::Point2(2.0, 4.0),
          shiny::nesting::geom::Point2(2.0, 2.0),
          shiny::nesting::geom::Point2(0.0, 2.0),
      }));
  const auto holed = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(
          rectangle(5.0, 5.0).outer(),
          {{shiny::nesting::geom::Point2(1.0, 1.0),
            shiny::nesting::geom::Point2(1.0, 2.0),
            shiny::nesting::geom::Point2(2.0, 2.0),
            shiny::nesting::geom::Point2(2.0, 1.0)}}));

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
  const auto rotated =
      shiny::nesting::geom::rotate(polygon, ResolvedRotation{.degrees = 90.0});
  const auto rotated_bounds = shiny::nesting::geom::compute_bounds(rotated);

  REQUIRE(rotated_bounds.min.x() == -1.0);
  REQUIRE(rotated_bounds.max.x() == 0.0);
  REQUIRE(rotated_bounds.min.y() == 0.0);
  REQUIRE(rotated_bounds.max.y() == 2.0);

  shiny::nesting::geom::DiscreteRotationSet rotations{{0.0, 90.0}};
  const auto transformed = shiny::nesting::geom::apply_transform(
      polygon,
      Transform2{.rotation_index = RotationIndex{1},
                 .translation = shiny::nesting::geom::Vector2(10.0, 20.0)},
      rotations);
  REQUIRE(transformed.has_value());
  const auto bounds = shiny::nesting::geom::compute_bounds(*transformed);
  REQUIRE(bounds.min.x() == 9.0);
  REQUIRE(bounds.max.x() == 10.0);
  REQUIRE(bounds.min.y() == 20.0);
  REQUIRE(bounds.max.y() == 22.0);
}

TEST_CASE("geometry validity rejects invalid input", "[geometry][validity]") {
  const PolygonWithHoles self_intersecting(Ring{
      shiny::nesting::geom::Point2(0.0, 0.0),
      shiny::nesting::geom::Point2(2.0, 2.0),
      shiny::nesting::geom::Point2(0.0, 2.0),
      shiny::nesting::geom::Point2(2.0, 0.0),
  });
  const auto self_intersection_validity =
      shiny::nesting::geom::validate_polygon(self_intersecting);
  REQUIRE_FALSE(self_intersection_validity.is_valid());
  REQUIRE(self_intersection_validity.issue ==
          PolygonValidityIssue::self_intersection);

  const PolygonWithHoles hole_outside(
      rectangle(4.0, 4.0).outer(), {Ring{
                                       shiny::nesting::geom::Point2(5.0, 5.0),
                                       shiny::nesting::geom::Point2(5.0, 6.0),
                                       shiny::nesting::geom::Point2(6.0, 6.0),
                                       shiny::nesting::geom::Point2(6.0, 5.0),
                                   }});
  const auto hole_validity =
      shiny::nesting::geom::validate_polygon(hole_outside);
  REQUIRE_FALSE(hole_validity.is_valid());
  REQUIRE(hole_validity.issue == PolygonValidityIssue::hole_outside_outer);
}

TEST_CASE("geometry spatial index queries overlapping boxes",
          "[geometry][spatial-index]") {
  // Removed: bucket-grid `SpatialIndex` was superseded by Boost.Geometry
  // `RTreeIndex`. See `geometry/rtree_index.hpp` and its dedicated tests.
  SUCCEED("SpatialIndex deleted in favour of RTreeIndex");
}
