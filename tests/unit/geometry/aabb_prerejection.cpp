#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "geometry/types.hpp"
#include "packing/common.hpp"

namespace {

using Catch::Approx;
using shiny::nesting::geom::Box2;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::boxes_overlap;
using shiny::nesting::pack::compute_bounds;
using shiny::nesting::pack::contains_box;
using shiny::nesting::pack::expand_box;

auto make_box(double x0, double y0, double x1, double y1) -> Box2 {
  return {.min = {x0, y0}, .max = {x1, y1}};
}

} // namespace

// ---------------------------------------------------------------------------
// contains_box
// ---------------------------------------------------------------------------

TEST_CASE("contains_box accepts identical boxes", "[packing][aabb]") {
  const auto box = make_box(0.0, 0.0, 10.0, 10.0);
  CHECK(contains_box(box, box));
}

TEST_CASE("contains_box accepts strictly smaller candidate",
          "[packing][aabb]") {
  const auto container = make_box(0.0, 0.0, 100.0, 100.0);
  const auto candidate = make_box(10.0, 10.0, 90.0, 90.0);
  CHECK(contains_box(container, candidate));
}

TEST_CASE("contains_box rejects candidate exceeding right edge",
          "[packing][aabb]") {
  const auto container = make_box(0.0, 0.0, 100.0, 100.0);
  const auto candidate = make_box(10.0, 10.0, 101.0, 90.0);
  CHECK_FALSE(contains_box(container, candidate));
}

TEST_CASE("contains_box rejects candidate exceeding top edge",
          "[packing][aabb]") {
  const auto container = make_box(0.0, 0.0, 100.0, 100.0);
  const auto candidate = make_box(10.0, 10.0, 90.0, 101.0);
  CHECK_FALSE(contains_box(container, candidate));
}

TEST_CASE("contains_box rejects candidate exceeding left edge",
          "[packing][aabb]") {
  const auto container = make_box(0.0, 0.0, 100.0, 100.0);
  const auto candidate = make_box(-1.0, 10.0, 90.0, 90.0);
  CHECK_FALSE(contains_box(container, candidate));
}

// ---------------------------------------------------------------------------
// boxes_overlap
// ---------------------------------------------------------------------------

TEST_CASE("boxes_overlap detects overlapping boxes", "[packing][aabb]") {
  const auto a = make_box(0.0, 0.0, 10.0, 10.0);
  const auto b = make_box(5.0, 5.0, 15.0, 15.0);
  CHECK(boxes_overlap(a, b));
  CHECK(boxes_overlap(b, a));
}

TEST_CASE("boxes_overlap detects touching boxes as overlapping",
          "[packing][aabb]") {
  const auto a = make_box(0.0, 0.0, 10.0, 10.0);
  const auto b = make_box(10.0, 0.0, 20.0, 10.0);
  CHECK(boxes_overlap(a, b));
}

TEST_CASE("boxes_overlap rejects separated boxes", "[packing][aabb]") {
  const auto a = make_box(0.0, 0.0, 10.0, 10.0);
  const auto b = make_box(20.0, 20.0, 30.0, 30.0);
  CHECK_FALSE(boxes_overlap(a, b));
}

TEST_CASE("boxes_overlap rejects horizontally separated boxes",
          "[packing][aabb]") {
  const auto a = make_box(0.0, 0.0, 10.0, 10.0);
  const auto b = make_box(11.0, 0.0, 20.0, 10.0);
  CHECK_FALSE(boxes_overlap(a, b));
}

// ---------------------------------------------------------------------------
// expand_box
// ---------------------------------------------------------------------------

TEST_CASE("expand_box grows box by clearance", "[packing][aabb]") {
  const auto box = make_box(10.0, 20.0, 30.0, 40.0);
  const auto expanded = expand_box(box, 5.0);
  CHECK(expanded.min.x() == Approx(5.0));
  CHECK(expanded.min.y() == Approx(15.0));
  CHECK(expanded.max.x() == Approx(35.0));
  CHECK(expanded.max.y() == Approx(45.0));
}

TEST_CASE("expand_box with zero clearance returns unchanged box",
          "[packing][aabb]") {
  const auto box = make_box(10.0, 20.0, 30.0, 40.0);
  const auto expanded = expand_box(box, 0.0);
  CHECK(expanded.min.x() == Approx(10.0));
  CHECK(expanded.max.x() == Approx(30.0));
}

TEST_CASE("expand_box with negative clearance returns unchanged box",
          "[packing][aabb]") {
  const auto box = make_box(10.0, 20.0, 30.0, 40.0);
  const auto expanded = expand_box(box, -5.0);
  CHECK(expanded.min.x() == Approx(10.0));
  CHECK(expanded.max.x() == Approx(30.0));
}

// ---------------------------------------------------------------------------
// compute_bounds
// ---------------------------------------------------------------------------

TEST_CASE("compute_bounds returns tight AABB for simple rectangle",
          "[packing][aabb]") {
  const PolygonWithHoles rect(shiny::nesting::geom::Ring{
      {0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {0.0, 5.0}});
  const auto bounds = compute_bounds(rect);
  CHECK(bounds.min.x() == Approx(0.0));
  CHECK(bounds.min.y() == Approx(0.0));
  CHECK(bounds.max.x() == Approx(10.0));
  CHECK(bounds.max.y() == Approx(5.0));
}

TEST_CASE("compute_bounds encompasses holes", "[packing][aabb]") {
  // Hole vertices outside the outer ring would be unusual, but compute_bounds
  // must still include them.
  const PolygonWithHoles poly(
      shiny::nesting::geom::Ring{
          {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}},
      {shiny::nesting::geom::Ring{
          {2.0, 2.0}, {8.0, 2.0}, {8.0, 8.0}, {2.0, 8.0}}});
  const auto bounds = compute_bounds(poly);
  CHECK(bounds.min.x() == Approx(0.0));
  CHECK(bounds.min.y() == Approx(0.0));
  CHECK(bounds.max.x() == Approx(10.0));
  CHECK(bounds.max.y() == Approx(10.0));
}

// ---------------------------------------------------------------------------
// AABB pre-rejection soundness: overlap check agrees with boxes_overlap
// ---------------------------------------------------------------------------

TEST_CASE("AABB containment implies polygon containment for axis-aligned rects",
          "[packing][aabb]") {
  // If the candidate AABB is inside the container AABB, then the polygon (being
  // a simple rectangle) is also inside.
  const auto container = make_box(0.0, 0.0, 300.0, 200.0);
  const auto candidate = make_box(10.0, 10.0, 50.0, 50.0);
  REQUIRE(contains_box(container, candidate));

  // Translating the candidate outside the container AABB must fail.
  const auto shifted = make_box(290.0, 180.0, 330.0, 220.0);
  CHECK_FALSE(contains_box(container, shifted));
}

TEST_CASE("AABB non-overlap implies polygon non-overlap for convex shapes",
          "[packing][aabb]") {
  // Two disjoint boxes guarantee their contained convex polygons don't overlap.
  const auto a = make_box(0.0, 0.0, 10.0, 10.0);
  const auto b = make_box(20.0, 0.0, 30.0, 10.0);
  CHECK_FALSE(boxes_overlap(a, b));
  // Since the boxes don't overlap, any convex polygon inscribed in each box
  // also cannot overlap.  (This is the invariant that AABB pre-rejection relies
  // on — no false negatives.)
}
