// Contract guard: compute_convex_nfp() must accept convex polygons that
// contain exactly collinear consecutive vertices on their outer ring.
//
// Background: CGAL::minkowski_sum_2() with EPICK is sensitive to degenerate
// edges in some configurations. Two upstream layers already simplify
// collinear vertices before they reach the EPICK boundary:
//   - normalize_request() in src/request.cpp (lines ~64-70 / 84-90),
//   - from_bg_polygon() in src/polygon_ops/boolean_ops.cpp (line ~68).
// This test pins the contract independently of those upstream layers, so
// future refactors that bypass them don't silently expose CGAL to
// collinear inputs again.

#include <catch2/catch_test_macros.hpp>

#include "geometry/types.hpp"
#include "nfp/convex_nfp.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::nfp::compute_convex_nfp;

}  // namespace

TEST_CASE("compute_convex_nfp tolerates collinear vertices on the outer ring",
          "[nfp][convex-nfp][collinear-regression]") {
  // Axis-aligned 4x2 rectangle expressed as a hexagon: the (2,0) and (4,0)
  // bottom edge has an explicit collinear vertex at (2,0) that exactly lies
  // on the segment (0,0)-(4,0). This mirrors the shape Boost.Geometry emits
  // for a fused triangle pair: the merge seam is preserved as an interior
  // collinear vertex.
  const Polygon fixed_with_collinear{
      .outer = {
          Point2{.x = 0.0, .y = 0.0},
          Point2{.x = 2.0, .y = 0.0},  // collinear with (0,0) and (4,0)
          Point2{.x = 4.0, .y = 0.0},
          Point2{.x = 4.0, .y = 2.0},
          Point2{.x = 0.0, .y = 2.0},
      }};

  const Polygon moving{
      .outer = {
          Point2{.x = 0.0, .y = 0.0},
          Point2{.x = 1.0, .y = 0.0},
          Point2{.x = 1.0, .y = 1.0},
          Point2{.x = 0.0, .y = 1.0},
      }};

  // Without upstream simplification (request normalization or boolean-op
  // post-processing), this convex hexagon would reach EPICK
  // CGAL::minkowski_sum_2() with a degenerate edge. The function must
  // tolerate that input shape.
  const auto result = compute_convex_nfp(fixed_with_collinear, moving);
  REQUIRE(result.ok());
  REQUIRE_FALSE(result.value().outer.empty());
}

TEST_CASE("compute_convex_nfp tolerates collinear vertices on both inputs",
          "[nfp][convex-nfp][collinear-regression]") {
  const Polygon fixed_with_collinear{
      .outer = {
          Point2{.x = 0.0, .y = 0.0},
          Point2{.x = 1.5, .y = 0.0},
          Point2{.x = 3.0, .y = 0.0},
          Point2{.x = 3.0, .y = 2.0},
          Point2{.x = 0.0, .y = 2.0},
      }};

  const Polygon moving_with_collinear{
      .outer = {
          Point2{.x = 0.0, .y = 0.0},
          Point2{.x = 1.0, .y = 0.0},
          Point2{.x = 1.0, .y = 0.5},
          Point2{.x = 1.0, .y = 1.0},  // collinear on the right edge
          Point2{.x = 0.0, .y = 1.0},
      }};

  const auto result =
      compute_convex_nfp(fixed_with_collinear, moving_with_collinear);
  REQUIRE(result.ok());
  REQUIRE_FALSE(result.value().outer.empty());
}
