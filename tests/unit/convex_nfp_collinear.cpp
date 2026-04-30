// Contract guard: compute_convex_nfp() must accept convex polygons that
// contain exactly collinear consecutive vertices on their outer ring.
//
// Background: convex Minkowski implementations are sensitive to degenerate
// edges in some configurations. Two upstream layers already simplify
// collinear vertices before they reach the pairwise-sum hull path:
//   - normalize_request() in src/request.cpp (lines ~64-70 / 84-90),
//   - from_bg_polygon() in src/polygon_ops/boolean_ops.cpp (line ~68).
// This test pins the contract independently of those upstream layers, so
// future refactors that bypass them don't silently regress the convex NFP
// kernel on
// collinear inputs again.

#include <catch2/catch_test_macros.hpp>

#include "geometry/types.hpp"
#include "nfp/convex_nfp.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::Ring;
using shiny::nesting::nfp::compute_convex_nfp;

} // namespace

TEST_CASE("compute_convex_nfp tolerates collinear vertices on the outer ring",
          "[nfp][convex-nfp][collinear-regression]") {
  // Axis-aligned 4x2 rectangle expressed as a hexagon: the (2,0) and (4,0)
  // bottom edge has an explicit collinear vertex at (2,0) that exactly lies
  // on the segment (0,0)-(4,0). This mirrors the shape Boost.Geometry emits
  // for a fused triangle pair: the merge seam is preserved as an interior
  // collinear vertex.
  const Polygon fixed_with_collinear(Ring{
      shiny::nesting::geom::Point2(0.0, 0.0),
      shiny::nesting::geom::Point2(2.0, 0.0), // collinear with (0,0) and (4,0)
      shiny::nesting::geom::Point2(4.0, 0.0),
      shiny::nesting::geom::Point2(4.0, 2.0),
      shiny::nesting::geom::Point2(0.0, 2.0),
  });

  const Polygon moving(Ring{shiny::nesting::geom::Point2(0.0, 0.0),
                            shiny::nesting::geom::Point2(1.0, 0.0),
                            shiny::nesting::geom::Point2(1.0, 1.0),
                            shiny::nesting::geom::Point2(0.0, 1.0)});

  // Without upstream simplification (request normalization or boolean-op
  // post-processing), this convex hexagon still reaches the convex NFP kernel
  // with a degenerate edge. The function must tolerate that input shape.
  const auto result = compute_convex_nfp(fixed_with_collinear, moving);
  REQUIRE(result.ok());
  REQUIRE_FALSE(result.value().outer().empty());
}

TEST_CASE("compute_convex_nfp tolerates collinear vertices on both inputs",
          "[nfp][convex-nfp][collinear-regression]") {
  const Polygon fixed_with_collinear(
      Ring{shiny::nesting::geom::Point2(0.0, 0.0),
           shiny::nesting::geom::Point2(1.5, 0.0),
           shiny::nesting::geom::Point2(3.0, 0.0),
           shiny::nesting::geom::Point2(3.0, 2.0),
           shiny::nesting::geom::Point2(0.0, 2.0)});

  const Polygon moving_with_collinear(
      Ring{shiny::nesting::geom::Point2(0.0, 0.0),
           shiny::nesting::geom::Point2(1.0, 0.0),
           shiny::nesting::geom::Point2(1.0, 0.5),
           shiny::nesting::geom::Point2(1.0, 1.0),
           shiny::nesting::geom::Point2(0.0, 1.0)});

  const auto result =
      compute_convex_nfp(fixed_with_collinear, moving_with_collinear);
  REQUIRE(result.ok());
  REQUIRE_FALSE(result.value().outer().empty());
}
