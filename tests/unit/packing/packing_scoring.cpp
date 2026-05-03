#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <limits>
#include <vector>

#include "geometry/queries/normalize.hpp"
#include "geometry/types.hpp"
#include "packing/collision_tracker.hpp"
#include "packing/sample_evaluator.hpp"
#include "packing/shape_penalty.hpp"

namespace {

namespace geom = shiny::nesting::geom;
namespace pack = shiny::nesting::pack;

// Build a closed axis-aligned rectangle as a normalized PolygonWithHoles.
// Winding and closure are handled by normalize_polygon.
auto make_rect(const double x0, const double y0, const double x1,
               const double y1) -> geom::PolygonWithHoles {
  return geom::normalize_polygon(
      geom::PolygonWithHoles(geom::Ring{geom::Point2(x0, y0),
                                        geom::Point2(x1, y0),
                                        geom::Point2(x1, y1),
                                        geom::Point2(x0, y1)}));
}

// Build a CollisionTracker with a 30×30 container and two items:
//   item 0 – 3×3 square at (25,25)–(28,28)  [the "moving" item]
//   item 1 – 5×5 square at ( 0, 0)–( 5, 5)  [a fixed neighbor]
auto make_two_item_tracker() -> pack::CollisionTracker {
  const auto container = make_rect(0, 0, 30, 30);
  std::vector<pack::CollisionTrackerItem> items{
      pack::CollisionTrackerItem{
          .item_id = 0, .polygon = make_rect(25, 25, 28, 28)},
      pack::CollisionTrackerItem{
          .item_id = 1, .polygon = make_rect(0, 0, 5, 5)},
  };
  return pack::CollisionTracker{container, std::move(items)};
}

} // namespace

// ---------------------------------------------------------------------------
// C1 — shape_penalty
// ---------------------------------------------------------------------------

TEST_CASE("shape_penalty scales with convex hull area of both polygons",
          "[shape-penalty]") {
  const auto unit_square  = make_rect(0, 0,  1,  1); // hull area = 1
  const auto large_square = make_rect(0, 0, 10, 10); // hull area = 100

  const double p_unit  = pack::shape_penalty(unit_square, unit_square);
  const double p_large = pack::shape_penalty(large_square, large_square);

  // sqrt(sqrt(1) * sqrt(1)) = 1.0
  REQUIRE_THAT(p_unit, Catch::Matchers::WithinAbs(1.0, 1e-9));
  // sqrt(sqrt(100) * sqrt(100)) = sqrt(10 * 10) = 10.0
  REQUIRE_THAT(p_large, Catch::Matchers::WithinAbs(10.0, 1e-9));
  // Strict ordering: larger hull area → larger penalty
  REQUIRE(p_large > p_unit);
}

TEST_CASE("shape_penalty is symmetric", "[shape-penalty]") {
  const auto narrow = make_rect(0, 0,  3, 4); // hull area = 12
  const auto wide   = make_rect(0, 0,  5, 6); // hull area = 30

  const double p_ab = pack::shape_penalty(narrow, wide);
  const double p_ba = pack::shape_penalty(wide, narrow);

  REQUIRE(p_ab == Catch::Approx(p_ba));
}

TEST_CASE("shape_penalty for a mixed pair lies strictly between same-size pair penalties",
          "[shape-penalty]") {
  const auto small = make_rect(0, 0,  1,  1); // hull area =   1
  const auto large = make_rect(0, 0, 10, 10); // hull area = 100

  const double p_ss = pack::shape_penalty(small, small);
  const double p_sl = pack::shape_penalty(small, large);
  const double p_ll = pack::shape_penalty(large, large);

  // Geometric-mean structure: p_sl = (p_ss * p_ll)^(1/2) in this case
  // because penalty = (hull_a * hull_b)^(1/4).  Regardless, strict
  // ordering must hold.
  REQUIRE(p_ss < p_sl);
  REQUIRE(p_sl < p_ll);
}

// ---------------------------------------------------------------------------
// C2 — sample_evaluator
// ---------------------------------------------------------------------------

TEST_CASE("evaluate_sample returns zero loss when candidate is inside container "
          "and far from all other items",
          "[sample-evaluator]") {
  const auto tracker = make_two_item_tracker();

  // Candidate for item 0: 3×3 at (18,18)–(21,21) — well inside the 30×30
  // container, far from item 1 at (0,0)–(5,5).
  // Pole distances: candidate pole centre ≈ (19.5,19.5), item-1 pole
  // centre = (2.5,2.5) → distance ≈ 24.0 >> pole_radii sum of 1.5+2.5=4.0
  // → overlap proxy == 0.  No outside-container area either.
  const auto candidate_far = make_rect(18, 18, 21, 21);
  const auto result = pack::evaluate_sample(
      tracker, 0, candidate_far, std::numeric_limits<double>::max());

  REQUIRE_FALSE(result.early_terminated);
  REQUIRE_THAT(result.weighted_loss, Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("evaluate_sample weighted loss is strictly higher for a fully overlapping placement",
          "[sample-evaluator]") {
  const auto tracker = make_two_item_tracker();

  const auto candidate_far        = make_rect(18, 18, 21, 21);
  const auto candidate_overlapping = make_rect(0, 0, 5, 5); // stacked on item 1

  const auto result_far = pack::evaluate_sample(
      tracker, 0, candidate_far, std::numeric_limits<double>::max());
  const auto result_overlapping = pack::evaluate_sample(
      tracker, 0, candidate_overlapping, std::numeric_limits<double>::max());

  // Full overlap of two 5×5 squares: area = 25, shape_penalty = 5.0
  // → weighted_loss = 25 * 5 = 125.  Far candidate: 0.  Strict ordering.
  REQUIRE(result_overlapping.weighted_loss > result_far.weighted_loss);
  REQUIRE_THAT(result_overlapping.weighted_loss,
               Catch::Matchers::WithinAbs(125.0, 1e-6));
}

TEST_CASE("evaluate_sample early-terminates when running loss exceeds best known threshold",
          "[sample-evaluator]") {
  // Container: 10×10 at (0,0).  One item (moving index 0) placed inside.
  const auto container = make_rect(0, 0, 10, 10);
  std::vector<pack::CollisionTrackerItem> items{
      pack::CollisionTrackerItem{.item_id = 0,
                                 .polygon = make_rect(1, 1, 3, 3)},
  };
  pack::CollisionTracker tracker{container, std::move(items)};

  // Candidate fully outside the 10×10 container (area outside = 25).
  // With best_known_loss = 0.0, the very first check
  //   weighted_loss(25) > 0.0 → early_terminated = true.
  const auto candidate_outside = make_rect(15, 15, 20, 20);
  const auto result = pack::evaluate_sample(tracker, 0, candidate_outside, 0.0);

  REQUIRE(result.early_terminated);
  REQUIRE(result.weighted_loss > 0.0);
}
