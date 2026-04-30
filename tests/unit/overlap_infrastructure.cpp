#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

#include "cache/penetration_depth_cache.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/penetration_depth.hpp"
#include "packing/collision_tracker.hpp"
#include "packing/overlap_proxy.hpp"
#include "predicates/point_location.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::PoleOfInaccessibility;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer =
          {
              {.x = min_x, .y = min_y},
              {.x = max_x, .y = min_y},
              {.x = max_x, .y = max_y},
              {.x = min_x, .y = max_y},
          },
  });
}

auto brute_force_pole(const PolygonWithHoles &polygon,
                      const std::size_t resolution) -> PoleOfInaccessibility {
  const auto bounds = shiny::nesting::geom::compute_bounds(polygon);
  PoleOfInaccessibility best{
      .center = {.x = (bounds.min.x + bounds.max.x) / 2.0,
                 .y = (bounds.min.y + bounds.max.y) / 2.0},
      .radius = 0.0,
  };

  const auto evaluate = [&](const Point2 &candidate) {
    const auto location =
        shiny::nesting::pred::locate_point_in_polygon(candidate, polygon);
    if (location.location == shiny::nesting::pred::PointLocation::exterior) {
      return;
    }

    const auto distance_squared =
        shiny::nesting::nfp::compute_penetration_depth_squared(polygon,
                                                               candidate);
    const auto radius = std::sqrt(std::max(0.0, distance_squared));
    if (radius > best.radius) {
      best = {.center = candidate, .radius = radius};
    }
  };

  evaluate(best.center);
  const auto safe_resolution = std::max<std::size_t>(2U, resolution);
  const auto step_x = shiny::nesting::geom::box_width(bounds) /
                      static_cast<double>(safe_resolution);
  const auto step_y = shiny::nesting::geom::box_height(bounds) /
                      static_cast<double>(safe_resolution);
  for (std::size_t row = 0; row <= safe_resolution; ++row) {
    for (std::size_t column = 0; column <= safe_resolution; ++column) {
      evaluate({
          .x = bounds.min.x + static_cast<double>(column) * step_x,
          .y = bounds.min.y + static_cast<double>(row) * step_y,
      });
    }
  }
  return best;
}

} // namespace

TEST_CASE("penetration depth returns squared distance to NFP boundary",
          "[nfp][penetration-depth]") {
  auto nfp = shiny::nesting::nfp::compute_convex_nfp(
      Polygon{.outer = rectangle(0.0, 0.0, 2.0, 2.0).outer},
      Polygon{.outer = rectangle(0.0, 0.0, 2.0, 2.0).outer});
  REQUIRE(nfp.ok());

  shiny::nesting::cache::PenetrationDepthCache cache;
  const auto depth = shiny::nesting::nfp::compute_penetration_depth_squared(
      nfp.value(), Point2{.x = 1.0, .y = 0.0}, &cache);
  REQUIRE(depth == Catch::Approx(1.0).margin(1e-8));

  const auto exterior = shiny::nesting::nfp::compute_penetration_depth_squared(
      nfp.value(), Point2{.x = 3.0, .y = 0.0}, &cache);
  REQUIRE(exterior == Catch::Approx(0.0).margin(1e-8));
}

TEST_CASE("overlap proxy distinguishes overlapping and separated shapes",
          "[packing][overlap-proxy]") {
  const auto lhs = rectangle(0.0, 0.0, 2.0, 2.0);
  const auto rhs_overlap = rectangle(1.0, 0.0, 3.0, 2.0);
  const auto rhs_separated = rectangle(4.0, 0.0, 6.0, 2.0);
  shiny::nesting::cache::PoleCache pole_cache;
  shiny::nesting::cache::PenetrationDepthCache pd_cache;

  const auto overlap_loss = shiny::nesting::pack::overlap_proxy_loss(
      lhs, rhs_overlap, &pole_cache, &pd_cache);
  const auto separated_loss = shiny::nesting::pack::overlap_proxy_loss(
      lhs, rhs_separated, &pole_cache, &pd_cache);
  const auto uncached_overlap_loss =
      shiny::nesting::pack::overlap_proxy_loss(lhs, rhs_overlap);

  REQUIRE(overlap_loss > 0.0);
  REQUIRE(separated_loss == Catch::Approx(0.0).margin(1e-8));
  REQUIRE(overlap_loss == Catch::Approx(uncached_overlap_loss).margin(1e-12));
  REQUIRE(pole_cache.size() == 3U);
  REQUIRE(pd_cache.size() > 0U);
}

TEST_CASE("pole of inaccessibility cache reuses cached result",
          "[packing][overlap-proxy]") {
  const auto polygon = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer =
          {
              {.x = 0.0, .y = 0.0},
              {.x = 6.0, .y = 0.0},
              {.x = 6.0, .y = 1.0},
              {.x = 2.0, .y = 1.0},
              {.x = 2.0, .y = 5.0},
              {.x = 0.0, .y = 5.0},
          },
  });
  shiny::nesting::cache::PoleCache pole_cache;
  shiny::nesting::cache::PenetrationDepthCache pd_cache;

  const auto first = shiny::nesting::pack::compute_pole_of_inaccessibility(
      polygon, 1e-3, &pole_cache, &pd_cache);
  const auto cached_pole_entries = pole_cache.size();
  const auto cached_pd_entries = pd_cache.size();
  const auto second = shiny::nesting::pack::compute_pole_of_inaccessibility(
      polygon, 1e-3, &pole_cache, &pd_cache);

  REQUIRE(cached_pole_entries == 1U);
  REQUIRE(cached_pd_entries > 0U);
  REQUIRE(pole_cache.size() == cached_pole_entries);
  REQUIRE(pd_cache.size() == cached_pd_entries);
  REQUIRE(second.center.x == Catch::Approx(first.center.x).margin(1e-12));
  REQUIRE(second.center.y == Catch::Approx(first.center.y).margin(1e-12));
  REQUIRE(second.radius == Catch::Approx(first.radius).margin(1e-12));
}

TEST_CASE("pole of inaccessibility matches high resolution reference on "
          "concave polygon",
          "[packing][overlap-proxy]") {
  const auto polygon = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer =
          {
              {.x = 0.0, .y = 0.0},
              {.x = 7.0, .y = 0.0},
              {.x = 7.0, .y = 2.0},
              {.x = 3.0, .y = 2.0},
              {.x = 3.0, .y = 7.0},
              {.x = 0.0, .y = 7.0},
          },
  });

  const auto reference = brute_force_pole(polygon, 256U);
  const auto polylabel =
      shiny::nesting::pack::compute_pole_of_inaccessibility(polygon);

  REQUIRE(polylabel.radius == Catch::Approx(reference.radius).margin(2e-2));
  REQUIRE(shiny::nesting::geom::point_distance(polylabel.center,
                                               reference.center) <= 5e-2);
}

TEST_CASE("pole of inaccessibility respects hole boundaries",
          "[packing][overlap-proxy]") {
  const auto polygon = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = rectangle(0.0, 0.0, 6.0, 6.0).outer,
      .holes = {rectangle(2.0, 2.0, 4.0, 4.0).outer},
  });

  const auto pole =
      shiny::nesting::pack::compute_pole_of_inaccessibility(polygon);
  const auto location =
      shiny::nesting::pred::locate_point_in_polygon(pole.center, polygon);
  const auto expected_radius = 2.0 / (1.0 + 1.0 / std::sqrt(2.0));

  REQUIRE(location.location == shiny::nesting::pred::PointLocation::interior);
  REQUIRE(location.inside_hole == false);
  REQUIRE(pole.radius == Catch::Approx(expected_radius).margin(1e-3));
}

TEST_CASE("collision tracker incremental updates match full recompute",
          "[packing][collision-tracker]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 10.0);
  std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1,
       .geometry_revision = 1,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.item_id = 2,
       .geometry_revision = 2,
       .polygon = rectangle(2.0, 0.0, 6.0, 4.0)},
      {.item_id = 3,
       .geometry_revision = 3,
       .polygon = rectangle(7.0, 0.0, 9.0, 2.0)},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  REQUIRE(tracker.total_loss() > 0.0);

  const auto moved_polygon = rectangle(5.0, 0.0, 9.0, 4.0);
  tracker.register_item_move(1U, moved_polygon);
  items[1].polygon = moved_polygon;
  shiny::nesting::pack::CollisionTracker recomputed(container, items);

  REQUIRE(tracker.total_loss() ==
          Catch::Approx(recomputed.total_loss()).margin(1e-8));
  REQUIRE(tracker.weighted_total_loss() ==
          Catch::Approx(recomputed.weighted_total_loss()).margin(1e-8));

  const auto original_weight = tracker.pair_weight(0U, 1U);
  tracker.update_gls_weights();
  REQUIRE(tracker.pair_weight(0U, 1U) >= original_weight);
}

TEST_CASE("collision tracker pair-loss cache reuses exact overlap states only",
          "[packing][collision-tracker]") {
  const auto container = rectangle(0.0, 0.0, 12.0, 12.0);
  const auto item_a = rectangle(0.0, 0.0, 4.0, 4.0);
  const auto item_b = rectangle(2.0, 0.0, 6.0, 4.0);
  const auto item_b_separated = rectangle(6.0, 0.0, 10.0, 4.0);
  std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1, .geometry_revision = 1, .polygon = item_a},
      {.item_id = 2, .geometry_revision = 2, .polygon = item_b},
      {.item_id = 3,
       .geometry_revision = 3,
       .polygon = rectangle(0.0, 6.0, 2.0, 8.0)},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  const auto initial_cache_size = tracker.pair_loss_cache_size();
  const auto initial_stats = tracker.pair_loss_cache_stats();
  REQUIRE(initial_cache_size == 1U);
  REQUIRE(initial_stats.hits == 0U);
  REQUIRE(initial_stats.misses == 3U);

  tracker.register_item_move(1U, item_b_separated);
  const auto after_separation_stats = tracker.pair_loss_cache_stats();
  REQUIRE(tracker.pair_loss_cache_size() == initial_cache_size);
  REQUIRE(after_separation_stats.hits == initial_stats.hits);
  REQUIRE(after_separation_stats.misses == initial_stats.misses + 2U);

  tracker.register_item_move(1U, item_b);
  const auto after_restore_stats = tracker.pair_loss_cache_stats();
  REQUIRE(tracker.pair_loss_cache_size() == initial_cache_size);
  REQUIRE(after_restore_stats.hits == after_separation_stats.hits + 1U);
  REQUIRE(after_restore_stats.misses == after_separation_stats.misses + 1U);
  REQUIRE(tracker.pair_loss(0U, 1U) > 0.0);
  REQUIRE(tracker.pair_exact_loss(0U, 1U) > 0.0);
}

// Regression for review finding "GLS decay branch fix": the
// `update_gls_weights` decay branch must gate on the `exact` flag so
// that a pair with proxy-only pressure does not collapse to the same
// decay path as a fully cleared pair. We can't reliably construct two
// polygons whose inscribed circles overlap while their material does
// not (inscribed circle ⊂ polygon, so the case is degenerate); the
// regression therefore verifies (a) that the configurable
// `weight_cap` is honoured, and (b) that an active pair amplifies
// while a separated pair decays toward 1.
TEST_CASE("collision tracker GLS weight update honours fix invariants",
          "[packing][collision-tracker][gls]") {
  const auto container = rectangle(0.0, 0.0, 100.0, 100.0);
  std::vector<shiny::nesting::pack::CollisionTrackerItem> items{
      {.item_id = 1,
       .geometry_revision = 1,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.item_id = 2,
       .geometry_revision = 2,
       .polygon = rectangle(2.0, 0.0, 6.0, 4.0)},
      {.item_id = 3,
       .geometry_revision = 3,
       .polygon = rectangle(50.0, 50.0, 54.0, 54.0)},
  };

  shiny::nesting::pack::CollisionTracker tracker(container, items);
  REQUIRE(tracker.pair_exact_loss(0U, 1U) > 0.0);
  REQUIRE(tracker.pair_exact_loss(0U, 2U) == Catch::Approx(0.0).margin(1e-12));
  REQUIRE(tracker.pair_loss(0U, 2U) == Catch::Approx(0.0).margin(1e-12));

  for (int i = 0; i < 5; ++i) {
    tracker.update_gls_weights();
  }
  // Active overlap pair amplifies (exact > 0 branch).
  REQUIRE(tracker.pair_weight(0U, 1U) > 1.0);
  // Separated pair decays toward 1 (loss == 0 branch).
  REQUIRE(tracker.pair_weight(0U, 2U) == Catch::Approx(1.0));

  // Configurable cap clamps the weight from above.
  constexpr double kSmallCap = 1.05;
  for (int i = 0; i < 50; ++i) {
    tracker.update_gls_weights(kSmallCap);
  }
  REQUIRE(tracker.pair_weight(0U, 1U) <= kSmallCap + 1e-12);
}
