#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numeric>
#include <vector>

#include "decomposition/convex_decomposition.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/validity.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/ifp.hpp"
#include "nfp/nfp.hpp"
#include "cache/nfp_cache.hpp"
#include "nfp/orbiting_nfp.hpp"
#include "predicates/point_location.hpp"
#include "util/status.hpp"

namespace {

using shiny::nesting::decomp::decompose_convex;
using shiny::nesting::decomp::is_convex;
using shiny::nesting::geom::Box2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonWithHoles;

auto rectangle(const double width, const double height) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{.outer = {
                                                  {.x = 0.0, .y = 0.0},
                                                  {.x = width, .y = 0.0},
                                                  {.x = width, .y = height},
                                                  {.x = 0.0, .y = height},
                                              }});
}

auto sum_area(const std::vector<Polygon> &polygons) -> double {
  return std::accumulate(polygons.begin(), polygons.end(), 0.0,
                         [](const double area, const Polygon &polygon) {
                           return area +
                                  shiny::nesting::geom::polygon_area(polygon);
                         });
}

auto sum_area(const std::vector<PolygonWithHoles> &polygons) -> double {
  return std::accumulate(polygons.begin(), polygons.end(), 0.0,
                         [](const double area, const PolygonWithHoles &polygon) {
                           return area +
                                  shiny::nesting::geom::polygon_area(polygon);
                         });
}

auto approx_equal(const double lhs, const double rhs) -> bool {
  return std::fabs(lhs - rhs) <= 1e-8;
}

auto point_in_region_set(
    const shiny::nesting::geom::Point2 &point,
    const std::vector<PolygonWithHoles> &regions) -> bool {
  return std::any_of(regions.begin(), regions.end(), [&](const auto &region) {
    return shiny::nesting::pred::locate_point_in_polygon(point, region).location !=
           shiny::nesting::pred::PointLocation::exterior;
  });
}

} // namespace

TEST_CASE("convex decomposition preserves area across convex, concave, and holed input",
          "[decomposition][nfp]") {
  const auto convex = rectangle(4.0, 3.0);
  auto convex_parts = decompose_convex(convex);
  REQUIRE(convex_parts.ok());
  REQUIRE(convex_parts.value().size() == 1U);
  REQUIRE(approx_equal(sum_area(convex_parts.value()),
                       shiny::nesting::geom::polygon_area(convex)));

  const auto l_shape = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {
          {.x = 0.0, .y = 0.0},
          {.x = 4.0, .y = 0.0},
          {.x = 4.0, .y = 1.0},
          {.x = 1.0, .y = 1.0},
          {.x = 1.0, .y = 4.0},
          {.x = 0.0, .y = 4.0},
      },
  });
  auto l_parts = decompose_convex(l_shape);
  REQUIRE(l_parts.ok());
  REQUIRE(l_parts.value().size() == 2U);
  REQUIRE(approx_equal(sum_area(l_parts.value()),
                       shiny::nesting::geom::polygon_area(l_shape)));
  for (const auto &part : l_parts.value()) {
    REQUIRE(is_convex(part));
  }

  const auto holed = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = rectangle(6.0, 6.0).outer,
      .holes = {{{.x = 2.0, .y = 2.0},
                 {.x = 2.0, .y = 4.0},
                 {.x = 4.0, .y = 4.0},
                 {.x = 4.0, .y = 2.0}}},
  });
  auto holed_parts = decompose_convex(holed);
  REQUIRE(holed_parts.ok());
  REQUIRE(!holed_parts.value().empty());
  REQUIRE(approx_equal(sum_area(holed_parts.value()),
                       shiny::nesting::geom::polygon_area(holed)));
  for (const auto &part : holed_parts.value()) {
    REQUIRE(is_convex(part));
  }
}

TEST_CASE("nfp helpers compute convex NFP, decomposition NFP, orbiting sliding, and IFR",
          "[nfp]") {
  const auto fixed_square = rectangle(2.0, 2.0);
  const auto moving_square = rectangle(1.0, 1.0);

  auto convex_nfp = shiny::nesting::nfp::compute_convex_nfp(
      Polygon{.outer = fixed_square.outer}, Polygon{.outer = moving_square.outer});
  REQUIRE(convex_nfp.ok());
  REQUIRE(shiny::nesting::geom::validate_polygon(convex_nfp.value()).is_valid());
  const auto convex_bounds = shiny::nesting::geom::compute_bounds(convex_nfp.value());
  REQUIRE(convex_bounds.min.x == -1.0);
  REQUIRE(convex_bounds.min.y == -1.0);
  REQUIRE(convex_bounds.max.x == 2.0);
  REQUIRE(convex_bounds.max.y == 2.0);
  REQUIRE(approx_equal(shiny::nesting::geom::polygon_area(convex_nfp.value()), 9.0));

  const auto concave = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {
          {.x = 0.0, .y = 0.0},
          {.x = 3.0, .y = 0.0},
          {.x = 3.0, .y = 1.0},
          {.x = 1.0, .y = 1.0},
          {.x = 1.0, .y = 3.0},
          {.x = 0.0, .y = 3.0},
      },
  });

  auto concave_nfp = shiny::nesting::nfp::compute_nfp(concave, moving_square);
  REQUIRE(concave_nfp.ok());
  REQUIRE(!concave_nfp.value().empty());
  for (const auto &polygon : concave_nfp.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }
  REQUIRE(sum_area(concave_nfp.value()) > 0.0);

  auto orbiting_concave =
      shiny::nesting::nfp::compute_orbiting_nfp(concave, moving_square);
  REQUIRE(orbiting_concave.ok());
  REQUIRE(!orbiting_concave.value().empty());
  for (const auto &polygon : orbiting_concave.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }
  REQUIRE(sum_area(orbiting_concave.value()) > 0.0);

  auto orbiting_nfp =
      shiny::nesting::nfp::compute_orbiting_nfp(fixed_square, moving_square);
  REQUIRE(orbiting_nfp.ok());
  REQUIRE(orbiting_nfp.value().size() == 1U);
  const auto orbiting_bounds =
      shiny::nesting::geom::compute_bounds(orbiting_nfp.value().front());
  REQUIRE(orbiting_bounds == convex_bounds);

  auto ifp = shiny::nesting::nfp::compute_ifp(rectangle(10.0, 6.0),
                                              rectangle(3.0, 2.0));
  REQUIRE(ifp.ok());
  REQUIRE(ifp.value().size() == 1U);
  const auto ifp_bounds = shiny::nesting::geom::compute_bounds(ifp.value().front());
  REQUIRE(ifp_bounds.min.x == 0.0);
  REQUIRE(ifp_bounds.min.y == 0.0);
  REQUIRE(ifp_bounds.max.x == 7.0);
  REQUIRE(ifp_bounds.max.y == 4.0);
}

TEST_CASE("general IFP excludes concave container notches", "[nfp][ifp]") {
  const auto l_shape = shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {
          {.x = 0.0, .y = 0.0},
          {.x = 4.0, .y = 0.0},
          {.x = 4.0, .y = 1.0},
          {.x = 1.0, .y = 1.0},
          {.x = 1.0, .y = 4.0},
          {.x = 0.0, .y = 4.0},
      },
  });
  const auto moving_square = rectangle(0.5, 0.5);

  auto ifp = shiny::nesting::nfp::compute_inner_fit_polygon(l_shape, moving_square);
  REQUIRE(ifp.ok());
  REQUIRE(!ifp.value().empty());
  for (const auto &polygon : ifp.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }

  REQUIRE(point_in_region_set({.x = 0.25, .y = 0.25}, ifp.value()));
  REQUIRE(point_in_region_set({.x = 0.25, .y = 3.0}, ifp.value()));
  REQUIRE(point_in_region_set({.x = 3.0, .y = 0.25}, ifp.value()));
  REQUIRE_FALSE(point_in_region_set({.x = 1.0, .y = 1.0}, ifp.value()));
  REQUIRE_FALSE(point_in_region_set({.x = 2.0, .y = 2.0}, ifp.value()));
}

TEST_CASE("nfp cache stores hits and evicts least recently used entries",
          "[nfp][cache]") {
  shiny::nesting::cache::NfpCache cache(
      {.policy = shiny::nesting::cache::CachePolicy::lru_bounded,
       .max_entries = 1U});
  const auto first_key =
      shiny::nesting::cache::make_nfp_cache_key(11U, 22U, 0.0, 90.0);
  const auto second_key =
      shiny::nesting::cache::make_nfp_cache_key(33U, 44U, 180.0, 270.0);

  REQUIRE(cache.get(first_key) == nullptr);

  const auto first_value = shiny::nesting::nfp::compute_ifp(rectangle(8.0, 5.0),
                                                            rectangle(2.0, 1.0));
  REQUIRE(first_value.ok());
  cache.put(first_key, first_value.value());
  REQUIRE(cache.size() == 1U);

  auto cached = cache.get(first_key);
  REQUIRE(cached != nullptr);
  REQUIRE(cached->size() == 1U);
  REQUIRE(shiny::nesting::geom::compute_bounds(cached->front()) ==
          Box2{.min = {.x = 0.0, .y = 0.0}, .max = {.x = 6.0, .y = 4.0}});

  const auto second_value = shiny::nesting::nfp::compute_ifp(rectangle(6.0, 4.0),
                                                             rectangle(1.0, 1.0));
  REQUIRE(second_value.ok());
  cache.put(second_key, second_value.value());
  REQUIRE(cache.size() == 1U);
  REQUIRE(cache.get(first_key) == nullptr);
  REQUIRE(cache.get(second_key) != nullptr);
}
