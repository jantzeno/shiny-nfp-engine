#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "fixtures/export_surface/mtg_fixture.hpp"
#include "geometry/decomposition/convex_decomposition.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/sanitize.hpp"
#include "geometry/queries/validity.hpp"
#include "geometry/transforms/transform.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/ifp.hpp"
#include "nfp/nfp.hpp"
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
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(width, 0.0),
          shiny::nesting::geom::Point2(width, height),
          shiny::nesting::geom::Point2(0.0, height),
      }));
}

auto sum_area(const std::vector<Polygon> &polygons) -> double {
  return std::accumulate(polygons.begin(), polygons.end(), 0.0,
                         [](const double area, const Polygon &polygon) {
                           return area +
                                  shiny::nesting::geom::polygon_area(polygon);
                         });
}

auto sum_area(const std::vector<PolygonWithHoles> &polygons) -> double {
  return std::accumulate(
      polygons.begin(), polygons.end(), 0.0,
      [](const double area, const PolygonWithHoles &polygon) {
        return area + shiny::nesting::geom::polygon_area(polygon);
      });
}

auto approx_equal(const double lhs, const double rhs) -> bool {
  return std::fabs(lhs - rhs) <= 1e-8;
}

auto point_in_region_set(const shiny::nesting::geom::Point2 &point,
                         const std::vector<PolygonWithHoles> &regions) -> bool {
  return std::any_of(regions.begin(), regions.end(), [&](const auto &region) {
    return shiny::nesting::pred::locate_point_in_polygon(point, region)
               .location != shiny::nesting::pred::PointLocation::exterior;
  });
}

auto mtg_piece_polygon(const shiny::nesting::test::mtg::MtgFixture &fixture,
                       const std::uint32_t piece_id)
    -> const PolygonWithHoles & {
  const auto it = std::find_if(
      fixture.pieces.begin(), fixture.pieces.end(),
      [piece_id](const auto &piece) { return piece.piece_id == piece_id; });
  REQUIRE(it != fixture.pieces.end());
  return it->polygon;
}

} // namespace

TEST_CASE("convex decomposition preserves area across convex, concave, and "
          "holed input",
          "[decomposition][nfp]") {
  const auto convex = rectangle(4.0, 3.0);
  auto convex_parts = decompose_convex(convex);
  REQUIRE(convex_parts.has_value());
  REQUIRE(convex_parts.value().size() == 1U);
  REQUIRE(approx_equal(sum_area(convex_parts.value()),
                       shiny::nesting::geom::polygon_area(convex)));

  const auto l_shape = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 4.0),
          shiny::nesting::geom::Point2(0.0, 4.0),
      }));
  auto l_parts = decompose_convex(l_shape);
  REQUIRE(l_parts.has_value());
  REQUIRE(l_parts.value().size() == 2U);
  REQUIRE(approx_equal(sum_area(l_parts.value()),
                       shiny::nesting::geom::polygon_area(l_shape)));
  for (const auto &part : l_parts.value()) {
    REQUIRE(is_convex(part));
  }

  const auto holed = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(
          rectangle(6.0, 6.0).outer(),
          {{shiny::nesting::geom::Point2(2.0, 2.0),
            shiny::nesting::geom::Point2(2.0, 4.0),
            shiny::nesting::geom::Point2(4.0, 4.0),
            shiny::nesting::geom::Point2(4.0, 2.0)}}));
  auto holed_parts = decompose_convex(holed);
  REQUIRE(holed_parts.has_value());
  REQUIRE(!holed_parts.value().empty());
  REQUIRE(approx_equal(sum_area(holed_parts.value()),
                       shiny::nesting::geom::polygon_area(holed)));
  for (const auto &part : holed_parts.value()) {
    REQUIRE(is_convex(part));
  }
}

TEST_CASE("nfp helpers compute convex NFP, decomposition NFP, orbiting "
          "sliding, and IFR",
          "[nfp]") {
  const auto fixed_square = rectangle(2.0, 2.0);
  const auto moving_square = rectangle(1.0, 1.0);

  auto convex_nfp = shiny::nesting::nfp::compute_convex_nfp(
      shiny::nesting::geom::Polygon(fixed_square.outer()),
      shiny::nesting::geom::Polygon(moving_square.outer()));
  REQUIRE(convex_nfp.has_value());
  REQUIRE(
      shiny::nesting::geom::validate_polygon(convex_nfp.value()).is_valid());
  const auto convex_bounds =
      shiny::nesting::geom::compute_bounds(convex_nfp.value());
  REQUIRE(convex_bounds.min.x() == -1.0);
  REQUIRE(convex_bounds.min.y() == -1.0);
  REQUIRE(convex_bounds.max.x() == 2.0);
  REQUIRE(convex_bounds.max.y() == 2.0);
  REQUIRE(approx_equal(shiny::nesting::geom::polygon_area(convex_nfp.value()),
                       9.0));

  const auto concave = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(3.0, 0.0),
          shiny::nesting::geom::Point2(3.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 3.0),
          shiny::nesting::geom::Point2(0.0, 3.0),
      }));

  auto concave_nfp = shiny::nesting::nfp::compute_nfp(concave, moving_square);
  REQUIRE(concave_nfp.has_value());
  REQUIRE(!concave_nfp.value().empty());
  for (const auto &polygon : concave_nfp.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }
  REQUIRE(sum_area(concave_nfp.value()) > 0.0);

  auto orbiting_concave =
      shiny::nesting::nfp::compute_orbiting_nfp(concave, moving_square);
  REQUIRE(orbiting_concave.has_value());
  REQUIRE(!orbiting_concave.value().empty());
  for (const auto &polygon : orbiting_concave.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }
  REQUIRE(sum_area(orbiting_concave.value()) > 0.0);

  auto orbiting_nfp =
      shiny::nesting::nfp::compute_orbiting_nfp(fixed_square, moving_square);
  REQUIRE(orbiting_nfp.has_value());
  REQUIRE(orbiting_nfp.value().size() == 1U);
  const auto orbiting_bounds =
      shiny::nesting::geom::compute_bounds(orbiting_nfp.value().front());
  REQUIRE(orbiting_bounds == convex_bounds);

  auto ifp = shiny::nesting::nfp::compute_ifp(rectangle(10.0, 6.0),
                                              rectangle(3.0, 2.0));
  REQUIRE(ifp.has_value());
  REQUIRE(ifp.value().size() == 1U);
  const auto ifp_bounds =
      shiny::nesting::geom::compute_bounds(ifp.value().front());
  REQUIRE(ifp_bounds.min.x() == 0.0);
  REQUIRE(ifp_bounds.min.y() == 0.0);
  REQUIRE(ifp_bounds.max.x() == 7.0);
  REQUIRE(ifp_bounds.max.y() == 4.0);
}

TEST_CASE("general IFP excludes concave container notches", "[nfp][ifp]") {
  const auto l_shape = shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 0.0),
          shiny::nesting::geom::Point2(4.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 1.0),
          shiny::nesting::geom::Point2(1.0, 4.0),
          shiny::nesting::geom::Point2(0.0, 4.0),
      }));
  const auto moving_square = rectangle(0.5, 0.5);

  auto ifp =
      shiny::nesting::nfp::compute_inner_fit_polygon(l_shape, moving_square);
  REQUIRE(ifp.has_value());
  REQUIRE(!ifp.value().empty());
  for (const auto &polygon : ifp.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }

  REQUIRE(point_in_region_set(shiny::nesting::geom::Point2(0.25, 0.25),
                              ifp.value()));
  REQUIRE(point_in_region_set(shiny::nesting::geom::Point2(0.25, 3.0),
                              ifp.value()));
  REQUIRE(point_in_region_set(shiny::nesting::geom::Point2(3.0, 0.25),
                              ifp.value()));
  REQUIRE_FALSE(
      point_in_region_set(shiny::nesting::geom::Point2(1.0, 1.0), ifp.value()));
  REQUIRE_FALSE(
      point_in_region_set(shiny::nesting::geom::Point2(2.0, 2.0), ifp.value()));
}

TEST_CASE("MTG actual-polygon CGAL regression pairs return typed NFP outcomes",
          "[nfp][actual-polygons][regression]") {
  struct PairCase {
    std::uint32_t fixed_piece_id;
    std::uint32_t moving_piece_id;
    double moving_rotation_degrees;
  };

  const auto fixture =
      shiny::nesting::test::mtg::load_mtg_fixture_with_actual_polygons();
  const std::vector<PairCase> cases{
      {.fixed_piece_id = 7U,
       .moving_piece_id = 4U,
       .moving_rotation_degrees = 90.0},
      {.fixed_piece_id = 4U,
       .moving_piece_id = 10U,
       .moving_rotation_degrees = 180.0},
      {.fixed_piece_id = 4U,
       .moving_piece_id = 16U,
       .moving_rotation_degrees = 270.0},
  };

  for (const auto &case_data : cases) {
    const auto &fixed = mtg_piece_polygon(fixture, case_data.fixed_piece_id);
    const auto moving = shiny::nesting::geom::rotate(
        mtg_piece_polygon(fixture, case_data.moving_piece_id),
        {.degrees = case_data.moving_rotation_degrees});

    INFO("fixed=" << case_data.fixed_piece_id
                  << " moving=" << case_data.moving_piece_id
                  << " moving_rot=" << case_data.moving_rotation_degrees);
    REQUIRE(shiny::nesting::geom::validate_polygon(fixed).is_valid());
    REQUIRE(shiny::nesting::geom::validate_polygon(moving).is_valid());

    const auto nfp = shiny::nesting::nfp::compute_nfp(fixed, moving);
    REQUIRE((nfp.has_value() ||
             nfp.error() == shiny::nesting::util::Status::computation_failed));
    if (nfp.has_value()) {
      REQUIRE_FALSE(nfp.value().empty());
      for (const auto &polygon : nfp.value()) {
        REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
      }
    }
  }
}

TEST_CASE("MTG actual-polygon regression pair computes exact NFP",
          "[nfp][actual-polygons][regression]") {
  const auto fixture =
      shiny::nesting::test::mtg::load_mtg_fixture_with_actual_polygons();
  const auto fixed = mtg_piece_polygon(fixture, 4U);
  const auto moving = shiny::nesting::geom::rotate(
      mtg_piece_polygon(fixture, 16U), {.degrees = 270.0});

  const auto nfp = shiny::nesting::nfp::compute_nfp(fixed, moving);
  REQUIRE(nfp.has_value());
  REQUIRE_FALSE(nfp.value().empty());
  for (const auto &polygon : nfp.value()) {
    REQUIRE(shiny::nesting::geom::validate_polygon(polygon).is_valid());
  }
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

  const auto first_value = shiny::nesting::nfp::compute_ifp(
      rectangle(8.0, 5.0), rectangle(2.0, 1.0));
  REQUIRE(first_value.has_value());
  cache.put(first_key,
            shiny::nesting::cache::NfpCacheValue{
                .polygons = first_value.value(),
                .accuracy = shiny::nesting::cache::NfpCacheAccuracy::exact,
                .status = shiny::nesting::util::Status::ok,
            });
  REQUIRE(cache.size() == 1U);

  auto cached = cache.get(first_key);
  REQUIRE(cached != nullptr);
  REQUIRE(cached->polygons.size() == 1U);
  REQUIRE(shiny::nesting::geom::compute_bounds(cached->polygons.front()) ==
          Box2{.min = shiny::nesting::geom::Point2(0.0, 0.0),
               .max = shiny::nesting::geom::Point2(6.0, 4.0)});

  const auto second_value = shiny::nesting::nfp::compute_ifp(
      rectangle(6.0, 4.0), rectangle(1.0, 1.0));
  REQUIRE(second_value.has_value());
  cache.put(second_key,
            shiny::nesting::cache::NfpCacheValue{
                .polygons = second_value.value(),
                .accuracy = shiny::nesting::cache::NfpCacheAccuracy::exact,
                .status = shiny::nesting::util::Status::ok,
            });
  REQUIRE(cache.size() == 1U);
  REQUIRE(cache.get(first_key) == nullptr);
  REQUIRE(cache.get(second_key) != nullptr);
}

TEST_CASE("compute_nfp sanitizes duplicate and collinear user geometry",
          "[nfp][sanitize]") {
  const auto noisy_fixed =
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(0.0, 0.0),
          shiny::nesting::geom::Point2(3.0, 0.0),
          shiny::nesting::geom::Point2(3.0, 0.0),
          shiny::nesting::geom::Point2(3.0, 1.0),
          shiny::nesting::geom::Point2(3.0, 2.0),
          shiny::nesting::geom::Point2(0.0, 2.0),
          shiny::nesting::geom::Point2(0.0, 0.0),
      });
  const auto moving = rectangle(1.0, 1.0);

  const auto sanitized = shiny::nesting::geom::sanitize_polygon(noisy_fixed);
  REQUIRE(sanitized.duplicate_vertices > 0U);

  const auto result = shiny::nesting::nfp::compute_nfp(noisy_fixed, moving);
  REQUIRE(result.has_value());
  REQUIRE_FALSE(result.value().empty());
}

TEST_CASE("actual-polygon MTG fixture loads as valid sanitized geometry",
          "[nfp][mtg][sanitize]") {
  const auto fixture =
      shiny::nesting::test::mtg::load_mtg_fixture_with_actual_polygons();

  REQUIRE(fixture.pieces.size() ==
          shiny::nesting::test::mtg::kBaselinePieceCount);
  for (const auto &piece : fixture.pieces) {
    const auto sanitized =
        shiny::nesting::geom::sanitize_polygon(piece.polygon);
    INFO(piece.piece_id);
    REQUIRE(sanitized.duplicate_vertices == 0U);
    REQUIRE(sanitized.zero_length_edges == 0U);
    REQUIRE(sanitized.sliver_rings == 0U);
    REQUIRE(shiny::nesting::geom::validate_polygon(piece.polygon).is_valid());
    REQUIRE(piece.polygon.holes().empty());
  }
}
