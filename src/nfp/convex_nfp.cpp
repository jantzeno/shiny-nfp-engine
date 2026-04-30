#include "nfp/convex_nfp.hpp"

#include <algorithm>
#include <vector>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/sanitize.hpp"
#include "geometry/validity.hpp"
#include "logging/shiny_log.hpp"
#include "polygon_ops/convex_hull.hpp"
#include "polygon_ops/simplify.hpp"

namespace shiny::nesting::nfp {
namespace {

[[nodiscard]] auto reflect_polygon(const geom::Polygon &polygon)
    -> geom::Polygon {
  geom::Polygon reflected;
  reflected.outer().reserve(polygon.outer().size());
  for (const auto &point : polygon.outer()) {
    reflected.outer().emplace_back(-point.x(), -point.y());
  }
  return geom::normalize_polygon(reflected);
}

[[nodiscard]] auto pairwise_sum_convex_nfp(const geom::Polygon &fixed,
                                           const geom::Polygon &moving)
    -> util::StatusOr<geom::PolygonWithHoles> {
  const auto reflected_moving = reflect_polygon(moving);
  const auto &fixed_ring = fixed.outer();
  const auto &moving_ring = reflected_moving.outer();
  if (fixed_ring.size() < 3U || moving_ring.size() < 3U) {
    return util::Status::invalid_input;
  }

  std::vector<geom::Point2> point_cloud;
  point_cloud.reserve(fixed_ring.size() * moving_ring.size());
  for (const auto &fixed_point : fixed_ring) {
    for (const auto &moving_point : moving_ring) {
      point_cloud.emplace_back(fixed_point.x() + moving_point.x(),
                               fixed_point.y() + moving_point.y());
    }
  }

  const auto hull = poly::compute_convex_hull(
      std::span<const geom::Point2>(point_cloud.data(), point_cloud.size()));
  auto simplified = poly::simplify_collinear_ring(hull.outer());
  if (simplified.size() < 3U) {
    return util::Status::computation_failed;
  }

  const auto polygon =
      geom::normalize_polygon(geom::PolygonWithHoles(std::move(simplified)));
  if (!geom::validate_polygon(polygon).is_valid() ||
      geom::polygon_area(polygon) <= 0.0) {
    return util::Status::computation_failed;
  }

  return polygon;
}

} // namespace

// No-Fit Polygon for two convex pieces.
//
// Identity:  NFP(A, B) = A ⊕ (-B)   (Minkowski sum of A with reflection of B)
// where -B = { -p : p ∈ B }. For convex polygons, that Minkowski sum is the
// convex hull of every pairwise vertex sum, which lets us stay entirely on the
// engine's Boost.Geometry path without exact-kernel bookkeeping.
//
// Returns a single connected polygon (with holes possible only when the
// input pieces themselves were not convex — guarded against above).
auto compute_convex_nfp(const geom::Polygon &fixed, const geom::Polygon &moving)
    -> util::StatusOr<geom::PolygonWithHoles> {
  const auto normalized_fixed = geom::Polygon(
      geom::sanitize_polygon(geom::PolygonWithHoles(fixed.outer()))
          .polygon.outer());
  const auto normalized_moving = geom::Polygon(
      geom::sanitize_polygon(geom::PolygonWithHoles(moving.outer()))
          .polygon.outer());

  const auto fixed_validity = geom::validate_polygon(normalized_fixed);
  const auto moving_validity = geom::validate_polygon(normalized_moving);

  if (!fixed_validity.is_valid() || !moving_validity.is_valid()) {
    return util::Status::invalid_input;
  }

  const auto fixed_convex = geom::polygon_is_convex(normalized_fixed);
  const auto moving_convex = geom::polygon_is_convex(normalized_moving);
  if (!fixed_convex || !moving_convex) {
    return util::Status::invalid_input;
  }

  const auto reflected_moving = reflect_polygon(normalized_moving);
  const auto reflected_validity = geom::validate_polygon(reflected_moving);
  if (!reflected_validity.is_valid()) {
    return util::Status::invalid_input;
  }

  auto pairwise_sum_nfp =
      pairwise_sum_convex_nfp(normalized_fixed, normalized_moving);
  if (pairwise_sum_nfp.ok()) {
    SHINY_DEBUG("nfp: convex pairwise-sum hull succeeded fixed_outer={} "
                "moving_outer={}",
                normalized_fixed.outer().size(),
                normalized_moving.outer().size());
    return pairwise_sum_nfp;
  }

  SHINY_DEBUG(
      "nfp: convex pairwise-sum hull failed fixed_outer={} moving_outer={}",
      normalized_fixed.outer().size(), normalized_moving.outer().size());
  return util::Status::computation_failed;
}

} // namespace shiny::nesting::nfp
