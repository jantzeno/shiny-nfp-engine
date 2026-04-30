#include "nfp/penetration_depth.hpp"

#include <algorithm>
#include <limits>
#include <span>

#include "cache/penetration_depth_cache.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/vector_ops.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::nfp {
namespace {

[[nodiscard]] auto point_segment_distance_squared(const geom::Point2 &point,
                                                  const geom::Segment2 &segment)
    -> double {
  const auto edge = geom::vector_between(segment.start, segment.end);
  const auto length_squared = geom::dot(edge, edge);
  if (length_squared == 0.0) {
    return geom::squared_distance(point, segment.start);
  }

  return geom::squared_distance(point,
                                geom::closest_point_on_segment(point, segment));
}

[[nodiscard]] auto ring_distance_squared(const geom::Point2 &point,
                                         std::span<const geom::Point2> ring)
    -> double {
  double min_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const geom::Segment2 segment{ring[index], ring[(index + 1U) % ring.size()]};
    min_distance =
        std::min(min_distance, point_segment_distance_squared(point, segment));
  }
  return min_distance == std::numeric_limits<double>::max() ? 0.0
                                                            : min_distance;
}

} // namespace

// Penetration Depth (squared) — minimum squared distance from `point` to
// the boundary of the NFP region. Non-zero only when `point` lies in the
// NFP interior. Returning d² rather than d avoids a sqrt in the hot loop
// of separation-driven evaluators; callers that need d compute sqrt
// themselves.
//
// Cache key includes `polygon_revision(nfp_polygon)` so geometry changes
// invalidate stale entries automatically. Coordinates are quantised to
// 1e-3 units (`kQuantizationScale = 1000`) which gives near-by hits a
// chance to share an entry; for 1mm-scale geometry that's 1µm bins.
//
// PERF: the cache lookup is performed BEFORE normalising the polygon so
// hot-path repeats avoid redundant O(n) ring rewrites. The polygon
// revision is read directly because it is invariant under our
// `normalize_polygon` rotations.
auto compute_penetration_depth_squared(
    const geom::PolygonWithHoles &nfp_polygon, const geom::Point2 &point,
    cache::PenetrationDepthCache *cache_ptr) -> double {
  const auto revision = geom::polygon_revision(nfp_polygon);
  const auto key = cache::make_penetration_depth_cache_key(revision, point);
  if (cache_ptr != nullptr) {
    if (auto cached = cache_ptr->get(key); cached != nullptr) {
      return *cached;
    }
  }

  const auto normalized = geom::normalize_polygon(nfp_polygon);
  const auto location = pred::locate_point_in_polygon(point, normalized);
  if (location.location == pred::PointLocation::exterior ||
      location.location == pred::PointLocation::boundary) {
    if (cache_ptr != nullptr) {
      cache_ptr->put(key, 0.0);
    }
    return 0.0;
  }

  double distance_squared = ring_distance_squared(point, normalized.outer());
  for (const auto &hole : normalized.holes()) {
    distance_squared =
        std::min(distance_squared, ring_distance_squared(point, hole));
  }

  if (cache_ptr != nullptr) {
    cache_ptr->put(key, distance_squared);
  }
  return distance_squared;
}

} // namespace shiny::nesting::nfp
