#include "packing/overlap_proxy.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <span>
#include <vector>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/vector_ops.hpp"
#include "nfp/penetration_depth.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack {
namespace {

// sqrt(2): half-cell-diagonal scaling factor used to bound how far the
// best point inside a square cell could be from its centre. For a square
// of half-side `h`, the farthest interior point is at distance
// `h * sqrt(2)` (a corner). The PoI search uses
// `distance(centre) + h * sqrt(2)` as an upper bound on the best signed
// distance any point in the cell can achieve, which prunes cells that
// cannot beat the current best within `epsilon`.
constexpr double kSqrt2 = 1.4142135623730951;

struct PoleSearchCell {
  geom::Point2 center{};
  double half_size{0.0};
  double distance{0.0};
  double max_distance{0.0};
};

struct PoleSearchCellOrder {
  [[nodiscard]] auto operator()(const PoleSearchCell &lhs,
                                const PoleSearchCell &rhs) const -> bool {
    return lhs.max_distance < rhs.max_distance;
  }
};

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

// Returns the minimum squared distance from `point` to any segment of
// `ring`. For an empty/degenerate ring (no segments) returns
// `numeric_limits<double>::max()` so callers reducing across multiple
// rings ignore the empty ring; importantly, an empty outer ring must
// NOT read as a perfect "boundary distance == 0" match because that
// would silently turn a degenerate polygon into the global PoI
// candidate.
[[nodiscard]] auto ring_distance_squared(const geom::Point2 &point,
                                         std::span<const geom::Point2> ring)
    -> double {
  double min_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const geom::Segment2 segment{ring[index], ring[(index + 1U) % ring.size()]};
    min_distance =
        std::min(min_distance, point_segment_distance_squared(point, segment));
  }
  return min_distance;
}

[[nodiscard]] auto
polygon_boundary_distance_squared(const geom::PolygonWithHoles &polygon,
                                  const geom::Point2 &point) -> double {
  double min_distance = ring_distance_squared(point, polygon.outer);
  for (const auto &hole : polygon.holes) {
    min_distance = std::min(
        min_distance,
        ring_distance_squared(point, std::span<const geom::Point2>(hole)));
  }
  return min_distance;
}

// Sign convention:
//   * positive  -> `point` is INSIDE the polygon; value is the distance
//                  to the nearest boundary (i.e. the largest inscribed
//                  circle radius achievable at this centre).
//   * zero      -> `point` lies ON the boundary.
//   * negative  -> `point` is OUTSIDE; value is -distance-to-boundary.
// The PoI search maximises this value, so positive interior distances
// dominate negative exterior ones automatically.
[[nodiscard]] auto
signed_distance_to_polygon(const geom::PolygonWithHoles &polygon,
                           const geom::Point2 &point,
                           cache::PenetrationDepthCache *pd_cache) -> double {
  const auto location = pred::locate_point_in_polygon(point, polygon);
  if (location.location == pred::PointLocation::boundary) {
    return 0.0;
  }

  if (location.location == pred::PointLocation::interior) {
    const auto distance_squared =
        nfp::compute_penetration_depth_squared(polygon, point, pd_cache);
    return std::sqrt(std::max(0.0, distance_squared));
  }

  return -std::sqrt(
      std::max(0.0, polygon_boundary_distance_squared(polygon, point)));
}

[[nodiscard]] auto make_pole_search_cell(const geom::PolygonWithHoles &polygon,
                                         const geom::Point2 &center,
                                         const double half_size,
                                         cache::PenetrationDepthCache *pd_cache)
    -> PoleSearchCell {
  const auto distance = signed_distance_to_polygon(polygon, center, pd_cache);
  return {
      .center = center,
      .half_size = half_size,
      .distance = distance,
      .max_distance = distance + half_size * kSqrt2,
  };
}

[[nodiscard]] auto better_pole_candidate(const PoleSearchCell &lhs,
                                         const PoleSearchCell &rhs) -> bool {
  if (lhs.distance != rhs.distance) {
    return lhs.distance > rhs.distance;
  }
  if (lhs.center.x != rhs.center.x) {
    return lhs.center.x < rhs.center.x;
  }
  return lhs.center.y < rhs.center.y;
}

// Builds the initial grid of square cells covering `bounds` with side
// length `cell_size`, pushing each cell into `queue`. Extracted so the
// loop body stays in lock-step with the child-subdivision loop below
// (both call `make_pole_search_cell` with the same conventions).
auto compute_initial_cells(
    const geom::PolygonWithHoles &normalized, const geom::Box2 &bounds,
    const double cell_size, cache::PenetrationDepthCache *pd_cache,
    std::priority_queue<PoleSearchCell, std::vector<PoleSearchCell>,
                        PoleSearchCellOrder> &queue) -> void {
  const auto half_size = cell_size / 2.0;
  for (double x = bounds.min.x; x < bounds.max.x; x += cell_size) {
    for (double y = bounds.min.y; y < bounds.max.y; y += cell_size) {
      queue.push(make_pole_search_cell(normalized,
                                       {.x = x + half_size, .y = y + half_size},
                                       half_size, pd_cache));
    }
  }
}

} // namespace

// Pole-of-Inaccessibility search via polylabel-style best-first quadtree.
//
// PoI = the interior point with maximum distance to the polygon boundary;
// equivalently, the centre of the largest inscribed circle. The search
// keeps a max-heap of square cells ordered by an upper bound on the best
// distance any point in that cell could achieve (`distance + half*sqrt(2)`).
// Cells whose upper bound is already within `epsilon` of the current best
// are discarded; the rest are subdivided into four children.
auto compute_pole_of_inaccessibility(const geom::PolygonWithHoles &polygon,
                                     const double epsilon,
                                     cache::PoleCache *pole_cache,
                                     cache::PenetrationDepthCache *pd_cache)
    -> PoleOfInaccessibility {
  const auto polygon_revision = geom::polygon_revision(polygon);
  return compute_pole_of_inaccessibility(polygon, polygon_revision, pole_cache,
                                         pd_cache, epsilon);
}

auto compute_pole_of_inaccessibility(const geom::PolygonWithHoles &polygon,
                                     const std::uint64_t polygon_revision,
                                     cache::PoleCache *pole_cache,
                                     cache::PenetrationDepthCache *pd_cache,
                                     const double epsilon)
    -> PoleOfInaccessibility {
  if (polygon.outer.empty()) {
    return {};
  }

  const auto effective_epsilon = std::max(epsilon, 1e-9);
  if (pole_cache != nullptr) {
    const auto key =
        cache::make_pole_cache_key(polygon_revision, effective_epsilon);
    if (auto cached = pole_cache->get(key); cached != nullptr) {
      return *cached;
    }
  }

  const auto normalized = geom::normalize_polygon(polygon);
  const auto bounds = geom::compute_bounds(normalized);
  const auto width = geom::box_width(bounds);
  const auto height = geom::box_height(bounds);
  const auto initial_cell_size = std::max(0.0, std::min(width, height));

  PoleSearchCell best =
      make_pole_search_cell(normalized,
                            {.x = (bounds.min.x + bounds.max.x) / 2.0,
                             .y = (bounds.min.y + bounds.max.y) / 2.0},
                            0.0, pd_cache);
  if (!normalized.outer.empty()) {
    const auto vertex_candidate = make_pole_search_cell(
        normalized, normalized.outer.front(), 0.0, pd_cache);
    if (better_pole_candidate(vertex_candidate, best)) {
      best = vertex_candidate;
    }
  }

  if (initial_cell_size > 0.0) {
    std::priority_queue<PoleSearchCell, std::vector<PoleSearchCell>,
                        PoleSearchCellOrder>
        queue;
    compute_initial_cells(normalized, bounds, initial_cell_size, pd_cache,
                          queue);

#ifndef NDEBUG
    double previous_max_distance = std::numeric_limits<double>::infinity();
#endif

    while (!queue.empty()) {
      const auto cell = queue.top();
      queue.pop();

#ifndef NDEBUG
      assert(cell.max_distance <= previous_max_distance + effective_epsilon);
      previous_max_distance = cell.max_distance;
#endif

      if (better_pole_candidate(cell, best)) {
        best = cell;
      }
      if (cell.max_distance <= best.distance + effective_epsilon) {
        continue;
      }

      const auto child_half_size = cell.half_size / 2.0;
      if (child_half_size <= 0.0) {
        continue;
      }

      for (const auto delta_x : {-child_half_size, child_half_size}) {
        for (const auto delta_y : {-child_half_size, child_half_size}) {
          queue.push(make_pole_search_cell(
              normalized,
              {.x = cell.center.x + delta_x, .y = cell.center.y + delta_y},
              child_half_size, pd_cache));
        }
      }
    }
  }

  PoleOfInaccessibility result{
      .center = best.center,
      .radius = std::max(0.0, best.distance),
  };
  if (pole_cache != nullptr) {
    pole_cache->put(
        cache::make_pole_cache_key(polygon_revision, effective_epsilon),
        result);
  }
  return result;
}

// Cheap overlap proxy via inscribed circles (poles).
//   penetration = r_lhs + r_rhs - dist(c_lhs, c_rhs)
// is positive iff the two inscribed circles intersect — a loose lower
// bound on actual polygon overlap and a smooth upper bound on
// "near-overlap" pressure. Hyperbolic decay `penetration / (1 + dist)`
// damps the value as the centres move apart so the gradient remains
// bounded for distant pairs.
//
// CAVEAT: poles are inscribed CIRCLES, not the polygons themselves. Two
// non-overlapping concave polygons CAN have intersecting poles. Callers
// that interpret a non-zero return as "actually overlapping" will be
// wrong; use this only as a smooth proxy / soft penalty term.
auto overlap_proxy_loss(const geom::PolygonWithHoles &lhs,
                        const geom::PolygonWithHoles &rhs,
                        cache::PoleCache *pole_cache,
                        cache::PenetrationDepthCache *pd_cache,
                        const double epsilon) -> double {
  return overlap_proxy_loss(lhs, geom::polygon_revision(lhs), rhs,
                            geom::polygon_revision(rhs), pole_cache, pd_cache,
                            epsilon);
}

auto overlap_proxy_loss(const geom::PolygonWithHoles &lhs,
                        const std::uint64_t lhs_revision,
                        const geom::PolygonWithHoles &rhs,
                        const std::uint64_t rhs_revision,
                        cache::PoleCache *pole_cache,
                        cache::PenetrationDepthCache *pd_cache,
                        const double epsilon) -> double {
  const auto lhs_pole = compute_pole_of_inaccessibility(
      lhs, lhs_revision, pole_cache, pd_cache, epsilon);
  const auto rhs_pole = compute_pole_of_inaccessibility(
      rhs, rhs_revision, pole_cache, pd_cache, epsilon);
  const auto center_distance =
      geom::point_distance(lhs_pole.center, rhs_pole.center);
  const auto penetration =
      std::max(0.0, lhs_pole.radius + rhs_pole.radius - center_distance);
  return penetration > 0.0 ? penetration / (1.0 + center_distance) : 0.0;
}

} // namespace shiny::nesting::pack
