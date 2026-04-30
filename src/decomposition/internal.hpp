#pragma once

#include "geometry/polygon.hpp"

namespace shiny::nesting::decomp::detail {

// Deterministic ordering for hole-free polygons. Sorts by (bbox.min.x,
// bbox.min.y, area). Used by both the triangulator and the greedy
// convex-merge to ensure identical inputs yield identical NFP cache keys
// downstream.
[[nodiscard]] inline auto polygon_origin_less(const geom::Polygon &lhs,
                                              const geom::Polygon &rhs)
    -> bool {
  const auto lhs_bounds = geom::compute_bounds(lhs);
  const auto rhs_bounds = geom::compute_bounds(rhs);

  if (lhs_bounds.min.x() != rhs_bounds.min.x()) {
    return lhs_bounds.min.x() < rhs_bounds.min.x();
  }
  if (lhs_bounds.min.y() != rhs_bounds.min.y()) {
    return lhs_bounds.min.y() < rhs_bounds.min.y();
  }

  const auto lhs_area = geom::polygon_area(lhs);
  const auto rhs_area = geom::polygon_area(rhs);
  if (lhs_area != rhs_area) {
    return lhs_area < rhs_area;
  }

  const auto &lhs_ring = lhs.outer();
  const auto &rhs_ring = rhs.outer();
  if (lhs_ring.size() != rhs_ring.size()) {
    return lhs_ring.size() < rhs_ring.size();
  }

  for (std::size_t index = 0; index < lhs_ring.size(); ++index) {
    if (lhs_ring[index] < rhs_ring[index]) {
      return true;
    }
    if (rhs_ring[index] < lhs_ring[index]) {
      return false;
    }
  }

  return false;
}

} // namespace shiny::nesting::decomp::detail
