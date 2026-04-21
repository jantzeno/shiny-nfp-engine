#pragma once

#include "geometry/polygon.hpp"

namespace shiny::nesting::decomp::detail {

// Deterministic ordering for hole-free polygons. Sorts by (bbox.min.x,
// bbox.min.y, area). Used by both the triangulator and the greedy
// convex-merge to ensure identical inputs yield identical NFP cache keys
// downstream.
[[nodiscard]] inline auto polygon_origin_less(const geom::Polygon &lhs,
                                              const geom::Polygon &rhs) -> bool {
  const auto lhs_bounds = geom::compute_bounds(lhs);
  const auto rhs_bounds = geom::compute_bounds(rhs);

  if (lhs_bounds.min.x != rhs_bounds.min.x) {
    return lhs_bounds.min.x < rhs_bounds.min.x;
  }
  if (lhs_bounds.min.y != rhs_bounds.min.y) {
    return lhs_bounds.min.y < rhs_bounds.min.y;
  }
  return geom::polygon_area(lhs) < geom::polygon_area(rhs);
}

} // namespace shiny::nesting::decomp::detail
