#pragma once

#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::decomp {

[[nodiscard]] auto is_convex(const geom::Polygon &polygon) -> bool;

// Decompose `polygon` into a set of convex sub-polygons.
//
// Implementation: adjacency-aware convex fusion over a CDT triangulation.
// `triangulate_polygon()` surfaces unconstrained triangle neighbours, and the
// merge loop only attempts unions across the current adjacency graph instead
// of restarting an all-pairs scan after each success. The convexity predicate
// is still enforced on the merged polygon, but candidate generation now
// follows the triangulation structure promised by the plan.
//
// Returns `invalid_input` for self-intersecting or zero-area inputs.
[[nodiscard]] auto decompose_convex(const geom::Polygon &polygon)
    -> std::expected<std::vector<geom::Polygon>, util::Status>;

[[nodiscard]] auto decompose_convex(const geom::PolygonWithHoles &polygon)
    -> std::expected<std::vector<geom::Polygon>, util::Status>;

} // namespace shiny::nesting::decomp