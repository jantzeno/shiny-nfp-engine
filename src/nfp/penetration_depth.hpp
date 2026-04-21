#pragma once

#include "cache/penetration_depth_cache.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::nfp {

// Penetration depth (squared) — minimum squared distance from `point`
// to the boundary of the NFP region. Non-zero only when `point` lies
// in the NFP interior (i.e. the moving piece overlaps the fixed piece
// for that reference position). Returning d² rather than d avoids a
// sqrt in the hot loop of separation-driven evaluators.
[[nodiscard]] auto compute_penetration_depth_squared(
    const geom::PolygonWithHoles &nfp_polygon, const geom::Point2 &point,
    cache::PenetrationDepthCache *cache = nullptr) -> double;

} // namespace shiny::nesting::nfp
