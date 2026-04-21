#pragma once

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::nfp {

[[nodiscard]] auto compute_convex_nfp(const geom::Polygon &fixed,
                                      const geom::Polygon &moving)
    -> util::StatusOr<geom::PolygonWithHoles>;

} // namespace shiny::nesting::nfp
