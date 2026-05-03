#pragma once

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::nfp {

[[nodiscard]] auto compute_convex_nfp(const geom::Polygon &fixed,
                                      const geom::Polygon &moving)
    -> std::expected<geom::PolygonWithHoles, util::Status>;

} // namespace shiny::nesting::nfp
