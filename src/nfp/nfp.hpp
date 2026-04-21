#pragma once

#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::nfp {

[[nodiscard]] auto compute_nfp(const geom::PolygonWithHoles &fixed,
                               const geom::PolygonWithHoles &moving)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>>;

} // namespace shiny::nesting::nfp
