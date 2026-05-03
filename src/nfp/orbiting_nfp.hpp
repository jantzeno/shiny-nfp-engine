#pragma once

#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::nfp {

[[nodiscard]] auto compute_orbiting_nfp(const geom::PolygonWithHoles &fixed,
                                        const geom::PolygonWithHoles &moving)
    -> std::expected<std::vector<geom::PolygonWithHoles>, util::Status>;

} // namespace shiny::nesting::nfp
