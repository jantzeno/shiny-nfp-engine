#pragma once

#include "geometry/types.hpp"

namespace shiny::nesting::pack {

[[nodiscard]] auto shape_penalty(const geom::PolygonWithHoles &lhs,
                                 const geom::PolygonWithHoles &rhs) -> double;

} // namespace shiny::nesting::pack
