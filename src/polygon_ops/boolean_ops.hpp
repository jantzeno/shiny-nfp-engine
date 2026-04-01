#pragma once

#include <vector>

#include "geometry/types.hpp"

namespace shiny::nfp::poly {

/**
 * @brief Computes the polygonal union of two regions.
 *
 * @param lhs Left-hand polygonal region.
 * @param rhs Right-hand polygonal region.
 * @return Zero or more polygons representing the union.
 */
[[nodiscard]] auto union_polygons(const geom::PolygonWithHoles &lhs,
                                  const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles>;

/**
 * @brief Computes the polygonal difference `lhs - rhs`.
 *
 * @param lhs Minuend polygonal region.
 * @param rhs Subtrahend polygonal region.
 * @return Zero or more polygons representing the difference.
 */
[[nodiscard]] auto difference_polygons(const geom::PolygonWithHoles &lhs,
                                       const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles>;

} // namespace shiny::nfp::poly