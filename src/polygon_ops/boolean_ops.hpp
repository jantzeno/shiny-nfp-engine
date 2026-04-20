#pragma once

#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::poly {

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
 * @brief Computes the polygonal intersection of two regions.
 *
 * @param lhs Left-hand polygonal region.
 * @param rhs Right-hand polygonal region.
 * @return Zero or more polygons representing the shared area.
 */
[[nodiscard]] auto intersection_polygons(const geom::PolygonWithHoles &lhs,
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

/**
 * @brief Computes the minimum Euclidean distance between two polygonal regions.
 *
 * @param lhs Left-hand polygonal region.
 * @param rhs Right-hand polygonal region.
 * @return Zero when the regions intersect or touch, otherwise the minimum
 *   boundary-to-boundary distance.
 */
[[nodiscard]] auto polygon_distance(const geom::PolygonWithHoles &lhs,
                                    const geom::PolygonWithHoles &rhs) -> double;

/**
 * @brief Inflates (or deflates) a polygon by a uniform distance.
 *
 * Uses Boost.Geometry buffer with miter joins.  A positive @p distance
 * grows the polygon outward; a negative distance shrinks it.
 *
 * @param polygon Source polygonal region.
 * @param distance Offset distance (positive = outward, negative = inward).
 * @return Zero or more polygons representing the buffered result.
 */
[[nodiscard]] auto buffer_polygon(const geom::PolygonWithHoles &polygon,
                                  double distance)
    -> std::vector<geom::PolygonWithHoles>;

} // namespace shiny::nesting::poly
