#pragma once

#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

/**
 * @brief Removes collinear interior vertices from one ring.
 */
[[nodiscard]] auto simplify_collinear_ring(std::span<const Point2> ring)
    -> Ring;

/**
 * @brief Simplifies the outer ring of a polygon.
 */
[[nodiscard]] auto simplify_polygon(const Polygon &polygon) -> Polygon;

/**
 * @brief Simplifies the outer ring of a polygon with Douglas-Peucker.
 */
[[nodiscard]] auto simplify_polygon_douglas_peucker(const Polygon &polygon,
                                                    double epsilon) -> Polygon;

/**
 * @brief Simplifies the outer ring and holes of a polygon-with-holes.
 */
[[nodiscard]] auto simplify_polygon(const PolygonWithHoles &polygon)
    -> PolygonWithHoles;

/**
 * @brief Simplifies the outer ring and holes of a polygon-with-holes with
 * Douglas-Peucker.
 */
[[nodiscard]] auto
simplify_polygon_douglas_peucker(const PolygonWithHoles &polygon,
                                 double epsilon) -> PolygonWithHoles;

} // namespace shiny::nesting::geom