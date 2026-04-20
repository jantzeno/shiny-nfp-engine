#pragma once

#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::poly {

/**
 * @brief Removes collinear interior vertices from one ring.
 */
[[nodiscard]] auto simplify_collinear_ring(std::span<const geom::Point2> ring)
    -> geom::Ring;

/**
 * @brief Simplifies the outer ring of a polygon.
 */
[[nodiscard]] auto simplify_polygon(const geom::Polygon &polygon)
    -> geom::Polygon;

/**
 * @brief Simplifies the outer ring of a polygon with Douglas-Peucker.
 */
[[nodiscard]] auto simplify_polygon_douglas_peucker(const geom::Polygon &polygon,
                                                    double epsilon)
    -> geom::Polygon;

/**
 * @brief Simplifies the outer ring and holes of a polygon-with-holes.
 */
[[nodiscard]] auto simplify_polygon(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles;

/**
 * @brief Simplifies the outer ring and holes of a polygon-with-holes with
 * Douglas-Peucker.
 */
[[nodiscard]] auto simplify_polygon_douglas_peucker(
    const geom::PolygonWithHoles &polygon, double epsilon)
    -> geom::PolygonWithHoles;

} // namespace shiny::nesting::poly
