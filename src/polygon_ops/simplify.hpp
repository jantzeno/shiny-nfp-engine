#pragma once

#include <span>

#include "geometry/types.hpp"

namespace shiny::nfp::poly {

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
 * @brief Simplifies the outer ring and holes of a polygon-with-holes.
 */
[[nodiscard]] auto simplify_polygon(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles;

} // namespace shiny::nfp::poly