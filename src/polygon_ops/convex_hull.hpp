#pragma once

#include <span>

#include "geometry/types.hpp"

namespace shiny::nfp::poly {

/**
 * @brief Computes the convex hull of an arbitrary point set.
 *
 * @param points Input point set.
 * @return Convex hull polygon.
 */
[[nodiscard]] auto compute_convex_hull(std::span<const geom::Point2> points)
    -> geom::Polygon;

/**
 * @brief Computes the convex hull of a polygon boundary.
 */
[[nodiscard]] auto compute_convex_hull(const geom::Polygon &polygon)
    -> geom::Polygon;

/**
 * @brief Computes the convex hull of a polygon-with-holes footprint.
 */
[[nodiscard]] auto compute_convex_hull(const geom::PolygonWithHoles &polygon)
    -> geom::Polygon;

} // namespace shiny::nfp::poly