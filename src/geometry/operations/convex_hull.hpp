#pragma once

#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

/**
 * @brief Computes the convex hull of an arbitrary point set.
 *
 * @param points Input point set.
 * @return Convex hull polygon.
 */
[[nodiscard]] auto compute_convex_hull(std::span<const Point2> points)
    -> Polygon;

/**
 * @brief Computes the convex hull of a polygon boundary.
 */
[[nodiscard]] auto compute_convex_hull(const Polygon &polygon) -> Polygon;

/**
 * @brief Computes the convex hull of a polygon-with-holes footprint.
 */
[[nodiscard]] auto compute_convex_hull(const PolygonWithHoles &polygon)
    -> Polygon;

} // namespace shiny::nesting::geom