#pragma once

#include <cmath>

#include "geometry/types.hpp"

namespace shiny::nesting::detail {

inline constexpr double kCanonicalCoordinateScale = 1000000000.0;

/**
 * @brief Quantizes one coordinate onto the canonical comparison grid.
 *
 * @param value Raw floating-point coordinate.
 * @return Canonicalized coordinate used for tolerant equality checks.
 */
[[nodiscard]] inline auto canonicalize_coordinate(double value) -> double {
  return std::nearbyint(value * kCanonicalCoordinateScale) /
         kCanonicalCoordinateScale;
}

/**
 * @brief Orders points lexicographically by `x` then `y`.
 */
[[nodiscard]] inline auto point_less(const geom::Point2 &lhs,
                                     const geom::Point2 &rhs) -> bool {
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }

  return lhs.y < rhs.y;
}

/**
 * @brief Compares points after canonical coordinate quantization.
 */
[[nodiscard]] inline auto near_same_point(const geom::Point2 &lhs,
                                          const geom::Point2 &rhs) -> bool {
  return canonicalize_coordinate(lhs.x) == canonicalize_coordinate(rhs.x) &&
         canonicalize_coordinate(lhs.y) == canonicalize_coordinate(rhs.y);
}

} // namespace shiny::nesting::detail