#pragma once

#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

/**
 * @brief Region accumulator used while merging placed-piece footprints.
 *
 * @par Thread Safety
 * - Container value type with no shared state.
 */
struct MergedRegion {
  std::vector<PolygonWithHoles> regions{};
};

/**
 * @brief Starts a merged-region accumulator from one polygon.
 */
[[nodiscard]] auto make_merged_region(const PolygonWithHoles &polygon)
    -> MergedRegion;

/**
 * @brief Merges two polygons into one merged-region accumulator.
 */
[[nodiscard]] auto merge_region(const PolygonWithHoles &lhs,
                                const PolygonWithHoles &rhs) -> MergedRegion;

/**
 * @brief Adds one polygon into an existing merged-region accumulator.
 */
[[nodiscard]] auto merge_polygon_into_region(const MergedRegion &region,
                                             const PolygonWithHoles &polygon)
    -> MergedRegion;

} // namespace shiny::nesting::geom