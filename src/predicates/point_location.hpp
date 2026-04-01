#pragma once

#include <cstdint>
#include <span>

#include "geometry/types.hpp"
#include "predicates/classify.hpp"

namespace shiny::nfp::pred {

/**
 * @brief Result of locating a point relative to one segment.
 *
 * @par Invariants
 * - `parametric_t` is meaningful when the relation is on the segment.
 *
 * @par Performance Notes
 * - Returned by value from low-cost boundary checks.
 */
struct PointOnSegmentResult {
  BoundaryRelation relation{BoundaryRelation::off_boundary};
  double parametric_t{0.0};
};

/**
 * @brief Result of locating a point relative to one ring.
 *
 * @par Invariants
 * - Boundary indices are `-1` when the point is not on the ring boundary.
 *
 * @par Performance Notes
 * - Bundles topological classification with the exact boundary feature hit.
 */
struct RingLocationResult {
  PointLocation location{PointLocation::exterior};
  std::int32_t edge_index{-1};
  std::int32_t vertex_index{-1};
};

/**
 * @brief Result of locating a point relative to a polygon with holes.
 *
 * @par Invariants
 * - `inside_hole` is only meaningful for interior/exterior classifications.
 * - Boundary indices remain `-1` when no boundary feature was hit.
 *
 * @par Performance Notes
 * - Encodes both outer-boundary and hole-boundary outcomes in one value.
 */
struct PolygonLocationResult {
  PointLocation location{PointLocation::exterior};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
  std::int32_t boundary_edge_index{-1};
  std::int32_t boundary_vertex_index{-1};
};

/**
 * @brief Locates a point relative to a single segment.
 *
 * @param point Query point in the same coordinate frame as `segment`.
 * @param segment Boundary segment to test.
 * @return Boundary relation and parametric location when the point lies on the
 *   segment.
 */
[[nodiscard]] auto locate_point_on_segment(const geom::Point2 &point,
                                           const geom::Segment2 &segment)
    -> PointOnSegmentResult;

/**
 * @brief Locates a point relative to one polygon ring.
 *
 * @par Algorithm Detail
 * - **Strategy**: Boundary test plus point-in-ring classification.
 * - **Steps**:
 *   1. Test the query point against each ring segment for exact boundary hits.
 *   2. If no boundary feature is hit, evaluate whether the point lies inside or
 *      outside the ring.
 *   3. Return the location and the boundary feature indices when applicable.
 *
 * @par Mathematical Basis
 * - Uses exact point-on-segment predicates for boundary detection, then applies
 *   a parity-style point-in-ring test for interior versus exterior.
 * - Separating boundary and parity stages yields stable topological labels for
 *   vertices, edge interiors, and strict interiors.
 *
 * @par Complexity
 * - **Time**: O(n) where `n` is the ring vertex count.
 * - **Space**: O(1).
 *
 * @par Invariants & Preconditions
 * - @pre `ring` should describe one simple boundary.
 *
 * @par Edge Cases & Degeneracies
 * - Distinguishes vertex hits from edge-interior hits.
 * - Handles degenerate one-point segments through the segment helper.
 *
 * @param point Query point in ring coordinates.
 * @param ring Ring to classify against.
 * @return Ring-level location plus matched boundary indices.
 */
[[nodiscard]] auto locate_point_in_ring(const geom::Point2 &point,
                                        std::span<const geom::Point2> ring)
    -> RingLocationResult;

/**
 * @brief Locates a point relative to a polygon with holes.
 *
 * @param point Query point in polygon coordinates.
 * @param polygon Polygon with holes to test.
 * @return Polygon-level location plus the hole or boundary feature involved.
 */
[[nodiscard]] auto
locate_point_in_polygon(const geom::Point2 &point,
                        const geom::PolygonWithHoles &polygon)
    -> PolygonLocationResult;

} // namespace shiny::nfp::pred