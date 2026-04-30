#pragma once

#include <vector>

namespace shiny::nesting::geom {

/**
 * @brief Cartesian point in model-space coordinates.
 *
 * Stores one polygon vertex or translation anchor in the library's geometric
 * kernel. Ordering and equality are value-based so normalized rings and cache
 * keys can compare points directly.
 *
 * @par Invariants
 * - Coordinates are interpreted in the repository's normalized 2D plane.
 *
 * @par Performance Notes
 * - Trivially copyable aggregate used heavily in rings, segments, and caches.
 */
struct Point2 {
  double x{};
  double y{};

  auto operator<=>(const Point2 &) const = default;
};

/**
 * @brief 2D displacement vector.
 *
 * Represents translation offsets and rotated edge deltas without introducing a
 * separate storage format from Point2.
 *
 * @par Invariants
 * - Components are interpreted as a displacement, not an absolute position.
 *
 * @par Performance Notes
 * - Stored directly inside transforms and placement records.
 */
struct Vector2 {
  double x{};
  double y{};

  auto operator<=>(const Vector2 &) const = default;
};

/**
 * @brief Axis-aligned bounding box over model-space coordinates.
 *
 * Captures coarse spatial extent for polygons, placements, and intermediate
 * geometric queries.
 *
 * @par Invariants
 * - `min` and `max` are expected to bound the same region.
 *
 * @par Performance Notes
 * - Used as a low-cost broad-phase summary before exact geometry checks.
 */
struct Box2 {
  Point2 min{};
  Point2 max{};

  auto operator<=>(const Box2 &) const = default;
};

/**
 * @brief Directed segment between two points.
 *
 * Used for boundary queries, contact extraction, and cut-plan generation.
 *
 * @par Invariants
 * - Endpoints are interpreted as an ordered pair unless a caller explicitly
 *   canonicalizes them.
 *
 * @par Performance Notes
 * - Value comparisons support segment deduplication after canonical ordering.
 */
struct Segment2 {
  Point2 start{};
  Point2 end{};

  auto operator<=>(const Segment2 &) const = default;
};

using Ring = std::vector<Point2>;

/**
 * @brief Simple polygon represented by one outer ring.
 *
 * This is the base representation for convex geometry and for intermediate
 * operations that do not need explicit cavities.
 *
 * @par Invariants
 * - Callers are expected to normalize winding and duplicate closure rules when
 *   required by an algorithm.
 *
 * @par Performance Notes
 * - Keeps storage minimal when holes are not needed.
 */
struct Polygon {
  Ring outer{};
};

/**
 * @brief Polygon with one outer boundary and zero or more hole rings.
 *
 * This is the repository's main shape representation for bins, pieces, and NFP
 * outputs.
 *
 * @par Invariants
 * - `outer` defines the exterior boundary and `holes` define excluded regions.
 *
 * @par Performance Notes
 * - Shared across decomposition, boolean ops, placement, and packing layers.
 */
struct PolygonWithHoles {
  Ring outer{};
  std::vector<Ring> holes{};
};

} // namespace shiny::nesting::geom