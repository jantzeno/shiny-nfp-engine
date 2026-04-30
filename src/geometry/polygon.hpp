#pragma once

#include <cstdint>
#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

[[nodiscard]] auto ring_signed_area(std::span<const Point2> ring) -> double;

[[nodiscard]] auto polygon_area(const Polygon &polygon) -> double;

[[nodiscard]] auto polygon_area(const PolygonWithHoles &polygon) -> double;

// Sum of `polygon_area(p)` for each polygon in the range. Used to fold a
// boolean-op result (which can be an empty set / single piece /
// multipolygon) into a single scalar — e.g., total intersection area for
// overlap detection or total clipped area for containment checks.
[[nodiscard]] auto polygon_area_sum(std::span<const PolygonWithHoles> polygons)
    -> double;

// Euclidean distance between two points. Centralised to keep the same
// hypot formulation across NFP, overlap proxy, and packing code.
[[nodiscard]] auto point_distance(const Point2 &lhs, const Point2 &rhs)
    -> double;

// Squared Euclidean distance — preferred when only ranking distances
// (avoids the sqrt). Centralised to eliminate the ad-hoc dx*dx + dy*dy
// reimplementations that previously appeared in cutting_sequence,
// pierce_point, polygon_ops/simplify, and others.
[[nodiscard]] auto squared_distance(const Point2 &lhs, const Point2 &rhs)
    -> double;

[[nodiscard]] auto compute_bounds(std::span<const Point2> ring) -> Box2;

[[nodiscard]] auto compute_bounds(const Polygon &polygon) -> Box2;

[[nodiscard]] auto compute_bounds(const PolygonWithHoles &polygon) -> Box2;

[[nodiscard]] auto polygon_is_convex(const Polygon &polygon) -> bool;

[[nodiscard]] auto box_width(const Box2 &box) -> double;

[[nodiscard]] auto box_height(const Box2 &box) -> double;

// Convert an axis-aligned bounding box into a rectangular polygon
// (CCW outer ring, no holes). Counter-clockwise winding so the
// polygon has positive signed area, matching the convention used by
// the rest of the geometry layer.
[[nodiscard]] auto box_to_polygon(const Box2 &box) -> PolygonWithHoles;

// Same as `box_to_polygon`, but the rectangle's width is clamped to
// `max_width` (>= 0). Used by the strip optimiser to truncate a
// container envelope to the current target strip width without
// shifting the origin.
[[nodiscard]] auto box_to_polygon_clamped(const Box2 &box, double max_width)
    -> PolygonWithHoles;

[[nodiscard]] auto boxes_overlap(const Box2 &lhs, const Box2 &rhs) -> bool;

[[nodiscard]] auto box_contains(const Box2 &container, const Box2 &candidate)
    -> bool;

[[nodiscard]] auto polygon_revision(const Polygon &polygon) -> std::uint64_t;

[[nodiscard]] auto polygon_revision(const PolygonWithHoles &polygon)
    -> std::uint64_t;

// Invoke `fn` for each ring (outer first, then every hole) of `polygon`.
// `fn` is called with `std::span<const Point2>`.
//
// Centralises the "for outer / for each hole" iteration pattern that
// appears across validity checks, candidate generation, normalization,
// and rendering. Keeps callers from forgetting the hole loop.
template <typename Fn>
auto for_each_ring(const PolygonWithHoles &polygon, Fn &&fn) -> void {
  fn(std::span<const Point2>{polygon.outer()});
  for (const auto &hole : polygon.holes()) {
    fn(std::span<const Point2>{hole});
  }
}

template <typename Fn>
auto for_each_vertex(const PolygonWithHoles &polygon, Fn &&fn) -> void {
  for_each_ring(polygon, [&](std::span<const Point2> ring) {
    for (const auto &vertex : ring) {
      fn(vertex);
    }
  });
}

} // namespace shiny::nesting::geom
