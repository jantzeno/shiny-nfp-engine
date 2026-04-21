#pragma once

#include <algorithm>
#include <cmath>

#include "geometry/polygon.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::geom {

[[nodiscard]] inline constexpr auto vector_between(const Point2 &start,
                                                   const Point2 &end) -> Vector2 {
  return {.x = end.x - start.x, .y = end.y - start.y};
}

[[nodiscard]] inline constexpr auto point_plus_vector(const Point2 &point,
                                                      const Vector2 &vector)
    -> Point2 {
  return {.x = point.x + vector.x, .y = point.y + vector.y};
}

[[nodiscard]] inline constexpr auto scale_vector(const Vector2 &vector,
                                                 const double scale) -> Vector2 {
  return {.x = vector.x * scale, .y = vector.y * scale};
}

[[nodiscard]] inline constexpr auto dot(const Vector2 &lhs, const Vector2 &rhs)
    -> double {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

[[nodiscard]] inline constexpr auto cross(const Vector2 &lhs, const Vector2 &rhs)
    -> double {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] inline auto vector_length(const Vector2 &vector) -> double {
  return std::hypot(vector.x, vector.y);
}

// Returns `vector / |vector|`, or the zero vector when `|vector| <=
// tolerance`. `tolerance` is in the same units as the input components
// (engine convention: millimeters); pass 0.0 to fall back to strict
// length-zero rejection.
[[nodiscard]] inline auto normalize_vector(const Vector2 &vector,
                                           const double tolerance = 0.0)
    -> Vector2 {
  const auto length = vector_length(vector);
  if (length <= tolerance) {
    return {};
  }
  return scale_vector(vector, 1.0 / length);
}

// Project `point` onto the line carrying `segment` and return the
// parametric position `t` clamped to [0, 1]. Degenerate (zero-length)
// segments yield `t = 0.0` so callers receive `segment.start` from
// `closest_point_on_segment`.
[[nodiscard]] inline auto project_point_to_segment(const Point2 &point,
                                                   const Segment2 &segment)
    -> double {
  const auto edge = vector_between(segment.start, segment.end);
  const auto length_squared = dot(edge, edge);
  if (length_squared <= 0.0) {
    return 0.0;
  }

  return std::clamp(dot(vector_between(segment.start, point), edge) / length_squared,
                    0.0, 1.0);
}

// Closest point on `segment` to `point`. For degenerate segments
// returns `segment.start` (see `project_point_to_segment`).
[[nodiscard]] inline auto closest_point_on_segment(const Point2 &point,
                                                   const Segment2 &segment)
    -> Point2 {
  return point_plus_vector(
      segment.start,
      scale_vector(vector_between(segment.start, segment.end),
                   project_point_to_segment(point, segment)));
}

[[nodiscard]] inline auto point_to_segment_distance(const Point2 &point,
                                                    const Segment2 &segment)
    -> double {
  return point_distance(point, closest_point_on_segment(point, segment));
}

// True when `point` lies within `tolerance` of `segment` (Euclidean
// distance, same coordinate units as the inputs). Degenerate segments
// reduce to a point-to-point test against `segment.start`.
[[nodiscard]] inline auto point_on_segment(const Point2 &point,
                                           const Segment2 &segment,
                                           const double tolerance) -> bool {
  return point_to_segment_distance(point, segment) <= tolerance;
}

static_assert(dot(Vector2{.x = 1.0, .y = 0.0}, Vector2{.x = 0.0, .y = 1.0}) ==
              0.0);
static_assert(cross(Vector2{.x = 1.0, .y = 0.0}, Vector2{.x = 0.0, .y = 1.0}) ==
              1.0);

} // namespace shiny::nesting::geom
