#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::pred {

/**
 * @brief Canonical orientation classification used by the predicate layer.
 */
enum class Orientation : std::int8_t {
  right_turn = -1,
  collinear = 0,
  left_turn = 1,
};

/**
 * @brief Canonical point-in-region classification.
 */
enum class PointLocation : std::int8_t {
  exterior = 0,
  boundary = 1,
  interior = 2,
};

/**
 * @brief Canonical segment-contact classification.
 */
enum class SegmentContactKind : std::int8_t {
  disjoint = 0,
  proper_intersection = 1,
  endpoint_touch = 2,
  collinear_overlap = 3,
  parallel_disjoint = 4,
};

/**
 * @brief Canonical relation between a point and a polygon boundary.
 */
enum class BoundaryRelation : std::int8_t {
  off_boundary = 0,
  on_edge_interior = 1,
  on_vertex = 2,
};

/**
 * @brief Returns the lexicographically smallest vertex index in a ring.
 *
 * @param ring Input ring.
 * @return Index of the smallest vertex by `x` then `y`.
 */
[[nodiscard]] auto
lexicographic_min_vertex_index(std::span<const geom::Point2> ring)
    -> std::size_t;

} // namespace shiny::nesting::pred