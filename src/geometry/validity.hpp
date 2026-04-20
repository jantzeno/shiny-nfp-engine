#pragma once

#include <cstdint>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

enum class PolygonValidityIssue : std::uint8_t {
  ok = 0,
  non_finite_coordinate = 1,
  too_few_vertices = 2,
  zero_area = 3,
  self_intersection = 4,
  hole_outside_outer = 5,
  hole_intersection = 6,
};

struct PolygonValidity {
  PolygonValidityIssue issue{PolygonValidityIssue::ok};
  std::int32_t ring_index{-1};
  std::int32_t edge_index{-1};

  [[nodiscard]] auto is_valid() const -> bool {
    return issue == PolygonValidityIssue::ok;
  }
};

[[nodiscard]] auto validate_polygon(const Polygon &polygon) -> PolygonValidity;

[[nodiscard]] auto validate_polygon(const PolygonWithHoles &polygon)
    -> PolygonValidity;

} // namespace shiny::nesting::geom
