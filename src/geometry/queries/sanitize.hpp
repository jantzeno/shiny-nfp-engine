#pragma once

#include <cstddef>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

struct PolygonSanitization {
  PolygonWithHoles polygon{};
  std::size_t duplicate_vertices{0};
  std::size_t zero_length_edges{0};
  std::size_t sliver_rings{0};
};

[[nodiscard]] auto sanitize_polygon(const PolygonWithHoles &polygon)
    -> PolygonSanitization;

} // namespace shiny::nesting::geom