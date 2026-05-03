#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::decomp {

struct TriangulationResult {
  std::vector<geom::Polygon> triangles{};
  std::vector<std::array<std::int32_t, 3>> neighbours{};
};

[[nodiscard]] auto triangulate_polygon(const geom::Polygon &polygon)
    -> std::expected<TriangulationResult, util::Status>;

[[nodiscard]] auto triangulate_polygon(const geom::PolygonWithHoles &polygon)
    -> std::expected<TriangulationResult, util::Status>;

} // namespace shiny::nesting::decomp