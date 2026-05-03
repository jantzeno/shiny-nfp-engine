#pragma once

#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

struct PortPoint {
  double x{0.0};
  double y{0.0};

  bool operator==(const PortPoint &) const = default;
};

using PortRing = std::vector<PortPoint>;

struct PortPolygon {
  PortRing outer{};
  std::vector<PortRing> holes{};
};

[[nodiscard]] auto to_port_polygon(const geom::PolygonWithHoles &polygon)
    -> PortPolygon;

[[nodiscard]] auto to_engine_polygon(const PortPolygon &polygon)
    -> geom::PolygonWithHoles;

} // namespace shiny::nesting::pack::sparrow::adapters