#include "packing/sparrow/adapters/geometry_adapter.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

namespace {

[[nodiscard]] auto to_port_ring(const geom::Ring &ring) -> PortRing {
  PortRing adapted;
  adapted.reserve(ring.size());
  for (const auto &point : ring) {
    adapted.push_back({.x = point.x(), .y = point.y()});
  }
  return adapted;
}

[[nodiscard]] auto to_engine_ring(const PortRing &ring) -> geom::Ring {
  geom::Ring adapted;
  adapted.reserve(ring.size());
  for (const auto &point : ring) {
    adapted.push_back(geom::Point2{point.x, point.y});
  }
  return adapted;
}

} // namespace

auto to_port_polygon(const geom::PolygonWithHoles &polygon) -> PortPolygon {
  PortPolygon adapted{.outer = to_port_ring(polygon.outer())};
  adapted.holes.reserve(polygon.holes().size());
  for (const auto &hole : polygon.holes()) {
    adapted.holes.push_back(to_port_ring(hole));
  }
  return adapted;
}

auto to_engine_polygon(const PortPolygon &polygon) -> geom::PolygonWithHoles {
  std::vector<geom::Ring> holes;
  holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    holes.push_back(to_engine_ring(hole));
  }
  return geom::PolygonWithHoles(to_engine_ring(polygon.outer),
                                std::move(holes));
}

} // namespace shiny::nesting::pack::sparrow::adapters