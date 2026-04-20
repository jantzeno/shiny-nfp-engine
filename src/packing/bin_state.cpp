#include "packing/bin_state.hpp"

#include <cmath>

namespace shiny::nesting::pack {
namespace {

[[nodiscard]] auto signed_area(const geom::Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

[[nodiscard]] auto polygon_area(const geom::PolygonWithHoles &polygon)
    -> double {
  long double area = std::abs(signed_area(polygon.outer));
  for (const auto &hole : polygon.holes) {
    area -= std::abs(signed_area(hole));
  }
  return static_cast<double>(area);
}

[[nodiscard]] auto
total_area(const std::vector<geom::PolygonWithHoles> &polygons) -> double {
  double area = 0.0;
  for (const auto &polygon : polygons) {
    area += polygon_area(polygon);
  }
  return area;
}

} // namespace

auto summarize_bin(const BinState &state) -> BinUtilizationSummary {
  const auto container_area = polygon_area(state.container);
  const auto occupied_area = total_area(state.occupied.regions);

  return {
      .bin_id = state.bin_id,
      .placement_count = state.placements.size(),
      .occupied_area = occupied_area,
      .container_area = container_area,
      .utilization =
          container_area > 0.0 ? occupied_area / container_area : 0.0,
  };
}

} // namespace shiny::nesting::pack