#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "packing/decoder.hpp"
#include "placement/config.hpp"

namespace shiny::nesting::pack {

struct ShelfState {
  double y{0.0};
  double height{0.0};
  double next_x{0.0};
  double used_min_x{0.0};
  double used_max_x{0.0};
};

struct BinPackingState {
  BinState bin_state{};
  geom::Box2 container_bounds{};
  std::vector<ShelfState> shelves{};
  std::vector<geom::Box2> occupied_bounds{};
  std::vector<geom::Box2> free_rectangles{};
  double occupied_area{0.0};
};

struct PlacementCandidate {
  std::size_t shelf_index{0};
  bool starts_new_shelf{false};
  place::Placement placement{};
  geom::ResolvedRotation resolved_rotation{};
  geom::PolygonWithHoles rotated_piece{};
  geom::Box2 translated_bounds{};
  double resulting_utilization{0.0};
};

struct PieceOrderingMetrics {
  std::size_t original_index{0};
  double width{0.0};
  double height{0.0};
  double polygon_area{0.0};
  double box_area{0.0};
  double max_dimension{0.0};
  double min_dimension{0.0};
  double box_waste_area{0.0};
  double fill_ratio{0.0};
};

} // namespace shiny::nesting::pack
