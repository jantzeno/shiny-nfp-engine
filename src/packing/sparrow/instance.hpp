#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "internal/request_normalization.hpp"

namespace shiny::nesting::pack::sparrow {

struct PortPiece {
  ExpandedPieceInstance instance{};
  std::uint64_t geometry_revision{0};
  geom::PolygonWithHoles polygon{};
  double value{1.0};
  std::vector<std::uint32_t> allowed_bin_ids{};
  std::optional<std::uint32_t> pinned_bin_id{};
  std::optional<geom::RotationIndex> forced_rotation{};
};

struct PortBin {
  ExpandedBinInstance instance{};
  geom::PolygonWithHoles polygon{};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
  std::vector<place::BedExclusionZone> exclusion_zones{};
  bool selected{true};
};

struct PortBinAssignment {
  std::uint32_t piece_id{0};
  std::vector<std::uint32_t> allowed_bin_ids{};
  std::optional<std::uint32_t> pinned_bin_id{};
};

struct PortInstance {
  std::vector<PortPiece> pieces{};
  std::vector<PortBin> bins{};
  std::vector<PortBinAssignment> assignments{};
  std::vector<std::uint32_t> frontier_bin_ids{};
  std::vector<std::uint32_t> selected_bin_ids{};
  ObjectiveMode objective_mode{ObjectiveMode::placement_count};
  bool allow_part_overflow{true};
  bool maintain_bed_assignment{false};
};

} // namespace shiny::nesting::pack::sparrow