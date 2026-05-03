#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "result.hpp"

namespace shiny::nesting::pack::sparrow {

struct PortPlacement {
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  geom::Point2 translation{};
  geom::ResolvedRotation resolved_rotation{};
  bool mirrored{false};
  ConstructivePlacementPhase phase{ConstructivePlacementPhase::primary_order};
};

struct PortOverflowLineage {
  std::uint32_t template_bin_id{0};
  std::uint32_t overflow_bin_id{0};
  std::uint32_t source_request_bin_id{0};
};

struct PortMultiBinState {
  std::vector<std::uint32_t> active_bin_ids{};
  std::vector<std::uint32_t> unplaced_piece_ids{};
  std::vector<PortOverflowLineage> overflow_lineage{};
};

struct SeedSolution {
  Layout layout{};
  ConstructiveReplay constructive{};
  std::vector<PortPlacement> placements{};
  PortMultiBinState multi_bin{};
  std::vector<std::uint32_t> unplaced_piece_ids{};
};

struct PortSearchTelemetry {
  std::size_t sampled_placements{0};
  std::size_t accepted_moves{0};
  std::size_t strike_count{0};
  std::size_t infeasible_pool_selections{0};
  std::size_t compression_attempts{0};
};

struct PortSolution {
  Layout layout{};
  ConstructiveReplay constructive{};
  StopReason stop_reason{StopReason::none};
  std::uint64_t effective_seed{0};
  PortMultiBinState multi_bin{};
  PortSearchTelemetry telemetry{};
};

} // namespace shiny::nesting::pack::sparrow