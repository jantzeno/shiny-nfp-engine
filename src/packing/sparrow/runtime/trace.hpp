#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::pack::sparrow::runtime {

struct SampledPlacementEvent {
  std::uint64_t seed{0};
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  std::size_t sample_index{0};
  geom::Point2 translation{};
  double rotation_degrees{0.0};
};

struct AcceptedMoveEvent {
  std::uint64_t seed{0};
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  double objective_before{0.0};
  double objective_after{0.0};
};

struct StrikeCountEvent {
  std::uint64_t seed{0};
  std::size_t iteration{0};
  std::size_t strike_count{0};
};

struct InfeasiblePoolSelectionEvent {
  std::uint64_t seed{0};
  std::uint32_t piece_id{0};
  std::size_t pool_size{0};
  std::size_t rank{0};
};

struct CompressionAttemptEvent {
  std::uint64_t seed{0};
  std::size_t iteration{0};
  double shrink_ratio{0.0};
  bool accepted{false};
};

struct TraceCapture {
  std::vector<SampledPlacementEvent> sampled_placements{};
  std::vector<AcceptedMoveEvent> accepted_moves{};
  std::vector<StrikeCountEvent> strike_counts{};
  std::vector<InfeasiblePoolSelectionEvent> infeasible_pool_selections{};
  std::vector<CompressionAttemptEvent> compression_attempts{};

  auto clear() -> void {
    sampled_placements.clear();
    accepted_moves.clear();
    strike_counts.clear();
    infeasible_pool_selections.clear();
    compression_attempts.clear();
  }

  [[nodiscard]] auto empty() const -> bool {
    return sampled_placements.empty() && accepted_moves.empty() &&
           strike_counts.empty() && infeasible_pool_selections.empty() &&
           compression_attempts.empty();
  }
};

} // namespace shiny::nesting::pack::sparrow::runtime