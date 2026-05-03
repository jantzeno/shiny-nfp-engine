#pragma once

#include <cstdint>
#include <optional>

#include "packing/sparrow/solution.hpp"

namespace shiny::nesting::pack::sparrow::eval {

struct ConstructiveSeedReplayMetadata {
  std::size_t placement_count{0};
  std::size_t unplaced_count{0};
  std::size_t frontier_change_count{0};
  std::size_t overflow_event_count{0};
  std::size_t exhaustion_event_count{0};
  std::optional<std::uint32_t> terminal_bin_id{};
};

struct ConstructiveSeedEvaluation {
  double score{0.0};
  double utilization_percent{0.0};
  bool complete_warm_start{false};
  std::uint64_t warm_start_signature{0};
  ConstructiveSeedReplayMetadata replay{};
};

[[nodiscard]] auto evaluate_constructive_seed(const SeedSolution &seed)
    -> ConstructiveSeedEvaluation;

} // namespace shiny::nesting::pack::sparrow::eval