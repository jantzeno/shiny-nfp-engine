#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

#include "observer.hpp"
#include "packing/layout.hpp"
#include "packing/sparrow/config.hpp"

namespace shiny::nesting::pack::sparrow::runtime {

struct ProgressSummary {
  std::size_t placed_count{0};
  double utilization_percent{0.0};
};

struct PortProgressSnapshot {
  pack::Layout current_layout{};
  pack::Layout best_layout{};
  std::optional<std::uint32_t> active_bin_id{};
  std::size_t sampled_placements{0};
  std::size_t accepted_moves{0};
  std::uint64_t elapsed_time_milliseconds{0};
  std::optional<std::uint64_t> remaining_time_milliseconds{};
  StopReason stop_reason{StopReason::none};
  SparrowPhase phase{SparrowPhase::constructive_seed};
  bool improved{false};
};

using ProgressListener = std::function<void(const PortProgressSnapshot &)>;

[[nodiscard]] auto summarize_progress(const PortProgressSnapshot &snapshot)
    -> ProgressSummary;

} // namespace shiny::nesting::pack::sparrow::runtime