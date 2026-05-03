#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "observer.hpp"
#include "packing/layout.hpp"
#include "request.hpp"

namespace shiny::nesting::runtime {

struct BinProgressSummary {
  std::uint32_t bin_id{0};
  std::size_t placed_count{0};
  double utilization_percent{0.0};
  bool active{false};
};

struct ProfileProgressSnapshot {
  SolveProfile profile{SolveProfile::balanced};
  ProgressPhase phase{ProgressPhase::none};
  std::string phase_detail{};
  pack::Layout current_layout{};
  pack::Layout best_layout{};
  std::optional<std::uint32_t> active_bin_id{};
  std::vector<BinProgressSummary> bin_summary{};
  std::size_t placed_count{0};
  double utilization_percent{0.0};
  std::uint64_t elapsed_time_milliseconds{0};
  std::optional<std::uint64_t> remaining_time_milliseconds{};
  StopReason stop_reason{StopReason::none};
  bool improved{false};
};

[[nodiscard]] auto layout_utilization_percent(const pack::Layout &layout)
    -> double;

[[nodiscard]] auto
summarize_bins(const pack::Layout &layout,
               const std::optional<std::uint32_t> &active_bin_id)
    -> std::vector<BinProgressSummary>;

[[nodiscard]] auto make_profile_progress_snapshot(
    SolveProfile profile, ProgressPhase phase, std::string phase_detail,
    const pack::Layout &current_layout, const pack::Layout &best_layout,
    const std::optional<std::uint32_t> &active_bin_id,
    std::uint64_t elapsed_time_milliseconds,
    const std::optional<std::uint64_t> &remaining_time_milliseconds,
    StopReason stop_reason, bool improved) -> ProfileProgressSnapshot;

} // namespace shiny::nesting::runtime

namespace shiny::nesting {

using runtime::BinProgressSummary;
using runtime::ProfileProgressSnapshot;

} // namespace shiny::nesting