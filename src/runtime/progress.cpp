#include "runtime/progress.hpp"

#include <unordered_map>
#include <utility>

namespace shiny::nesting::runtime {

auto layout_utilization_percent(const pack::Layout &layout) -> double {
  double total_occupied_area = 0.0;
  double total_container_area = 0.0;
  for (const auto &bin : layout.bins) {
    total_occupied_area += bin.utilization.occupied_area;
    total_container_area += bin.utilization.container_area;
  }
  if (total_container_area <= 0.0) {
    return 0.0;
  }
  return (total_occupied_area / total_container_area) * 100.0;
}

auto summarize_bins(const pack::Layout &layout,
                    const std::optional<std::uint32_t> &active_bin_id)
    -> std::vector<BinProgressSummary> {
  std::unordered_map<std::uint32_t, std::size_t> placed_count_by_bin;
  placed_count_by_bin.reserve(layout.placement_trace.size());
  for (const auto &placement : layout.placement_trace) {
    ++placed_count_by_bin[placement.bin_id];
  }

  std::vector<BinProgressSummary> summary;
  summary.reserve(layout.bins.size());
  for (const auto &bin : layout.bins) {
    const double utilization_percent =
        bin.utilization.container_area <= 0.0
            ? 0.0
            : (bin.utilization.occupied_area / bin.utilization.container_area) *
                  100.0;
    summary.push_back(BinProgressSummary{
        .bin_id = bin.bin_id,
        .placed_count = placed_count_by_bin[bin.bin_id],
        .utilization_percent = utilization_percent,
        .active = active_bin_id.has_value() && *active_bin_id == bin.bin_id,
    });
  }
  return summary;
}

auto make_profile_progress_snapshot(
    const SolveProfile profile, const ProgressPhase phase,
    std::string phase_detail, const pack::Layout &current_layout,
    const pack::Layout &best_layout,
    const std::optional<std::uint32_t> &active_bin_id,
    const std::uint64_t elapsed_time_milliseconds,
    const std::optional<std::uint64_t> &remaining_time_milliseconds,
    const StopReason stop_reason, const bool improved)
    -> ProfileProgressSnapshot {
  return {
      .profile = profile,
      .phase = phase,
      .phase_detail = std::move(phase_detail),
      .current_layout = current_layout,
      .best_layout = best_layout,
      .active_bin_id = active_bin_id,
      .bin_summary = summarize_bins(current_layout, active_bin_id),
      .placed_count = current_layout.placement_trace.size(),
      .utilization_percent = layout_utilization_percent(current_layout),
      .elapsed_time_milliseconds = elapsed_time_milliseconds,
      .remaining_time_milliseconds = remaining_time_milliseconds,
      .stop_reason = stop_reason,
      .improved = improved,
  };
}

} // namespace shiny::nesting::runtime