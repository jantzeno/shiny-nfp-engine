#include "packing/sparrow/runtime/progress.hpp"

namespace shiny::nesting::pack::sparrow::runtime {

auto summarize_progress(const PortProgressSnapshot &snapshot)
    -> ProgressSummary {
  double occupied_area = 0.0;
  double container_area = 0.0;
  for (const auto &bin : snapshot.current_layout.bins) {
    occupied_area += bin.utilization.occupied_area;
    container_area += bin.utilization.container_area;
  }

  return {
      .placed_count = snapshot.current_layout.placement_trace.size(),
      .utilization_percent = container_area <= 0.0
                                 ? 0.0
                                 : (occupied_area / container_area) * 100.0,
  };
}

} // namespace shiny::nesting::pack::sparrow::runtime