#include "packing/sparrow/adapters/progress_adapter.hpp"

#include <string>

#include "packing/sparrow/config.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

auto to_profile_progress_snapshot(const runtime::PortProgressSnapshot &snapshot)
    -> ProfileProgressSnapshot {
  const auto profile = snapshot.phase == SparrowPhase::profile_maximum_search
                           ? SolveProfile::maximum_search
                           : SolveProfile::balanced;
  const auto phase = snapshot.stop_reason == StopReason::none
                         ? ProgressPhase::refinement
                         : ProgressPhase::completed;
  return ::shiny::nesting::runtime::make_profile_progress_snapshot(
      profile, phase, std::string(runtime_phase_name(snapshot.phase)),
      snapshot.current_layout, snapshot.best_layout, snapshot.active_bin_id,
      snapshot.elapsed_time_milliseconds, snapshot.remaining_time_milliseconds,
      snapshot.stop_reason, snapshot.improved);
}

} // namespace shiny::nesting::pack::sparrow::adapters