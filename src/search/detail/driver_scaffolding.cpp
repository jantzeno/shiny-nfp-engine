#include "search/detail/driver_scaffolding.hpp"

namespace shiny::nesting::search::detail {

auto driver_interrupted(const SolveControl &control,
                        const runtime::TimeBudget &time_budget,
                        const runtime::Stopwatch &stopwatch) noexcept -> bool {
  return control.cancellation.stop_requested() || time_budget.expired(stopwatch);
}

auto driver_stop_reason(const SolveControl &control,
                        const runtime::TimeBudget &time_budget,
                        const runtime::Stopwatch &stopwatch,
                        const bool hit_iteration_limit) noexcept -> StopReason {
  if (control.cancellation.stop_requested()) {
    return StopReason::cancelled;
  }
  if (time_budget.expired(stopwatch)) {
    return StopReason::time_limit_reached;
  }
  if (hit_iteration_limit) {
    return StopReason::iteration_limit_reached;
  }
  return StopReason::completed;
}

auto driver_make_budget(const SolveControl &control,
                        const runtime::TimeBudget &time_budget,
                        const runtime::Stopwatch &stopwatch,
                        const std::size_t iterations_completed) noexcept -> BudgetState {
  return {
      .iteration_limit_enabled = control.iteration_limit > 0U,
      .iteration_limit = control.iteration_limit,
      .iterations_completed = iterations_completed,
      .time_limit_enabled = time_budget.enabled(),
      .time_limit_milliseconds = time_budget.limit_milliseconds(),
      .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
      .cancellation_requested = control.cancellation.stop_requested(),
  };
}

auto driver_empty_result(StrategyKind strategy, SearchReplay replay,
                         const SolveControl &control,
                         const runtime::TimeBudget &time_budget,
                         const runtime::Stopwatch &stopwatch) noexcept -> NestingResult {
  return {
      .strategy = strategy,
      .total_parts = 0,
      .budget = driver_make_budget(control, time_budget, stopwatch, 0),
      .stop_reason = StopReason::completed,
      .search = std::move(replay),
  };
}

} // namespace shiny::nesting::search::detail
