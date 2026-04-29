#include "search/detail/driver_scaffolding.hpp"

#include <algorithm>

namespace shiny::nesting::search::detail {

auto OperationBudget::iteration_limit() const noexcept -> std::size_t {
  if (!external_limit_enabled) {
    return internal_limit;
  }
  return std::min(external_limit, internal_limit);
}

auto OperationBudget::external_limit_reached(
    const std::size_t operations_completed) const noexcept -> bool {
  return external_limit_enabled && operations_completed >= external_limit;
}

auto make_operation_budget(const SolveControl &control,
                           const std::size_t internal_limit) noexcept
    -> OperationBudget {
  return {
      .external_limit_enabled = control.operation_limit > 0U,
      .external_limit = control.operation_limit,
      .internal_limit = internal_limit,
  };
}

auto driver_interrupted(const SolveControl &control,
                        const runtime::TimeBudget &time_budget,
                        const runtime::Stopwatch &stopwatch) noexcept -> bool {
  return control.cancellation.stop_requested() ||
         time_budget.expired(stopwatch);
}

auto driver_stop_reason(const SolveControl &control,
                        const runtime::TimeBudget &time_budget,
                        const runtime::Stopwatch &stopwatch,
                        const bool hit_operation_limit) noexcept -> StopReason {
  if (control.cancellation.stop_requested()) {
    return StopReason::cancelled;
  }
  if (time_budget.expired(stopwatch)) {
    return StopReason::time_limit_reached;
  }
  if (control.operation_limit > 0U && hit_operation_limit) {
    return StopReason::operation_limit_reached;
  }
  return StopReason::completed;
}

// auto driver_make_budget(const SolveControl &control,
//                         const runtime::TimeBudget &time_budget,
//                         const runtime::Stopwatch &stopwatch,
//                         const std::size_t operations_completed) noexcept
//     -> BudgetState {
//   return {
//       .operation_limit_enabled = control.operation_limit > 0U,
//       .operation_limit = control.operation_limit,
//       .operations_completed = operations_completed,
//       .time_limit_enabled = time_budget.enabled(),
//       .time_limit_milliseconds = time_budget.limit_milliseconds(),
//       .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
//       .cancellation_requested = control.cancellation.stop_requested(),
//   };
// }

auto driver_empty_result(StrategyKind strategy, SearchReplay replay) noexcept
    -> NestingResult {
  return {
      .strategy = strategy,
      .total_parts = 0,
      // .budget = driver_make_budget(control, time_budget, stopwatch, 0),
      .stop_reason = StopReason::completed,
      .search = std::move(replay),
  };
}

} // namespace shiny::nesting::search::detail
