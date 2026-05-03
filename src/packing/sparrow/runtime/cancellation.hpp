#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "observer.hpp"
#include "runtime/cancellation.hpp"
#include "runtime/timing.hpp"

namespace shiny::nesting::pack::sparrow::runtime {

enum class StopTrigger : std::uint8_t {
  none = 0,
  cancellation = 1,
  time_limit = 2,
  operation_limit = 3,
};

struct TerminationStatus {
  bool cancellation_requested{false};
  bool time_limit_reached{false};
  bool operation_limit_enabled{false};
  std::optional<std::uint64_t> remaining_time_milliseconds{};
};

struct TerminationDecision {
  StopTrigger trigger{StopTrigger::none};
  StopReason stop_reason{StopReason::completed};
  std::optional<std::uint64_t> remaining_time_milliseconds{};

  [[nodiscard]] auto stop_requested() const -> bool {
    return trigger != StopTrigger::none;
  }
};

struct TerminationContext {
  ::shiny::nesting::runtime::CancellationToken cancellation{};
  const ::shiny::nesting::runtime::TimeBudget *time_budget{nullptr};
  const ::shiny::nesting::runtime::Stopwatch *stopwatch{nullptr};
  std::size_t operation_limit{0};
};

[[nodiscard]] inline auto
capture_termination_status(const TerminationContext &context)
    -> TerminationStatus {
  const bool time_budget_active = context.time_budget != nullptr &&
                                  context.stopwatch != nullptr &&
                                  context.time_budget->enabled();
  return {
      .cancellation_requested = context.cancellation.stop_requested(),
      .time_limit_reached = time_budget_active &&
                            context.time_budget->expired(*context.stopwatch),
      .operation_limit_enabled = context.operation_limit > 0U,
      .remaining_time_milliseconds =
          time_budget_active ? std::optional<std::uint64_t>(
                                   context.time_budget->remaining_milliseconds(
                                       *context.stopwatch))
                             : std::nullopt,
  };
}

[[nodiscard]] inline auto
interruption_requested(const TerminationStatus &status) -> bool {
  return status.cancellation_requested || status.time_limit_reached;
}

[[nodiscard]] inline auto classify_stop(const TerminationStatus &status,
                                        const bool hit_operation_limit)
    -> TerminationDecision {
  if (status.cancellation_requested) {
    return {
        .trigger = StopTrigger::cancellation,
        .stop_reason = StopReason::cancelled,
        .remaining_time_milliseconds = status.remaining_time_milliseconds,
    };
  }
  if (status.time_limit_reached) {
    return {
        .trigger = StopTrigger::time_limit,
        .stop_reason = StopReason::time_limit_reached,
        .remaining_time_milliseconds = status.remaining_time_milliseconds,
    };
  }
  if (status.operation_limit_enabled && hit_operation_limit) {
    return {
        .trigger = StopTrigger::operation_limit,
        .stop_reason = StopReason::operation_limit_reached,
        .remaining_time_milliseconds = status.remaining_time_milliseconds,
    };
  }
  return {
      .trigger = StopTrigger::none,
      .stop_reason = StopReason::completed,
      .remaining_time_milliseconds = status.remaining_time_milliseconds,
  };
}

[[nodiscard]] inline auto stop_trigger_name(const StopTrigger trigger)
    -> std::string_view {
  switch (trigger) {
  case StopTrigger::none:
    return "none";
  case StopTrigger::cancellation:
    return "cancellation";
  case StopTrigger::time_limit:
    return "time_limit";
  case StopTrigger::operation_limit:
    return "operation_limit";
  }
  return "unknown";
}

} // namespace shiny::nesting::pack::sparrow::runtime