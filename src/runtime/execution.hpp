#pragma once

#include <cstddef>
#include <cstdint>

namespace shiny::nfp::runtime {

/**
 * @brief Configures run-scoped observability and execution controls.
 *
 * Short paragraph describing the non-geometric settings that govern retained
 * event history, timestamps, worker provisioning, time budgets, and
 * cancellation support for one execution session.
 *
 * @par Invariants
 * - `max_retained_events` stays within the supported bounded history range.
 * - `progress_event_interval_ms` stays within the supported runtime interval
 *   range.
 * - `worker_count` stays within the supported bounded worker range.
 *
 * @par Performance Notes
 * - Retained event history is capped explicitly to avoid unbounded growth.
 */
struct ExecutionControlConfig {
  bool capture_timestamps{true};
  bool retain_event_history{true};
  std::size_t max_retained_events{256};
  bool record_event_timeline{true};
  std::uint32_t progress_event_interval_ms{250};
  std::uint32_t worker_count{1};
  std::uint32_t time_budget_ms{0};
  bool allow_cancellation{true};

  /**
   * @brief Reports whether the execution-control settings are supported.
   *
   * Short paragraph describing the bounded ranges used by runtime-level
   * bookkeeping and execution control.
   *
   * @return `true` when the event-retention and timing settings fall within
   *   the supported ranges.
   *
   * @pre None.
   * @post Does not modify the config.
   * @par Determinism
   * - Deterministic for a fixed config state.
   */
  [[nodiscard]] auto is_valid() const -> bool {
    return max_retained_events >= 1U && max_retained_events <= 10'000U &&
           progress_event_interval_ms >= 50U &&
           progress_event_interval_ms <= 5'000U && worker_count >= 1U &&
           worker_count <= 256U && time_budget_ms <= 86'400'000U;
  }
};

} // namespace shiny::nfp::runtime