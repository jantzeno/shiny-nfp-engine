#pragma once

#include <functional>

#include "runtime/execution.hpp"
#include "search/observer.hpp"

namespace shiny::nfp::search {

/**
 * @brief Registers one run-scoped observer callback.
 *
 * Short paragraph describing the callback boundary used to receive immutable
 * event payloads during one search run.
 *
 * @par Invariants
 * - `on_event` may be empty when the caller does not want live callbacks.
 *
 * @par Performance Notes
 * - Callback delivery happens synchronously on the emitting run thread.
 * - Parallel runs may serialize callback delivery per run to preserve event
 *   ordering and retained-history coherence.
 */
struct SearchObserver {
  std::function<void(const SearchEvent &event)> on_event{};

  /**
   * @brief Reports whether a live observer callback is installed.
   *
   * Short paragraph describing the presence check used before event delivery.
   *
   * @return `true` when `on_event` contains a callable target.
   *
   * @pre None.
   * @post Does not modify the observer.
   * @par Determinism
   * - Deterministic for a fixed observer state.
   */
  [[nodiscard]] auto installed() const -> bool {
    return static_cast<bool>(on_event);
  }
};

/**
 * @brief Groups run-scoped observer and execution-control surfaces.
 *
 * Short paragraph describing the live callback, retained-history, timeout, and
 * cancellation controls attached to one explicit search request.
 *
 * @par Invariants
 * - Empty callbacks leave execution behavior unchanged aside from retained
 *   result history.
 *
 * @par Performance Notes
 * - Cancellation checks occur only at safe boundaries between search steps
 *   and worker-task checkpoints.
 * - Observer delivery may synchronize per run without serializing unrelated
 *   solver runs.
 */
struct SearchExecutionConfig {
  runtime::ExecutionControlConfig control{};
  SearchObserver observer{};
  std::function<bool()> cancellation_requested{};
};

} // namespace shiny::nfp::search