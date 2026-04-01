#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <utility>

#include "search/execution.hpp"
#include "search/result.hpp"

namespace shiny::nfp::search::detail {

class RunInterruptionGate {
public:
  using ElapsedMsCallback = std::function<std::uint64_t()>;

  RunInterruptionGate(const SearchExecutionConfig &execution,
                      ElapsedMsCallback elapsed_ms)
      : execution_(execution), elapsed_ms_(std::move(elapsed_ms)) {}

  [[nodiscard]] auto poll() -> bool {
    if (interrupted()) {
      return true;
    }

    if (poll_time_budget()) {
      return true;
    }

    if (!execution_.control.allow_cancellation ||
        !execution_.cancellation_requested) {
      return false;
    }

    std::scoped_lock lock(cancellation_mutex_);
    if (interrupted()) {
      return true;
    }
    if (poll_time_budget()) {
      return true;
    }
    if (execution_.cancellation_requested()) {
      set_state(InterruptionState::cancelled);
      return true;
    }

    return false;
  }

  [[nodiscard]] auto interrupted() const -> bool {
    return state_.load(std::memory_order_acquire) != InterruptionState::none;
  }

  [[nodiscard]] auto status() const -> SearchRunStatus {
    switch (state_.load(std::memory_order_acquire)) {
    case InterruptionState::none:
      return SearchRunStatus::completed;
    case InterruptionState::timed_out:
      return SearchRunStatus::timed_out;
    case InterruptionState::cancelled:
      return SearchRunStatus::cancelled;
    }

    return SearchRunStatus::completed;
  }

private:
  enum class InterruptionState : std::uint8_t {
    none = 0,
    timed_out = 1,
    cancelled = 2,
  };

  [[nodiscard]] auto poll_time_budget() -> bool {
    if (execution_.control.time_budget_ms == 0U) {
      return false;
    }
    if (elapsed_ms_() < execution_.control.time_budget_ms) {
      return false;
    }

    set_state(InterruptionState::timed_out);
    return true;
  }

  auto set_state(InterruptionState state) -> void {
    auto expected = InterruptionState::none;
    state_.compare_exchange_strong(expected, state, std::memory_order_acq_rel,
                                   std::memory_order_acquire);
  }

  const SearchExecutionConfig &execution_;
  ElapsedMsCallback elapsed_ms_;
  std::atomic<InterruptionState> state_{InterruptionState::none};
  std::mutex cancellation_mutex_{};
};

} // namespace shiny::nfp::search::detail