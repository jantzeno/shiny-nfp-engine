#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "packing/layout.hpp"

namespace shiny::nesting {

enum class StopReason : std::uint8_t {
  none = 0,
  completed = 1,
  cancelled = 2,
  iteration_limit_reached = 3,
  time_limit_reached = 4,
  invalid_request = 5,
};

enum class ProgressPhase : std::uint8_t {
  none = 0,
  part_placement = 1,   // constructive pass or BRKGA chromosome evaluation
  part_refinement = 2,  // SA/ALNS/GDRR/LAHC move step or BRKGA polishing pass
  completed = 3,
};

struct BudgetState {
  bool iteration_limit_enabled{false};
  std::size_t iteration_limit{0};
  std::size_t iterations_completed{0};
  bool time_limit_enabled{false};
  std::uint64_t time_limit_milliseconds{0};
  std::uint64_t elapsed_milliseconds{0};
  bool cancellation_requested{false};
};

struct ProgressSnapshot {
  std::size_t sequence{0};
  std::size_t placed_parts{0};
  std::size_t total_parts{0};
  pack::Layout layout{};
  BudgetState budget{};
  StopReason stop_reason{StopReason::none};
  ProgressPhase phase{ProgressPhase::none};
  std::string phase_detail{};
  double utilization_percent{0.0};
  bool improved{false};
};

using ProgressObserver = std::function<void(const ProgressSnapshot &)>;

class ProgressThrottle {
public:
  explicit ProgressThrottle(
      std::uint64_t min_interval_ms = kDefaultIntervalMs)
      : min_interval_ms_{min_interval_ms} {}

  [[nodiscard]] auto should_emit() -> bool {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_emit_time_)
            .count();
    if (static_cast<std::uint64_t>(elapsed_ms) >= min_interval_ms_) {
      last_emit_time_ = now;
      return true;
    }
    return false;
  }

  void reset() { last_emit_time_ = std::chrono::steady_clock::now(); }

  static constexpr std::uint64_t kDefaultIntervalMs = 100;

private:
  std::uint64_t min_interval_ms_;
  std::chrono::steady_clock::time_point last_emit_time_{};
};

} // namespace shiny::nesting
