#include "runtime/timing.hpp"

namespace shiny::nesting::runtime {

Stopwatch::Stopwatch() : start_(std::chrono::steady_clock::now()) {}

void Stopwatch::reset() { start_ = std::chrono::steady_clock::now(); }

auto Stopwatch::elapsed_milliseconds() const -> std::uint64_t {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_)
          .count());
}

TimeBudget::TimeBudget(const std::uint64_t limit_milliseconds)
    : limit_milliseconds_(limit_milliseconds) {}

auto TimeBudget::limit_milliseconds() const -> std::uint64_t {
  return limit_milliseconds_;
}

auto TimeBudget::enabled() const -> bool { return limit_milliseconds_ > 0U; }

auto TimeBudget::expired(const Stopwatch &stopwatch) const -> bool {
  return enabled() && stopwatch.elapsed_milliseconds() >= limit_milliseconds_;
}

auto TimeBudget::remaining_milliseconds(const Stopwatch &stopwatch) const
    -> std::uint64_t {
  if (!enabled()) {
    return 0U;
  }
  const auto elapsed = stopwatch.elapsed_milliseconds();
  return elapsed >= limit_milliseconds_ ? 0U : limit_milliseconds_ - elapsed;
}

} // namespace shiny::nesting::runtime
