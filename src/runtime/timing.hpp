#pragma once

#include <chrono>
#include <cstdint>

namespace shiny::nesting::runtime {

class Stopwatch {
public:
  Stopwatch();

  void reset();

  [[nodiscard]] auto elapsed_milliseconds() const -> std::uint64_t;

private:
  std::chrono::steady_clock::time_point start_{};
};

class TimeBudget {
public:
  explicit TimeBudget(std::uint64_t limit_milliseconds = 0);

  [[nodiscard]] auto limit_milliseconds() const -> std::uint64_t;

  [[nodiscard]] auto enabled() const -> bool;

  [[nodiscard]] auto expired(const Stopwatch &stopwatch) const -> bool;

  [[nodiscard]] auto remaining_milliseconds(const Stopwatch &stopwatch) const
      -> std::uint64_t;

private:
  std::uint64_t limit_milliseconds_{0};
};

} // namespace shiny::nesting::runtime
