#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "observer.hpp"
#include "runtime/cancellation.hpp"
#include "runtime/progress.hpp"

namespace shiny::nesting {

namespace pack {
struct PackerWorkspace;
}

enum class SeedProgressionMode : std::uint8_t {
  increment = 0,
  decrement = 1,
  random = 2,
};

struct SolveControl {
  ProgressObserver on_progress{};
  runtime::CancellationToken cancellation{};
  std::size_t operation_limit{0};
  std::uint64_t time_limit_milliseconds{0};
  std::uint64_t random_seed{0};
  bool cancellation_requested{false};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
  pack::PackerWorkspace *workspace{nullptr};
};

using ProfileProgressObserver =
    std::function<void(const ProfileProgressSnapshot &)>;

struct ProfileSolveControl {
  ProfileProgressObserver on_progress{};
  runtime::CancellationToken cancellation{};
  std::size_t operation_limit{0};
  std::uint64_t random_seed{0};
  bool cancellation_requested{false};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
  pack::PackerWorkspace *workspace{nullptr};
};

} // namespace shiny::nesting