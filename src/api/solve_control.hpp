#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>

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
  bool emit_empty_transitions{false};
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

/// Derive a 64-bit seed from an mt19937 engine by consuming two draws.
/// Two calls with the same RNG state always produce the same seed, giving
/// fully deterministic results when seeded identically.
[[nodiscard]] inline auto seed_from_rng(std::mt19937 &rng) -> std::uint64_t {
  return (static_cast<std::uint64_t>(rng()) << 32U) | rng();
}

/// Return the next seed in the increment progression.
[[nodiscard]] constexpr auto increment_seed(std::uint64_t seed)
    -> std::uint64_t {
  return seed + 1U;
}

/// Return the previous seed in the decrement progression.
[[nodiscard]] constexpr auto decrement_seed(std::uint64_t seed)
    -> std::uint64_t {
  return seed - 1U;
}

} // namespace shiny::nesting