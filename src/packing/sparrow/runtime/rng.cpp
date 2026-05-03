#include "packing/sparrow/runtime/rng.hpp"

#include <limits>

namespace shiny::nesting::pack::sparrow::runtime {

namespace {

[[nodiscard]] auto mix(std::uint64_t value) -> std::uint64_t {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

} // namespace

auto SplitMix64Rng::next_u64() -> std::uint64_t {
  state_ = mix(state_);
  return state_;
}

auto SplitMix64Rng::uniform_index(const std::size_t upper_bound)
    -> std::size_t {
  if (upper_bound == 0U) {
    return 0U;
  }
  return static_cast<std::size_t>(next_u64() % upper_bound);
}

auto SplitMix64Rng::uniform_real() -> double {
  constexpr double kDenominator =
      static_cast<double>(std::numeric_limits<std::uint64_t>::max());
  return static_cast<double>(next_u64()) / kDenominator;
}

auto derive_worker_seed(const std::uint64_t worker_seed_base,
                        const std::size_t worker_index) -> std::uint64_t {
  return mix(worker_seed_base + static_cast<std::uint64_t>(worker_index));
}

} // namespace shiny::nesting::pack::sparrow::runtime