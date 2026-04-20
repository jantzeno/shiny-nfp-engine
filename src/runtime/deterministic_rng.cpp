#include "runtime/deterministic_rng.hpp"

namespace shiny::nesting::runtime {

DeterministicRng::DeterministicRng(const std::uint64_t seed)
    : seed_(seed), generator_(seed) {}

auto DeterministicRng::seed() const -> std::uint64_t { return seed_; }

auto DeterministicRng::next_u64() -> std::uint64_t { return generator_(); }

auto DeterministicRng::uniform_index(const std::size_t upper_bound)
    -> std::size_t {
  if (upper_bound == 0U) {
    return 0U;
  }
  std::uniform_int_distribution<std::size_t> distribution(0U, upper_bound - 1U);
  return distribution(generator_);
}

auto DeterministicRng::uniform_real() -> double {
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(generator_);
}

auto DeterministicRng::bernoulli(const double probability) -> bool {
  std::bernoulli_distribution distribution(probability);
  return distribution(generator_);
}

} // namespace shiny::nesting::runtime
