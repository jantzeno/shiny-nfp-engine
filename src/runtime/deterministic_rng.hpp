#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace shiny::nesting::runtime {

class DeterministicRng {
public:
  explicit DeterministicRng(std::uint64_t seed = 0);

  [[nodiscard]] auto seed() const -> std::uint64_t;

  [[nodiscard]] auto next_u64() -> std::uint64_t;

  [[nodiscard]] auto uniform_index(std::size_t upper_bound) -> std::size_t;

  [[nodiscard]] auto uniform_real() -> double;

  [[nodiscard]] auto bernoulli(double probability) -> bool;

  template <typename T> void shuffle(std::vector<T> &values) {
    std::shuffle(values.begin(), values.end(), generator_);
  }

private:
  std::uint64_t seed_{0};
  std::mt19937_64 generator_{};
};

} // namespace shiny::nesting::runtime
