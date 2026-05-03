#pragma once

#include <cstddef>
#include <cstdint>

namespace shiny::nesting::pack::sparrow::runtime {

class SplitMix64Rng {
public:
  explicit SplitMix64Rng(std::uint64_t seed = 0) : state_(seed) {}

  [[nodiscard]] auto seed() const -> std::uint64_t { return state_; }

  [[nodiscard]] auto next_u64() -> std::uint64_t;

  [[nodiscard]] auto uniform_index(std::size_t upper_bound) -> std::size_t;

  [[nodiscard]] auto uniform_real() -> double;

private:
  std::uint64_t state_{0};
};

[[nodiscard]] auto derive_worker_seed(std::uint64_t worker_seed_base,
                                      std::size_t worker_index)
    -> std::uint64_t;

} // namespace shiny::nesting::pack::sparrow::runtime