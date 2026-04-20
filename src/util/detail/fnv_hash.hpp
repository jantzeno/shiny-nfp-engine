#pragma once

#include <cstdint>
#include <initializer_list>

namespace shiny::nesting::detail {

inline constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
inline constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

/**
 * @brief Mixes one value into an FNV-1a style 64-bit hash seed.
 */
[[nodiscard]] inline auto fnv_hash_mix(std::uint64_t seed, std::uint64_t value)
    -> std::uint64_t {
  seed ^= value;
  seed *= kFnvPrime;
  return seed;
}

/**
 * @brief Hashes a small sequence of 64-bit values with FNV-1a mixing.
 */
[[nodiscard]] inline auto
fnv_hash_values(std::initializer_list<std::uint64_t> values) -> std::uint64_t {
  auto seed = kFnvOffsetBasis;
  for (const auto value : values) {
    seed = fnv_hash_mix(seed, value);
  }
  return seed;
}

} // namespace shiny::nesting::detail