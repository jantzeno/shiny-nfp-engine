#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>

namespace shiny::nesting::runtime::hash {

inline constexpr std::uint64_t kFnv1aOffsetBasis = 0xcbf29ce484222325ULL;
inline constexpr std::uint64_t kFnv1aPrime = 0x100000001b3ULL;

// Golden-ratio reciprocal scaled to 64 bits (φ⁻¹ × 2⁶⁴). Used as a
// universal mixing/salting constant: it has good bit-distribution and
// no algebraic relationship to typical input distributions, so XOR'ing
// with it scrambles correlated values cheaply. Appears inside `combine`
// below and is exposed for sites that need a stand-alone salt.
inline constexpr std::uint64_t kGoldenRatio64 = 0x9e3779b97f4a7c15ULL;

// Boost-style hash combiner (golden-ratio splash + bit-shift mixer).
// Used to fold individual std::hash<T> outputs into a composite key
// hash. See
// <https://www.boost.org/doc/libs/1_84_0/libs/container_hash/doc/html/hash.html>.
[[nodiscard]] constexpr auto combine(const std::size_t seed,
                                     const std::size_t value) noexcept
    -> std::size_t {
  return seed ^ (value + kGoldenRatio64 + (seed << 6U) + (seed >> 2U));
}

// Variadic helper that hashes each argument with std::hash<T> and folds
// the result through `combine`. Convenient for composite key hashes.
template <typename... Args>
[[nodiscard]] auto combine_hashes(const Args &...args) noexcept -> std::size_t {
  std::size_t seed = 0;
  ((seed = combine(seed, std::hash<Args>{}(args))), ...);
  return seed;
}

// Streaming primitive: mix `bytes` into an existing FNV-1a hash. Use
// when building a hash incrementally over heterogeneous inputs (e.g.
// polygon ring coordinate lists where each ring contributes a length
// followed by N point pairs).
constexpr auto fnv1a_mix(std::uint64_t &hash,
                         const std::span<const std::byte> bytes) noexcept
    -> void {
  for (const auto byte : bytes) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= kFnv1aPrime;
  }
}

template <typename T>
auto fnv1a_mix_value(std::uint64_t &hash, const T &value) noexcept -> void {
  static_assert(std::is_trivially_copyable_v<T>,
                "fnv1a_mix_value requires trivially-copyable values");
  fnv1a_mix(hash, std::as_bytes(std::span<const T, 1>(&value, 1)));
}

// 64-bit FNV-1a over a byte range. Stable across runs and platforms;
// suitable for seeding RNGs from chromosome data or signing layout
// orderings.
[[nodiscard]] constexpr auto
fnv1a(const std::span<const std::byte> bytes) noexcept -> std::uint64_t {
  std::uint64_t hash = kFnv1aOffsetBasis;
  fnv1a_mix(hash, bytes);
  return hash;
}

[[nodiscard]] inline auto fnv1a(const std::string_view text) noexcept
    -> std::uint64_t {
  return fnv1a(std::as_bytes(std::span(text.data(), text.size())));
}

// Convenience overload for hashing a span of trivially-copyable values
// (e.g. uint32 piece-id sequences for layout signatures).
template <typename T>
[[nodiscard]] auto fnv1a_of(const std::span<const T> values) noexcept
    -> std::uint64_t {
  static_assert(std::is_trivially_copyable_v<T>,
                "fnv1a_of requires trivially-copyable values");
  return fnv1a(std::as_bytes(values));
}

} // namespace shiny::nesting::runtime::hash
