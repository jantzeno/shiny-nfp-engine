#pragma once

#include <compare>
#include <cstdint>

namespace shiny::nfp::cache {

/**
 * @brief Monotonic geometry version used to invalidate cached results.
 *
 * Distinguishes shape revisions without coupling cache keys to the full polygon
 * payload.
 *
 * @par Invariants
 * - Higher values represent newer geometry revisions for the same asset.
 *
 * @par Performance Notes
 * - Kept as a compact scalar so revision checks remain cheap in cache keys.
 */
struct GeometryRevision {
  std::uint64_t value{0};

  auto operator<=>(const GeometryRevision &) const = default;
};

/**
 * @brief Algorithm implementation version used in cache keys.
 *
 * Allows cached NFP results to be invalidated when extraction logic or
 * tolerance-sensitive behavior changes.
 *
 * @par Invariants
 * - A change in semantics should increment the stored value.
 *
 * @par Performance Notes
 * - Small scalar wrapper intended for cheap comparison and hashing.
 */
struct AlgorithmRevision {
  std::uint32_t value{0};

  auto operator<=>(const AlgorithmRevision &) const = default;
};

} // namespace shiny::nfp::cache