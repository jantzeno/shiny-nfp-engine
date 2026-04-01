#pragma once

#include <cstddef>

namespace shiny::nfp::cache {

/**
 * @brief Selects the eviction policy used by a cache store.
 *
 * The default policy keeps every inserted value, while the bounded mode evicts
 * least-recently-used entries once a configured capacity is reached.
 */
enum class CachePolicy {
  unbounded = 0,
  lru_bounded = 1,
};

/**
 * @brief Configures cache retention strategy for engine-local stores.
 *
 * This lightweight value is threaded through the geometry, NFP, placement, and
 * search engines so they can all share the same retention policy.
 *
 * @par Invariants
 * - `max_entries` is only meaningful when `policy` is `lru_bounded`.
 *
 * @par Performance Notes
 * - Defaults preserve the original unbounded cache behavior.
 */
struct CachePolicyConfig {
  CachePolicy policy{CachePolicy::unbounded};
  std::size_t max_entries{0};
};

} // namespace shiny::nfp::cache