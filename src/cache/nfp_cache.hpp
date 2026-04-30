#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cache/cache_policy.hpp"
#include "cache/lru_cache.hpp"
#include "geometry/types.hpp"
#include "runtime/hash.hpp"
#include "util/status.hpp"

// Per-pair NFP result cache (Plan §2.5 + §11.3).
//
// Cache key is (fixed_revision, moving_revision, fixed_rotation,
// moving_rotation). Revisions are stable identifiers for the
// underlying piece geometry — any geometry change must bump the
// revision so stale entries become unreachable rather than served.
// Rotations are quantised to milli-degrees so float comparisons
// round consistently; callers that vary rotation continuously will
// see cache MISS for the non-quantised difference, which is the
// desired behaviour.
namespace shiny::nesting::cache {

enum class NfpCacheEntryKind : std::uint8_t {
  exact = 0,
  conservative_bbox_fallback = 1,
};

enum class NfpCacheAccuracy : std::uint8_t {
  exact = 0,
  conservative_bbox_fallback = 1,
};

struct NfpCacheKey {
  std::uint64_t fixed_revision{0};
  std::uint64_t moving_revision{0};
  std::int64_t fixed_rotation_millidegrees{0};
  std::int64_t moving_rotation_millidegrees{0};
  NfpCacheEntryKind entry_kind{NfpCacheEntryKind::exact};

  auto operator==(const NfpCacheKey &) const -> bool = default;
};

struct NfpCacheKeyHash {
  [[nodiscard]] auto operator()(const NfpCacheKey &key) const noexcept
      -> std::size_t {
    return runtime::hash::combine_hashes(
        key.fixed_revision, key.moving_revision,
        key.fixed_rotation_millidegrees, key.moving_rotation_millidegrees,
        static_cast<std::uint8_t>(key.entry_kind));
  }
};

struct NfpCacheValue {
  std::vector<geom::PolygonWithHoles> polygons{};
  NfpCacheAccuracy accuracy{NfpCacheAccuracy::exact};
  util::Status status{util::Status::ok};
};

using NfpCache = LruCache<NfpCacheKey, NfpCacheValue, NfpCacheKeyHash>;

[[nodiscard]] inline auto default_nfp_cache_config() -> CacheStoreConfig {
  return CacheStoreConfig{.policy = CachePolicy::lru_bounded,
                          .max_entries = 50'000U};
}

[[nodiscard]] auto make_nfp_cache_key(
    std::uint64_t fixed_revision, std::uint64_t moving_revision,
    double fixed_rotation_degrees, double moving_rotation_degrees,
    NfpCacheEntryKind entry_kind = NfpCacheEntryKind::exact) -> NfpCacheKey;

} // namespace shiny::nesting::cache
