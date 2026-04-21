#pragma once

#include <cstddef>
#include <cstdint>

#include "cache/cache_policy.hpp"
#include "cache/lru_cache.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::cache {

struct CollisionPairLossCacheKey {
  std::uint64_t lhs_polygon_revision{0};
  std::uint64_t rhs_polygon_revision{0};

  auto operator==(const CollisionPairLossCacheKey &) const -> bool = default;
};

struct CollisionPairLossCacheValue {
  double loss{0.0};
  double exact{0.0};
};

struct CollisionPairLossCacheStats {
  std::uint64_t hits{0};
  std::uint64_t misses{0};
};

struct CollisionPairLossCacheKeyHash {
  [[nodiscard]] auto operator()(const CollisionPairLossCacheKey &key) const noexcept
      -> std::size_t {
    return runtime::hash::combine_hashes(key.lhs_polygon_revision,
                                         key.rhs_polygon_revision);
  }
};

using CollisionPairLossCache =
    LruCache<CollisionPairLossCacheKey, CollisionPairLossCacheValue,
             CollisionPairLossCacheKeyHash>;

[[nodiscard]] inline auto default_collision_pair_loss_cache_config()
    -> CacheStoreConfig {
  return CacheStoreConfig{.policy = CachePolicy::lru_bounded,
                          .max_entries = 8192U};
}

[[nodiscard]] inline auto make_collision_pair_loss_cache_key(
    const std::uint64_t lhs_polygon_revision,
    const std::uint64_t rhs_polygon_revision) -> CollisionPairLossCacheKey {
  if (lhs_polygon_revision <= rhs_polygon_revision) {
    return CollisionPairLossCacheKey{
        .lhs_polygon_revision = lhs_polygon_revision,
        .rhs_polygon_revision = rhs_polygon_revision,
    };
  }
  return CollisionPairLossCacheKey{
      .lhs_polygon_revision = rhs_polygon_revision,
      .rhs_polygon_revision = lhs_polygon_revision,
  };
}

} // namespace shiny::nesting::cache
