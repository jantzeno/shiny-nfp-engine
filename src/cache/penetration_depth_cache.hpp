#pragma once

#include <cstddef>
#include <cstdint>

#include "cache/cache_policy.hpp"
#include "cache/lru_cache.hpp"
#include "geometry/types.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::cache {

struct PenetrationDepthCacheKey {
  std::uint64_t nfp_revision{0};
  std::int64_t x_quantized{0};
  std::int64_t y_quantized{0};

  auto operator==(const PenetrationDepthCacheKey &) const -> bool = default;
};

struct PenetrationDepthCacheKeyHash {
  [[nodiscard]] auto
  operator()(const PenetrationDepthCacheKey &key) const noexcept
      -> std::size_t {
    return runtime::hash::combine_hashes(key.nfp_revision, key.x_quantized,
                                         key.y_quantized);
  }
};

using PenetrationDepthCache =
    LruCache<PenetrationDepthCacheKey, double, PenetrationDepthCacheKeyHash>;

[[nodiscard]] inline auto default_pd_cache_config() -> CacheStoreConfig {
  return CacheStoreConfig{.policy = CachePolicy::lru_bounded,
                          .max_entries = 200'000U};
}

[[nodiscard]] auto make_penetration_depth_cache_key(std::uint64_t nfp_revision,
                                                    const geom::Point2 &point)
    -> PenetrationDepthCacheKey;

} // namespace shiny::nesting::cache
