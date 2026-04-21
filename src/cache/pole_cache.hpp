#pragma once

#include <cstddef>
#include <cstdint>

#include "cache/cache_policy.hpp"
#include "cache/lru_cache.hpp"
#include "geometry/types.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::pack {

struct PoleOfInaccessibility {
  geom::Point2 center{};
  double radius{0.0};
};

} // namespace shiny::nesting::pack

namespace shiny::nesting::cache {

struct PoleCacheKey {
  std::uint64_t polygon_revision{0};
  std::int64_t epsilon_quantized{0};

  auto operator==(const PoleCacheKey &) const -> bool = default;
};

struct PoleCacheKeyHash {
  [[nodiscard]] auto operator()(const PoleCacheKey &key) const noexcept
      -> std::size_t {
    return runtime::hash::combine_hashes(key.polygon_revision,
                                         key.epsilon_quantized);
  }
};

using PoleCache =
    LruCache<PoleCacheKey, pack::PoleOfInaccessibility, PoleCacheKeyHash>;

[[nodiscard]] inline auto default_pole_cache_config() -> CacheStoreConfig {
  return CacheStoreConfig{.policy = CachePolicy::lru_bounded,
                          .max_entries = 50'000U};
}

[[nodiscard]] auto make_pole_cache_key(std::uint64_t polygon_revision,
                                       double epsilon) -> PoleCacheKey;

} // namespace shiny::nesting::cache
