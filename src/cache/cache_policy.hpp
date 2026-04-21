#pragma once

#include <cstddef>
#include <cstdint>

namespace shiny::nesting::cache {

enum class CachePolicy : std::uint8_t {
  // Bounded LRU eviction. The default. Production runs and tests
  // should always pick this; `max_entries` upper-bounds the resident
  // set size and prevents the cache from growing without bound during
  // a long metaheuristic search where many distinct
  // (geometry_revision, rotation) keys are visited.
  lru_bounded = 1,
  // Disable eviction entirely. ONLY for short-lived, bounded-key tests
  // where pressure cannot grow. Footgun for production use — a long
  // metaheuristic search will balloon memory. Sites that select this
  // should justify it inline.
  unbounded = 0,
};

struct CacheStoreConfig {
  CachePolicy policy{CachePolicy::lru_bounded};
  std::size_t max_entries{1024};
};

} // namespace shiny::nesting::cache
