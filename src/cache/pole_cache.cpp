#include "cache/pole_cache.hpp"

#include <algorithm>
#include <cmath>

namespace shiny::nesting::cache {
namespace {

// Quantises the requested `epsilon` into an integer key so the LRU
// cache can detect "same polygon, same tolerance" lookups. The scale
// (1e6) caps key resolution at ~1e-6: any two callers requesting
// epsilons closer than that to one another COALESCE into the same
// cache entry. This is intentional — the PoI search itself is bounded
// below by `effective_epsilon = max(epsilon, 1e-9)` in
// overlap_proxy.cpp, so finer epsilons would not produce materially
// different results yet would defeat reuse across nearby callers.
//
// TODO(OCP): if a caller ever needs configurable quantisation
// resolution, expose this scale via a `PoleCacheConfig` struct and
// thread it through `make_pole_cache_key`. Holding off today because
// the only caller pair (overlap_proxy compute/put) shares the
// constant trivially via this file and adding a config struct would
// touch every cache-key construction site for no behavioural gain.
constexpr double kEpsilonQuantizationScale = 1'000'000.0;

} // namespace

auto make_pole_cache_key(const std::uint64_t polygon_revision,
                         const double epsilon) -> PoleCacheKey {
  return {
      .polygon_revision = polygon_revision,
      .epsilon_quantized = static_cast<std::int64_t>(
          std::llround(std::max(0.0, epsilon) * kEpsilonQuantizationScale)),
  };
}

} // namespace shiny::nesting::cache
