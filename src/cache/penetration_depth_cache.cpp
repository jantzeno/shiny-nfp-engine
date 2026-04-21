#include "cache/penetration_depth_cache.hpp"

#include <cmath>

namespace shiny::nesting::cache {
namespace {

constexpr double kQuantizationScale = 1'000.0;

[[nodiscard]] auto quantize(const double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * kQuantizationScale));
}

} // namespace

auto make_penetration_depth_cache_key(const std::uint64_t nfp_revision,
                                      const geom::Point2 &point)
    -> PenetrationDepthCacheKey {
  return {
      .nfp_revision = nfp_revision,
      .x_quantized = quantize(point.x),
      .y_quantized = quantize(point.y),
  };
}

} // namespace shiny::nesting::cache
