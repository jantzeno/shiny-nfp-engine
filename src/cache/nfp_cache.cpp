#include "cache/nfp_cache.hpp"

#include <cmath>

#include "geometry/transform.hpp"

namespace shiny::nesting::cache {
namespace {

constexpr double kAngleScale = 1'000.0;

[[nodiscard]] auto quantize_angle(const double degrees) -> std::int64_t {
  return static_cast<std::int64_t>(
      std::llround(geom::normalize_angle_degrees(degrees) * kAngleScale));
}

} // namespace

auto make_nfp_cache_key(const std::uint64_t fixed_revision,
                        const std::uint64_t moving_revision,
                        const double fixed_rotation_degrees,
                        const double moving_rotation_degrees,
                        const NfpCacheEntryKind entry_kind) -> NfpCacheKey {
  return {
      .fixed_revision = fixed_revision,
      .moving_revision = moving_revision,
      .fixed_rotation_millidegrees = quantize_angle(fixed_rotation_degrees),
      .moving_rotation_millidegrees = quantize_angle(moving_rotation_degrees),
      .entry_kind = entry_kind,
  };
}

} // namespace shiny::nesting::cache
