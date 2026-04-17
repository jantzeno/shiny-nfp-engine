#include "placement/config.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kGrainAngleEpsilon = 1e-9;

[[nodiscard]] auto angle_is_axis_aligned(double angle, double axis) -> bool {
  return std::fabs(angle - axis) <= kGrainAngleEpsilon;
}

[[nodiscard]] auto normalize_half_turn(double degrees) -> double {
  auto normalized = std::fmod(degrees, 180.0);
  if (normalized < 0.0) {
    normalized += 180.0;
  }
  if (std::fabs(normalized - 180.0) <= kGrainAngleEpsilon) {
    return 0.0;
  }
  return normalized;
}

[[nodiscard]] auto point_is_finite(const shiny::nfp::geom::Point2 &point)
    -> bool {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

} // namespace

namespace shiny::nfp::place {

auto BedExclusionZone::is_valid() const -> bool {
  if (region.outer.size() < 3U) {
    return false;
  }

  return std::all_of(region.outer.begin(), region.outer.end(), point_is_finite);
}

auto PlacementConfig::is_valid() const -> bool {
  if (part_clearance < 0.0 || !std::isfinite(part_clearance)) {
    return false;
  }

  switch (bed_grain_direction) {
  case BedGrainDirection::unrestricted:
  case BedGrainDirection::along_x:
  case BedGrainDirection::along_y:
    break;
  }

  const auto &angles = allowed_rotations.angles_degrees;
  if (angles.empty() || angles.size() > 360U) {
    return false;
  }

  auto sorted = angles;
  std::sort(sorted.begin(), sorted.end());

  for (const auto angle : sorted) {
    if (!std::isfinite(angle) || angle < 0.0 || angle >= 360.0) {
      return false;
    }
  }

  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
    return false;
  }

  return std::all_of(exclusion_zones.begin(), exclusion_zones.end(),
                     [](const auto &zone) { return zone.is_valid(); });
}

auto resolve_rotation(geom::RotationIndex rotation_index,
                      const PlacementConfig &config)
    -> std::optional<geom::ResolvedRotation> {
  if (!config.is_valid()) {
    return std::nullopt;
  }

  const auto &angles = config.allowed_rotations.angles_degrees;
  if (rotation_index.value >= angles.size()) {
    return std::nullopt;
  }

  return geom::ResolvedRotation{.degrees = angles[rotation_index.value]};
}

auto rotation_is_allowed(geom::RotationIndex rotation_index,
                         const PlacementConfig &config) -> bool {
  return resolve_rotation(rotation_index, config).has_value();
}

auto exclusion_zone_applies_to_bin(const BedExclusionZone &zone,
                                   const std::uint32_t bin_id) -> bool {
  return !zone.bin_id.has_value() || *zone.bin_id == bin_id;
}

auto grain_compatibility_allows_rotation(geom::ResolvedRotation rotation,
                                         BedGrainDirection bed_grain_direction,
                                         PartGrainCompatibility compatibility)
    -> bool {
  switch (compatibility) {
  case PartGrainCompatibility::unrestricted:
    return true;
  case PartGrainCompatibility::parallel_to_bed:
  case PartGrainCompatibility::perpendicular_to_bed:
    break;
  }

  if (bed_grain_direction == BedGrainDirection::unrestricted) {
    return true;
  }

  const auto normalized = normalize_half_turn(rotation.degrees);
  const auto parallel_to_x = angle_is_axis_aligned(normalized, 0.0);
  const auto parallel_to_y = angle_is_axis_aligned(normalized, 90.0);

  bool parallel = false;
  bool perpendicular = false;
  switch (bed_grain_direction) {
  case BedGrainDirection::unrestricted:
    return true;
  case BedGrainDirection::along_x:
    parallel = parallel_to_x;
    perpendicular = parallel_to_y;
    break;
  case BedGrainDirection::along_y:
    parallel = parallel_to_y;
    perpendicular = parallel_to_x;
    break;
  }

  switch (compatibility) {
  case PartGrainCompatibility::unrestricted:
    return true;
  case PartGrainCompatibility::parallel_to_bed:
    return parallel;
  case PartGrainCompatibility::perpendicular_to_bed:
    return perpendicular;
  }

  return false;
}

} // namespace shiny::nfp::place
