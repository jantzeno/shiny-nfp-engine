#pragma once

#include <cstdint>
#include <vector>

#include "types.hpp"

namespace shiny::nfp::geom {

/**
 * @brief Index into a discrete rotation set.
 *
 * Separates cached rotation identity from the floating-point angle value used
 * to realize that rotation.
 *
 * @par Invariants
 * - `value` is expected to reference a valid entry in the active rotation set.
 *
 * @par Performance Notes
 * - Small wrapper used directly in cache keys and placement records.
 */
struct RotationIndex {
  std::uint16_t value{};

  auto operator<=>(const RotationIndex &) const = default;
};

/**
 * @brief Concrete resolved rotation angle in degrees.
 *
 * Stores the floating-point angle selected from a discrete rotation set after
 * configuration resolution.
 *
 * @par Invariants
 * - The value is interpreted in degrees.
 *
 * @par Performance Notes
 * - Kept separate from RotationIndex so caches can stay index-based.
 */
struct ResolvedRotation {
  double degrees{};

  auto operator<=>(const ResolvedRotation &) const = default;
};

/**
 * @brief Discrete rigid transform used by placement and packing.
 *
 * Couples one configured rotation with a translation vector.
 *
 * @par Invariants
 * - `rotation_index` and `translation` describe the same transformed pose.
 *
 * @par Performance Notes
 * - Aggregates only index and translation so transformed polygons can be
 *   materialized lazily.
 */
struct Transform2 {
  RotationIndex rotation_index{};
  Vector2 translation{};
};

/**
 * @brief Configured set of allowed piece rotations.
 *
 * Carries the discrete angle choices used by constructive decoding and search.
 *
 * @par Invariants
 * - `angles_degrees` is interpreted as an ordered discrete domain.
 *
 * @par Performance Notes
 * - Shared directly across placement, decoder, and search requests.
 */
struct DiscreteRotationSet {
  std::vector<double> angles_degrees{};
};

} // namespace shiny::nfp::geom