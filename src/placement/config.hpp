#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "geometry/transform.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::place {

/**
 * @brief Describes the bed grain axis for one solve run.
 *
 * This explicit value keeps grain semantics separate from ordinary rotation
 * filtering while still working with the repository's discrete rotation sets.
 *
 * @par Invariants
 * - `unrestricted` means grain does not constrain placement admissibility.
 * - `along_x` and `along_y` describe an undirected bed axis.
 *
 * @par Performance Notes
 * - Copied directly into placement and packing config surfaces.
 */
enum class BedGrainDirection : std::int8_t {
  unrestricted = 0,
  along_x = 1,
  along_y = 2,
};

/**
 * @brief Describes how one part's grain must relate to the bed grain axis.
 *
 * This explicit solve-run input participates in admissibility and cache
 * identity rather than being inferred from allowed rotations alone.
 *
 * @par Invariants
 * - `unrestricted` means the part has no grain-specific admissibility rule.
 * - Parallel and perpendicular rules are evaluated against the resolved bed
 *   grain axis.
 *
 * @par Performance Notes
 * - Stored on piece and placement request surfaces by value.
 */
enum class PartGrainCompatibility : std::int8_t {
  unrestricted = 0,
  parallel_to_bed = 1,
  perpendicular_to_bed = 2,
};

/**
 * @brief Selects which bed corner candidate ranking should treat as the start.
 */
enum class PlacementStartCorner : std::int8_t {
  bottom_left = 0,
  bottom_right = 1,
  top_left = 2,
  top_right = 3,
};

/**
 * @brief One explicit forbidden bed region for clamps or tooling.
 *
 * This type keeps manufacturing keep-outs semantically distinct from ordinary
 * cavities and holes even though both are ultimately polygonal geometry.
 *
 * @par Invariants
 * - `region.outer` must describe a finite polygon with at least three points.
 * - Exclusion zones do not create usable cavities for part-in-part placement.
 *
 * @par Performance Notes
 * - Stored directly on the solve-run config so legality and cache identity see
 *   the same value surface.
 */
struct BedExclusionZone {
  std::uint32_t zone_id{0};
  std::optional<std::uint32_t> bin_id{};
  geom::Polygon region{};

  /**
   * @brief Reports whether the keep-out region is structurally usable.
   *
   * @return `true` when the region contains a finite outer polygon.
   *
   * @pre None.
   * @post Does not modify the exclusion zone.
   * @par Determinism
   * - Deterministic for a fixed zone state.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Configures candidate generation and ranking for placement queries.
 *
 * @par Invariants
 * - `allowed_rotations` defines the only rotation indices that may be resolved.
 * - Exclusion zones remain explicit keep-outs rather than ordinary cavities.
 *
 * @par Performance Notes
 * - Shared directly by placement and packing requests.
 */
struct PlacementConfig {
  double part_clearance{0.0};
  geom::DiscreteRotationSet allowed_rotations{{0.0, 90.0, 180.0, 270.0}};
  BedGrainDirection bed_grain_direction{BedGrainDirection::unrestricted};
  bool enable_part_in_part_placement{false};
  bool explore_concave_candidates{false};
  std::vector<BedExclusionZone> exclusion_zones{};

  /**
   * @brief Reports whether the placement config is internally consistent.
   *
   * @return `true` when candidate generation can safely use this config.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Resolves a discrete rotation index to its configured angle.
 *
 * @param rotation_index Index into `config.allowed_rotations`.
 * @param config Placement configuration supplying the discrete rotation set.
 * @return Resolved angle when the index is valid, otherwise `std::nullopt`.
 */
[[nodiscard]] auto resolve_rotation(geom::RotationIndex rotation_index,
                                    const PlacementConfig &config)
    -> std::optional<geom::ResolvedRotation>;

/**
 * @brief Reports whether a rotation index is allowed by the config.
 *
 * @param rotation_index Index to test.
 * @param config Placement configuration supplying the discrete rotation set.
 * @return `true` when the rotation index can be resolved.
 */
[[nodiscard]] auto rotation_is_allowed(geom::RotationIndex rotation_index,
                                       const PlacementConfig &config) -> bool;

[[nodiscard]] auto exclusion_zone_applies_to_bin(const BedExclusionZone &zone,
                                                 std::uint32_t bin_id) -> bool;

/**
 * @brief Reports whether one resolved rotation satisfies a part grain rule.
 *
 * Short paragraph describing the admissibility rule that compares one part's
 * resolved grain axis against the configured bed grain direction.
 *
 * @param rotation Resolved rotation for the candidate pose.
 * @param bed_grain_direction Bed grain axis for the current solve run.
 * @param compatibility Part-specific grain rule.
 * @return `true` when the resolved rotation satisfies the requested grain
 *   relation.
 *
 * @pre `rotation` should come from a valid placement rotation index.
 * @post Does not modify any input.
 * @par Determinism
 * - Deterministic for fixed rotation and grain inputs.
 */
[[nodiscard]] auto grain_compatibility_allows_rotation(
    geom::ResolvedRotation rotation, BedGrainDirection bed_grain_direction,
    PartGrainCompatibility compatibility) -> bool;

} // namespace shiny::nesting::place
