#pragma once

#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "placement/config.hpp"

namespace shiny::nesting::place {

/**
 * @brief Selects the ranking heuristic used for feasible placement candidates.
 */
enum class PlacementPolicy : std::uint32_t {
  bottom_left = 0,
  minimum_length = 1,
  maximum_utilization = 2,
};

/**
 * @brief Describes where a placement candidate originated.
 */
enum class PlacementCandidateSource : std::int8_t {
  constructive_boundary = 0,
  bin_boundary = 1,
  hole_boundary = 2,
  perfect_fit = 3,
  perfect_slide = 4,
  concave_boundary = 5,
};

/**
 * @brief Selected pose for placing one piece into one bin.
 */
struct Placement {
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  geom::RotationIndex rotation_index{};
  geom::Point2 translation{};
  bool mirrored{false};
};

/**
 * @brief Full geometry request for one placement query.
 *
 * @par Invariants
 * - All revision fields must correspond to the supplied geometry payloads.
 *
 * @par Performance Notes
 * - Revision fields are used to build stable cache keys for ranked results.
 */
struct PlacementRequest {
  std::uint32_t bin_id{0};
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles piece{};
  geom::PolygonWithHoles bin{};
  geom::PolygonWithHoles merged_region{};
  std::vector<geom::PolygonWithHoles> holes{};
  geom::RotationIndex rotation_index{};
  PartGrainCompatibility part_grain_compatibility{
      PartGrainCompatibility::unrestricted};
  PlacementStartCorner start_corner{PlacementStartCorner::bottom_left};
  std::uint64_t piece_geometry_revision{0};
  std::uint64_t bin_geometry_revision{0};
  std::uint64_t merged_region_revision{0};
  std::uint64_t hole_set_revision{0};
  PlacementConfig config{};
};

/**
 * @brief One feasible placement candidate before final selection.
 */
struct PlacementCandidate {
  geom::Point2 translation{};
  geom::RotationIndex rotation_index{};
  double score{0.0};
  PlacementCandidateSource source{
      PlacementCandidateSource::constructive_boundary};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
};

/**
 * @brief Candidate list sorted according to a placement policy.
 *
 * @par Invariants
 * - `candidates.front()` is the best candidate whenever the list is non-empty.
 *
 * @par Performance Notes
 * - Exposes `best()` so callers can avoid repeating emptiness checks.
 */
struct RankedPlacementSet {
  std::vector<PlacementCandidate> candidates{};
  PlacementPolicy policy{PlacementPolicy::bottom_left};

  /**
   * @brief Returns the best-ranked candidate when one exists.
   *
   * @return Pointer to the first candidate, or `nullptr` when the set is empty.
   */
  [[nodiscard]] auto best() const -> const PlacementCandidate * {
    return candidates.empty() ? nullptr : &candidates.front();
  }
};

} // namespace shiny::nesting::place
