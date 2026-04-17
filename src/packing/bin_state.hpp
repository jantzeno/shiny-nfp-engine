#pragma once

#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "packing/layout.hpp"
#include "placement/config.hpp"
#include "polygon_ops/merge_region.hpp"

namespace shiny::nfp::pack {

/**
 * @brief Mutable per-bin state used during constructive decoding.
 *
 * Stores the container, current occupied region, discovered holes, and placed
 * parts while the decoder incrementally builds a layout.
 *
 * @par Invariants
 * - Revisions track when container, occupied region, or hole state changes.
 *
 * @par Performance Notes
 * - Caches merged occupancy and holes to avoid recomputing them for every piece
 *   query.
 */
struct BinState {
  std::uint32_t bin_id{0};
  geom::PolygonWithHoles container{};
  std::uint64_t container_geometry_revision{0};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
  poly::MergedRegion occupied{};
  std::uint64_t occupied_region_revision{0};
  std::vector<geom::PolygonWithHoles> holes{};
  std::uint64_t hole_set_revision{0};
  std::vector<PlacedPiece> placements{};
  BinUtilizationSummary utilization{};
};

/**
 * @brief Computes utilization metrics for the current bin state.
 *
 * @param state Bin state to summarize.
 * @return Utilization summary derived from the occupied region and placements.
 */
[[nodiscard]] auto summarize_bin(const BinState &state)
    -> BinUtilizationSummary;

} // namespace shiny::nfp::pack
