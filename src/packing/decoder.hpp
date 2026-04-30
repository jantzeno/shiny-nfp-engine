#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "geometry/transform.hpp"
#include "packing/bin_state.hpp"
#include "packing/config.hpp"

namespace shiny::nesting::pack {

/**
 * @brief One input piece supplied to the constructive decoder.
 *
 * @par Invariants
 * - `geometry_revision` must advance when `polygon` changes.
 *
 * @par Performance Notes
 * - The revision is copied directly into downstream cache keys.
 */
struct PieceInput {
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint64_t geometry_revision{0};
  bool allow_mirror{false};
  std::optional<geom::DiscreteRotationSet> allowed_rotations{};
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
  std::vector<std::uint32_t> allowed_bin_ids{};
  bool restricted_to_allowed_bins{false};
};

/**
 * @brief Explicit geometry for one bin available to the decoder.
 *
 * @par Invariants
 * - `bin_id` must be unique within one request.
 *
 * @par Performance Notes
 * - The decoder opens bins lazily in request order, but never synthesizes
 *   geometry beyond what is listed here.
 */
struct BinInput {
  std::uint32_t bin_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint64_t geometry_revision{0};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
};

/**
 * @brief Complete input bundle for one constructive decode run.
 *
 * @par Invariants
 * - Piece order is semantically significant and is consumed as provided.
 *
 * @par Performance Notes
 * - Search reuses this type directly when reevaluating piece orders.
 */
struct DecoderRequest {
  std::vector<BinInput> bins{};
  std::vector<PieceInput> pieces{};
  place::PlacementPolicy policy{place::PlacementPolicy::bottom_left};
  PackingConfig config{};
};

using InterruptionProbe = std::function<bool()>;

/**
 * @brief Output of one constructive decode run.
 *
 * @par Invariants
 * - `layout` mirrors the more placement-centric view of the same `bins` state.
 * - `interrupted` reports whether execution stopped at a caller-provided safe
 *   interruption boundary.
 *
 * @par Performance Notes
 * - Exposes both bin-state and export-oriented views to avoid recomputation.
 */
struct DecoderResult {
  std::vector<BinState> bins{};
  Layout layout{};
  bool interrupted{false};
};

} // namespace shiny::nesting::pack
