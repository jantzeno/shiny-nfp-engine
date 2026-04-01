#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "cache/cache_policy.hpp"
#include "nfp/engine.hpp"
#include "packing/bin_state.hpp"
#include "packing/config.hpp"

namespace shiny::nfp::pack {

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
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
};

/**
 * @brief Prototype geometry for bins opened by the decoder.
 *
 * @par Invariants
 * - Every bin opened during one decode starts from this prototype geometry.
 *
 * @par Performance Notes
 * - Stored once in the request and copied into each new bin state.
 */
struct BinPrototype {
  std::uint32_t base_bin_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint64_t geometry_revision{0};
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
  BinPrototype bin{};
  std::vector<PieceInput> pieces{};
  place::PlacementPolicy policy{place::PlacementPolicy::bottom_left};
  PackingConfig config{};
  std::size_t max_bin_count{0};
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

/**
 * @brief Order-sensitive constructive packing engine.
 *
 * Consumes a piece order and incrementally places parts into bins using the
 * placement engine and cached NFP queries.
 *
 * @par Invariants
 * - Cache contents are tied to the current geometry and algorithm revisions.
 *
 * @par Performance Notes
 * - Owns convex, nonconvex, and decomposition caches through `NfpEngine`.
 */
class ConstructiveDecoder {
public:
  explicit ConstructiveDecoder(cache::CachePolicyConfig cache_policy = {});

  /**
   * @brief Decodes the requested piece order into a concrete layout.
   *
   * @par Algorithm Detail
   * - **Strategy**: Greedy constructive decoding over the configured piece
   *   order.
   * - **Steps**:
   *   1. Open the prototype bin and iterate pieces in request order.
   *   2. Query feasible placements against current occupancy, holes, and the
   *      bin/container boundaries.
   *   3. Place the best-ranked candidate or open a new bin when allowed.
   *
   * @par Mathematical Basis
   * - Feasible placements are sampled from translational contact boundaries
   *   (NFP/IFP geometry), then filtered by containment and non-overlap
   *   predicates.
   * - Greedy objective ordering induces a deterministic sequence over this
   *   feasible set.
   *
   * @par Complexity
   * - **Time**: O(p * q) where `p` is the piece count and `q` is the aggregate
   *   placement-query cost.
   * - **Space**: O(p + b) where `b` is the number of opened bins.
   *
   * @par Invariants & Preconditions
   * - @pre Request configuration must be valid.
   * - @pre Input polygons should already be normalized.
   *
   * @par Edge Cases & Degeneracies
   * - Leaves pieces unplaced when no legal candidate exists and new bins are
   *   disallowed.
   * - Preserves the input order when multiple candidates tie after ranking.
   *
   * @param request Bin prototype, piece order, and decode policy.
   * @param interruption_requested Optional run-scoped interruption probe used
   *   to stop at safe decode boundaries.
   * @return Concrete decode result containing bin states and exported layout
   *   state.
   */
  [[nodiscard]] auto
  decode(const DecoderRequest &request,
         const InterruptionProbe &interruption_requested = {}) -> DecoderResult;

  /**
   * @brief Clears all geometry caches owned by the decoder.
   */
  auto clear_caches() -> void;

  /**
   * @brief Returns the current convex NFP cache size.
   */
  [[nodiscard]] auto convex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the current nonconvex NFP cache size.
   */
  [[nodiscard]] auto nonconvex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the current decomposition cache size.
   */
  [[nodiscard]] auto decomposition_cache_size() const -> std::size_t;

private:
  NfpEngine nfp_engine_{};
  std::uint64_t decode_generation_{0};
};

} // namespace shiny::nfp::pack