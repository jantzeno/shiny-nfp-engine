#pragma once

#include <cstddef>

#include "cache/cache_policy.hpp"
#include "cache/stores.hpp"
#include "nfp/cache_keys.hpp"
#include "search/request.hpp"
#include "search/result.hpp"

namespace shiny::nfp::search {

/**
 * @brief Deterministic local-search engine over constructive decoder orders.
 *
 * Evaluates adjacent swaps and one-piece insert moves above the constructive
 * decoder to reduce unplaced pieces, bins, and utilization loss.
 *
 * @par Invariants
 * - Reevaluation cache entries are keyed by piece order, geometry revisions,
 * and search revision.
 *
 * @par Performance Notes
 * - Reuses cached decode evaluations across repeated runs and neighboring
 *   orders.
 */
class JostleSearch {
public:
  explicit JostleSearch(cache::CachePolicyConfig cache_policy = {})
      : cache_policy_(cache_policy), reevaluation_cache_(cache_policy),
        decoder_(cache_policy) {}

  /**
   * @brief Improves a constructive decode by exploring local order changes.
   *
   * @par Algorithm Detail
   * - **Strategy**: Deterministic neighborhood search with jostle swaps,
   *   insert-kick moves, and bounded plateau traversal.
   * - **Steps**:
   *   1. Decode the baseline identity order and record it as the initial best.
   *   2. Explore adjacent-swap neighbors for the current order.
   *   3. If no strict jostle improvement is found, explore one-piece insert
   *      neighbors and continue until the plateau budget is exhausted.
   *
   * @par Mathematical Basis
   * - Defines a neighborhood graph over piece permutations using adjacent swaps
   *   and one-piece insert moves.
   * - Each neighbor is scored by decoding into geometric feasibility space,
   *   then compared under the lexicographic packing objective.
   *
   * @par Complexity
   * - **Time**: O(i * n^2 * d) where `i` is the realized iteration count, `n`
   *   is the piece count, and `d` is the decode cost per evaluated order.
   * - **Space**: O(c) where `c` is the reevaluation cache footprint.
   *
   * @par Invariants & Preconditions
   * - @pre Search and decoder configs must be valid.
   * - @pre Request geometry should already be normalized.
   *
   * @par Edge Cases & Degeneracies
   * - Stops immediately when configs are invalid.
   * - Accepts equal-score plateau moves but limits consecutive stalls by
   *   `plateau_budget`.
   *
   * @param request Constructive decode request plus local-search parameters.
   * @return Baseline, best result, per-iteration progress trace, retained
   *   event history, and terminal run status.
   */
  [[nodiscard]] auto improve(const SearchRequest &request) -> SearchResult;

  /**
   * @brief Clears cached reevaluated orders and decoder-side geometry caches.
   */
  auto clear_caches() -> void;

  /**
   * @brief Returns the number of cached layout evaluations.
   */
  [[nodiscard]] auto reevaluation_cache_size() const -> std::size_t;

private:
  cache::CachePolicyConfig cache_policy_{};
  cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation>
      reevaluation_cache_{};
  pack::ConstructiveDecoder decoder_{};
};

} // namespace shiny::nfp::search