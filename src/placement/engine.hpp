#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "cache/cache_policy.hpp"
#include "cache/stores.hpp"
#include "nfp/types.hpp"
#include "placement/types.hpp"

namespace shiny::nfp::place {

/**
 * @brief Generates, filters, ranks, and caches placement candidates.
 *
 * This engine is the bridge between geometric feasibility data and the
 * constructive decoder's choice of the next placed pose.
 *
 * @par Invariants
 * - Cached ranked candidate sets are keyed by geometry revisions and policy.
 *
 * @par Performance Notes
 * - Reuses ranked candidate sets across repeated placement queries.
 */
class PlacementEngine {
public:
  explicit PlacementEngine(cache::CachePolicyConfig cache_policy = {})
      : cache_(cache_policy) {}

  /**
   * @brief Converts one NFP/IFP result into raw placement candidates.
   *
   * @param result Geometric feasibility result to convert.
   * @param rotation_index Rotation index shared by all emitted candidates.
   * @param config Placement config used to interpret the rotation set.
   * @param source Candidate provenance tag.
   * @param inside_hole Whether the candidates come from a hole placement path.
   * @param hole_index Hole index for hole-originating candidates.
   * @return Unfiltered placement candidates extracted from the geometry result.
   */
  [[nodiscard]] auto extract_candidates(
      const NfpResult &result, geom::RotationIndex rotation_index,
      const PlacementConfig &config = {},
      PlacementCandidateSource source = PlacementCandidateSource::nfp_boundary,
      bool inside_hole = false, std::int32_t hole_index = -1) const
      -> std::vector<PlacementCandidate>;

  /**
   * @brief Removes infeasible or duplicate candidates for one placement query.
   *
   * @param request Full placement request state.
   * @param candidates Raw candidate set to filter.
   * @return Feasible candidates that survive geometric and policy checks.
   */
  [[nodiscard]] auto
  filter_candidates(const PlacementRequest &request,
                    std::span<const PlacementCandidate> candidates) const
      -> std::vector<PlacementCandidate>;

  /**
   * @brief Sorts feasible candidates according to the requested policy.
   *
   * @param request Full placement request state.
   * @param candidates Feasible candidates to rank.
   * @param policy Ranking heuristic to apply.
   * @return Ranked candidate set with best-first ordering.
   */
  [[nodiscard]] auto
  rank_candidates(const PlacementRequest &request,
                  std::span<const PlacementCandidate> candidates,
                  PlacementPolicy policy) const -> RankedPlacementSet;

  /**
   * @brief Runs the full candidate generation pipeline for one piece query.
   *
   * @par Algorithm Detail
   * - **Strategy**: Candidate extraction from feasibility regions followed by
   *   filtering, ranking, and cache reuse.
   * - **Steps**:
   *   1. Extract candidates from occupied-region NFP, container IFP, and hole
   *      IFP inputs.
   *   2. Filter infeasible or duplicate candidates against the current bin
   *      state.
   *   3. Rank the survivors with the selected placement policy and cache the
   *      result.
   *
   * @param request Full placement request state.
   * @param occupied_region_nfp Feasibility against current occupied geometry.
   * @param bin_ifp Container feasibility region.
   * @param hole_ifps Hole feasibility regions.
   * @param policy Ranking heuristic to apply.
   * @return Ranked candidate set for the request.
   */
  [[nodiscard]] auto query_candidates(const PlacementRequest &request,
                                      const NfpResult *occupied_region_nfp,
                                      const NfpResult *bin_ifp,
                                      std::span<const NfpResult> hole_ifps,
                                      PlacementPolicy policy)
      -> RankedPlacementSet;

  /**
   * @brief Clears cached placement-query results.
   */
  auto clear_cache() -> void;

  /**
   * @brief Returns the number of cached ranked placement sets.
   */
  [[nodiscard]] auto cache_size() const -> std::size_t;

private:
  cache::CacheStore<cache::PlacementQueryKey, RankedPlacementSet> cache_{};
};

} // namespace shiny::nfp::place
