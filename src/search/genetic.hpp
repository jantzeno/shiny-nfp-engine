#pragma once

#include <cstddef>

#include "cache/cache_policy.hpp"
#include "cache/stores.hpp"
#include "nfp/cache_keys.hpp"
#include "search/request.hpp"
#include "search/result.hpp"

namespace shiny::nfp::search {

/**
 * @brief Deterministic genetic-search engine above the constructive decoder.
 *
 * Evolves piece-order populations with tournament selection, ordered
 * crossover, bounded mutation, elitism, and optional nested local-search
 * polishing.
 *
 * @par Invariants
 * - Reevaluation cache entries are keyed by piece order, geometry revisions,
 *   and the GA search revision.
 *
 * @par Performance Notes
 * - Reuses cached decode evaluations across generations and repeated runs.
 */
class GeneticSearch {
public:
  explicit GeneticSearch(cache::CachePolicyConfig cache_policy = {})
      : cache_policy_(cache_policy), reevaluation_cache_(cache_policy),
        decoder_(cache_policy) {}

  /**
   * @brief Improves a constructive decode by evolving piece-order populations.
   *
   * @param request Constructive decode request plus genetic-search,
   *   local-polishing, and execution-control settings.
   * @return Baseline, best result, generation-level progress trace, retained
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