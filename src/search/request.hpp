#pragma once

#include "packing/decoder.hpp"
#include "search/config.hpp"
#include "search/execution.hpp"

namespace shiny::nfp::search {

/**
 * @brief Full input bundle for one search-engine run.
 *
 * Combines the constructive decode request with local-search polishing,
 * genetic-search planning, and run-scoped observer plus execution controls.
 *
 * @par Invariants
 * - `decoder_request`, any active search config, and `execution.control` must
 *   all be valid before execution.
 *
 * @par Performance Notes
 * - Reuses `pack::DecoderRequest` directly so reevaluation needs no adapter.
 */
struct SearchRequest {
  pack::DecoderRequest decoder_request{};
  LocalSearchConfig local_search{};
  GeneticSearchConfig genetic_search{};
  SearchExecutionConfig execution{};
};

} // namespace shiny::nfp::search