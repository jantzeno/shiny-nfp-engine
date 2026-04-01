#pragma once

#include <cstdint>
#include <functional>

#include "decomposition/decomposition_result.hpp"
#include "nfp/graph_types.hpp"
#include "nfp/requests.hpp"
#include "nfp/types.hpp"

namespace shiny::nfp {

/**
 * @brief Reports whether graph extraction produced a usable nonconvex NFP.
 */
enum class ExtractionStatus : std::int8_t {
  success = 0,
  empty = 1,
  invalid_graph = 2,
};

/**
 * @brief Output of the production nonconvex graph-extraction pipeline.
 *
 * @par Invariants
 * - `result`, `graph`, and `status` describe the same extraction run.
 */
struct NonconvexNfpResult {
  NfpResult result{};
  ArrangementGraph graph{};
  ExtractionStatus status{ExtractionStatus::success};
};

/**
 * @brief Builds the arrangement graph induced by pairwise convex components.
 *
 * @param request Nonconvex input polygons and algorithm revision.
 * @return Raw arrangement graph before pruning and loop extraction.
 */
[[nodiscard]] auto build_arrangement_graph(const NonconvexNfpRequest &request)
    -> ArrangementGraph;

/**
 * @brief Removes graph fragments that cannot contribute to valid extracted
 * loops.
 *
 * @param graph Raw arrangement graph.
 * @return Pruned arrangement graph.
 */
[[nodiscard]] auto prune_arrangement_graph(ArrangementGraph graph)
    -> ArrangementGraph;

/**
 * @brief Extracts NFP loops and special features from a pruned arrangement
 * graph.
 *
 * @param graph Pruned arrangement graph.
 * @return Extracted NFP result.
 */
[[nodiscard]] auto extract_nfp_from_graph(const ArrangementGraph &graph)
    -> NfpResult;

/**
 * @brief Computes a nonconvex NFP through decomposition plus graph extraction.
 *
 * @par Algorithm Detail
 * - **Strategy**: Decompose into convex parts, build pairwise convex NFP
 *   arrangement geometry, then extract loops from the induced graph.
 *
 * @par Mathematical Basis
 * - Represents each polygon as a union of convex components and composes
 *   pairwise convex contact regions in translation space.
 * - Extracts valid boundary loops from the induced arrangement graph to recover
 *   the nonconvex NFP topology.
 *
 * @param request Nonconvex input polygons and algorithm revision.
 * @return Nonconvex NFP result with graph diagnostics.
 */
[[nodiscard]] auto
compute_nonconvex_graph_nfp(const NonconvexNfpRequest &request)
    -> NonconvexNfpResult;

} // namespace shiny::nfp

namespace shiny::nfp::detail {

/**
 * @brief Builds an arrangement graph from already computed convex
 * decompositions.
 *
 * @param request Nonconvex request describing the original pair.
 * @param piece_a Convex decomposition of piece A.
 * @param piece_b Convex decomposition of piece B.
 * @return Arrangement graph assembled from pairwise convex NFP contributions.
 */
[[nodiscard]] auto build_arrangement_graph_from_decompositions(
    const NonconvexNfpRequest &request,
    const decomp::DecompositionResult &piece_a,
    const decomp::DecompositionResult &piece_b,
    const std::function<bool()> &interruption_requested = {})
    -> ArrangementGraph;

[[nodiscard]] auto prune_arrangement_graph_with_interruption(
    ArrangementGraph graph,
    const std::function<bool()> &interruption_requested = {})
    -> ArrangementGraph;

[[nodiscard]] auto extract_nfp_from_graph_with_interruption(
    const ArrangementGraph &graph,
    const std::function<bool()> &interruption_requested = {}) -> NfpResult;

} // namespace shiny::nfp::detail