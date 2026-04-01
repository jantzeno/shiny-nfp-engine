#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "decomposition/decomposition_result.hpp"
#include "nfp/requests.hpp"
#include "nfp/types.hpp"

namespace shiny::nfp {

/**
 * @brief Classifies the active contact type during orbital tracing.
 */
enum class OrbitalTouchKind : std::int8_t {
  vertex_vertex = 0,
  vertex_edge = 1,
  edge_vertex = 2,
  edge_edge = 3,
};

/**
 * @brief Reports whether the orbital verifier completed successfully.
 */
enum class OrbitalVerifierStatus : std::int8_t {
  success = 0,
  empty = 1,
  invalid = 2,
};

/**
 * @brief One sampled contact configuration from the orbital verifier.
 *
 * @par Thread Safety
 * - Container value type with no shared state.
 */
struct OrbitalToucher {
  geom::Point2 translation{};
  OrbitalTouchKind kind{OrbitalTouchKind::vertex_vertex};
  std::vector<geom::Point2> contact_points{};
};

/**
 * @brief Feasible translation segment discovered during orbital propagation.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct OrbitalFeasibleTranslation {
  geom::Segment2 segment{};
  bool trimmed{false};
};

/**
 * @brief Summary of one traced orbital loop.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct OrbitalLoopTrace {
  geom::Point2 start{};
  NfpFeatureKind kind{NfpFeatureKind::outer_loop};
  std::size_t segment_count{0};
  bool completed{false};
};

/**
 * @brief Diagnostic output of the orbital-verifier path.
 *
 * @par Invariants
 * - `result`, `touchers`, `feasible_translations`, and `traces` all describe
 *   the same verifier execution.
 */
struct OrbitalVerifierResult {
  NfpResult result{};
  std::vector<OrbitalToucher> touchers{};
  std::vector<OrbitalFeasibleTranslation> feasible_translations{};
  std::vector<OrbitalLoopTrace> traces{};
  std::vector<geom::Point2> inner_loop_starts{};
  OrbitalVerifierStatus status{OrbitalVerifierStatus::success};
  bool completed{false};
};

/**
 * @brief Computes a diagnostic NFP trace by orbiting one polygon around
 * another.
 *
 * @par Algorithm Detail
 * - **Strategy**: Follow feasible contact transitions around the translational
 *   boundary and reconstruct loops from the visited states.
 *
 * @par Mathematical Basis
 * - Traverses the boundary of the translational contact manifold by advancing
 *   between active contact constraints.
 * - Loop closure over feasible contact transitions reconstructs outer and inner
 *   NFP boundary cycles for diagnostic comparison.
 *
 * @param request Nonconvex input polygons and algorithm revision.
 * @return Diagnostic orbital verifier result.
 */
[[nodiscard]] auto
compute_orbital_verifier_nfp(const NonconvexNfpRequest &request)
    -> OrbitalVerifierResult;

} // namespace shiny::nfp

namespace shiny::nfp::detail {

/**
 * @brief Runs the orbital verifier from already computed convex decompositions.
 *
 * @param request Nonconvex request describing the original pair.
 * @param piece_a Convex decomposition of piece A.
 * @param piece_b Convex decomposition of piece B.
 * @return Diagnostic verifier result derived from the provided decompositions.
 */
[[nodiscard]] auto compute_orbital_verifier_from_decompositions(
    const NonconvexNfpRequest &request,
    const decomp::DecompositionResult &piece_a,
    const decomp::DecompositionResult &piece_b) -> OrbitalVerifierResult;

} // namespace shiny::nfp::detail