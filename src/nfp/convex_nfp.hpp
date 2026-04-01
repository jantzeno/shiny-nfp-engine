#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "cache/stores.hpp"
#include "geometry/types.hpp"
#include "nfp/cache_keys.hpp"
#include "nfp/requests.hpp"
#include "nfp/types.hpp"

namespace shiny::nfp {

/**
 * @brief Vertex annotation used while ordering a convex NFP boundary.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct ConvexOrderingVertex {
  geom::Point2 point{};
  std::size_t source_edge_index{0};
  double polar_key{0.0};
};

/**
 * @brief Canonical edge sequence used by the convex merge routine.
 *
 * @par Invariants
 * - `edges` and `source_indices` describe the same cyclic sequence.
 */
struct ConvexEdgeSequence {
  std::vector<geom::Vector2> edges{};
  std::vector<std::size_t> source_indices{};
};

/**
 * @brief Computes the convex no-fit polygon for two convex inputs.
 *
 * @par Algorithm Detail
 * - **Strategy**: Convex edge-merge / Minkowski-boundary construction.
 * - **Steps**:
 *   1. Build canonical directed edge sequences for both inputs.
 *   2. Merge edges by polar order.
 *   3. Reconstruct the output loop and classify special perfect-fit features.
 *
 * @par Mathematical Basis
 * - Constructs the boundary of the Minkowski sum of piece A and the reflected
 *   boundary of piece B.
 * - Polar-angle edge merge over two convex chains yields the translational
 *   contact loop without self-intersections.
 *
 * @par Complexity
 * - **Time**: O(n + m).
 * - **Space**: O(n + m).
 *
 * @par Invariants & Preconditions
 * - @pre Request rings must be normalized and convex.
 *
 * @param request Convex input polygons and rotations.
 * @return Convex NFP arrangement.
 */
[[nodiscard]] auto compute_convex_nfp(const ConvexNfpRequest &request)
    -> NfpResult;

/**
 * @brief Computes or retrieves a convex NFP using the supplied cache store.
 *
 * @param request Convex input polygons and rotations.
 * @param geometry_a_revision Revision tag for piece A geometry.
 * @param geometry_b_revision Revision tag for piece B geometry.
 * @param cache_store Cache store keyed by pair/rotation identity.
 * @return Cached or newly computed convex NFP arrangement.
 */
[[nodiscard]] auto compute_convex_nfp(
    const ConvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    cache::CacheStore<cache::PairRotationKey, NfpResult> &cache_store)
    -> NfpResult;

/**
 * @brief Builds the directed edge sequence for one convex ring.
 *
 * @param ring Normalized convex ring.
 * @return Directed edges and their source indices in traversal order.
 */
[[nodiscard]] auto
build_convex_edge_sequence(std::span<const geom::Point2> ring)
    -> ConvexEdgeSequence;

/**
 * @brief Orders NFP vertices into a stable canonical sequence.
 *
 * @param result Convex NFP result to analyze.
 * @return Canonically ordered vertex annotations.
 */
[[nodiscard]] auto order_convex_nfp_vertices(const NfpResult &result)
    -> std::vector<ConvexOrderingVertex>;

} // namespace shiny::nfp