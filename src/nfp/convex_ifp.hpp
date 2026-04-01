#pragma once

#include "cache/stores.hpp"
#include "nfp/cache_keys.hpp"
#include "nfp/requests.hpp"
#include "nfp/types.hpp"

namespace shiny::nfp {

/**
 * @brief Computes the convex inner-fit polygon for one container/piece pair.
 *
 * @par Algorithm Detail
 * - **Strategy**: Convex feasible-translation construction derived from support
 *   relationships between the container and the reflected piece.
 *
 * @par Mathematical Basis
 * - Computes feasible translations as the intersection of half-planes induced
 *   by container edges and the reflected convex piece.
 * - Equivalent to clipping the translational configuration space to placements
 *   where every piece vertex remains inside the convex container.
 *
 * @par Complexity
 * - **Time**: O(n + m).
 * - **Space**: O(n + m).
 *
 * @param request Convex container and piece geometry.
 * @return Convex IFP arrangement.
 */
[[nodiscard]] auto compute_convex_ifp(const ConvexIfpRequest &request)
    -> NfpResult;

/**
 * @brief Computes or retrieves a convex IFP using the supplied cache store.
 *
 * @param request Convex container and piece geometry.
 * @param geometry_a_revision Revision tag for the container geometry.
 * @param geometry_b_revision Revision tag for the piece geometry.
 * @param cache_store Cache store keyed by pair/rotation identity.
 * @return Cached or newly computed convex IFP arrangement.
 */
[[nodiscard]] auto compute_convex_ifp(
    const ConvexIfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    cache::CacheStore<cache::PairRotationKey, NfpResult> &cache_store)
    -> NfpResult;

} // namespace shiny::nfp