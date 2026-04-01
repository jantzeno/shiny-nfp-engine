#pragma once

#include <cstddef>
#include <functional>

#include "cache/cache_policy.hpp"
#include "decomposition/decompose.hpp"
#include "nfp/nonconvex_nfp.hpp"
#include "nfp/orbital_verifier.hpp"
#include "nfp/requests.hpp"
#include "nfp/types.hpp"

namespace shiny::nfp {

/**
 * @brief Cache-owning facade for convex, nonconvex, and verifier NFP services.
 *
 * @par Invariants
 * - Convex, nonconvex, and decomposition caches remain versioned through the
 *   request revisions passed into each entrypoint.
 *
 * @par Performance Notes
 * - Centralizes reuse of convex NFP, nonconvex NFP, and decomposition results.
 */
class NfpEngine {
public:
  explicit NfpEngine(cache::CachePolicyConfig cache_policy = {})
      : convex_cache_(cache_policy), nonconvex_cache_(cache_policy),
        decomposition_engine_(cache_policy) {}

  /**
   * @brief Computes a convex NFP using the engine-owned cache.
   *
   * @param request Convex input polygons and discrete rotations.
   * @param geometry_a_revision Revision tag for piece A geometry.
   * @param geometry_b_revision Revision tag for piece B geometry.
   * @return Cached or newly computed convex NFP result.
   */
  [[nodiscard]] auto
  compute_convex_nfp(const ConvexNfpRequest &request,
                     cache::GeometryRevision geometry_a_revision,
                     cache::GeometryRevision geometry_b_revision) -> NfpResult;

  /**
   * @brief Computes a convex IFP using the engine-owned cache.
   *
   * @param request Convex container and piece geometry.
   * @param geometry_a_revision Revision tag for the container geometry.
   * @param geometry_b_revision Revision tag for the piece geometry.
   * @return Cached or newly computed convex IFP result.
   */
  [[nodiscard]] auto
  compute_convex_ifp(const ConvexIfpRequest &request,
                     cache::GeometryRevision geometry_a_revision,
                     cache::GeometryRevision geometry_b_revision) -> NfpResult;

  /**
   * @brief Decomposes a polygon using the shared decomposition engine cache.
   *
   * @param request Polygon and decomposition settings.
   * @param geometry_revision Revision tag for the polygon geometry.
   * @return Cached or newly computed convex decomposition result.
   */
  [[nodiscard]] auto
  decompose_polygon(const decomp::DecompositionRequest &request,
                    cache::GeometryRevision geometry_revision)
      -> decomp::DecompositionResult;

  /**
   * @brief Computes the production nonconvex NFP via graph extraction.
   *
   * @param request Nonconvex input polygons and algorithm revision.
   * @param geometry_a_revision Revision tag for piece A geometry.
   * @param geometry_b_revision Revision tag for piece B geometry.
   * @param interruption_requested Optional run-scoped interruption probe used
   *   to stop at safe graph-construction and extraction boundaries.
   * @return Cached or newly computed graph-extracted nonconvex NFP.
   */
  [[nodiscard]] auto compute_nonconvex_graph_nfp(
      const NonconvexNfpRequest &request,
      cache::GeometryRevision geometry_a_revision,
      cache::GeometryRevision geometry_b_revision,
      const std::function<bool()> &interruption_requested = {})
      -> NonconvexNfpResult;

  /**
   * @brief Runs the orbital verifier for a nonconvex pair.
   *
   * @param request Nonconvex input polygons and algorithm revision.
   * @param geometry_a_revision Revision tag for piece A geometry.
   * @param geometry_b_revision Revision tag for piece B geometry.
   * @return Diagnostic orbital verifier result.
   */
  [[nodiscard]] auto
  compute_orbital_verifier_nfp(const NonconvexNfpRequest &request,
                               cache::GeometryRevision geometry_a_revision,
                               cache::GeometryRevision geometry_b_revision)
      -> OrbitalVerifierResult;

  /**
   * @brief Clears only the convex NFP/IFP cache.
   */
  auto clear_convex_cache() -> void;

  /**
   * @brief Clears only the nonconvex NFP cache.
   */
  auto clear_nonconvex_cache() -> void;

  /**
   * @brief Clears only the decomposition cache.
   */
  auto clear_decomposition_cache() -> void;

  /**
   * @brief Clears all engine-owned caches.
   */
  auto clear_caches() -> void;

  /**
   * @brief Returns the number of cached convex results.
   */
  [[nodiscard]] auto convex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the number of cached nonconvex results.
   */
  [[nodiscard]] auto nonconvex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the number of cached decompositions.
   */
  [[nodiscard]] auto decomposition_cache_size() const -> std::size_t;

private:
  cache::CacheStore<cache::PairRotationKey, NfpResult> convex_cache_{};
  cache::CacheStore<cache::NonconvexNfpCacheKey, NonconvexNfpResult>
      nonconvex_cache_{};
  decomp::DecompositionEngine decomposition_engine_{};
};

} // namespace shiny::nfp