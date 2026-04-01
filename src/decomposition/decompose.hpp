#pragma once

#include <cstddef>
#include <cstdint>

#include "cache/cache_policy.hpp"
#include "cache/stores.hpp"
#include "decomposition/decomposition_result.hpp"
#include "geometry/transform.hpp"
#include "geometry/types.hpp"
#include "nfp/cache_keys.hpp"

namespace shiny::nfp::decomp {

/**
 * @brief Input bundle for convex decomposition of one rotated piece.
 *
 * Couples source geometry with the piece identity, resolved rotation, and the
 * requested CGAL partitioning strategy.
 *
 * @par Invariants
 * - `polygon` is expected to be normalized before decomposition-sensitive cache
 *   keys are reused.
 *
 * @par Performance Notes
 * - Carries exactly the fields needed to build a piece-rotation cache key.
 */
struct DecompositionRequest {
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles polygon{};
  geom::ResolvedRotation rotation{};
  DecompositionAlgorithm algorithm{
      DecompositionAlgorithm::cgal_optimal_convex_partition};
};

/**
 * @brief Decomposes one polygon into convex components without external cache.
 *
 * @par Algorithm Detail
 * - **Strategy**: Convex decomposition via CGAL partitioning over the
 *   normalized, rotated polygon.
 * - **Steps**:
 *   1. Normalize the input polygon and apply the requested resolved rotation.
 *   2. Partition the rotated polygon into convex components with the selected
 *      CGAL backend.
 *   3. Validate convexity, area conservation, and orientation of the result.
 *
 * @par Mathematical Basis
 * - Decomposes a simple polygon into a finite set of convex components whose
 *   union matches the source boundary and area.
 * - Convexity constraints ensure each component is suitable for linear-time
 *   convex pairwise NFP composition downstream.
 *
 * @par Complexity
 * - **Time**: O(n log n + p) where `n` is the source vertex count and `p` is
 *   the backend partition cost.
 * - **Space**: O(n + k) where `k` is the total component vertex count.
 *
 * @par Invariants & Preconditions
 * - @pre The source polygon should be simple.
 * - @pre Rotation and piece identity must match the geometry supplied.
 *
 * @par Edge Cases & Degeneracies
 * - Rejects topology that cannot be expressed as convex components.
 * - Surfaces area and orientation mismatches explicitly in the validity field.
 *
 * @param request Piece, rotation, and algorithm selection for decomposition.
 * @return Convex components plus validation metadata.
 * @see DecompositionEngine
 */
[[nodiscard]] auto decompose_polygon(const DecompositionRequest &request)
    -> DecompositionResult;

/**
 * @brief Decomposes one polygon and reuses a caller-provided cache store.
 *
 * @param request Piece, rotation, and algorithm selection for decomposition.
 * @param geometry_revision Revision marker used in the cache key.
 * @param cache_store Cache used to reuse prior piece-rotation decompositions.
 * @return Convex components plus validation metadata.
 * @see decompose_polygon(const DecompositionRequest &)
 */
[[nodiscard]] auto decompose_polygon(
    const DecompositionRequest &request,
    cache::GeometryRevision geometry_revision,
    cache::CacheStore<cache::PieceRotationKey, DecompositionResult>
        &cache_store) -> DecompositionResult;

/**
 * @brief Validates that a decomposition covers the source polygon correctly.
 *
 * @param source Original polygon being decomposed.
 * @param result Candidate decomposition result to check.
 * @return Validation category describing the first detected mismatch.
 */
[[nodiscard]] auto validate_decomposition(const geom::PolygonWithHoles &source,
                                          const DecompositionResult &result)
    -> DecompositionValidity;

/**
 * @brief Cache-owning façade for repeated decomposition requests.
 *
 * Owns the piece-rotation decomposition cache so higher layers can reuse convex
 * partitions without manually threading cache stores around.
 *
 * @par Invariants
 * - Cache keys combine piece identity, rotation, and geometry revision.
 *
 * @par Performance Notes
 * - Repeated calls with unchanged geometry reuse cached decompositions.
 */
class DecompositionEngine {
public:
  explicit DecompositionEngine(cache::CachePolicyConfig cache_policy = {})
      : cache_(cache_policy) {}

  /**
   * @brief Decomposes one piece and stores or reuses the cached result.
   *
   * @param request Piece, rotation, and algorithm selection for decomposition.
   * @param geometry_revision Revision marker used to invalidate stale cache
   *   entries.
   * @return Convex decomposition result for the given piece state.
   */
  [[nodiscard]] auto
  decompose_polygon(const DecompositionRequest &request,
                    cache::GeometryRevision geometry_revision)
      -> DecompositionResult;

  /**
   * @brief Clears all cached decomposition entries.
   */
  auto clear_cache() -> void;

  /**
   * @brief Returns the number of cached piece-rotation decompositions.
   *
   * @return Current cache cardinality.
   */
  [[nodiscard]] auto cache_size() const -> std::size_t;

private:
  cache::CacheStore<cache::PieceRotationKey, DecompositionResult> cache_{};
};

} // namespace shiny::nfp::decomp