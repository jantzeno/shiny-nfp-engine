#pragma once

#include <cstdint>

#include "geometry/transform.hpp"
#include "nfp/requests.hpp"
#include "nfp/revisions.hpp"

namespace shiny::nfp::cache {

/**
 * @brief Cache key for one piece under one resolved rotation.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct PieceRotationKey {
  std::uint32_t piece_id{0};
  geom::ResolvedRotation rotation{};
  GeometryRevision geometry_revision{};

  auto operator<=>(const PieceRotationKey &) const = default;
};

/**
 * @brief Cache key for a two-piece interaction under resolved rotations.
 *
 * @par Invariants
 * - `inside` distinguishes IFP-style container queries from ordinary pairwise
 *   NFP queries.
 */
struct PairRotationKey {
  std::uint32_t piece_a_id{0};
  geom::ResolvedRotation rotation_a{};
  std::uint32_t piece_b_id{0};
  geom::ResolvedRotation rotation_b{};
  bool inside{false};
  GeometryRevision geometry_a_revision{};
  GeometryRevision geometry_b_revision{};

  auto operator<=>(const PairRotationKey &) const = default;
};

/**
 * @brief Cache key for the production nonconvex NFP pipeline.
 */
struct NonconvexNfpCacheKey {
  PairRotationKey pair{};
  AlgorithmRevision algorithm_revision{};

  auto operator<=>(const NonconvexNfpCacheKey &) const = default;
};

/**
 * @brief Cache key for one placement-engine feasibility query.
 */
struct PlacementQueryKey {
  std::uint32_t bin_id{0};
  std::uint32_t piece_id{0};
  geom::RotationIndex rotation_index{};
  std::uint64_t piece_geometry_revision{0};
  std::uint64_t bin_geometry_revision{0};
  std::uint64_t merged_region_revision{0};
  std::uint64_t hole_set_revision{0};
  std::uint32_t policy_id{0};

  auto operator<=>(const PlacementQueryKey &) const = default;
};

/**
 * @brief Cache key for one layout evaluation in local search.
 */
struct LayoutEvalKey {
  std::uint64_t piece_order_hash{0};
  std::uint64_t rotation_assignment_hash{0};
  std::uint64_t bin_policy_hash{0};
  std::uint32_t search_revision{0};

  auto operator<=>(const LayoutEvalKey &) const = default;
};

/**
 * @brief Builds a piece/rotation cache key.
 */
[[nodiscard]] inline auto
make_piece_rotation_key(std::uint32_t piece_id, geom::ResolvedRotation rotation,
                        GeometryRevision geometry_revision)
    -> PieceRotationKey {
  return {
      .piece_id = piece_id,
      .rotation = rotation,
      .geometry_revision = geometry_revision,
  };
}

/**
 * @brief Builds a pair/rotation cache key for convex NFP queries.
 */
[[nodiscard]] inline auto make_pair_rotation_key(
    const ConvexNfpRequest &request, GeometryRevision geometry_a_revision,
    GeometryRevision geometry_b_revision) -> PairRotationKey {
  return {
      .piece_a_id = request.piece_a_id,
      .rotation_a = request.rotation_a,
      .piece_b_id = request.piece_b_id,
      .rotation_b = request.rotation_b,
      .inside = false,
      .geometry_a_revision = geometry_a_revision,
      .geometry_b_revision = geometry_b_revision,
  };
}

/**
 * @brief Builds a pair/rotation cache key for convex IFP queries.
 */
[[nodiscard]] inline auto make_pair_rotation_key(
    const ConvexIfpRequest &request, GeometryRevision geometry_a_revision,
    GeometryRevision geometry_b_revision) -> PairRotationKey {
  return {
      .piece_a_id = request.container_id,
      .rotation_a = request.container_rotation,
      .piece_b_id = request.piece_id,
      .rotation_b = request.piece_rotation,
      .inside = true,
      .geometry_a_revision = geometry_a_revision,
      .geometry_b_revision = geometry_b_revision,
  };
}

/**
 * @brief Builds a nonconvex NFP cache key from a request and geometry
 * revisions.
 */
[[nodiscard]] inline auto make_nonconvex_nfp_cache_key(
    const NonconvexNfpRequest &request, GeometryRevision geometry_a_revision,
    GeometryRevision geometry_b_revision) -> NonconvexNfpCacheKey {
  return {
      .pair =
          {
              .piece_a_id = request.piece_a_id,
              .rotation_a = request.rotation_a,
              .piece_b_id = request.piece_b_id,
              .rotation_b = request.rotation_b,
              .inside = false,
              .geometry_a_revision = geometry_a_revision,
              .geometry_b_revision = geometry_b_revision,
          },
      .algorithm_revision = request.algorithm_revision,
  };
}

} // namespace shiny::nfp::cache