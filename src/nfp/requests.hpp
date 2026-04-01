#pragma once

#include <cstdint>

#include "geometry/transform.hpp"
#include "geometry/types.hpp"
#include "nfp/revisions.hpp"

namespace shiny::nfp {

/**
 * @brief Input bundle for convex no-fit polygon computation.
 *
 * @par Invariants
 * - `convex_a` and `convex_b` are expected to be normalized convex rings.
 */
struct ConvexNfpRequest {
  std::uint32_t piece_a_id{0};
  std::uint32_t piece_b_id{0};
  geom::Ring convex_a{};
  geom::Ring convex_b{};
  geom::ResolvedRotation rotation_a{};
  geom::ResolvedRotation rotation_b{};
};

/**
 * @brief Input bundle for convex inner-fit polygon computation.
 *
 * @par Invariants
 * - `container` is the enclosing region and `piece` is the translated shape.
 */
struct ConvexIfpRequest {
  std::uint32_t container_id{0};
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles container{};
  geom::PolygonWithHoles piece{};
  geom::ResolvedRotation container_rotation{};
  geom::ResolvedRotation piece_rotation{};
};

/**
 * @brief Input bundle for nonconvex NFP generation.
 *
 * @par Invariants
 * - `piece_a` and `piece_b` are expected to be normalized simple polygons with
 *   holes represented explicitly.
 *
 * @par Performance Notes
 * - Carries an algorithm revision so cache entries can be invalidated when the
 *   nonconvex kernel changes.
 */
struct NonconvexNfpRequest {
  std::uint32_t piece_a_id{0};
  std::uint32_t piece_b_id{0};
  geom::PolygonWithHoles piece_a{};
  geom::PolygonWithHoles piece_b{};
  geom::ResolvedRotation rotation_a{};
  geom::ResolvedRotation rotation_b{};
  cache::AlgorithmRevision algorithm_revision{};
};

} // namespace shiny::nfp