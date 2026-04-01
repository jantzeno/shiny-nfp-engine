#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"

namespace shiny::nfp::decomp {

/**
 * @brief Selects the convex decomposition backend.
 *
 * Both modes rely on CGAL partitioning, with the optimal path favoring fewer
 * convex components and the approximate path favoring throughput.
 */
enum class DecompositionAlgorithm : std::int8_t {
  cgal_optimal_convex_partition = 0,
  cgal_approx_convex_partition = 1,
};

/**
 * @brief Describes whether a decomposition result preserves required topology.
 *
 * The status distinguishes structural, orientation, and area regressions so
 * callers can diagnose invalid convex partitions without inspecting every
 * component manually.
 */
enum class DecompositionValidity : std::int8_t {
  unknown = 0,
  valid = 1,
  invalid_topology = 2,
  invalid_orientation = 3,
  area_mismatch = 4,
  nonconvex_component = 5,
};

/**
 * @brief One convex component extracted from a source polygon.
 *
 * Tracks which source component produced the convex piece and whether the ring
 * was normalized before downstream use.
 *
 * @par Invariants
 * - `outer` is expected to be convex when `validity` is `valid` in the parent
 *   result.
 *
 * @par Performance Notes
 * - Designed for direct reuse in convex-pair NFP generation.
 */
struct ConvexComponent {
  geom::Ring outer{};
  std::size_t source_component_index{0};
  bool normalized{false};
};

/**
 * @brief Complete output of one polygon decomposition request.
 *
 * Packages the extracted convex components together with validation status,
 * aggregate signed area, and whether the result came from cache.
 *
 * @par Invariants
 * - `components` should collectively cover the source polygon when `validity`
 *   is `valid`.
 *
 * @par Performance Notes
 * - `cached` lets callers report cache behavior without a second side channel.
 */
struct DecompositionResult {
  std::vector<ConvexComponent> components{};
  DecompositionValidity validity{DecompositionValidity::unknown};
  double signed_area{0.0};
  bool cached{false};
};

} // namespace shiny::nfp::decomp