#pragma once

#include <cstdint>

#include "placement/config.hpp"

namespace shiny::nesting::pack {

/**
 * @brief Selects how coincident cut segments are simplified.
 *
 * These modes control whether cut-plan generation leaves raw segment output
 * untouched or removes redundant coincident coverage.
 */
enum class SharedCutOptimizationMode : std::int8_t {
  off = 0,
  remove_fully_covered_coincident_segments = 1,
};

/**
 * @brief Selects the AABB constructive heuristic used by the bounding-box
 * packer.
 *
 * `shelf` preserves the original row/shelf behavior, `skyline` stacks against
 * the current step skyline, and `free_rectangle_backfill` places against the
 * largest remaining free rectangles.
 */
enum class BoundingBoxHeuristic : std::int8_t {
  shelf = 0,
  skyline = 1,
  free_rectangle_backfill = 2,
};

/**
 * @brief Heuristic settings specific to the bounding-box constructive packer.
 *
 * @par Invariants
 * - `heuristic` must be one of the supported bounded constructive strategies.
 */
struct BoundingBoxPackingConfig {
  BoundingBoxHeuristic heuristic{BoundingBoxHeuristic::shelf};

  /**
   * @brief Reports whether the selected heuristic is supported.
   *
   * @return `true` when the heuristic value maps to one of the implemented
   * constructive modes.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Bounds deterministic reordering attempts for constructive packers.
 *
 * Shared by constructive flows that evaluate a fixed set of deterministic
 * piece-order variants instead of stochastic optimization.
 *
 * @par Invariants
 * - `max_attempts` must stay within the supported bounded range.
 *
 * @par Performance Notes
 * - Higher attempt counts scale linearly with constructive decode cost.
 */
struct DeterministicAttemptConfig {
  std::uint32_t max_attempts{1};

  /**
   * @brief Reports whether the bounded deterministic attempt settings are
   * internally consistent.
   *
   * @return `true` when the attempt budget stays within the supported range.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Controls cut-plan simplification after layout generation.
 *
 * @par Invariants
 * - Simplification behavior is only meaningful when the selected mode is not
 *   `off`.
 *
 * @par Performance Notes
 * - Validation is cheap and can be checked on every decode request.
 */
struct LaserCutOptimizationConfig {
  SharedCutOptimizationMode mode{SharedCutOptimizationMode::off};
  bool require_exact_collinearity{true};
  bool preserve_visible_notches{true};

  /**
   * @brief Reports whether the cut optimization settings are internally
   * consistent.
   *
   * @return `true` when the selected option combination is supported.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief High-level constructive packing configuration.
 *
 * Combines placement settings, hole-first placement behavior, and optional cut
 * optimization settings.
 *
 * @par Invariants
 * - The nested placement and cut-optimization configs must both be valid.
 *
 * @par Performance Notes
 * - Shared verbatim across constructive packing requests.
 */
struct PackingConfig {
  place::PlacementConfig placement{};
  bool enable_hole_first_placement{true};
  BoundingBoxPackingConfig bounding_box{};
  DeterministicAttemptConfig deterministic_attempts{};
  LaserCutOptimizationConfig laser_cut_optimization{};

  /**
   * @brief Reports whether the full packing config can be used by the decoder.
   *
   * @return `true` when all nested config surfaces are valid.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

} // namespace shiny::nesting::pack
