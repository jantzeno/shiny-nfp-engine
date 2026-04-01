#pragma once

#include <cstdint>

#include "placement/config.hpp"

namespace shiny::nfp::pack {

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
 * - Shared verbatim across decoder and search requests.
 */
struct PackingConfig {
  place::PlacementConfig placement{};
  bool enable_hole_first_placement{true};
  LaserCutOptimizationConfig laser_cut_optimization{};

  /**
   * @brief Reports whether the full packing config can be used by the decoder.
   *
   * @return `true` when all nested config surfaces are valid.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

} // namespace shiny::nfp::pack