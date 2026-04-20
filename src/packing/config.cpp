#include "packing/config.hpp"

namespace shiny::nesting::pack {

auto BoundingBoxPackingConfig::is_valid() const -> bool {
  switch (heuristic) {
  case BoundingBoxHeuristic::shelf:
  case BoundingBoxHeuristic::skyline:
  case BoundingBoxHeuristic::free_rectangle_backfill:
    return true;
  }

  return false;
}

auto DeterministicAttemptConfig::is_valid() const -> bool {
  return max_attempts >= 1U && max_attempts <= 32U;
}

auto LaserCutOptimizationConfig::is_valid() const -> bool {
  switch (mode) {
  case SharedCutOptimizationMode::off:
  case SharedCutOptimizationMode::remove_fully_covered_coincident_segments:
    return true;
  }

  return false;
}

auto PackingConfig::is_valid() const -> bool {
  return placement.is_valid() && bounding_box.is_valid() &&
         deterministic_attempts.is_valid() && laser_cut_optimization.is_valid();
}

} // namespace shiny::nesting::pack