#pragma once

#include <cstddef>

#include "packing/sparrow/sample/search_placement.hpp"

namespace shiny::nesting::pack::sparrow::sample {

struct CoordinateDescentConfig {
  std::size_t iteration_budget{8};
  double translation_step{1.0};
  double min_translation_step{0.1};
  double angle_step_degrees{0.0};
  double min_angle_step_degrees{0.0};
  bool enable_rotation_axis{false};
};

struct CoordinateDescentResult {
  SearchPlacementCandidate best_candidate{};
  std::size_t iterations_completed{0};
  bool stopped_by_iteration_budget{false};
};

[[nodiscard]] auto refine_placement(
    const quantify::CollisionTracker &tracker, std::size_t moving_index,
    const SearchPlacementCandidate &initial_candidate,
    const CoordinateDescentConfig &config) -> CoordinateDescentResult;

} // namespace shiny::nesting::pack::sparrow::sample