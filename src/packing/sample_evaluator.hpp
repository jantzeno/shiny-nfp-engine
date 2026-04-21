#pragma once

#include <cstddef>

#include "geometry/types.hpp"
#include "packing/collision_tracker.hpp"

namespace shiny::nesting::pack {

struct SampleEvaluation {
  double weighted_loss{0.0};
  bool early_terminated{false};
};

[[nodiscard]] auto evaluate_sample(const CollisionTracker &tracker,
                                   std::size_t moving_index,
                                   const geom::PolygonWithHoles &candidate_polygon,
                                   double best_known_loss)
    -> SampleEvaluation;

} // namespace shiny::nesting::pack
