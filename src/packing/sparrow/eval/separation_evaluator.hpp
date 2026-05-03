#pragma once

#include <cstddef>

#include "packing/sparrow/adapters/geometry_adapter.hpp"
#include "packing/sparrow/eval/sample_eval.hpp"
#include "packing/sparrow/quantify/collision_tracker.hpp"

namespace shiny::nesting::pack::sparrow::eval {

[[nodiscard]] auto
evaluate_separation_candidate(const quantify::CollisionTracker &tracker,
                              std::size_t moving_index,
                              const adapters::PortPolygon &candidate_polygon,
                              double best_known_loss) -> SampleEval;

} // namespace shiny::nesting::pack::sparrow::eval