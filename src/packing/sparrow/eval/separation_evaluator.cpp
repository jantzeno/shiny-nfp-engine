#include "packing/sparrow/eval/separation_evaluator.hpp"

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/polygon.hpp"

namespace shiny::nesting::pack::sparrow::eval {

auto evaluate_separation_candidate(
    const quantify::CollisionTracker &tracker, const std::size_t moving_index,
    const adapters::PortPolygon &candidate_polygon,
    const double best_known_loss) -> SampleEval {
  const auto candidate = adapters::to_engine_polygon(candidate_polygon);
  const BoundedLossContext context{.best_known_loss = best_known_loss};

  auto evaluation = accumulate_weighted_loss(
      context, 0.0,
      tracker.container_weight(moving_index) *
          geom::polygon_area_sum(geom::difference_polygons(
              candidate, tracker.container_polygon())));
  if (evaluation.early_terminated) {
    return evaluation;
  }

  for (std::size_t other = 0; other < tracker.item_count(); ++other) {
    if (other == moving_index) {
      continue;
    }
    evaluation = accumulate_weighted_loss(
        context, evaluation.weighted_loss,
        tracker.pair_weight(moving_index, other) *
            geom::polygon_area_sum(geom::intersection_polygons(
                candidate, tracker.item_polygon(other))));
    if (evaluation.early_terminated) {
      return evaluation;
    }
  }

  return evaluation;
}

} // namespace shiny::nesting::pack::sparrow::eval