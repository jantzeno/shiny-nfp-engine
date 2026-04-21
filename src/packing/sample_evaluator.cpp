#include "packing/sample_evaluator.hpp"

#include "geometry/polygon.hpp"
#include "packing/collision_tracker.hpp"
#include "polygon_ops/boolean_ops.hpp"

namespace shiny::nesting::pack {

// Evaluate the weighted loss the tracker WOULD record if `moving_index`
// were repositioned to `candidate_polygon`. The tracker is NOT mutated.
//
// Early termination: as we sum container-loss + per-pair losses, any
// running total exceeding `best_known_loss` aborts immediately and sets
// `early_terminated = true`. Callers (item_mover) supply their current
// best so this short-circuits most negative samples.
auto evaluate_sample(const CollisionTracker &tracker, const std::size_t moving_index,
                     const geom::PolygonWithHoles &candidate_polygon,
                     const double best_known_loss) -> SampleEvaluation {
  SampleEvaluation result{};
  const auto candidate_revision = geom::polygon_revision(candidate_polygon);

  const auto outside_loss =
      geom::polygon_area_sum(poly::difference_polygons(candidate_polygon, tracker.container()));
  result.weighted_loss += tracker.container_weight(moving_index) * outside_loss;
  if (result.weighted_loss > best_known_loss) {
    result.early_terminated = true;
    return result;
  }

  for (std::size_t other = 0; other < tracker.item_count(); ++other) {
    if (other == moving_index) {
      continue;
    }
    const auto [pair_loss, _] = compute_polygon_pair_loss(
        candidate_polygon, candidate_revision, tracker.item(other).polygon,
        tracker.item_polygon_revision(other), tracker.pole_cache(),
        tracker.penetration_depth_cache());
    result.weighted_loss += tracker.pair_weight(moving_index, other) * pair_loss;
    if (result.weighted_loss > best_known_loss) {
      result.early_terminated = true;
      return result;
    }
  }

  return result;
}

} // namespace shiny::nesting::pack
