#include "packing/sparrow/sample/coordinate_descent.hpp"

#include <array>

#include "geometry/transforms/transform.hpp"
#include "geometry/vector_ops.hpp"
#include "packing/sparrow/eval/separation_evaluator.hpp"

namespace shiny::nesting::pack::sparrow::sample {

namespace {

[[nodiscard]] auto polygon_centroid(const geom::PolygonWithHoles &polygon)
    -> geom::Point2 {
  const auto bounds = geom::compute_bounds(polygon);
  return geom::Point2{(bounds.min.x() + bounds.max.x()) / 2.0,
                      (bounds.min.y() + bounds.max.y()) / 2.0};
}

[[nodiscard]] auto rotate_about_point(const geom::PolygonWithHoles &polygon,
                                      const geom::Point2 center,
                                      const double degrees)
    -> geom::PolygonWithHoles {
  return geom::translate(
      geom::rotate(
          geom::translate(polygon, geom::Vector2(-center.x(), -center.y())),
          geom::ResolvedRotation{degrees}),
      geom::Vector2(center.x(), center.y()));
}

} // namespace

auto refine_placement(const quantify::CollisionTracker &tracker,
                      const std::size_t moving_index,
                      const SearchPlacementCandidate &initial_candidate,
                      const CoordinateDescentConfig &config)
    -> CoordinateDescentResult {
  CoordinateDescentResult result{.best_candidate = initial_candidate};
  if (config.iteration_budget == 0U) {
    return result;
  }

  std::array axes{geom::Vector2{1.0, 0.0}, geom::Vector2{0.0, 1.0},
                  geom::Vector2{1.0, 1.0}, geom::Vector2{1.0, -1.0}};
  double translation_step = config.translation_step;
  double angle_step = config.angle_step_degrees;

  const auto consider = [&](const geom::PolygonWithHoles &polygon,
                            const geom::Point2 translation,
                            const double rotation_degrees) {
    const auto evaluation = eval::evaluate_separation_candidate(
        tracker, moving_index, adapters::to_port_polygon(polygon),
        result.best_candidate.weighted_loss);
    if (!evaluation.early_terminated &&
        evaluation.weighted_loss + eval::kLossComparisonEpsilon <
            result.best_candidate.weighted_loss) {
      result.best_candidate = {
          .polygon = polygon,
          .translation = translation,
          .rotation_degrees = rotation_degrees,
          .weighted_loss = evaluation.weighted_loss,
          .from_constructive_seed =
              result.best_candidate.from_constructive_seed,
      };
      return true;
    }
    return false;
  };

  for (std::size_t iteration = 0; iteration < config.iteration_budget;
       ++iteration) {
    ++result.iterations_completed;
    const bool translation_active =
        translation_step > 0.0 &&
        translation_step >= config.min_translation_step;
    const bool rotation_active = config.enable_rotation_axis &&
                                 angle_step > 0.0 &&
                                 angle_step >= config.min_angle_step_degrees;
    if (!translation_active && !rotation_active) {
      break;
    }

    bool improved_translation = false;
    if (translation_active) {
      for (const auto &axis : axes) {
        const auto forward_polygon =
            geom::translate(result.best_candidate.polygon,
                            geom::Vector2(axis.x() * translation_step,
                                          axis.y() * translation_step));
        improved_translation =
            consider(forward_polygon,
                     geom::Point2{
                         result.best_candidate.translation.x() +
                             axis.x() * translation_step,
                         result.best_candidate.translation.y() +
                             axis.y() * translation_step,
                     },
                     result.best_candidate.rotation_degrees) ||
            improved_translation;

        const auto backward_polygon =
            geom::translate(result.best_candidate.polygon,
                            geom::Vector2(-axis.x() * translation_step,
                                          -axis.y() * translation_step));
        improved_translation =
            consider(backward_polygon,
                     geom::Point2{
                         result.best_candidate.translation.x() -
                             axis.x() * translation_step,
                         result.best_candidate.translation.y() -
                             axis.y() * translation_step,
                     },
                     result.best_candidate.rotation_degrees) ||
            improved_translation;
      }
      translation_step *= improved_translation ? 1.1 : 0.5;
    }

    bool improved_rotation = false;
    if (rotation_active) {
      const auto centroid = polygon_centroid(result.best_candidate.polygon);
      const auto forward_polygon = rotate_about_point(
          result.best_candidate.polygon, centroid, angle_step);
      improved_rotation =
          consider(forward_polygon, result.best_candidate.translation,
                   result.best_candidate.rotation_degrees + angle_step) ||
          improved_rotation;

      const auto backward_polygon = rotate_about_point(
          result.best_candidate.polygon, centroid, -angle_step);
      improved_rotation =
          consider(backward_polygon, result.best_candidate.translation,
                   result.best_candidate.rotation_degrees - angle_step) ||
          improved_rotation;
      angle_step *= improved_rotation ? 1.1 : 0.5;
    }

    if (iteration + 1U == config.iteration_budget) {
      result.stopped_by_iteration_budget = true;
    }
  }

  return result;
}

} // namespace shiny::nesting::pack::sparrow::sample