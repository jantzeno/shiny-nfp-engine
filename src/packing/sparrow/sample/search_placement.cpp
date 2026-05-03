#include "packing/sparrow/sample/search_placement.hpp"

#include <algorithm>
#include <limits>

#include "geometry/polygon.hpp"
#include "geometry/transforms/transform.hpp"
#include "packing/sparrow/eval/separation_evaluator.hpp"
#include "packing/sparrow/sample/uniform_sampler.hpp"

namespace shiny::nesting::pack::sparrow::sample {

namespace {

[[nodiscard]] auto anchor_candidate(const geom::PolygonWithHoles &base_polygon,
                                    const geom::Point2 target,
                                    const double rotation_degrees)
    -> geom::PolygonWithHoles {
  const auto rotated =
      geom::rotate(base_polygon, geom::ResolvedRotation{rotation_degrees});
  const auto rotated_bounds = geom::compute_bounds(rotated);
  return geom::translate(rotated,
                         geom::Vector2(target.x() - rotated_bounds.min.x(),
                                       target.y() - rotated_bounds.min.y()));
}

[[nodiscard]] auto find_seed_placement(const SeedSolution *seed_solution,
                                       const std::uint32_t piece_id)
    -> const PortPlacement * {
  if (seed_solution == nullptr) {
    return nullptr;
  }
  const auto it = std::find_if(seed_solution->placements.begin(),
                               seed_solution->placements.end(),
                               [piece_id](const PortPlacement &placement) {
                                 return placement.piece_id == piece_id;
                               });
  return it == seed_solution->placements.end() ? nullptr : &*it;
}

} // namespace

auto SearchPlacementPolicy::for_profile(const SolveProfile profile)
    -> SearchPlacementPolicy {
  switch (profile) {
  case SolveProfile::quick:
    return {.profile = profile, .global_samples = 4U, .focused_samples = 2U};
  case SolveProfile::balanced:
    return {.profile = profile, .global_samples = 8U, .focused_samples = 4U};
  case SolveProfile::maximum_search:
    return {.profile = profile, .global_samples = 16U, .focused_samples = 8U};
  }
  return {.profile = profile, .global_samples = 8U, .focused_samples = 4U};
}

auto search_placement(const SearchPlacementRequest &request,
                      runtime::SplitMix64Rng &rng)
    -> std::optional<SearchPlacementResult> {
  if (request.tracker == nullptr ||
      request.moving_index >= request.tracker->item_count()) {
    return std::nullopt;
  }

  const auto &tracker = *request.tracker;
  const auto &current_polygon = tracker.item_polygon(request.moving_index);
  const auto current_bounds = geom::compute_bounds(current_polygon);
  const auto base_polygon =
      geom::translate(current_polygon, geom::Vector2(-current_bounds.min.x(),
                                                     -current_bounds.min.y()));
  const auto bounds = geom::compute_bounds(tracker.container_polygon());
  const auto rotations = request.allowed_rotations_degrees.empty()
                             ? std::span<const double>{}
                             : request.allowed_rotations_degrees;

  SearchPlacementResult result;
  SearchPlacementCandidate best_candidate{
      .polygon = current_polygon,
      .translation = current_bounds.min,
      .weighted_loss = std::numeric_limits<double>::max(),
      .from_constructive_seed = false,
  };

  const auto consider = [&](const geom::Point2 translation,
                            const double rotation_degrees,
                            const bool from_constructive_seed) {
    const auto candidate_polygon =
        anchor_candidate(base_polygon, translation, rotation_degrees);
    const auto evaluation = eval::evaluate_separation_candidate(
        tracker, request.moving_index,
        adapters::to_port_polygon(candidate_polygon),
        best_candidate.weighted_loss);
    if (!evaluation.early_terminated &&
        evaluation.weighted_loss + eval::kLossComparisonEpsilon <
            best_candidate.weighted_loss) {
      best_candidate = {
          .polygon = candidate_polygon,
          .translation = translation,
          .rotation_degrees = rotation_degrees,
          .weighted_loss = evaluation.weighted_loss,
          .from_constructive_seed = from_constructive_seed,
      };
    }
  };

  if (const auto *seed_placement = find_seed_placement(
          request.seed_solution, tracker.item_piece_id(request.moving_index));
      seed_placement != nullptr) {
    consider(seed_placement->translation,
             seed_placement->resolved_rotation.degrees, true);
  }

  const auto global_samples = sample_uniform_placements(
      rng, bounds, request.policy.global_samples, rotations);
  result.sampled_candidates = global_samples.size();
  for (const auto &sample : global_samples) {
    consider(sample.translation, sample.rotation_degrees, false);
  }

  const geom::Point2 focus_center = best_candidate.translation;
  const double width = geom::box_width(bounds);
  const double height = geom::box_height(bounds);
  for (std::size_t index = 0; index < request.policy.focused_samples; ++index) {
    const double x =
        std::clamp(focus_center.x() + (rng.uniform_real() - 0.5) * width * 0.25,
                   bounds.min.x(), bounds.max.x());
    const double y = std::clamp(focus_center.y() +
                                    (rng.uniform_real() - 0.5) * height * 0.25,
                                bounds.min.y(), bounds.max.y());
    const double rotation =
        rotations.empty() ? 0.0
                          : rotations[rng.uniform_index(rotations.size())];
    consider(geom::Point2{x, y}, rotation, false);
  }
  result.focused_candidates = request.policy.focused_samples;

  if (best_candidate.weighted_loss == std::numeric_limits<double>::max()) {
    return std::nullopt;
  }
  result.best_candidate = best_candidate;
  return result;
}

} // namespace shiny::nesting::pack::sparrow::sample