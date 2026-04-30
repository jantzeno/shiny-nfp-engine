#include "packing/item_mover.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <utility>

#include "geometry/polygon.hpp"
#include "geometry/transforms/transform.hpp"
#include "geometry/vector_ops.hpp"
#include "nfp/ifp.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr std::size_t kRegionSampleAttempts = 16U;

[[nodiscard]] auto translate_by_delta(const geom::PolygonWithHoles &polygon,
                                      const geom::Vector2 delta)
    -> geom::PolygonWithHoles {
  return geom::translate(polygon, delta);
}

struct RingCentroid {
  geom::Point2 centroid{};
  double signed_area{0.0};
};

// Centroid is accumulated in `long double` to defer the precision floor
// to the final cast back to `double`. For typical layouts (10⁵ vertices,
// coordinates ~10³) the long-double accumulator keeps cancellation error
// well below the 1e-12 zero-area threshold below; the eventual cast to
// `double` is the only precision floor in this routine.
[[nodiscard]] auto ring_centroid(const geom::Ring &ring)
    -> std::optional<RingCentroid> {
  if (ring.empty()) {
    return std::nullopt;
  }

  long double twice_area = 0.0L;
  long double centroid_x = 0.0L;
  long double centroid_y = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    const auto &current = ring[index];
    const auto &next = ring[next_index];
    const long double edge_cross =
        static_cast<long double>(current.x()) * next.y() -
        static_cast<long double>(next.x()) * current.y();
    twice_area += edge_cross;
    centroid_x +=
        (static_cast<long double>(current.x()) + next.x()) * edge_cross;
    centroid_y +=
        (static_cast<long double>(current.y()) + next.y()) * edge_cross;
  }

  if (std::fabs(static_cast<double>(twice_area)) <= 1e-12) {
    long double average_x = 0.0L;
    long double average_y = 0.0L;
    for (const auto &point : ring) {
      average_x += point.x();
      average_y += point.y();
    }
    const auto scale = 1.0L / static_cast<long double>(ring.size());
    return RingCentroid{
        .centroid = geom::Point2(static_cast<double>(average_x * scale),
                                 static_cast<double>(average_y * scale)),
        .signed_area = 0.0,
    };
  }

  const auto scale = 1.0L / (3.0L * twice_area);
  return RingCentroid{
      .centroid = geom::Point2(static_cast<double>(centroid_x * scale),
                               static_cast<double>(centroid_y * scale)),
      .signed_area = static_cast<double>(twice_area / 2.0L),
  };
}

[[nodiscard]] auto polygon_centroid(const geom::PolygonWithHoles &polygon)
    -> geom::Point2 {
  if (polygon.outer().empty()) {
    return {};
  }

  long double weighted_x = 0.0L;
  long double weighted_y = 0.0L;
  long double total_area = 0.0L;

  if (const auto outer = ring_centroid(polygon.outer()); outer.has_value()) {
    weighted_x +=
        static_cast<long double>(outer->centroid.x()) * outer->signed_area;
    weighted_y +=
        static_cast<long double>(outer->centroid.y()) * outer->signed_area;
    total_area += outer->signed_area;
  }
  for (const auto &hole : polygon.holes()) {
    if (const auto contribution = ring_centroid(hole);
        contribution.has_value()) {
      weighted_x += static_cast<long double>(contribution->centroid.x()) *
                    contribution->signed_area;
      weighted_y += static_cast<long double>(contribution->centroid.y()) *
                    contribution->signed_area;
      total_area += contribution->signed_area;
    }
  }

  if (std::fabs(static_cast<double>(total_area)) > 1e-12) {
    return {static_cast<double>(weighted_x / total_area),
            static_cast<double>(weighted_y / total_area)};
  }

  long double average_x = 0.0L;
  long double average_y = 0.0L;
  for (const auto &point : polygon.outer()) {
    average_x += point.x();
    average_y += point.y();
  }
  const auto scale = 1.0L / static_cast<long double>(polygon.outer().size());
  return {static_cast<double>(average_x * scale),
          static_cast<double>(average_y * scale)};
}

[[nodiscard]] auto rotate_about_point(const geom::PolygonWithHoles &polygon,
                                      const geom::Point2 center,
                                      const double degrees)
    -> geom::PolygonWithHoles {
  return geom::translate(
      geom::rotate(
          geom::translate(polygon, geom::Vector2(-center.x(), -center.y())),
          geom::ResolvedRotation{.degrees = degrees}),
      geom::Vector2(center.x(), center.y()));
}

// Coarse:fine = 1:2 of the descent budget. The coarse pass widens the
// initial step (`coarse_step_ratio`) to escape local basins quickly,
// then the fine pass spends the remaining 2/3 polishing with the smaller
// `fine_step_ratio`. The bias toward fine refinement is empirical:
// Sparrow §5 and our regression set show that placing too few iterations
// in the fine pass leaves residual sub-step overlaps after the coarse
// step has already located the right basin.
[[nodiscard]] auto split_descent_budget(const std::size_t total_iterations)
    -> std::pair<std::size_t, std::size_t> {
  if (total_iterations == 0U) {
    return {0U, 0U};
  }
  if (total_iterations == 1U) {
    return {1U, 0U};
  }

  const auto coarse_iterations =
      std::max<std::size_t>(1U, total_iterations / 3U);
  return {coarse_iterations, total_iterations - coarse_iterations};
}

template <typename CandidateEvaluator>
auto run_descent_pass(const std::span<const geom::Vector2> axes, ItemMove &best,
                      CandidateEvaluator &&evaluate_candidate,
                      const double translation_start_step,
                      const double translation_min_step,
                      const bool enable_rotation_axis,
                      const double angle_start_degrees,
                      const double min_angle_step_degrees,
                      const std::size_t iteration_budget) -> void {
  double translation_step = translation_start_step;
  double angle_step_degrees = angle_start_degrees;
  std::size_t iteration = 0U;

  // Probe a forward/backward axis pair; returns true iff `best` strictly
  // improved (within the same 1e-9 tolerance the candidate evaluator uses).
  const auto try_pair = [&](const geom::PolygonWithHoles &forward,
                            const geom::PolygonWithHoles &backward) -> bool {
    const auto previous_loss = best.weighted_loss;
    evaluate_candidate(forward);
    evaluate_candidate(backward);
    return best.weighted_loss + 1e-9 < previous_loss;
  };

  while (iteration < iteration_budget) {
    const bool translation_active =
        translation_step > 0.0 && translation_step >= translation_min_step;
    const bool rotation_active = enable_rotation_axis &&
                                 angle_step_degrees > 0.0 &&
                                 angle_step_degrees >= min_angle_step_degrees;
    if (!translation_active && !rotation_active) {
      break;
    }

    bool improved_translation = false;
    if (translation_active) {
      for (const auto &axis : axes) {
        const auto basis = best.polygon;
        const auto forward = translate_by_delta(
            basis, geom::Vector2(axis.x() * translation_step,
                                 axis.y() * translation_step));
        const auto backward = translate_by_delta(
            basis, geom::Vector2(-axis.x() * translation_step,
                                 -axis.y() * translation_step));
        if (try_pair(forward, backward)) {
          improved_translation = true;
        }
      }
      translation_step *= improved_translation ? 1.1 : 0.5;
    }

    bool improved_rotation = false;
    if (rotation_active) {
      const auto basis = best.polygon;
      const auto centroid = polygon_centroid(basis);
      const auto forward =
          rotate_about_point(basis, centroid, angle_step_degrees);
      const auto backward =
          rotate_about_point(basis, centroid, -angle_step_degrees);
      if (try_pair(forward, backward)) {
        improved_rotation = true;
      }
      angle_step_degrees *= improved_rotation ? 1.1 : 0.5;
    }

    ++iteration;
  }
}

[[nodiscard]] auto sample_point_in_region(const geom::PolygonWithHoles &region,
                                          runtime::DeterministicRng &rng)
    -> std::optional<geom::Point2> {
  if (region.outer().empty()) {
    return std::nullopt;
  }

  const auto bounds = geom::compute_bounds(region);
  for (std::size_t attempt = 0; attempt < kRegionSampleAttempts; ++attempt) {
    const geom::Point2 sample(
        bounds.min.x() + rng.uniform_real() * geom::box_width(bounds),
        bounds.min.y() + rng.uniform_real() * geom::box_height(bounds));
    if (pred::locate_point_in_polygon(sample, region).location !=
        pred::PointLocation::exterior) {
      return sample;
    }
  }

  for (const auto &vertex : region.outer()) {
    if (pred::locate_point_in_polygon(vertex, region).location !=
        pred::PointLocation::exterior) {
      return vertex;
    }
  }

  return std::nullopt;
}

[[nodiscard]] auto
sample_target_from_regions(const std::vector<geom::PolygonWithHoles> &regions,
                           runtime::DeterministicRng &rng)
    -> std::optional<geom::Point2> {
  if (regions.empty()) {
    return std::nullopt;
  }

  double total_area = 0.0;
  for (const auto &region : regions) {
    total_area += geom::polygon_area(region);
  }

  if (total_area <= 0.0) {
    for (const auto &region : regions) {
      if (const auto sample = sample_point_in_region(region, rng);
          sample.has_value()) {
        return sample;
      }
    }
    return std::nullopt;
  }

  double remaining = rng.uniform_real() * total_area;
  for (const auto &region : regions) {
    remaining -= geom::polygon_area(region);
    if (remaining > 0.0) {
      continue;
    }
    if (const auto sample = sample_point_in_region(region, rng);
        sample.has_value()) {
      return sample;
    }
  }

  return sample_point_in_region(regions.back(), rng);
}

} // namespace

// Move a single colliding item to a lower-loss position.
//
// Search budget (per call):
//   1. `global_samples` random anchor placements inside the piece's
//      inner-fit region — coarse exploration that respects general
//      container geometry, not just the container bbox.
//   2. For each currently-colliding pair, generate `focused_samples`
//      candidates moving away from the collision partner along the
//      vector between bbox centres. Step size scales by container
//      diameter × `coarse_step_ratio`.
//   3. Coordinate descent in two passes:
//      * coarse translation refinement `coarse_step_ratio →
//      coarse_min_step_ratio`
//      * fine translation refinement `fine_step_ratio → min_step_ratio`
//      Both passes probe H, V, +diag, −diag and, when enabled, a
//      centroid-preserving rotation axis (±`angle_step_degrees`).
//      Translation and rotation keep independent ×1.1 / ×0.5 adaptive
//      schedules so one axis family cannot inflate the other's step size.
//   4. Diagonal axes are intentionally NOT unit-normalised — see the
//      comment at the descent loop for the rationale (√2× reach bias
//      enables overlap resolution within the fixed iteration budget).
//
// Returns `nullopt` if no candidate strictly improves the current loss
// (within 1e-9 tolerance) — caller should leave the item alone.
auto move_item(const CollisionTracker &tracker, const std::size_t item_index,
               const ItemMoverConfig &config, runtime::DeterministicRng &rng)
    -> std::optional<ItemMove> {
  const auto &current_item = tracker.item(item_index);
  const auto current_bounds = geom::compute_bounds(current_item.polygon);
  const auto container_bounds = geom::compute_bounds(tracker.container());
  const auto local_piece = geom::translate(
      current_item.polygon,
      geom::Vector2(-current_bounds.min.x(), -current_bounds.min.y()));
  auto move_regions_or =
      nfp::compute_inner_fit_polygon(tracker.container(), local_piece);
  if (!move_regions_or.ok()) {
    return std::nullopt;
  }
  const auto &move_regions = move_regions_or.value();
  if (move_regions.empty()) {
    return std::nullopt;
  }
  const auto current_loss =
      evaluate_sample(tracker, item_index, current_item.polygon,
                      std::numeric_limits<double>::max())
          .weighted_loss;

  ItemMove best{.polygon = current_item.polygon, .weighted_loss = current_loss};

  const auto evaluate_candidate = [&](const geom::PolygonWithHoles &candidate) {
    const auto evaluation =
        evaluate_sample(tracker, item_index, candidate, best.weighted_loss);
    if (!evaluation.early_terminated &&
        evaluation.weighted_loss + 1e-9 < best.weighted_loss) {
      best = {.polygon = candidate, .weighted_loss = evaluation.weighted_loss};
    }
  };

  for (std::size_t sample = 0; sample < config.global_samples; ++sample) {
    const auto target = sample_target_from_regions(move_regions, rng);
    if (!target.has_value()) {
      break;
    }
    evaluate_candidate(translate_by_delta(
        current_item.polygon,
        geom::Vector2(target->x() - current_bounds.min.x(),
                      target->y() - current_bounds.min.y())));
  }

  const auto current_center =
      geom::Point2((current_bounds.min.x() + current_bounds.max.x()) / 2.0,
                   (current_bounds.min.y() + current_bounds.max.y()) / 2.0);
  for (std::size_t other = 0; other < tracker.item_count(); ++other) {
    if (other == item_index) {
      continue;
    }
    if (tracker.pair_loss(item_index, other) <= 0.0) {
      continue;
    }
    const auto other_bounds = geom::compute_bounds(tracker.item(other).polygon);
    const auto other_center =
        geom::Point2((other_bounds.min.x() + other_bounds.max.x()) / 2.0,
                     (other_bounds.min.y() + other_bounds.max.y()) / 2.0);
    const auto direction = geom::normalize_vector(
        geom::vector_between(other_center, current_center));
    const auto resolved_direction =
        direction == geom::Vector2{} ? geom::Vector2(1.0, 0.0) : direction;
    const auto step = std::max(geom::box_width(container_bounds),
                               geom::box_height(container_bounds)) *
                      config.coarse_step_ratio;
    for (std::size_t sample = 0; sample < config.focused_samples; ++sample) {
      const auto scale =
          step * (0.5 + static_cast<double>(sample) / config.focused_samples);
      evaluate_candidate(translate_by_delta(
          current_item.polygon, geom::scale_vector(resolved_direction, scale)));
    }
  }

  // Coordinate descent axes. Diagonals are intentionally NOT
  // unit-normalised — leaving them as {±1, ±1} gives diagonal probes a
  // √2× larger reach than H/V at the same `step`. This biases the
  // descent toward diagonal exploration and, empirically, lets the
  // limited iteration budget resolve overlaps that span ~½ the
  // container diameter even when the fine pass is already small. If
  // this were ever changed to unit-normalised diagonals, the step
  // ratios and/or descent budget would need to rise to compensate.
  // TODO(item-mover): make this axis set configurable via
  // ItemMoverConfig — an OCP improvement for callers that want, e.g.,
  // additional 30°/60° axes or container-aligned anisotropic probes.
  std::array<geom::Vector2, 4> axes{{
      geom::Vector2(1.0, 0.0),
      geom::Vector2(0.0, 1.0),
      geom::Vector2(1.0, 1.0),
      geom::Vector2(1.0, -1.0),
  }};
  const auto container_diameter = std::max(geom::box_width(container_bounds),
                                           geom::box_height(container_bounds));
  const auto [coarse_iterations, fine_iterations] =
      split_descent_budget(config.coordinate_descent_iterations);
  const bool rotation_enabled =
      config.enable_rotation_axis && !current_item.rotation_locked &&
      config.angle_step_degrees > 0.0 && config.min_angle_step_degrees > 0.0;
  run_descent_pass(axes, best, evaluate_candidate,
                   container_diameter * config.coarse_step_ratio,
                   container_diameter * config.coarse_min_step_ratio,
                   rotation_enabled, config.angle_step_degrees,
                   config.min_angle_step_degrees, coarse_iterations);
  run_descent_pass(axes, best, evaluate_candidate,
                   container_diameter * config.fine_step_ratio,
                   container_diameter * config.min_step_ratio, rotation_enabled,
                   config.angle_step_degrees, config.min_angle_step_degrees,
                   fine_iterations);

  if (best.weighted_loss + 1e-9 < current_loss) {
    return best;
  }
  return std::nullopt;
}

} // namespace shiny::nesting::pack
