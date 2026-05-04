#include "packing/irregular/core.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "geometry/transforms/transform.hpp"
#include "logging/shiny_log.hpp"
#include "nfp/ifp.hpp"
#include "packing/common.hpp"
#include "packing/irregular/candidate_generation.hpp"

namespace shiny::nesting::pack::detail {
namespace {

[[nodiscard]] auto
bounds_corner_for_start_corner(const geom::Box2 &bounds,
                               const place::PlacementStartCorner start_corner)
    -> geom::Point2 {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
    return bounds.min;
  case place::PlacementStartCorner::bottom_right:
    return geom::Point2{bounds.max.x(), bounds.min.y()};
  case place::PlacementStartCorner::top_left:
    return geom::Point2{bounds.min.x(), bounds.max.y()};
  case place::PlacementStartCorner::top_right:
    return bounds.max;
  }
  return bounds.min;
}

auto append_unique_anchor(std::vector<AnchorPoint> &anchors,
                          const AnchorPoint &candidate) -> void {
  const auto duplicate = std::find_if(
      anchors.begin(), anchors.end(), [&](const AnchorPoint &existing) {
        return almost_equal(existing.point.x(), candidate.point.x()) &&
               almost_equal(existing.point.y(), candidate.point.y()) &&
               existing.source == candidate.source;
      });
  if (duplicate == anchors.end()) {
    anchors.push_back(candidate);
  }
}

auto append_ring_anchors(std::vector<AnchorPoint> &anchors,
                         const std::span<const geom::Point2> ring,
                         const place::PlacementCandidateSource source) -> void {
  for (const auto &point : ring) {
    append_unique_anchor(anchors, {.point = point, .source = source});
  }
}

auto append_box_anchors(std::vector<AnchorPoint> &anchors,
                        const geom::Box2 &box,
                        const place::PlacementCandidateSource source) -> void {
  append_unique_anchor(anchors, {.point = box.min, .source = source});
  append_unique_anchor(
      anchors,
      {.point = geom::Point2{box.max.x(), box.min.y()}, .source = source});
  append_unique_anchor(anchors, {.point = box.max, .source = source});
  append_unique_anchor(
      anchors,
      {.point = geom::Point2{box.min.x(), box.max.y()}, .source = source});
}

[[nodiscard]] auto
start_corner_on_right(const place::PlacementStartCorner start_corner) -> bool {
  return start_corner == place::PlacementStartCorner::bottom_right ||
         start_corner == place::PlacementStartCorner::top_right;
}

[[nodiscard]] auto
start_corner_on_top(const place::PlacementStartCorner start_corner) -> bool {
  return start_corner == place::PlacementStartCorner::top_left ||
         start_corner == place::PlacementStartCorner::top_right;
}

[[nodiscard]] auto
point_for_start_corner(const geom::Point2 &point, const geom::Box2 &container,
                       const place::PlacementStartCorner start_corner)
    -> geom::Point2 {
  return geom::Point2{
      start_corner_on_right(start_corner)
          ? container.min.x() + (container.max.x() - point.x())
          : point.x(),
      start_corner_on_top(start_corner)
          ? container.min.y() + (container.max.y() - point.y())
          : point.y(),
  };
}

[[nodiscard]] auto
box_for_start_corner(const geom::Box2 &box, const geom::Box2 &container,
                     const place::PlacementStartCorner start_corner)
    -> geom::Box2 {
  return pack::normalize_box(
      point_for_start_corner(box.min, container, start_corner),
      point_for_start_corner(box.max, container, start_corner));
}

[[nodiscard]] auto unique_sorted_values(std::vector<double> values)
    -> std::vector<double> {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end(),
                           [](const double lhs, const double rhs) {
                             return almost_equal(lhs, rhs);
                           }),
               values.end());
  return values;
}

[[nodiscard]] auto collect_anchors(const WorkingBin &bin,
                                   const double part_spacing)
    -> std::vector<AnchorPoint> {
  std::vector<AnchorPoint> anchors;
  append_ring_anchors(anchors, bin.state.container.outer(),
                      place::PlacementCandidateSource::bin_boundary);
  append_box_anchors(anchors, geom::compute_bounds(bin.state.container),
                     place::PlacementCandidateSource::bin_boundary);
  for (const auto &hole : bin.state.container.holes()) {
    append_ring_anchors(anchors, hole,
                        place::PlacementCandidateSource::constructive_boundary);
  }

  for (const auto &region : bin.cached_free_regions) {
    append_ring_anchors(anchors, region.outer(),
                        place::PlacementCandidateSource::constructive_boundary);
    append_box_anchors(anchors, geom::compute_bounds(region),
                       place::PlacementCandidateSource::constructive_boundary);
    for (const auto &hole : region.holes()) {
      append_ring_anchors(
          anchors, hole,
          place::PlacementCandidateSource::constructive_boundary);
    }
  }

  if (part_spacing <= 0.0) {
    for (const auto &zone : bin.exclusion_regions) {
      append_ring_anchors(
          anchors, zone.outer(),
          place::PlacementCandidateSource::constructive_boundary);
    }
    for (const auto &placement : bin.state.placements) {
      append_ring_anchors(
          anchors, placement.polygon.outer(),
          place::PlacementCandidateSource::constructive_boundary);
      append_box_anchors(
          anchors, geom::compute_bounds(placement.polygon),
          place::PlacementCandidateSource::constructive_boundary);
    }
  }

  for (const auto &hole : bin.state.holes) {
    append_ring_anchors(anchors, hole.outer(),
                        place::PlacementCandidateSource::hole_boundary);
  }
  return anchors;
}

[[nodiscard]] auto
candidate_reference_points(const geom::PolygonWithHoles &polygon,
                           const geom::Box2 &bounds,
                           const place::PlacementStartCorner start_corner)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> points(polygon.outer().begin(),
                                   polygon.outer().end());
  points.push_back(bounds.min);
  points.emplace_back(bounds.max.x(), bounds.min.y());
  points.push_back(bounds.max);
  points.emplace_back(bounds.min.x(), bounds.max.y());
  points.push_back(bounds_corner_for_start_corner(bounds, start_corner));
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  return points;
}

[[nodiscard]] auto translation_points_from_anchors(
    const std::span<const AnchorPoint> anchors,
    const std::span<const geom::Point2> reference_points)
    -> std::vector<GeneratedCandidatePoint> {
  std::vector<GeneratedCandidatePoint> translations;
  translations.reserve(anchors.size() * reference_points.size());
  for (const auto &anchor : anchors) {
    for (const auto &reference_point : reference_points) {
      translations.push_back({
          .translation = geom::Point2{anchor.point.x() - reference_point.x(),
                                      anchor.point.y() - reference_point.y()},
          .source = anchor.source,
      });
    }
  }
  return translations;
}

[[nodiscard]] auto
skyline_candidate_min_xs(const geom::Box2 &container_bounds,
                         const std::span<const geom::Box2> occupied_bounds,
                         const double width) -> std::vector<double> {
  std::vector<double> candidates{
      container_bounds.min.x(),
      container_bounds.max.x() - width,
  };
  candidates.reserve(2U + occupied_bounds.size() * 4U);
  for (const auto &occupied : occupied_bounds) {
    candidates.push_back(occupied.min.x());
    candidates.push_back(occupied.max.x());
    candidates.push_back(occupied.min.x() - width);
    candidates.push_back(occupied.max.x() - width);
  }

  auto unique = unique_sorted_values(std::move(candidates));
  unique.erase(std::remove_if(unique.begin(), unique.end(),
                              [&](const double value) {
                                return value < container_bounds.min.x() -
                                                   kDistanceEpsilon ||
                                       value + width >
                                           container_bounds.max.x() +
                                               kDistanceEpsilon;
                              }),
               unique.end());
  return unique;
}

[[nodiscard]] auto translation_points_from_bbox_skyline(
    const WorkingBin &bin, const geom::Box2 &rotated_bounds,
    const double part_spacing) -> std::vector<GeneratedCandidatePoint> {
  const auto container_bounds = geom::compute_bounds(bin.state.container);
  const auto start_corner = bin.state.start_corner;
  const auto canonical_container =
      box_for_start_corner(container_bounds, container_bounds, start_corner);
  std::vector<geom::Box2> canonical_occupied_bounds;
  canonical_occupied_bounds.reserve(bin.placement_bounds.size());
  for (const auto &occupied : bin.placement_bounds) {
    canonical_occupied_bounds.push_back(box_for_start_corner(
        pack::spacing_reservation_bounds(occupied, part_spacing),
        container_bounds, start_corner));
  }

  const double width = geom::box_width(rotated_bounds);
  const double height = geom::box_height(rotated_bounds);
  std::vector<GeneratedCandidatePoint> translations;
  const auto candidate_xs = skyline_candidate_min_xs(
      canonical_container, canonical_occupied_bounds, width);
  translations.reserve(candidate_xs.size());

  for (const double candidate_min_x : candidate_xs) {
    double candidate_min_y = canonical_container.min.y();
    for (const auto &occupied : canonical_occupied_bounds) {
      if (!pack::intervals_overlap_interior(
              candidate_min_x, candidate_min_x + width, occupied.min.x(),
              occupied.max.x())) {
        continue;
      }
      candidate_min_y = std::max(candidate_min_y, occupied.max.y());
    }

    const geom::Box2 canonical_candidate{
        .min = geom::Point2{candidate_min_x, candidate_min_y},
        .max = geom::Point2{candidate_min_x + width, candidate_min_y + height},
    };
    const auto actual_candidate = box_for_start_corner(
        canonical_candidate, container_bounds, start_corner);
    if (!geom::box_contains(container_bounds, actual_candidate)) {
      continue;
    }
    translations.push_back({
        .translation =
            geom::Point2{actual_candidate.min.x() - rotated_bounds.min.x(),
                         actual_candidate.min.y() - rotated_bounds.min.y()},
        .source = place::PlacementCandidateSource::constructive_boundary,
    });
  }
  return translations;
}

[[nodiscard]] auto obstacle_candidates_for(const WorkingBin &bin)
    -> std::vector<CandidateGenerationObstacle> {
  std::vector<CandidateGenerationObstacle> obstacles;
  obstacles.reserve(bin.state.placements.size());
  for (const auto &placement : bin.state.placements) {
    obstacles.push_back({
        .geometry_revision = placement.piece_geometry_revision,
        .polygon = placement.polygon,
        .translation = geom::Vector2{placement.placement.translation.x(),
                                     placement.placement.translation.y()},
        .rotation = placement.resolved_rotation,
    });
  }
  return obstacles;
}

struct PlacementSearchDiagnostics {
  std::size_t rotations_tried{0};
  std::size_t candidates_before_limit{0};
  std::size_t candidates_after_limit{0};
  std::size_t rejected_by_containment{0};
  std::size_t rejected_by_overlap{0};
  std::size_t rejected_by_spacing{0};
  std::size_t nfp_generation_failures{0};
  std::size_t nfp_exact_cache_hits{0};
  std::size_t nfp_fallback_cache_hits{0};
  std::size_t nfp_exact_computations{0};
  std::size_t nfp_bbox_fallbacks{0};
  std::size_t fallback_candidates{0};
  const char *best_rejected_reason{"none"};
  double best_rejected_score{std::numeric_limits<double>::lowest()};
};

[[nodiscard]] auto bbox_overlaps_existing_placement(
    const geom::Box2 &candidate_bbox,
    const std::span<const geom::Box2> placement_bounds) -> bool {
  return std::any_of(placement_bounds.begin(), placement_bounds.end(),
                     [&](const auto &placed_bbox) {
                       return pack::boxes_overlap(candidate_bbox, placed_bbox);
                     });
}

auto note_rejected_candidate(PlacementSearchDiagnostics &diagnostics,
                             const char *reason, const double score) -> void {
  if (score > diagnostics.best_rejected_score) {
    diagnostics.best_rejected_score = score;
    diagnostics.best_rejected_reason = reason;
  }
}

[[nodiscard]] auto bbox_can_contain_shape(const geom::Box2 &container_bbox,
                                          const geom::Box2 &piece_bbox)
    -> bool {
  return geom::box_width(container_bbox) + kDistanceEpsilon >=
             geom::box_width(piece_bbox) &&
         geom::box_height(container_bbox) + kDistanceEpsilon >=
             geom::box_height(piece_bbox);
}

[[nodiscard]] auto
region_can_fit_piece_exact(const geom::PolygonWithHoles &region,
                           const geom::PolygonWithHoles &piece)
    -> std::optional<bool> {
  auto ifp = nfp::compute_ifp(region, piece);
  if (!ifp.has_value()) {
    return std::nullopt;
  }
  return !ifp.value().empty();
}

auto try_exact_fit_candidate_impl(WorkingBin &bin, const PieceInstance &piece,
                                  const NormalizedRequest &request,
                                  const SolveControl &control,
                                  PlacementAttemptContext &attempt_context)
    -> PlacementSearchResult {
  ensure_free_regions_cached(bin, request.request.execution);
  const auto &free_regions = bin.cached_free_regions;
  const auto &region_bboxes = bin.cached_region_bboxes;
  const auto &rotations =
      allowed_rotations_for(*piece.source, request.request.execution);
  const auto rotation_count = geom::rotation_count(rotations);
  const auto rotation_begin = piece.forced_rotation_index.has_value()
                                  ? piece.forced_rotation_index->value
                                  : 0U;
  const auto rotation_end =
      piece.forced_rotation_index.has_value()
          ? std::min<std::size_t>(rotation_count, rotation_begin + 1U)
          : rotation_count;

  std::optional<CandidatePlacement> best;
  const auto container_bounds = geom::compute_bounds(bin.state.container);

  for (std::size_t rotation_value = rotation_begin;
       rotation_value < rotation_end; ++rotation_value) {
    if (interrupted(control)) {
      return {.status = PlacementSearchStatus::interrupted};
    }

    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        geom::resolve_rotation(rotation_index, rotations);
    if (!resolved_rotation.has_value()) {
      continue;
    }

    const auto mirror_count = piece.source->allow_mirror ? 2U : 1U;
    for (std::size_t mirror_index = 0; mirror_index < mirror_count;
         ++mirror_index) {
      const bool mirrored = mirror_index == 1U;
      const auto source_polygon = mirrored ? geom::mirror(piece.source->polygon)
                                           : piece.source->polygon;
      const auto refined_angles =
          rotations.range_degrees.has_value()
              ? geom::local_refinement_angles(*rotations.range_degrees,
                                              resolved_rotation->degrees)
              : std::vector<double>{resolved_rotation->degrees};
      const auto mirrored_revision = effective_geometry_revision(
          piece.source->geometry_revision, mirrored);

      for (const auto refined_angle : refined_angles) {
        const geom::ResolvedRotation active_rotation{.degrees = refined_angle};
        const auto rotated_piece =
            geom::rotate(source_polygon, active_rotation);
        if (rotated_piece.outer().empty()) {
          continue;
        }

        const auto rotated_bounds = geom::compute_bounds(rotated_piece);
        for (std::size_t region_index = 0; region_index < free_regions.size();
             ++region_index) {
          if (!bbox_can_contain_shape(region_bboxes[region_index],
                                      rotated_bounds)) {
            continue;
          }

          auto ifp =
              nfp::compute_ifp(free_regions[region_index], rotated_piece);
          if (!ifp.has_value()) {
            continue;
          }

          for (const auto &feasible_region : ifp.value()) {
            geom::for_each_vertex(feasible_region, [&](const geom::Point2
                                                           &translation) {
              ++attempt_context.candidate_evaluations_completed;

              const geom::Box2 candidate_bbox{
                  .min = geom::Point2{rotated_bounds.min.x() + translation.x(),
                                      rotated_bounds.min.y() + translation.y()},
                  .max = geom::Point2{rotated_bounds.max.x() + translation.x(),
                                      rotated_bounds.max.y() + translation.y()},
              };
              const bool any_bbox_fit =
                  request.request.execution.irregular
                          .enable_direct_overlap_check
                      ? geom::box_contains(container_bounds, candidate_bbox)
                      : std::any_of(region_bboxes.begin(), region_bboxes.end(),
                                    [&](const auto &region_bbox) {
                                      return geom::box_contains(region_bbox,
                                                                candidate_bbox);
                                    });
              if (!any_bbox_fit) {
                return;
              }

              const geom::Vector2 translation_vector{translation.x(),
                                                     translation.y()};
              const auto transformed_piece =
                  geom::translate(rotated_piece, translation_vector);
              const bool fits =
                  request.request.execution.irregular
                          .enable_direct_overlap_check
                      ? fits_bin_direct(transformed_piece, candidate_bbox, bin,
                                        request.request.execution)
                      : fits_any_region(transformed_piece, candidate_bbox,
                                        free_regions, region_bboxes);
              if (!fits) {
                return;
              }
              if (!respects_spacing(transformed_piece, bin,
                                    request.request.execution)) {
                return;
              }

              const auto hole_index = hole_index_for_candidate(
                  transformed_piece, candidate_bbox, bin.state.holes);
              CandidatePlacement candidate{
                  .placement =
                      {
                          .piece_id = piece.expanded.expanded_piece_id,
                          .bin_id = bin.state.bin_id,
                          .rotation_index = rotation_index,
                          .translation =
                              geom::Point2{translation.x(), translation.y()},
                          .mirrored = mirrored,
                      },
                  .piece_geometry_revision = mirrored_revision,
                  .resolved_rotation = active_rotation,
                  .polygon = transformed_piece,
                  .bounds = candidate_bbox,
                  .source = place::PlacementCandidateSource::perfect_fit,
                  .nfp_accuracy = cache::NfpCacheAccuracy::exact,
                  .inside_hole = hole_index >= 0,
                  .hole_index = hole_index,
                  .score = compute_candidate_score(
                      bin, request.request.execution, transformed_piece,
                      candidate_bbox),
              };

              if (!best.has_value() ||
                  better_candidate(bin,
                                   request.request.execution.placement_policy,
                                   candidate, *best)) {
                best = std::move(candidate);
              }
            });
          }
        }
      }
    }
  }

  if (!best.has_value()) {
    return {.status = PlacementSearchStatus::no_candidate};
  }

  return {.status = PlacementSearchStatus::found, .candidate = std::move(best)};
}

} // namespace

auto try_exact_fit_candidate(WorkingBin &bin, const PieceInstance &piece,
                             const NormalizedRequest &request,
                             const SolveControl &control,
                             PlacementAttemptContext &attempt_context)
    -> PlacementSearchResult {
  return try_exact_fit_candidate_impl(bin, piece, request, control,
                                      attempt_context);
}

auto frontier_exhaustion_status_for_piece(
    WorkingBin &bin, const PieceInstance &piece,
    const ExecutionPolicy &execution, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch) -> FrontierExhaustionStatus {
  if (!piece_allows_bin(piece, bin.state.bin_id)) {
    return FrontierExhaustionStatus::exhausted;
  }

  ensure_free_regions_cached(bin, execution);
  const auto &free_regions = bin.cached_free_regions;
  const auto &region_bboxes = bin.cached_region_bboxes;
  if (free_regions.empty()) {
    return FrontierExhaustionStatus::exhausted;
  }

  const auto &rotations = allowed_rotations_for(*piece.source, execution);
  const auto rotation_count = geom::rotation_count(rotations);
  const auto rotation_begin = piece.forced_rotation_index.has_value()
                                  ? piece.forced_rotation_index->value
                                  : 0U;
  const auto rotation_end =
      piece.forced_rotation_index.has_value()
          ? std::min<std::size_t>(rotation_count, rotation_begin + 1U)
          : rotation_count;

  for (std::size_t rotation_value = rotation_begin;
       rotation_value < rotation_end; ++rotation_value) {
    if (time_budget.expired(stopwatch)) {
      return FrontierExhaustionStatus::fit_may_exist;
    }
    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        geom::resolve_rotation(rotation_index, rotations);
    if (!resolved_rotation.has_value()) {
      continue;
    }

    const auto mirror_count = piece.source->allow_mirror ? 2U : 1U;
    for (std::size_t mirror_index = 0; mirror_index < mirror_count;
         ++mirror_index) {
      const bool mirrored = mirror_index == 1U;
      const auto source_polygon = mirrored ? geom::mirror(piece.source->polygon)
                                           : piece.source->polygon;
      const auto refined_angles =
          rotations.range_degrees.has_value()
              ? geom::local_refinement_angles(*rotations.range_degrees,
                                              resolved_rotation->degrees)
              : std::vector<double>{resolved_rotation->degrees};

      for (const auto refined_angle : refined_angles) {
        const auto rotated_piece = geom::rotate(
            source_polygon, geom::ResolvedRotation{.degrees = refined_angle});
        if (rotated_piece.outer().empty()) {
          continue;
        }

        const auto rotated_bbox = geom::compute_bounds(rotated_piece);
        for (std::size_t region_index = 0; region_index < free_regions.size();
             ++region_index) {
          if (!bbox_can_contain_shape(region_bboxes[region_index],
                                      rotated_bbox)) {
            continue;
          }

          const auto exact_fit = region_can_fit_piece_exact(
              free_regions[region_index], rotated_piece);
          if (!exact_fit.has_value()) {
            SHINY_DEBUG("frontier exhaustion probe: ifp failed piece={} bin={} "
                        "rotation={} mirrored={}",
                        piece.expanded.expanded_piece_id, bin.state.bin_id,
                        rotation_value, mirrored);
            return FrontierExhaustionStatus::fit_may_exist;
          }
          if (*exact_fit) {
            return FrontierExhaustionStatus::fit_may_exist;
          }
        }
      }
    }
  }

  return FrontierExhaustionStatus::exhausted;
}

auto find_best_for_bin(
    WorkingBin &bin, const PieceInstance &piece,
    const NormalizedRequest &request, const SolveControl &control,
    const runtime::TimeBudget &time_budget, const runtime::Stopwatch &stopwatch,
    ProgressThrottle &search_throttle, const std::size_t placed_parts,
    const std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, PackerSearchMetrics *search_metrics,
    runtime::DeterministicRng *rng) -> PlacementSearchResult {
  if (!piece_allows_bin(piece, bin.state.bin_id)) {
    return {.status = PlacementSearchStatus::no_candidate};
  }

  ensure_free_regions_cached(bin, request.request.execution);
  const auto &free_regions = bin.cached_free_regions;
  if (free_regions.empty()) {
    return {.status = PlacementSearchStatus::no_candidate};
  }
  const auto &region_bboxes = bin.cached_region_bboxes;

  const auto strategy = request.request.execution.irregular.candidate_strategy;
  const bool use_anchor_candidates =
      strategy == CandidateStrategy::anchor_vertex ||
      strategy == CandidateStrategy::nfp_hybrid;
  const bool use_nfp_candidates = strategy != CandidateStrategy::anchor_vertex;
  const auto anchors =
      use_anchor_candidates
          ? collect_anchors(bin, request.request.execution.part_spacing)
          : std::vector<AnchorPoint>{};
  const auto &rotations =
      allowed_rotations_for(*piece.source, request.request.execution);
  const auto rotation_count = geom::rotation_count(rotations);
  const auto rotation_begin = piece.forced_rotation_index.has_value()
                                  ? piece.forced_rotation_index->value
                                  : 0U;
  const auto rotation_end =
      piece.forced_rotation_index.has_value()
          ? std::min<std::size_t>(rotation_count, rotation_begin + 1U)
          : rotation_count;

  std::optional<CandidatePlacement> best;
  std::vector<CandidatePlacement> top_k;
  std::size_t local_eval_count = 0;
  bool search_done = false;
  PlacementSearchStatus stop_status = PlacementSearchStatus::no_candidate;
  PlacementSearchDiagnostics diagnostics;

  for (std::size_t rotation_value = rotation_begin;
       rotation_value < rotation_end && !search_done; ++rotation_value) {
    if (interrupted(control) || time_budget.expired(stopwatch)) {
      stop_status = PlacementSearchStatus::interrupted;
      break;
    }

    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        geom::resolve_rotation(rotation_index, rotations);
    if (!resolved_rotation.has_value()) {
      continue;
    }
    ++diagnostics.rotations_tried;

    const auto mirror_count = piece.source->allow_mirror ? 2U : 1U;
    for (std::size_t mirror_index = 0;
         mirror_index < mirror_count && !search_done; ++mirror_index) {
      const bool mirrored = mirror_index == 1U;
      const auto source_polygon = mirrored ? geom::mirror(piece.source->polygon)
                                           : piece.source->polygon;
      const auto refined_angles =
          rotations.range_degrees.has_value()
              ? geom::local_refinement_angles(*rotations.range_degrees,
                                              resolved_rotation->degrees)
              : std::vector<double>{resolved_rotation->degrees};
      const auto mirrored_revision = effective_geometry_revision(
          piece.source->geometry_revision, mirrored);
      const auto container_bounds = geom::compute_bounds(bin.state.container);

      for (const auto refined_angle : refined_angles) {
        const geom::ResolvedRotation active_rotation{.degrees = refined_angle};
        const auto rotated_piece =
            geom::rotate(source_polygon, active_rotation);
        if (rotated_piece.outer().empty()) {
          continue;
        }

        const auto rotated_bounds = geom::compute_bounds(rotated_piece);
        const auto reference_points = candidate_reference_points(
            rotated_piece, rotated_bounds, bin.state.start_corner);

        std::vector<GeneratedCandidatePoint> candidate_points;
        if (use_anchor_candidates) {
          auto anchor_points =
              translation_points_from_anchors(anchors, reference_points);
          candidate_points.insert(candidate_points.end(), anchor_points.begin(),
                                  anchor_points.end());
          auto skyline_points = translation_points_from_bbox_skyline(
              bin, rotated_bounds, request.request.execution.part_spacing);
          candidate_points.insert(candidate_points.end(),
                                  skyline_points.begin(), skyline_points.end());
        }
        if (use_nfp_candidates) {
          const auto obstacles = obstacle_candidates_for(bin);
          try {
            CandidateGenerationDiagnostics candidate_generation_diagnostics{};
            auto nfp_points = generate_nfp_candidate_points(
                bin.state.container, bin.exclusion_regions, obstacles,
                rotated_piece, mirrored_revision, active_rotation, strategy,
                nfp_cache, &candidate_generation_diagnostics);
            if (nfp_points.has_value()) {
              if (search_metrics != nullptr) {
                search_metrics->record(candidate_generation_diagnostics);
              }
              diagnostics.nfp_exact_cache_hits +=
                  candidate_generation_diagnostics.exact_nfp_cache_hits;
              diagnostics.nfp_fallback_cache_hits +=
                  candidate_generation_diagnostics.conservative_bbox_cache_hits;
              diagnostics.nfp_exact_computations +=
                  candidate_generation_diagnostics.exact_nfp_computations;
              diagnostics.nfp_bbox_fallbacks +=
                  candidate_generation_diagnostics.conservative_bbox_fallbacks;
              diagnostics.fallback_candidates +=
                  candidate_generation_diagnostics
                      .conservative_bbox_fallback_candidates;
              auto points = std::move(nfp_points).value();
              candidate_points.insert(candidate_points.end(), points.begin(),
                                      points.end());
            } else {
              ++diagnostics.nfp_generation_failures;
              SHINY_DEBUG("placement_search: nfp candidate generation failed "
                          "piece={} bin={} rotation={} strategy={} status={}",
                          piece.expanded.expanded_piece_id, bin.state.bin_id,
                          rotation_value, static_cast<int>(strategy),
                          static_cast<int>(nfp_points.error()));
            }
          } catch (const std::exception &ex) {
            ++diagnostics.nfp_generation_failures;
            SHINY_DEBUG("placement_search: nfp candidate generation threw "
                        "piece={} bin={} rotation={} strategy={} error={}",
                        piece.expanded.expanded_piece_id, bin.state.bin_id,
                        rotation_value, static_cast<int>(strategy), ex.what());
          }
        }
        diagnostics.candidates_before_limit += candidate_points.size();
        std::vector<GeneratedCandidatePoint> bbox_clear_candidate_points;
        std::vector<GeneratedCandidatePoint> bbox_overlap_candidate_points;
        bbox_clear_candidate_points.reserve(candidate_points.size());
        bbox_overlap_candidate_points.reserve(candidate_points.size());
        for (const auto &candidate_point : candidate_points) {
          const geom::Vector2 translation{candidate_point.translation.x(),
                                          candidate_point.translation.y()};
          const geom::Box2 candidate_bbox{
              .min = geom::Point2{rotated_bounds.min.x() + translation.x(),
                                  rotated_bounds.min.y() + translation.y()},
              .max = geom::Point2{rotated_bounds.max.x() + translation.x(),
                                  rotated_bounds.max.y() + translation.y()},
          };
          const bool any_bbox_fit =
              request.request.execution.irregular.enable_direct_overlap_check
                  ? geom::box_contains(container_bounds, candidate_bbox)
                  : std::any_of(region_bboxes.begin(), region_bboxes.end(),
                                [&](const auto &region_bbox) {
                                  return geom::box_contains(region_bbox,
                                                            candidate_bbox);
                                });
          if (any_bbox_fit) {
            if (bbox_overlaps_existing_placement(candidate_bbox,
                                                 bin.placement_bounds)) {
              bbox_overlap_candidate_points.push_back(candidate_point);
            } else {
              bbox_clear_candidate_points.push_back(candidate_point);
            }
          } else {
            ++diagnostics.rejected_by_containment;
            note_rejected_candidate(diagnostics, "containment", 0.0);
          }
        }
        candidate_points = std::move(bbox_clear_candidate_points);
        candidate_points.insert(candidate_points.end(),
                                bbox_overlap_candidate_points.begin(),
                                bbox_overlap_candidate_points.end());
        limit_candidate_points(candidate_points, request.request.execution,
                               bin.state.start_corner, rng);
        diagnostics.candidates_after_limit += candidate_points.size();

        for (const auto &candidate_point : candidate_points) {
          ++attempt_context.candidate_evaluations_completed;
          ++local_eval_count;
          if (local_eval_count % kInterruptCheckInterval == 0) {
            if (interrupted(control) || time_budget.expired(stopwatch)) {
              stop_status = PlacementSearchStatus::interrupted;
              search_done = true;
              break;
            }

            if (search_throttle.should_emit()) {
              emit_search_progress(
                  control, placed_parts, total_parts,
                  attempt_context.candidate_evaluations_completed,
                  std::format(
                      "Searching rotation {}/{}, candidates evaluated: {}",
                      rotation_value + 1U, rotation_count,
                      attempt_context.candidate_evaluations_completed));
            }
          }

          const geom::Vector2 translation{candidate_point.translation.x(),
                                          candidate_point.translation.y()};
          const geom::Box2 candidate_bbox{
              .min = geom::Point2{rotated_bounds.min.x() + translation.x(),
                                  rotated_bounds.min.y() + translation.y()},
              .max = geom::Point2{rotated_bounds.max.x() + translation.x(),
                                  rotated_bounds.max.y() + translation.y()},
          };
          const bool any_bbox_fit =
              request.request.execution.irregular.enable_direct_overlap_check
                  ? geom::box_contains(container_bounds, candidate_bbox)
                  : std::any_of(region_bboxes.begin(), region_bboxes.end(),
                                [&](const auto &region_bbox) {
                                  return geom::box_contains(region_bbox,
                                                            candidate_bbox);
                                });
          if (!any_bbox_fit) {
            ++diagnostics.rejected_by_containment;
            note_rejected_candidate(diagnostics, "containment", 0.0);
            continue;
          }

          const auto transformed_piece =
              geom::translate(rotated_piece, translation);
          const bool fits =
              request.request.execution.irregular.enable_direct_overlap_check
                  ? fits_bin_direct(transformed_piece, candidate_bbox, bin,
                                    request.request.execution)
                  : fits_any_region(transformed_piece, candidate_bbox,
                                    free_regions, region_bboxes);
          if (!fits) {
            ++diagnostics.rejected_by_overlap;
            note_rejected_candidate(
                diagnostics, "overlap_or_exact_containment",
                compute_candidate_score(bin, request.request.execution,
                                        transformed_piece, candidate_bbox));
            continue;
          }
          if (!respects_spacing(transformed_piece, bin,
                                request.request.execution)) {
            ++diagnostics.rejected_by_spacing;
            note_rejected_candidate(
                diagnostics, "spacing",
                compute_candidate_score(bin, request.request.execution,
                                        transformed_piece, candidate_bbox));
            continue;
          }

          const auto hole_index = hole_index_for_candidate(
              transformed_piece, candidate_bbox, bin.state.holes);
          CandidatePlacement candidate{
              .placement =
                  {
                      .piece_id = piece.expanded.expanded_piece_id,
                      .bin_id = bin.state.bin_id,
                      .rotation_index = rotation_index,
                      .translation =
                          geom::Point2{translation.x(), translation.y()},
                      .mirrored = mirrored,
                  },
              .piece_geometry_revision = mirrored_revision,
              .resolved_rotation = active_rotation,
              .polygon = transformed_piece,
              .bounds = candidate_bbox,
              .source = hole_index >= 0
                            ? place::PlacementCandidateSource::hole_boundary
                            : candidate_point.source,
              .nfp_accuracy = candidate_point.nfp_accuracy,
              .inside_hole = hole_index >= 0,
              .hole_index = hole_index,
              .score =
                  compute_candidate_score(bin, request.request.execution,
                                          transformed_piece, candidate_bbox),
          };

          if (rng != nullptr) {
            if (top_k.size() < kTopKCandidates) {
              top_k.push_back(std::move(candidate));
            } else {
              std::size_t worst_idx = 0;
              for (std::size_t k = 1; k < top_k.size(); ++k) {
                if (better_candidate(bin,
                                     request.request.execution.placement_policy,
                                     top_k[worst_idx], top_k[k])) {
                  worst_idx = k;
                }
              }
              if (better_candidate(bin,
                                   request.request.execution.placement_policy,
                                   candidate, top_k[worst_idx])) {
                top_k[worst_idx] = std::move(candidate);
              }
            }
          } else if (!best.has_value() ||
                     better_candidate(
                         bin, request.request.execution.placement_policy,
                         candidate, *best)) {
            best = std::move(candidate);
          }
        }
      }
    }
  }

  if (rng != nullptr && !top_k.empty()) {
    auto chosen_candidate = std::move(top_k[rng->uniform_index(top_k.size())]);
    if (search_metrics != nullptr &&
        chosen_candidate.nfp_accuracy ==
            cache::NfpCacheAccuracy::conservative_bbox_fallback) {
      ++search_metrics->selected_fallback_placements;
    }
    return {.status = PlacementSearchStatus::found,
            .candidate = std::move(chosen_candidate)};
  }
  if (best.has_value()) {
    if (search_metrics != nullptr &&
        best->nfp_accuracy ==
            cache::NfpCacheAccuracy::conservative_bbox_fallback) {
      ++search_metrics->selected_fallback_placements;
    }
    return {.status = PlacementSearchStatus::found,
            .candidate = std::move(best)};
  }
  SHINY_DEBUG(
      "placement_search: no candidate piece={} bin={} allowed_bins={} "
      "rotations_tried={} candidates_before_limit={} "
      "candidates_after_limit={} rejected_containment={} "
      "rejected_overlap={} rejected_spacing={} nfp_failures={} "
      "nfp_exact_cache_hits={} nfp_fallback_cache_hits={} "
      "nfp_exact_computations={} nfp_bbox_fallbacks={} "
      "fallback_candidates={} best_rejected_reason={} placed_in_bin={} "
      "free_regions={}",
      piece.expanded.expanded_piece_id, bin.state.bin_id,
      piece.allowed_expanded_bin_ids.size(), diagnostics.rotations_tried,
      diagnostics.candidates_before_limit, diagnostics.candidates_after_limit,
      diagnostics.rejected_by_containment, diagnostics.rejected_by_overlap,
      diagnostics.rejected_by_spacing, diagnostics.nfp_generation_failures,
      diagnostics.nfp_exact_cache_hits, diagnostics.nfp_fallback_cache_hits,
      diagnostics.nfp_exact_computations, diagnostics.nfp_bbox_fallbacks,
      diagnostics.fallback_candidates, diagnostics.best_rejected_reason,
      bin.state.placements.size(), free_regions.size());
  return {.status = stop_status};
}

// auto find_best_for_bin(
//     WorkingBin &bin, const PieceInstance &piece,
//     const NormalizedRequest &request, const runtime::TimeBudget &time_budget,
//     const runtime::Stopwatch &stopwatch, const SolveControl &control,
//     ProgressThrottle &search_throttle, const std::size_t placed_parts,
//     const std::size_t total_parts, PlacementAttemptContext &attempt_context,
//     cache::NfpCache *nfp_cache, PackerSearchMetrics *search_metrics,
//     runtime::DeterministicRng *rng) -> PlacementSearchResult {
//   if (!piece_allows_bin(piece, bin.state.bin_id)) {
//     return {.status = PlacementSearchStatus::no_candidate};
//   }

//   ensure_free_regions_cached(bin, request.request.execution);
//   const auto &free_regions = bin.cached_free_regions;
//   if (free_regions.empty()) {
//     return {.status = PlacementSearchStatus::no_candidate};
//   }
//   const auto &region_bboxes = bin.cached_region_bboxes;

//   const auto strategy =
//   request.request.execution.irregular.candidate_strategy; const bool
//   use_anchor_candidates =
//       strategy == CandidateStrategy::anchor_vertex ||
//       strategy == CandidateStrategy::nfp_perfect ||
//       strategy == CandidateStrategy::nfp_hybrid;
//   const bool use_nfp_candidates = strategy !=
//   CandidateStrategy::anchor_vertex; const auto anchors =
//       use_anchor_candidates
//           ? collect_anchors(bin, request.request.execution.part_spacing)
//           : std::vector<AnchorPoint>{};
//   const auto &rotations =
//       allowed_rotations_for(*piece.source, request.request.execution);
//   const auto rotation_count = geom::rotation_count(rotations);
//   const auto rotation_begin = piece.forced_rotation_index.has_value()
//                                   ? piece.forced_rotation_index->value
//                                   : 0U;
//   const auto rotation_end =
//       piece.forced_rotation_index.has_value()
//           ? std::min<std::size_t>(rotation_count, rotation_begin + 1U)
//           : rotation_count;

//   std::optional<CandidatePlacement> best;
//   std::vector<CandidatePlacement> top_k;
//   std::size_t local_eval_count = 0;
//   bool search_done = false;
//   PlacementSearchStatus stop_status = PlacementSearchStatus::no_candidate;
//   PlacementSearchDiagnostics diagnostics;

//   for (std::size_t rotation_value = rotation_begin;
//        rotation_value < rotation_end && !search_done; ++rotation_value) {
//     if (interrupted(control, time_budget, stopwatch)) {
//       stop_status = PlacementSearchStatus::interrupted;
//       break;
//     }

//     const geom::RotationIndex rotation_index{
//         static_cast<std::uint16_t>(rotation_value)};
//     const auto resolved_rotation =
//         geom::resolve_rotation(rotation_index, rotations);
//     if (!resolved_rotation.has_value()) {
//       continue;
//     }
//     ++diagnostics.rotations_tried;

//     const auto mirror_count = piece.source->allow_mirror ? 2U : 1U;
//     for (std::size_t mirror_index = 0;
//          mirror_index < mirror_count && !search_done; ++mirror_index) {
//       const bool mirrored = mirror_index == 1U;
//       const auto source_polygon = mirrored ?
//       geom::mirror(piece.source->polygon)
//                                            : piece.source->polygon;
//       const auto refined_angles =
//           rotations.range_degrees.has_value()
//               ? geom::local_refinement_angles(*rotations.range_degrees,
//                                               resolved_rotation->degrees)
//               : std::vector<double>{resolved_rotation->degrees};
//       const auto mirrored_revision = effective_geometry_revision(
//           piece.source->geometry_revision, mirrored);
//       const auto container_bounds =
//       geom::compute_bounds(bin.state.container);

//       for (const auto refined_angle : refined_angles) {
//         const geom::ResolvedRotation active_rotation{.degrees =
//         refined_angle}; const auto rotated_piece =
//             geom::rotate(source_polygon, active_rotation);
//         if (rotated_piece.outer.empty()) {
//           continue;
//         }

//         const auto rotated_bounds = geom::compute_bounds(rotated_piece);
//         const auto reference_points = candidate_reference_points(
//             rotated_piece, rotated_bounds, bin.state.start_corner);

//         std::vector<GeneratedCandidatePoint> candidate_points;
//         if (use_anchor_candidates) {
//           auto anchor_points =
//               translation_points_from_anchors(anchors, reference_points);
//           candidate_points.insert(candidate_points.end(),
//           anchor_points.begin(),
//                                   anchor_points.end());
//           auto skyline_points = translation_points_from_bbox_skyline(
//               bin, rotated_bounds, request.request.execution.part_spacing);
//           candidate_points.insert(candidate_points.end(),
//                                   skyline_points.begin(),
//                                   skyline_points.end());
//         }
//         if (use_nfp_candidates) {
//           const auto obstacles = obstacle_candidates_for(bin);
//           try {
//             CandidateGenerationDiagnostics
//             candidate_generation_diagnostics{}; auto nfp_points =
//             generate_nfp_candidate_points(
//                 bin.state.container, bin.exclusion_regions, obstacles,
//                 rotated_piece, mirrored_revision, active_rotation, strategy,
//                 nfp_cache, &candidate_generation_diagnostics);
//             if (nfp_points.has_value()) {
//               if (search_metrics != nullptr) {
//                 search_metrics->record(candidate_generation_diagnostics);
//               }
//               diagnostics.nfp_exact_cache_hits +=
//                   candidate_generation_diagnostics.exact_nfp_cache_hits;
//               diagnostics.nfp_fallback_cache_hits +=
//                   candidate_generation_diagnostics.conservative_bbox_cache_hits;
//               diagnostics.nfp_exact_computations +=
//                   candidate_generation_diagnostics.exact_nfp_computations;
//               diagnostics.nfp_bbox_fallbacks +=
//                   candidate_generation_diagnostics.conservative_bbox_fallbacks;
//               diagnostics.fallback_candidates +=
//                   candidate_generation_diagnostics
//                       .conservative_bbox_fallback_candidates;
//               auto points = std::move(nfp_points).value();
//               candidate_points.insert(candidate_points.end(), points.begin(),
//                                       points.end());
//             } else {
//               ++diagnostics.nfp_generation_failures;
//               SHINY_DEBUG("placement_search: nfp candidate generation failed
//               "
//                           "piece={} bin={} rotation={} strategy={}
//                           status={}", piece.expanded.expanded_piece_id,
//                           bin.state.bin_id, rotation_value,
//                           static_cast<int>(strategy),
//                           static_cast<int>(nfp_points.error()));
//             }
//           } catch (const std::exception &ex) {
//             ++diagnostics.nfp_generation_failures;
//             SHINY_DEBUG("placement_search: nfp candidate generation threw "
//                         "piece={} bin={} rotation={} strategy={} error={}",
//                         piece.expanded.expanded_piece_id, bin.state.bin_id,
//                         rotation_value, static_cast<int>(strategy),
//                         ex.what());
//           }
//         }
//         diagnostics.candidates_before_limit += candidate_points.size();
//         std::vector<GeneratedCandidatePoint> bbox_clear_candidate_points;
//         std::vector<GeneratedCandidatePoint> bbox_overlap_candidate_points;
//         bbox_clear_candidate_points.reserve(candidate_points.size());
//         bbox_overlap_candidate_points.reserve(candidate_points.size());
//         for (const auto &candidate_point : candidate_points) {
//           const geom::Vector2 translation{
//               .x = candidate_point.translation.x,
//               .y = candidate_point.translation.y,
//           };
//           const geom::Box2 candidate_bbox{
//               .min = {.x = rotated_bounds.min.x + translation.x,
//                       .y = rotated_bounds.min.y + translation.y},
//               .max = {.x = rotated_bounds.max.x + translation.x,
//                       .y = rotated_bounds.max.y + translation.y},
//           };
//           const bool any_bbox_fit =
//               request.request.execution.irregular.enable_direct_overlap_check
//                   ? geom::box_contains(container_bounds, candidate_bbox)
//                   : std::any_of(region_bboxes.begin(), region_bboxes.end(),
//                                 [&](const auto &region_bbox) {
//                                   return geom::box_contains(region_bbox,
//                                                             candidate_bbox);
//                                 });
//           if (any_bbox_fit) {
//             if (bbox_overlaps_existing_placement(candidate_bbox,
//                                                  bin.placement_bounds)) {
//               bbox_overlap_candidate_points.push_back(candidate_point);
//             } else {
//               bbox_clear_candidate_points.push_back(candidate_point);
//             }
//           } else {
//             ++diagnostics.rejected_by_containment;
//             note_rejected_candidate(diagnostics, "containment", 0.0);
//           }
//         }
//         candidate_points = std::move(bbox_clear_candidate_points);
//         candidate_points.insert(candidate_points.end(),
//                                 bbox_overlap_candidate_points.begin(),
//                                 bbox_overlap_candidate_points.end());
//         limit_candidate_points(candidate_points, request.request.execution,
//                                bin.state.start_corner, rng);
//         diagnostics.candidates_after_limit += candidate_points.size();

//         for (const auto &candidate_point : candidate_points) {
//           ++attempt_context.candidate_evaluations_completed;
//           ++local_eval_count;
//           if (local_eval_count % kInterruptCheckInterval == 0) {
//             if (interrupted(control, time_budget, stopwatch)) {
//               stop_status = PlacementSearchStatus::interrupted;
//               search_done = true;
//               break;
//             }
//             if (attempt_context.budget_milliseconds > 0 &&
//                 (stopwatch.elapsed_milliseconds() -
//                  attempt_context.started_milliseconds) >
//                     attempt_context.budget_milliseconds) {
//               SHINY_DEBUG("find_best_for_bin: per-piece budget exceeded ({}ms
//               "
//                           "> {}ms limit)",
//                           stopwatch.elapsed_milliseconds() -
//                               attempt_context.started_milliseconds,
//                           attempt_context.budget_milliseconds);
//               stop_status =
//               PlacementSearchStatus::placement_budget_exhausted; search_done
//               = true; break;
//             }
//             if (search_throttle.should_emit()) {
//               emit_search_progress(
//                   control, placed_parts, total_parts,
//                   attempt_context.candidate_evaluations_completed,
//                   make_budget(control, time_budget, stopwatch, 0U),
//                   std::format(
//                       "Searching rotation {}/{}, candidates evaluated: {}",
//                       rotation_value + 1U, rotation_count,
//                       attempt_context.candidate_evaluations_completed));
//             }
//           }

//           const geom::Vector2 translation{
//               .x = candidate_point.translation.x,
//               .y = candidate_point.translation.y,
//           };
//           const geom::Box2 candidate_bbox{
//               .min = {.x = rotated_bounds.min.x + translation.x,
//                       .y = rotated_bounds.min.y + translation.y},
//               .max = {.x = rotated_bounds.max.x + translation.x,
//                       .y = rotated_bounds.max.y + translation.y},
//           };
//           const bool any_bbox_fit =
//               request.request.execution.irregular.enable_direct_overlap_check
//                   ? geom::box_contains(container_bounds, candidate_bbox)
//                   : std::any_of(region_bboxes.begin(), region_bboxes.end(),
//                                 [&](const auto &region_bbox) {
//                                   return geom::box_contains(region_bbox,
//                                                             candidate_bbox);
//                                 });
//           if (!any_bbox_fit) {
//             ++diagnostics.rejected_by_containment;
//             note_rejected_candidate(diagnostics, "containment", 0.0);
//             continue;
//           }

//           const auto transformed_piece =
//               geom::translate(rotated_piece, translation);
//           const bool fits =
//               request.request.execution.irregular.enable_direct_overlap_check
//                   ? fits_bin_direct(transformed_piece, candidate_bbox, bin,
//                                     request.request.execution)
//                   : fits_any_region(transformed_piece, candidate_bbox,
//                                     free_regions, region_bboxes);
//           if (!fits) {
//             ++diagnostics.rejected_by_overlap;
//             note_rejected_candidate(
//                 diagnostics, "overlap_or_exact_containment",
//                 compute_candidate_score(bin, request.request.execution,
//                                         transformed_piece, candidate_bbox));
//             continue;
//           }
//           if (!respects_spacing(transformed_piece, bin,
//                                 request.request.execution)) {
//             ++diagnostics.rejected_by_spacing;
//             note_rejected_candidate(
//                 diagnostics, "spacing",
//                 compute_candidate_score(bin, request.request.execution,
//                                         transformed_piece, candidate_bbox));
//             continue;
//           }

//           const auto hole_index = hole_index_for_candidate(
//               transformed_piece, candidate_bbox, bin.state.holes);
//           CandidatePlacement candidate{
//               .placement =
//                   {
//                       .piece_id = piece.expanded.expanded_piece_id,
//                       .bin_id = bin.state.bin_id,
//                       .rotation_index = rotation_index,
//                       .translation = {.x = translation.x, .y =
//                       translation.y}, .mirrored = mirrored,
//                   },
//               .piece_geometry_revision = mirrored_revision,
//               .resolved_rotation = active_rotation,
//               .polygon = transformed_piece,
//               .bounds = candidate_bbox,
//               .source = hole_index >= 0
//                             ? place::PlacementCandidateSource::hole_boundary
//                             : candidate_point.source,
//               .nfp_accuracy = candidate_point.nfp_accuracy,
//               .inside_hole = hole_index >= 0,
//               .hole_index = hole_index,
//               .score =
//                   compute_candidate_score(bin, request.request.execution,
//                                           transformed_piece, candidate_bbox),
//           };

//           if (rng != nullptr) {
//             if (top_k.size() < kTopKCandidates) {
//               top_k.push_back(std::move(candidate));
//             } else {
//               std::size_t worst_idx = 0;
//               for (std::size_t k = 1; k < top_k.size(); ++k) {
//                 if (better_candidate(bin,
//                                      request.request.execution.placement_policy,
//                                      top_k[worst_idx], top_k[k])) {
//                   worst_idx = k;
//                 }
//               }
//               if (better_candidate(bin,
//                                    request.request.execution.placement_policy,
//                                    candidate, top_k[worst_idx])) {
//                 top_k[worst_idx] = std::move(candidate);
//               }
//             }
//           } else if (!best.has_value() ||
//                      better_candidate(
//                          bin, request.request.execution.placement_policy,
//                          candidate, *best)) {
//             best = std::move(candidate);
//           }
//         }
//       }
//     }
//   }

//   if (rng != nullptr && !top_k.empty()) {
//     auto chosen_candidate =
//     std::move(top_k[rng->uniform_index(top_k.size())]); if (search_metrics !=
//     nullptr &&
//         chosen_candidate.nfp_accuracy ==
//             cache::NfpCacheAccuracy::conservative_bbox_fallback) {
//       ++search_metrics->selected_fallback_placements;
//     }
//     return {.status = PlacementSearchStatus::found,
//             .candidate = std::move(chosen_candidate)};
//   }
//   if (best.has_value()) {
//     if (search_metrics != nullptr &&
//         best->nfp_accuracy ==
//             cache::NfpCacheAccuracy::conservative_bbox_fallback) {
//       ++search_metrics->selected_fallback_placements;
//     }
//     return {.status = PlacementSearchStatus::found,
//             .candidate = std::move(best)};
//   }
//   SHINY_DEBUG(
//       "placement_search: no candidate piece={} bin={} allowed_bins={} "
//       "rotations_tried={} candidates_before_limit={} "
//       "candidates_after_limit={} rejected_containment={} "
//       "rejected_overlap={} rejected_spacing={} nfp_failures={} "
//       "nfp_exact_cache_hits={} nfp_fallback_cache_hits={} "
//       "nfp_exact_computations={} nfp_bbox_fallbacks={} "
//       "fallback_candidates={} best_rejected_reason={} placed_in_bin={} "
//       "free_regions={}",
//       piece.expanded.expanded_piece_id, bin.state.bin_id,
//       piece.allowed_expanded_bin_ids.size(), diagnostics.rotations_tried,
//       diagnostics.candidates_before_limit,
//       diagnostics.candidates_after_limit,
//       diagnostics.rejected_by_containment, diagnostics.rejected_by_overlap,
//       diagnostics.rejected_by_spacing, diagnostics.nfp_generation_failures,
//       diagnostics.nfp_exact_cache_hits, diagnostics.nfp_fallback_cache_hits,
//       diagnostics.nfp_exact_computations, diagnostics.nfp_bbox_fallbacks,
//       diagnostics.fallback_candidates, diagnostics.best_rejected_reason,
//       bin.state.placements.size(), free_regions.size());
//   return {.status = stop_status};
// }

} // namespace shiny::nesting::pack::detail
