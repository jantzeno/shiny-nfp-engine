#include "packing/irregular/core.hpp"

#include <algorithm>
#include <format>
#include <optional>
#include <utility>
#include <vector>

#include "geometry/transform.hpp"
#include "logging/shiny_log.hpp"
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
    return {.x = bounds.max.x, .y = bounds.min.y};
  case place::PlacementStartCorner::top_left:
    return {.x = bounds.min.x, .y = bounds.max.y};
  case place::PlacementStartCorner::top_right:
    return bounds.max;
  }
  return bounds.min;
}

auto append_unique_anchor(std::vector<AnchorPoint> &anchors,
                          const AnchorPoint &candidate) -> void {
  const auto duplicate = std::find_if(
      anchors.begin(), anchors.end(), [&](const AnchorPoint &existing) {
        return almost_equal(existing.point.x, candidate.point.x) &&
               almost_equal(existing.point.y, candidate.point.y) &&
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

[[nodiscard]] auto collect_anchors(const WorkingBin &bin,
                                   const double part_spacing)
    -> std::vector<AnchorPoint> {
  std::vector<AnchorPoint> anchors;
  append_ring_anchors(anchors, bin.state.container.outer,
                      place::PlacementCandidateSource::bin_boundary);
  for (const auto &hole : bin.state.container.holes) {
    append_ring_anchors(anchors, hole,
                        place::PlacementCandidateSource::constructive_boundary);
  }

  if (part_spacing > 0.0) {
    for (const auto &region : bin.cached_free_regions) {
      append_ring_anchors(
          anchors, region.outer,
          place::PlacementCandidateSource::constructive_boundary);
      for (const auto &hole : region.holes) {
        append_ring_anchors(
            anchors, hole,
            place::PlacementCandidateSource::constructive_boundary);
      }
    }
  } else {
    for (const auto &zone : bin.exclusion_regions) {
      append_ring_anchors(
          anchors, zone.outer,
          place::PlacementCandidateSource::constructive_boundary);
    }
    for (const auto &placement : bin.state.placements) {
      append_ring_anchors(
          anchors, placement.polygon.outer,
          place::PlacementCandidateSource::constructive_boundary);
    }
  }

  for (const auto &hole : bin.state.holes) {
    append_ring_anchors(anchors, hole.outer,
                        place::PlacementCandidateSource::hole_boundary);
  }
  return anchors;
}

[[nodiscard]] auto
candidate_reference_points(const geom::PolygonWithHoles &polygon,
                           const geom::Box2 &bounds,
                           const place::PlacementStartCorner start_corner)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> points = polygon.outer;
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
          .translation =
              {
                  .x = anchor.point.x - reference_point.x,
                  .y = anchor.point.y - reference_point.y,
              },
          .source = anchor.source,
      });
    }
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
        .translation = {.x = placement.placement.translation.x,
                        .y = placement.placement.translation.y},
        .rotation = placement.resolved_rotation,
    });
  }
  return obstacles;
}

} // namespace

auto find_best_for_bin(
    WorkingBin &bin, const PieceInstance &piece,
    const NormalizedRequest &request, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch, const SolveControl &control,
    ProgressThrottle &search_throttle, const std::size_t placed_parts,
    const std::size_t total_parts, const std::uint64_t per_piece_budget_ms,
    cache::NfpCache *nfp_cache, runtime::DeterministicRng *rng)
    -> std::optional<CandidatePlacement> {
  if (!piece_allows_bin(piece, bin.state.bin_id)) {
    return std::nullopt;
  }

  ensure_free_regions_cached(bin, request.request.execution);
  const auto &free_regions = bin.cached_free_regions;
  if (free_regions.empty()) {
    return std::nullopt;
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
  const auto piece_start_ms = stopwatch.elapsed_milliseconds(); // HERE
  std::size_t eval_count = 0;
  bool search_done = false;

  for (std::size_t rotation_value = rotation_begin;
       rotation_value < rotation_end && !search_done; ++rotation_value) {
    if (interrupted(control, time_budget, stopwatch)) {
      break;
    }

    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        geom::resolve_rotation(rotation_index, rotations);
    if (!resolved_rotation.has_value()) {
      continue;
    }

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
        if (rotated_piece.outer.empty()) {
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
        }
        if (use_nfp_candidates) {
          const auto obstacles = obstacle_candidates_for(bin);
          auto nfp_points = generate_nfp_candidate_points(
              bin.state.container, bin.exclusion_regions, obstacles,
              rotated_piece, mirrored_revision, active_rotation, strategy,
              nfp_cache);
          if (nfp_points.ok()) {
            auto points = std::move(nfp_points).value();
            candidate_points.insert(candidate_points.end(), points.begin(),
                                    points.end());
          }
        }
        limit_candidate_points(candidate_points, request.request.execution,
                               bin.state.start_corner, rng);

        for (const auto &candidate_point : candidate_points) {
          if (++eval_count % kInterruptCheckInterval == 0) {
            if (interrupted(control, time_budget, stopwatch)) {
              search_done = true;
              break;
            }
            if (per_piece_budget_ms > 0 &&
                (stopwatch.elapsed_milliseconds() - piece_start_ms) >
                    per_piece_budget_ms) {
              SHINY_DEBUG("find_best_for_bin: per-piece budget exceeded ({}ms "
                          "> {}ms limit)",
                          stopwatch.elapsed_milliseconds() - piece_start_ms,
                          per_piece_budget_ms);
              search_done = true;
              break;
            }
            if (search_throttle.should_emit()) {
              emit_search_progress(
                  control, placed_parts, total_parts,
                  make_budget(control, time_budget, stopwatch, 0U),
                  std::format(
                      "Searching rotation {}/{}, candidates evaluated: {}",
                      rotation_value + 1U, rotation_count, eval_count));
            }
          }

          const geom::Vector2 translation{
              .x = candidate_point.translation.x,
              .y = candidate_point.translation.y,
          };
          const geom::Box2 candidate_bbox{
              .min = {.x = rotated_bounds.min.x + translation.x,
                      .y = rotated_bounds.min.y + translation.y},
              .max = {.x = rotated_bounds.max.x + translation.x,
                      .y = rotated_bounds.max.y + translation.y},
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
          if (!fits || !respects_spacing(transformed_piece, bin,
                                         request.request.execution)) {
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
                      .translation = {.x = translation.x, .y = translation.y},
                      .mirrored = mirrored,
                  },
              .piece_geometry_revision = mirrored_revision,
              .resolved_rotation = active_rotation,
              .polygon = transformed_piece,
              .bounds = candidate_bbox,
              .source = hole_index >= 0
                            ? place::PlacementCandidateSource::hole_boundary
                            : candidate_point.source,
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
    return std::move(top_k[rng->uniform_index(top_k.size())]);
  }
  return best;
}

} // namespace shiny::nesting::pack::detail
