#include "placement/engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "cache/key_hash.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/convex_hull.hpp"
#include "predicates/segment_intersection.hpp"

namespace shiny::nfp::place {
namespace {

constexpr double kAreaEpsilon = 1e-9;

[[nodiscard]] auto as_polygon_with_holes(const BedExclusionZone &zone)
    -> geom::PolygonWithHoles {
  return {.outer = zone.region.outer};
}

[[nodiscard]] auto midpoint(const geom::Point2 &lhs, const geom::Point2 &rhs)
    -> geom::Point2 {
  return {
      .x = (lhs.x + rhs.x) / 2.0,
      .y = (lhs.y + rhs.y) / 2.0,
  };
}

[[nodiscard]] auto translate_point(const geom::Point2 &point,
                                   const geom::Point2 &translation)
    -> geom::Point2 {
  return {
      .x = point.x + translation.x,
      .y = point.y + translation.y,
  };
}

[[nodiscard]] auto translate_ring(const geom::Ring &ring,
                                  const geom::Point2 &translation)
    -> geom::Ring {
  geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(translate_point(point, translation));
  }
  return translated;
}

[[nodiscard]] auto translate_polygon(const geom::PolygonWithHoles &polygon,
                                     const geom::Point2 &translation)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, translation);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, translation));
  }
  return translated;
}

[[nodiscard]] auto signed_area(const geom::Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

[[nodiscard]] auto polygon_area(const geom::PolygonWithHoles &polygon)
    -> double {
  long double area = std::abs(signed_area(polygon.outer));
  for (const auto &hole : polygon.holes) {
    area -= std::abs(signed_area(hole));
  }
  return static_cast<double>(area);
}

[[nodiscard]] auto
total_polygon_area(std::span<const geom::PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += polygon_area(polygon);
  }
  return total;
}

void append_unique_candidate(std::vector<PlacementCandidate> &candidates,
                             PlacementCandidate candidate) {
  const auto same_identity = [&candidate](const auto &entry) {
    return entry.rotation_index == candidate.rotation_index &&
           entry.translation == candidate.translation;
  };

  if (!candidate.inside_hole) {
    const auto existing_non_hole =
        std::find_if(candidates.begin(), candidates.end(),
                     [&same_identity](const auto &entry) {
                       return same_identity(entry) && !entry.inside_hole;
                     });
    if (existing_non_hole != candidates.end()) {
      return;
    }

    const auto first_match =
        std::find_if(candidates.begin(), candidates.end(), same_identity);
    if (first_match == candidates.end()) {
      candidates.push_back(std::move(candidate));
      return;
    }

    *first_match = std::move(candidate);
    candidates.erase(std::remove_if(std::next(first_match), candidates.end(),
                                    [&same_identity](const auto &entry) {
                                      return same_identity(entry);
                                    }),
                     candidates.end());
    return;
  }

  const auto existing_non_hole =
      std::find_if(candidates.begin(), candidates.end(),
                   [&same_identity](const auto &entry) {
                     return same_identity(entry) && !entry.inside_hole;
                   });
  if (existing_non_hole != candidates.end()) {
    return;
  }

  const auto existing_same_hole =
      std::find_if(candidates.begin(), candidates.end(),
                   [&candidate, &same_identity](const auto &entry) {
                     return same_identity(entry) && entry.inside_hole &&
                            entry.hole_index == candidate.hole_index;
                   });
  if (existing_same_hole == candidates.end()) {
    candidates.push_back(std::move(candidate));
  }
}

void append_ring_candidates(std::vector<PlacementCandidate> &candidates,
                            const geom::Ring &ring,
                            geom::RotationIndex rotation_index,
                            PlacementCandidateSource source, bool inside_hole,
                            std::int32_t hole_index,
                            bool explore_concave_candidates) {
  for (const auto &point : ring) {
    append_unique_candidate(candidates, PlacementCandidate{
                                            .translation = point,
                                            .rotation_index = rotation_index,
                                            .source = source,
                                            .inside_hole = inside_hole,
                                            .hole_index = hole_index,
                                        });
  }

  if (!explore_concave_candidates || ring.size() < 2U) {
    return;
  }

  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    append_unique_candidate(
        candidates, PlacementCandidate{
                        .translation = midpoint(ring[index], ring[next_index]),
                        .rotation_index = rotation_index,
                        .source = PlacementCandidateSource::concave_boundary,
                        .inside_hole = inside_hole,
                        .hole_index = hole_index,
                    });
  }
}

[[nodiscard]] auto squared_distance(const geom::Point2 &lhs,
                                    const geom::Point2 &rhs) -> double {
  const auto dx = lhs.x - rhs.x;
  const auto dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

[[nodiscard]] auto distance_to_segment(const geom::Point2 &point,
                                       const geom::Segment2 &segment)
    -> double {
  const auto dx = segment.end.x - segment.start.x;
  const auto dy = segment.end.y - segment.start.y;
  const auto length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) {
    return std::sqrt(squared_distance(point, segment.start));
  }

  const auto projection =
      ((point.x - segment.start.x) * dx + (point.y - segment.start.y) * dy) /
      length_squared;
  const auto clamped = std::clamp(projection, 0.0, 1.0);
  const geom::Point2 closest{
      .x = segment.start.x + clamped * dx,
      .y = segment.start.y + clamped * dy,
  };
  return std::sqrt(squared_distance(point, closest));
}

[[nodiscard]] auto segment_distance(const geom::Segment2 &lhs,
                                    const geom::Segment2 &rhs) -> double {
  const auto contact = pred::classify_segment_contact(lhs, rhs);
  if (contact.kind != pred::SegmentContactKind::disjoint &&
      contact.kind != pred::SegmentContactKind::parallel_disjoint) {
    return 0.0;
  }

  return std::min(
      {distance_to_segment(lhs.start, rhs), distance_to_segment(lhs.end, rhs),
       distance_to_segment(rhs.start, lhs), distance_to_segment(rhs.end, lhs)});
}

[[nodiscard]] auto ring_distance(const geom::Ring &lhs, const geom::Ring &rhs)
    -> double {
  if (lhs.size() < 2U || rhs.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t lhs_index = 0; lhs_index < lhs.size(); ++lhs_index) {
    const auto lhs_next = (lhs_index + 1U) % lhs.size();
    const geom::Segment2 lhs_segment{lhs[lhs_index], lhs[lhs_next]};
    for (std::size_t rhs_index = 0; rhs_index < rhs.size(); ++rhs_index) {
      const auto rhs_next = (rhs_index + 1U) % rhs.size();
      const geom::Segment2 rhs_segment{rhs[rhs_index], rhs[rhs_next]};
      minimum_distance = std::min(minimum_distance,
                                  segment_distance(lhs_segment, rhs_segment));
      if (minimum_distance == 0.0) {
        return 0.0;
      }
    }
  }
  return minimum_distance;
}

[[nodiscard]] auto
minimum_boundary_distance(const geom::PolygonWithHoles &piece,
                          const geom::PolygonWithHoles &occupied_region)
    -> double {
  if (piece.outer.size() < 2U || occupied_region.outer.size() < 2U) {
    return std::numeric_limits<double>::infinity();
  }

  double minimum_distance = ring_distance(piece.outer, occupied_region.outer);
  for (const auto &hole : occupied_region.holes) {
    minimum_distance =
        std::min(minimum_distance, ring_distance(piece.outer, hole));
    if (minimum_distance == 0.0) {
      return 0.0;
    }
  }
  return minimum_distance;
}

[[nodiscard]] auto is_polygon_contained(const geom::PolygonWithHoles &container,
                                        const geom::PolygonWithHoles &polygon)
    -> bool {
  return poly::difference_polygons(polygon, container).empty();
}

[[nodiscard]] auto
overlaps_occupied_region(const geom::PolygonWithHoles &piece,
                         const geom::PolygonWithHoles &occupied_region)
    -> bool {
  if (occupied_region.outer.empty()) {
    return false;
  }

  const auto remaining = poly::difference_polygons(piece, occupied_region);
  return total_polygon_area(remaining) + kAreaEpsilon < polygon_area(piece);
}

[[nodiscard]] auto
overlaps_any_exclusion_zone(const geom::PolygonWithHoles &piece,
                            std::span<const BedExclusionZone> exclusion_zones)
    -> bool {
  for (const auto &zone : exclusion_zones) {
    if (overlaps_occupied_region(piece, as_polygon_with_holes(zone))) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto bottom_left_tiebreak(const PlacementCandidate &lhs,
                                        const PlacementCandidate &rhs) -> bool {
  if (lhs.translation.y != rhs.translation.y) {
    return lhs.translation.y < rhs.translation.y;
  }
  if (lhs.translation.x != rhs.translation.x) {
    return lhs.translation.x < rhs.translation.x;
  }
  if (lhs.rotation_index != rhs.rotation_index) {
    return lhs.rotation_index < rhs.rotation_index;
  }
  if (lhs.inside_hole != rhs.inside_hole) {
    return lhs.inside_hole;
  }
  return static_cast<std::int32_t>(lhs.source) <
         static_cast<std::int32_t>(rhs.source);
}

[[nodiscard]] auto bounding_width(const PlacementRequest &request,
                                  const geom::PolygonWithHoles &piece)
    -> double {
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();

  auto accumulate_ring = [&min_x, &max_x](const geom::Ring &ring) {
    for (const auto &point : ring) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }
  };

  accumulate_ring(piece.outer);
  accumulate_ring(request.merged_region.outer);

  if (!std::isfinite(min_x) || !std::isfinite(max_x)) {
    return 0.0;
  }
  return max_x - min_x;
}

[[nodiscard]] auto utilization_score(const PlacementRequest &request,
                                     const geom::PolygonWithHoles &piece)
    -> double {
  std::vector<geom::PolygonWithHoles> occupied;
  if (request.merged_region.outer.empty()) {
    occupied.push_back(piece);
  } else {
    occupied = poly::union_polygons(request.merged_region, piece);
  }

  std::vector<geom::Point2> hull_points;
  for (const auto &polygon : occupied) {
    hull_points.insert(hull_points.end(), polygon.outer.begin(),
                       polygon.outer.end());
  }

  if (hull_points.empty()) {
    return 0.0;
  }

  const auto hull =
      poly::compute_convex_hull(std::span<const geom::Point2>(hull_points));
  const auto hull_area =
      polygon_area(geom::PolygonWithHoles{.outer = hull.outer});
  if (hull_area <= kAreaEpsilon) {
    return 0.0;
  }
  return total_polygon_area(occupied) / hull_area;
}

[[nodiscard]] auto make_policy_id(const PlacementConfig &config,
                                  PartGrainCompatibility grain_compatibility,
                                  PlacementPolicy policy) -> std::uint32_t {
  std::size_t seed =
      std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(policy));
  cache::hash::combine(seed, std::hash<double>{}(config.part_clearance));
  for (const auto angle : config.allowed_rotations.angles_degrees) {
    cache::hash::combine(seed, std::hash<double>{}(angle));
  }
  cache::hash::combine(seed, std::hash<std::int8_t>{}(static_cast<std::int8_t>(
                                 config.bed_grain_direction)));
  cache::hash::combine(seed,
                       std::hash<bool>{}(config.enable_part_in_part_placement));
  cache::hash::combine(seed,
                       std::hash<bool>{}(config.explore_concave_candidates));
  cache::hash::combine(seed, std::hash<std::int8_t>{}(static_cast<std::int8_t>(
                                 grain_compatibility)));
  for (const auto &zone : config.exclusion_zones) {
    cache::hash::combine(seed, std::hash<std::uint32_t>{}(zone.zone_id));
    for (const auto &point : zone.region.outer) {
      cache::hash::combine(seed, std::hash<double>{}(point.x));
      cache::hash::combine(seed, std::hash<double>{}(point.y));
    }
  }
  return static_cast<std::uint32_t>(seed ^ (seed >> 32U));
}

} // namespace

auto PlacementEngine::extract_candidates(const NfpResult &result,
                                         geom::RotationIndex rotation_index,
                                         const PlacementConfig &config,
                                         PlacementCandidateSource source,
                                         bool inside_hole,
                                         std::int32_t hole_index) const
    -> std::vector<PlacementCandidate> {
  std::vector<PlacementCandidate> candidates;

  for (const auto &loop : result.loops) {
    append_ring_candidates(candidates, loop.vertices, rotation_index, source,
                           inside_hole, hole_index,
                           config.explore_concave_candidates);
  }

  for (const auto &point : result.perfect_fit_points) {
    append_unique_candidate(candidates,
                            PlacementCandidate{
                                .translation = point,
                                .rotation_index = rotation_index,
                                .source = PlacementCandidateSource::perfect_fit,
                                .inside_hole = inside_hole,
                                .hole_index = hole_index,
                            });
  }

  for (const auto &segment : result.perfect_sliding_segments) {
    append_unique_candidate(
        candidates, PlacementCandidate{
                        .translation = segment.start,
                        .rotation_index = rotation_index,
                        .source = PlacementCandidateSource::perfect_sliding,
                        .inside_hole = inside_hole,
                        .hole_index = hole_index,
                    });
    append_unique_candidate(
        candidates, PlacementCandidate{
                        .translation = segment.end,
                        .rotation_index = rotation_index,
                        .source = PlacementCandidateSource::perfect_sliding,
                        .inside_hole = inside_hole,
                        .hole_index = hole_index,
                    });

    if (config.explore_concave_candidates) {
      append_unique_candidate(
          candidates, PlacementCandidate{
                          .translation = midpoint(segment.start, segment.end),
                          .rotation_index = rotation_index,
                          .source = PlacementCandidateSource::concave_boundary,
                          .inside_hole = inside_hole,
                          .hole_index = hole_index,
                      });
    }
  }

  return candidates;
}

auto PlacementEngine::filter_candidates(
    const PlacementRequest &request,
    std::span<const PlacementCandidate> candidates) const
    -> std::vector<PlacementCandidate> {
  std::vector<PlacementCandidate> filtered;

  if (!request.config.is_valid()) {
    return filtered;
  }

  const auto resolved_rotation =
      resolve_rotation(request.rotation_index, request.config);
  if (!resolved_rotation.has_value() ||
      !grain_compatibility_allows_rotation(*resolved_rotation,
                                           request.config.bed_grain_direction,
                                           request.part_grain_compatibility)) {
    return filtered;
  }

  const auto piece_area = polygon_area(request.piece);
  if (piece_area <= kAreaEpsilon) {
    return filtered;
  }

  for (const auto &candidate : candidates) {
    if (candidate.rotation_index != request.rotation_index) {
      continue;
    }

    if (candidate.inside_hole) {
      if (!request.config.enable_part_in_part_placement ||
          candidate.hole_index < 0 ||
          static_cast<std::size_t>(candidate.hole_index) >=
              request.holes.size()) {
        continue;
      }
    }

    const auto translated_piece =
        translate_polygon(request.piece, candidate.translation);

    if (!is_polygon_contained(request.bin, translated_piece)) {
      continue;
    }

    if (candidate.inside_hole &&
        !is_polygon_contained(
            request.holes[static_cast<std::size_t>(candidate.hole_index)],
            translated_piece)) {
      continue;
    }

    if (overlaps_occupied_region(translated_piece, request.merged_region)) {
      continue;
    }

    if (overlaps_any_exclusion_zone(translated_piece,
                                    request.config.exclusion_zones)) {
      continue;
    }

    if (request.config.part_clearance > 0.0 &&
        request.merged_region.outer.size() >= 2U &&
        minimum_boundary_distance(translated_piece, request.merged_region) <
            request.config.part_clearance) {
      continue;
    }

    append_unique_candidate(filtered, candidate);
  }

  return filtered;
}

auto PlacementEngine::rank_candidates(
    const PlacementRequest &request,
    std::span<const PlacementCandidate> candidates,
    PlacementPolicy policy) const -> RankedPlacementSet {
  RankedPlacementSet ranked{.policy = policy};
  ranked.candidates.assign(candidates.begin(), candidates.end());

  for (auto &candidate : ranked.candidates) {
    const auto translated_piece =
        translate_polygon(request.piece, candidate.translation);

    switch (policy) {
    case PlacementPolicy::bottom_left:
      candidate.score = candidate.translation.y;
      break;
    case PlacementPolicy::minimum_length:
      candidate.score = bounding_width(request, translated_piece);
      break;
    case PlacementPolicy::maximum_utilization:
      candidate.score = utilization_score(request, translated_piece);
      break;
    }
  }

  auto better = [policy](const PlacementCandidate &lhs,
                         const PlacementCandidate &rhs) {
    switch (policy) {
    case PlacementPolicy::bottom_left:
      if (lhs.score != rhs.score) {
        return lhs.score < rhs.score;
      }
      return bottom_left_tiebreak(lhs, rhs);
    case PlacementPolicy::minimum_length:
      if (lhs.score != rhs.score) {
        return lhs.score < rhs.score;
      }
      return bottom_left_tiebreak(lhs, rhs);
    case PlacementPolicy::maximum_utilization:
      if (lhs.score != rhs.score) {
        return lhs.score > rhs.score;
      }
      return bottom_left_tiebreak(lhs, rhs);
    }

    return false;
  };

  std::sort(ranked.candidates.begin(), ranked.candidates.end(), better);
  return ranked;
}

auto PlacementEngine::query_candidates(const PlacementRequest &request,
                                       const NfpResult *occupied_region_nfp,
                                       const NfpResult *bin_ifp,
                                       std::span<const NfpResult> hole_ifps,
                                       PlacementPolicy policy)
    -> RankedPlacementSet {
  RankedPlacementSet empty{.policy = policy};
  if (!rotation_is_allowed(request.rotation_index, request.config)) {
    return empty;
  }

  const cache::PlacementQueryKey cache_key{
      .bin_id = request.bin_id,
      .piece_id = request.piece_id,
      .rotation_index = request.rotation_index,
      .piece_geometry_revision = request.piece_geometry_revision,
      .bin_geometry_revision = request.bin_geometry_revision,
      .merged_region_revision = request.merged_region_revision,
      .hole_set_revision = request.hole_set_revision,
      .policy_id = make_policy_id(request.config,
                                  request.part_grain_compatibility, policy),
  };
  if (const auto *cached = cache_.get(cache_key); cached != nullptr) {
    return *cached;
  }

  std::vector<PlacementCandidate> extracted;

  if (occupied_region_nfp != nullptr) {
    const auto occupied_candidates = extract_candidates(
        *occupied_region_nfp, request.rotation_index, request.config,
        PlacementCandidateSource::nfp_boundary);
    extracted.insert(extracted.end(), occupied_candidates.begin(),
                     occupied_candidates.end());
  }

  if (bin_ifp != nullptr) {
    const auto bin_candidates =
        extract_candidates(*bin_ifp, request.rotation_index, request.config,
                           PlacementCandidateSource::bin_ifp);
    extracted.insert(extracted.end(), bin_candidates.begin(),
                     bin_candidates.end());
  }

  for (std::size_t index = 0; index < hole_ifps.size(); ++index) {
    const auto hole_candidates =
        extract_candidates(hole_ifps[index], request.rotation_index,
                           request.config, PlacementCandidateSource::hole_ifp,
                           true, static_cast<std::int32_t>(index));
    extracted.insert(extracted.end(), hole_candidates.begin(),
                     hole_candidates.end());
  }

  const auto filtered = filter_candidates(request, extracted);
  auto ranked = rank_candidates(request, filtered, policy);
  cache_.put(cache_key, ranked);
  return ranked;
}

auto PlacementEngine::clear_cache() -> void { cache_.clear(); }

auto PlacementEngine::cache_size() const -> std::size_t {
  return cache_.size();
}

} // namespace shiny::nfp::place
