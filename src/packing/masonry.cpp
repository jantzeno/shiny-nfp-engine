#include "packing/masonry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include "geometry/normalize.hpp"
#include "nfp/revisions.hpp"
#include "placement/config.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/simplify.hpp"
#include "predicates/orientation.hpp"
#include "predicates/segment_intersection.hpp"
#include "util/detail/fnv_hash.hpp"

namespace shiny::nfp::pack {
namespace {

constexpr double kAreaEpsilon = 1e-9;
constexpr double kCoordinateSnap = 1e-10;
constexpr double kShelfComparisonEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

struct CandidateSelection {
  place::PlacementCandidate candidate{};
  geom::ResolvedRotation resolved_rotation{};
  geom::PolygonWithHoles rotated_piece{};
};

struct BoundingBox {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

struct ShelfChoice {
  CandidateSelection selection{};
  BoundingBox bounds{};
  std::size_t shelf_index{0};
  bool started_new_shelf{false};
};

[[nodiscard]] auto as_polygon_with_holes(const place::BedExclusionZone &zone)
    -> geom::PolygonWithHoles {
  return {.outer = zone.region.outer};
}

[[nodiscard]] auto snap_coordinate(double value) -> double {
  if (std::fabs(value) < kCoordinateSnap) {
    return 0.0;
  }
  return value;
}

[[nodiscard]] auto almost_equal(double lhs, double rhs, double tolerance)
    -> bool {
  return std::fabs(lhs - rhs) <= std::max(tolerance, kShelfComparisonEpsilon);
}

[[nodiscard]] auto is_ring_convex(std::span<const geom::Point2> ring) -> bool {
  if (ring.size() < 3U) {
    return false;
  }

  pred::Orientation expected = pred::Orientation::collinear;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto &a = ring[index];
    const auto &b = ring[(index + 1U) % ring.size()];
    const auto &c = ring[(index + 2U) % ring.size()];
    const auto orientation = pred::orient({a, b, c});
    if (orientation == pred::Orientation::collinear) {
      continue;
    }
    if (expected == pred::Orientation::collinear) {
      expected = orientation;
      continue;
    }
    if (orientation != expected) {
      return false;
    }
  }

  return expected != pred::Orientation::collinear;
}

[[nodiscard]] auto
normalize_convex_ifp_shape(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  return poly::simplify_polygon(geom::normalize_polygon(polygon));
}

[[nodiscard]] auto supports_convex_ifp(const geom::PolygonWithHoles &container,
                                       const geom::PolygonWithHoles &piece)
    -> bool {
  const auto normalized_container = normalize_convex_ifp_shape(container);
  const auto normalized_piece = normalize_convex_ifp_shape(piece);
  if (!is_ring_convex(normalized_container.outer) ||
      !is_ring_convex(normalized_piece.outer)) {
    return false;
  }

  return std::all_of(normalized_container.holes.begin(),
                     normalized_container.holes.end(),
                     [](const auto &hole) { return is_ring_convex(hole); });
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

[[nodiscard]] auto rotate_point(const geom::Point2 &point, double degrees)
    -> geom::Point2 {
  const auto radians = degrees * kPi / 180.0;
  auto cosine = std::cos(radians);
  auto sine = std::sin(radians);

  cosine = snap_coordinate(cosine);
  sine = snap_coordinate(sine);

  return {
      .x = snap_coordinate(point.x * cosine - point.y * sine),
      .y = snap_coordinate(point.x * sine + point.y * cosine),
  };
}

[[nodiscard]] auto rotate_ring(const geom::Ring &ring, double degrees)
    -> geom::Ring {
  geom::Ring rotated;
  rotated.reserve(ring.size());
  for (const auto &point : ring) {
    rotated.push_back(rotate_point(point, degrees));
  }
  return rotated;
}

[[nodiscard]] auto rotate_polygon(const geom::PolygonWithHoles &polygon,
                                  double degrees) -> geom::PolygonWithHoles {
  geom::PolygonWithHoles rotated{};
  rotated.outer = rotate_ring(polygon.outer, degrees);
  rotated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    rotated.holes.push_back(rotate_ring(hole, degrees));
  }
  return geom::normalize_polygon(rotated);
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

void append_unique_candidate(std::vector<place::PlacementCandidate> &candidates,
                             place::PlacementCandidate candidate) {
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

void append_ring_candidates(std::vector<place::PlacementCandidate> &candidates,
                            const geom::Ring &ring,
                            geom::RotationIndex rotation_index,
                            place::PlacementCandidateSource source,
                            bool inside_hole, std::int32_t hole_index,
                            bool explore_concave_candidates) {
  for (const auto &point : ring) {
    append_unique_candidate(candidates, place::PlacementCandidate{
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
        candidates,
        place::PlacementCandidate{
            .translation = midpoint(ring[index], ring[next_index]),
            .rotation_index = rotation_index,
            .source = place::PlacementCandidateSource::concave_boundary,
            .inside_hole = inside_hole,
            .hole_index = hole_index,
        });
  }
}

void append_nfp_candidates(std::vector<place::PlacementCandidate> &candidates,
                           const NfpResult &result,
                           geom::RotationIndex rotation_index,
                           const place::PlacementConfig &config,
                           place::PlacementCandidateSource source,
                           bool inside_hole = false,
                           std::int32_t hole_index = -1) {
  for (const auto &loop : result.loops) {
    append_ring_candidates(candidates, loop.vertices, rotation_index, source,
                           inside_hole, hole_index,
                           config.explore_concave_candidates);
  }

  for (const auto &point : result.perfect_fit_points) {
    append_unique_candidate(
        candidates, place::PlacementCandidate{
                        .translation = point,
                        .rotation_index = rotation_index,
                        .source = place::PlacementCandidateSource::perfect_fit,
                        .inside_hole = inside_hole,
                        .hole_index = hole_index,
                    });
  }

  for (const auto &segment : result.perfect_sliding_segments) {
    append_unique_candidate(
        candidates,
        place::PlacementCandidate{
            .translation = segment.start,
            .rotation_index = rotation_index,
            .source = place::PlacementCandidateSource::perfect_sliding,
            .inside_hole = inside_hole,
            .hole_index = hole_index,
        });
    append_unique_candidate(
        candidates,
        place::PlacementCandidate{
            .translation = segment.end,
            .rotation_index = rotation_index,
            .source = place::PlacementCandidateSource::perfect_sliding,
            .inside_hole = inside_hole,
            .hole_index = hole_index,
        });

    if (config.explore_concave_candidates) {
      append_unique_candidate(
          candidates,
          place::PlacementCandidate{
              .translation = midpoint(segment.start, segment.end),
              .rotation_index = rotation_index,
              .source = place::PlacementCandidateSource::concave_boundary,
              .inside_hole = inside_hole,
              .hole_index = hole_index,
          });
    }
  }
}

void append_container_anchor_candidates(
    std::vector<place::PlacementCandidate> &candidates,
    const geom::PolygonWithHoles &container,
    const geom::PolygonWithHoles &piece, geom::RotationIndex rotation_index,
    place::PlacementCandidateSource source, bool inside_hole = false,
    std::int32_t hole_index = -1) {
  if (container.outer.empty() || piece.outer.empty()) {
    return;
  }

  for (const auto &container_point : container.outer) {
    for (const auto &piece_point : piece.outer) {
      append_unique_candidate(
          candidates, place::PlacementCandidate{
                          .translation =
                              {
                                  .x = container_point.x - piece_point.x,
                                  .y = container_point.y - piece_point.y,
                              },
                          .rotation_index = rotation_index,
                          .source = source,
                          .inside_hole = inside_hole,
                          .hole_index = hole_index,
                      });
    }
  }
}

[[nodiscard]] auto make_transient_geometry_revision(
    std::uint64_t build_generation, std::uint64_t scope_revision,
    std::uint32_t owner_id, std::size_t component_index, std::uint64_t salt)
    -> cache::GeometryRevision {
  return {.value = detail::fnv_hash_values(
              {salt, build_generation, scope_revision,
               static_cast<std::uint64_t>(owner_id),
               static_cast<std::uint64_t>(component_index)})};
}

[[nodiscard]] auto
make_transient_piece_id(std::uint64_t build_generation, std::uint32_t owner_id,
                        std::size_t component_index, std::uint64_t salt)
    -> std::uint32_t {
  const auto hash = detail::fnv_hash_values(
      {salt, build_generation, static_cast<std::uint64_t>(owner_id),
       static_cast<std::uint64_t>(component_index)});
  return static_cast<std::uint32_t>(hash ^ (hash >> 32U));
}

void append_ifp_candidates(
    std::vector<place::PlacementCandidate> &candidates, NfpEngine &nfp_engine,
    const geom::PolygonWithHoles &container, std::uint32_t container_id,
    cache::GeometryRevision container_geometry_revision,
    const geom::PolygonWithHoles &piece, std::uint32_t piece_id,
    cache::GeometryRevision piece_geometry_revision,
    const geom::ResolvedRotation &piece_rotation,
    geom::RotationIndex rotation_index, const place::PlacementConfig &config,
    place::PlacementCandidateSource source, bool inside_hole = false,
    std::int32_t hole_index = -1) {
  if (!supports_convex_ifp(container, piece)) {
    append_container_anchor_candidates(candidates, container, piece,
                                       rotation_index, source, inside_hole,
                                       hole_index);
    return;
  }

  const auto ifp_result = nfp_engine.compute_convex_ifp(
      {
          .container_id = container_id,
          .piece_id = piece_id,
          .container = normalize_convex_ifp_shape(container),
          .piece = normalize_convex_ifp_shape(piece),
          .container_rotation = {},
          .piece_rotation = piece_rotation,
      },
      container_geometry_revision, piece_geometry_revision);

  append_nfp_candidates(candidates, ifp_result, rotation_index, config, source,
                        inside_hole, hole_index);
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
    const geom::Segment2 lhs_segment{
        lhs[lhs_index], lhs_next < lhs.size() ? lhs[lhs_next] : lhs.front()};
    for (std::size_t rhs_index = 0; rhs_index < rhs.size(); ++rhs_index) {
      const auto rhs_next = (rhs_index + 1U) % rhs.size();
      const geom::Segment2 rhs_segment{
          rhs[rhs_index], rhs_next < rhs.size() ? rhs[rhs_next] : rhs.front()};
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

[[nodiscard]] auto overlaps_any_occupied_region(
    const geom::PolygonWithHoles &piece,
    std::span<const geom::PolygonWithHoles> occupied_regions) -> bool {
  for (const auto &region : occupied_regions) {
    if (overlaps_occupied_region(piece, region)) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto overlaps_any_exclusion_zone(
    const geom::PolygonWithHoles &piece,
    std::span<const place::BedExclusionZone> exclusion_zones,
    const std::uint32_t bin_id) -> bool {
  for (const auto &zone : exclusion_zones) {
    if (!place::exclusion_zone_applies_to_bin(zone, bin_id)) {
      continue;
    }
    if (overlaps_occupied_region(piece, as_polygon_with_holes(zone))) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto minimum_distance_to_regions(
    const geom::PolygonWithHoles &piece,
    std::span<const geom::PolygonWithHoles> occupied_regions) -> double {
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (const auto &region : occupied_regions) {
    minimum_distance =
        std::min(minimum_distance, minimum_boundary_distance(piece, region));
    if (minimum_distance == 0.0) {
      return 0.0;
    }
  }
  return minimum_distance;
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

[[nodiscard]] auto primary_edge_distance(
    const BoundingBox &container_bounds, const BoundingBox &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  if (start_corner_on_top(start_corner)) {
    return container_bounds.max_y - piece_bounds.max_y;
  }
  return piece_bounds.min_y - container_bounds.min_y;
}

[[nodiscard]] auto secondary_edge_distance(
    const BoundingBox &container_bounds, const BoundingBox &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  if (start_corner_on_right(start_corner)) {
    return container_bounds.max_x - piece_bounds.max_x;
  }
  return piece_bounds.min_x - container_bounds.min_x;
}

[[nodiscard]] auto
corner_tiebreak(const BoundingBox &container_bounds,
                const BoundingBox &lhs_bounds, const BoundingBox &rhs_bounds,
                const place::PlacementCandidate &lhs,
                const place::PlacementCandidate &rhs,
                const place::PlacementStartCorner start_corner) -> bool {
  const double lhs_primary =
      primary_edge_distance(container_bounds, lhs_bounds, start_corner);
  const double rhs_primary =
      primary_edge_distance(container_bounds, rhs_bounds, start_corner);
  if (lhs_primary != rhs_primary) {
    return lhs_primary < rhs_primary;
  }

  const double lhs_secondary =
      secondary_edge_distance(container_bounds, lhs_bounds, start_corner);
  const double rhs_secondary =
      secondary_edge_distance(container_bounds, rhs_bounds, start_corner);
  if (lhs_secondary != rhs_secondary) {
    return lhs_secondary < rhs_secondary;
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

[[nodiscard]] auto
filter_candidates(const BinState &state,
                  const geom::PolygonWithHoles &rotated_piece,
                  geom::RotationIndex rotation_index,
                  place::PartGrainCompatibility grain_compatibility,
                  const PackingConfig &config,
                  std::span<const place::PlacementCandidate> candidates)
    -> std::vector<place::PlacementCandidate> {
  std::vector<place::PlacementCandidate> filtered;

  if (!config.is_valid()) {
    return filtered;
  }

  const auto resolved_rotation =
      place::resolve_rotation(rotation_index, config.placement);
  if (!resolved_rotation.has_value() ||
      !place::grain_compatibility_allows_rotation(
          *resolved_rotation, config.placement.bed_grain_direction,
          grain_compatibility)) {
    return filtered;
  }

  const auto piece_area = polygon_area(rotated_piece);
  if (piece_area <= kAreaEpsilon) {
    return filtered;
  }

  for (const auto &candidate : candidates) {
    if (candidate.rotation_index != rotation_index) {
      continue;
    }

    if (candidate.inside_hole) {
      if (!config.placement.enable_part_in_part_placement ||
          candidate.hole_index < 0 ||
          static_cast<std::size_t>(candidate.hole_index) >=
              state.holes.size()) {
        continue;
      }
    }

    const auto translated_piece =
        translate_polygon(rotated_piece, candidate.translation);

    if (!is_polygon_contained(state.container, translated_piece)) {
      continue;
    }

    if (candidate.inside_hole &&
        !is_polygon_contained(
            state.holes[static_cast<std::size_t>(candidate.hole_index)],
            translated_piece)) {
      continue;
    }

    if (overlaps_any_occupied_region(translated_piece,
                                     state.occupied.regions)) {
      continue;
    }

    if (overlaps_any_exclusion_zone(
            translated_piece, config.placement.exclusion_zones, state.bin_id)) {
      continue;
    }

    if (config.placement.part_clearance > 0.0 &&
        minimum_distance_to_regions(translated_piece, state.occupied.regions) <
            config.placement.part_clearance) {
      continue;
    }

    append_unique_candidate(filtered, candidate);
  }

  return filtered;
}

[[nodiscard]] auto collect_holes(const poly::MergedRegion &occupied)
    -> std::vector<geom::PolygonWithHoles> {
  std::vector<geom::PolygonWithHoles> holes;
  for (const auto &region : occupied.regions) {
    for (const auto &hole : region.holes) {
      holes.push_back(
          geom::normalize_polygon(geom::PolygonWithHoles{.outer = hole}));
    }
  }
  return holes;
}

auto refresh_bin_state(BinState &state) -> void {
  state.holes = collect_holes(state.occupied);
  ++state.hole_set_revision;
  state.utilization = summarize_bin(state);
}

[[nodiscard]] auto make_empty_bin(const BinInput &bin) -> BinState {
  BinState state{};
  state.bin_id = bin.bin_id;
  state.container = geom::normalize_polygon(bin.polygon);
  state.container_geometry_revision = bin.geometry_revision;
  state.start_corner = bin.start_corner;
  state.utilization = summarize_bin(state);
  return state;
}

[[nodiscard]] auto piece_allows_bin(const PieceInput &piece,
                                    const std::uint32_t bin_id) -> bool {
  return piece.allowed_bin_ids.empty() ||
         std::find(piece.allowed_bin_ids.begin(), piece.allowed_bin_ids.end(),
                   bin_id) != piece.allowed_bin_ids.end();
}

void apply_selection(BinState &state, const PieceInput &piece,
                     const CandidateSelection &selection,
                     std::vector<PlacementTraceEntry> &trace,
                     bool opened_new_bin) {
  const auto translated_piece = translate_polygon(
      selection.rotated_piece, selection.candidate.translation);

  state.placements.push_back({
      .placement = {.piece_id = piece.piece_id,
                    .bin_id = state.bin_id,
                    .rotation_index = selection.candidate.rotation_index,
                    .translation = selection.candidate.translation},
      .resolved_rotation = selection.resolved_rotation,
      .polygon = translated_piece,
      .source = selection.candidate.source,
      .inside_hole = selection.candidate.inside_hole,
      .hole_index = selection.candidate.hole_index,
      .score = selection.candidate.score,
  });

  state.occupied =
      poly::merge_polygon_into_region(state.occupied, translated_piece);
  ++state.occupied_region_revision;
  refresh_bin_state(state);

  trace.push_back({
      .piece_id = piece.piece_id,
      .bin_id = state.bin_id,
      .rotation_index = selection.candidate.rotation_index,
      .resolved_rotation = selection.resolved_rotation,
      .translation = selection.candidate.translation,
      .source = selection.candidate.source,
      .opened_new_bin = opened_new_bin,
      .inside_hole = selection.candidate.inside_hole,
      .hole_index = selection.candidate.hole_index,
      .score = selection.candidate.score,
  });
}

[[nodiscard]] auto compute_bounds(const geom::PolygonWithHoles &polygon)
    -> BoundingBox {
  BoundingBox bounds{
      .min_x = std::numeric_limits<double>::infinity(),
      .min_y = std::numeric_limits<double>::infinity(),
      .max_x = -std::numeric_limits<double>::infinity(),
      .max_y = -std::numeric_limits<double>::infinity(),
  };

  auto accumulate_ring = [&bounds](const geom::Ring &ring) {
    for (const auto &point : ring) {
      bounds.min_x = std::min(bounds.min_x, point.x);
      bounds.min_y = std::min(bounds.min_y, point.y);
      bounds.max_x = std::max(bounds.max_x, point.x);
      bounds.max_y = std::max(bounds.max_y, point.y);
    }
  };

  accumulate_ring(polygon.outer);
  for (const auto &hole : polygon.holes) {
    accumulate_ring(hole);
  }

  if (!std::isfinite(bounds.min_x)) {
    return {};
  }

  return bounds;
}

[[nodiscard]] auto total_utilization(const std::vector<BinState> &bins)
    -> double {
  double total = 0.0;
  for (const auto &bin : bins) {
    total += bin.utilization.utilization;
  }
  return total;
}

auto record_progress(MasonryResult &result, const MasonryRequest &request,
                     std::uint32_t current_piece_id) -> void {
  const auto placed_piece_count = result.layout.placement_trace.size();
  const auto unplaced_piece_count = result.layout.unplaced_piece_ids.size();

  result.progress.push_back({
      .algorithm_kind = AlgorithmKind::masonry_builder,
      .current_piece_id = current_piece_id,
      .processed_piece_count = placed_piece_count + unplaced_piece_count,
      .piece_count = request.decoder_request.pieces.size(),
      .bin_count = result.bins.size(),
      .placed_piece_count = placed_piece_count,
      .unplaced_piece_count = unplaced_piece_count,
      .total_utilization = total_utilization(result.bins),
  });

  if (request.observer.installed()) {
    request.observer.on_progress(result.progress.back());
  }
}

[[nodiscard]] auto collect_shelf_bases(const BinState &state, double tolerance)
    -> std::vector<double> {
  std::vector<double> shelves;
  shelves.reserve(state.placements.size());

  for (const auto &placement : state.placements) {
    const auto bounds = compute_bounds(placement.polygon);
    const auto existing =
        std::find_if(shelves.begin(), shelves.end(), [&](double shelf_y) {
          return almost_equal(shelf_y, bounds.min_y, tolerance);
        });
    if (existing == shelves.end()) {
      shelves.push_back(bounds.min_y);
    }
  }

  std::sort(shelves.begin(), shelves.end());
  return shelves;
}

[[nodiscard]] auto
classify_shelf(const BinState &state,
               const geom::PolygonWithHoles &translated_piece,
               const MasonryConfig &config) -> std::pair<std::size_t, bool> {
  const auto shelves =
      collect_shelf_bases(state, config.shelf_alignment_tolerance);
  const auto bounds = compute_bounds(translated_piece);

  for (std::size_t index = 0; index < shelves.size(); ++index) {
    if (almost_equal(shelves[index], bounds.min_y,
                     config.shelf_alignment_tolerance)) {
      return {index, false};
    }
  }

  const auto insertion =
      std::lower_bound(shelves.begin(), shelves.end(), bounds.min_y);
  return {static_cast<std::size_t>(std::distance(shelves.begin(), insertion)),
          true};
}

[[nodiscard]] auto shelf_choice_better(const ShelfChoice &lhs,
                                       const ShelfChoice &rhs,
                                       const BinState &state,
                                       const MasonryConfig &config) -> bool {
  const auto container_bounds = compute_bounds(state.container);
  const double lhs_primary =
      primary_edge_distance(container_bounds, lhs.bounds, state.start_corner);
  const double rhs_primary =
      primary_edge_distance(container_bounds, rhs.bounds, state.start_corner);
  if (!almost_equal(lhs_primary, rhs_primary,
                    config.shelf_alignment_tolerance)) {
    return lhs_primary < rhs_primary;
  }

  const double lhs_secondary =
      secondary_edge_distance(container_bounds, lhs.bounds, state.start_corner);
  const double rhs_secondary =
      secondary_edge_distance(container_bounds, rhs.bounds, state.start_corner);
  if (!almost_equal(lhs_secondary, rhs_secondary,
                    config.shelf_alignment_tolerance)) {
    return lhs_secondary < rhs_secondary;
  }

  return corner_tiebreak(container_bounds, lhs.bounds, rhs.bounds,
                         lhs.selection.candidate, rhs.selection.candidate,
                         state.start_corner);
}

[[nodiscard]] auto rank_candidate_set(
    const BinState &state, const geom::PolygonWithHoles &rotated_piece,
    const MasonryConfig &masonry,
    std::vector<place::PlacementCandidate> candidates,
    geom::ResolvedRotation resolved_rotation) -> std::optional<ShelfChoice> {
  std::optional<ShelfChoice> best;

  for (auto &candidate : candidates) {
    const auto translated_piece =
        translate_polygon(rotated_piece, candidate.translation);
    const auto bounds = compute_bounds(translated_piece);
    const auto [shelf_index, started_new_shelf] =
        classify_shelf(state, translated_piece, masonry);
    candidate.score = primary_edge_distance(compute_bounds(state.container),
                                            bounds, state.start_corner);

    ShelfChoice choice{
        .selection = {.candidate = candidate,
                      .resolved_rotation = resolved_rotation,
                      .rotated_piece = rotated_piece},
        .bounds = bounds,
        .shelf_index = shelf_index,
        .started_new_shelf = started_new_shelf,
    };

    if (!best.has_value() ||
        shelf_choice_better(choice, *best, state, masonry)) {
      best = std::move(choice);
    }
  }

  return best;
}

[[nodiscard]] auto find_best_for_rotation(
    NfpEngine &nfp_engine, std::uint64_t build_generation,
    const BinState &state, const PieceInput &piece,
    geom::RotationIndex rotation_index, const PackingConfig &config,
    const MasonryConfig &masonry) -> std::optional<ShelfChoice> {
  const auto resolved_rotation =
      place::resolve_rotation(rotation_index, config.placement);
  if (!resolved_rotation.has_value()) {
    return std::nullopt;
  }

  const auto rotated_piece =
      rotate_polygon(piece.polygon, resolved_rotation->degrees);

  auto collect_hole_candidates = [&]() {
    std::vector<place::PlacementCandidate> extracted;
    for (std::size_t index = 0; index < state.holes.size(); ++index) {
      append_ifp_candidates(
          extracted, nfp_engine, state.holes[index],
          make_transient_piece_id(build_generation, state.bin_id, index,
                                  0x684f4c45ULL),
          make_transient_geometry_revision(build_generation,
                                           state.hole_set_revision,
                                           state.bin_id, index, 0x684f4c45ULL),
          rotated_piece, piece.piece_id, {.value = piece.geometry_revision},
          *resolved_rotation, rotation_index, config.placement,
          place::PlacementCandidateSource::hole_ifp, true,
          static_cast<std::int32_t>(index));
    }
    return filter_candidates(state, rotated_piece, rotation_index,
                             piece.grain_compatibility, config, extracted);
  };

  auto collect_regular_candidates = [&]() {
    std::vector<place::PlacementCandidate> extracted;
    append_ifp_candidates(extracted, nfp_engine, state.container, state.bin_id,
                          {.value = state.container_geometry_revision},
                          rotated_piece, piece.piece_id,
                          {.value = piece.geometry_revision},
                          *resolved_rotation, rotation_index, config.placement,
                          place::PlacementCandidateSource::bin_ifp);

    for (std::size_t index = 0; index < state.occupied.regions.size();
         ++index) {
      const auto region_piece_id = make_transient_piece_id(
          build_generation, state.bin_id, index, 0x52454749ULL);
      const auto region_geometry_revision = make_transient_geometry_revision(
          build_generation, state.occupied_region_revision, state.bin_id, index,
          0x52454749ULL);
      const auto &region = state.occupied.regions[index];
      const NonconvexNfpRequest nfp_request{
          .piece_a_id = region_piece_id,
          .piece_b_id = piece.piece_id,
          .piece_a = region,
          .piece_b = rotated_piece,
          .rotation_a = {},
          .rotation_b = *resolved_rotation,
          .algorithm_revision = {.value = 1},
      };
      const auto nfp_result = nfp_engine.compute_nonconvex_graph_nfp(
          nfp_request, region_geometry_revision,
          {.value = piece.geometry_revision});
      append_nfp_candidates(extracted, nfp_result.result, rotation_index,
                            config.placement,
                            place::PlacementCandidateSource::nfp_boundary);
    }

    return filter_candidates(state, rotated_piece, rotation_index,
                             piece.grain_compatibility, config, extracted);
  };

  if (config.enable_hole_first_placement &&
      config.placement.enable_part_in_part_placement && !state.holes.empty()) {
    if (auto hole_candidates = collect_hole_candidates();
        !hole_candidates.empty()) {
      return rank_candidate_set(state, rotated_piece, masonry,
                                std::move(hole_candidates), *resolved_rotation);
    }
  }

  auto regular_candidates = collect_regular_candidates();
  return rank_candidate_set(state, rotated_piece, masonry,
                            std::move(regular_candidates), *resolved_rotation);
}

[[nodiscard]] auto
find_best_for_bin(NfpEngine &nfp_engine, const BinState &state,
                  std::uint64_t build_generation, const PieceInput &piece,
                  const MasonryRequest &request) -> std::optional<ShelfChoice> {
  std::optional<ShelfChoice> best;
  const auto rotation_count = request.decoder_request.config.placement
                                  .allowed_rotations.angles_degrees.size();

  for (std::size_t index = 0; index < rotation_count; ++index) {
    const auto rotation_index =
        geom::RotationIndex{.value = static_cast<std::uint16_t>(index)};
    const auto candidate = find_best_for_rotation(
        nfp_engine, build_generation, state, piece, rotation_index,
        request.decoder_request.config, request.masonry);
    if (candidate.has_value() &&
        (!best.has_value() ||
         shelf_choice_better(*candidate, *best, state, request.masonry))) {
      best = candidate;
    }
  }

  return best;
}

} // namespace

auto MasonryConfig::is_valid() const -> bool {
  return shelf_alignment_tolerance >= 0.0 &&
         std::isfinite(shelf_alignment_tolerance);
}

MasonryBuilder::MasonryBuilder(cache::CachePolicyConfig cache_policy)
    : nfp_engine_(cache_policy) {}

auto MasonryBuilder::build(const MasonryRequest &request) -> MasonryResult {
  MasonryResult result{};
  const auto build_generation = ++build_generation_;

  if (!request.masonry.is_valid() ||
      !request.decoder_request.config.is_valid() ||
      request.decoder_request.bins.empty()) {
    for (const auto &piece : request.decoder_request.pieces) {
      result.layout.unplaced_piece_ids.push_back(piece.piece_id);
    }
    return result;
  }

  std::vector<bool> opened_bins(request.decoder_request.bins.size(), false);
  for (const auto &piece : request.decoder_request.pieces) {
    bool placed = false;

    for (auto &bin : result.bins) {
      if (!piece_allows_bin(piece, bin.bin_id)) {
        continue;
      }

      if (const auto selection = find_best_for_bin(
              nfp_engine_, bin, build_generation, piece, request);
          selection.has_value()) {
        apply_selection(bin, piece, selection->selection,
                        result.layout.placement_trace, false);
        result.trace.push_back({
            .piece_id = piece.piece_id,
            .bin_id = bin.bin_id,
            .shelf_index = selection->shelf_index,
            .rotation_index = selection->selection.candidate.rotation_index,
            .resolved_rotation = selection->selection.resolved_rotation,
            .translation = selection->selection.candidate.translation,
            .source = selection->selection.candidate.source,
            .opened_new_bin = false,
            .started_new_shelf = selection->started_new_shelf,
            .inside_hole = selection->selection.candidate.inside_hole,
            .hole_index = selection->selection.candidate.hole_index,
            .score = selection->selection.candidate.score,
        });
        placed = true;
        break;
      }
    }

    if (placed) {
      record_progress(result, request, piece.piece_id);
      continue;
    }

    for (std::size_t request_bin_index = 0;
         request_bin_index < request.decoder_request.bins.size();
         ++request_bin_index) {
      if (opened_bins[request_bin_index]) {
        continue;
      }
      const BinInput &bin_input =
          request.decoder_request.bins[request_bin_index];
      if (!piece_allows_bin(piece, bin_input.bin_id)) {
        continue;
      }

      auto new_bin = make_empty_bin(bin_input);
      if (const auto selection = find_best_for_bin(
              nfp_engine_, new_bin, build_generation, piece, request);
          selection.has_value()) {
        apply_selection(new_bin, piece, selection->selection,
                        result.layout.placement_trace, true);
        result.trace.push_back({
            .piece_id = piece.piece_id,
            .bin_id = new_bin.bin_id,
            .shelf_index = selection->shelf_index,
            .rotation_index = selection->selection.candidate.rotation_index,
            .resolved_rotation = selection->selection.resolved_rotation,
            .translation = selection->selection.candidate.translation,
            .source = selection->selection.candidate.source,
            .opened_new_bin = true,
            .started_new_shelf = true,
            .inside_hole = selection->selection.candidate.inside_hole,
            .hole_index = selection->selection.candidate.hole_index,
            .score = selection->selection.candidate.score,
        });
        opened_bins[request_bin_index] = true;
        result.bins.push_back(std::move(new_bin));
        placed = true;
        break;
      }
    }

    if (placed) {
      record_progress(result, request, piece.piece_id);
      continue;
    }

    result.layout.unplaced_piece_ids.push_back(piece.piece_id);
    record_progress(result, request, piece.piece_id);
  }

  result.layout.bins.reserve(result.bins.size());
  for (const auto &bin : result.bins) {
    result.layout.bins.push_back({
        .bin_id = bin.bin_id,
        .container = bin.container,
        .occupied = bin.occupied,
        .placements = bin.placements,
        .utilization = bin.utilization,
    });
  }

  return result;
}

auto MasonryBuilder::clear_caches() -> void { nfp_engine_.clear_caches(); }

auto MasonryBuilder::convex_cache_size() const -> std::size_t {
  return nfp_engine_.convex_cache_size();
}

auto MasonryBuilder::nonconvex_cache_size() const -> std::size_t {
  return nfp_engine_.nonconvex_cache_size();
}

auto MasonryBuilder::decomposition_cache_size() const -> std::size_t {
  return nfp_engine_.decomposition_cache_size();
}

} // namespace shiny::nfp::pack
