#include "nfp/orbiting_nfp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "decomposition/convex_decomposition.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/sanitize.hpp"
#include "geometry/validity.hpp"
#include "geometry/vector_ops.hpp"
#include "logging/shiny_log.hpp"
#include "nfp/convex_nfp.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/convex_hull.hpp"
#include "polygon_ops/greedy_merge.hpp"
#include "polygon_ops/simplify.hpp"

// Burke et al. orbital NFP — computes No-Fit Polygon by sliding the
// "orbiting" polygon around the boundary of the "stationary" polygon and
// recording the trajectory of its reference point.
//
// High-level loop (function `trace_nfp_boundary`):
//   1. Place orbiting polygon at a known boundary contact (start_position,
//      typically bottom-of-stationary touching top-of-orbiting).
//   2. At each step, identify all current contact features (vertex-edge,
//      edge-vertex, edge-edge) via `find_contacts`.
//   3. Generate candidate translation vectors from contact geometry
//      (`compute_translation_vectors`); each contact constrains motion
//      along the contacting edge.
//   4. Filter out vectors that would revisit recently-visited path points
//      (perfect-fit dedup in `handle_perfect_fit`).
//   5. Score remaining vectors and pick one favoring CCW orbit + previous
//      direction continuity (`select_translation_vector`).
//   6. Probe the chosen vector for first-collision with `ray_segment_
//      intersection`; advance the reference point up to that collision
//      minus a contact tolerance.
//   7. Append the new position to the path. Terminate when the path
//      closes back on `start_position` (within tolerance), no progress is
//      made, or `kMaxIterations` is hit.
//   8. `build_path_polygon` simplifies collinear vertices, validates,
//      and returns the closed loop as the NFP.
//
// Robustness pitfalls (the code addresses these but each is fragile):
//   - Stuck states: `recover_contact` snaps the orbiting reference back
//     onto the stationary boundary if floating-point drift loses contact.
//   - Local back-and-forth oscillation: the visited-position dedup uses
//     a 10× tolerance window (kContactTolerance * 10) — large enough to
//     prevent micro-bounce, small enough not to suppress legitimate moves.
//   - Step capping: each slide step is clamped to a fraction of the
//     COMBINED stationary+orbiting bounding-box diameter
//     (`kMaxSlideStepDiameterFraction * combined_diameter`) rather than
//     a fixed coordinate value. This keeps the per-step distance
//     "moderately long relative to the geometry" without baking in a
//     unit assumption (the historical 1000.0 cap silently sub-sampled
//     long edges in non-mm units or large sheets).
//   - Numerical: ray-segment intersection uses `kContactTolerance` slack
//     on both `t` and `u` parameters to admit grazing intersections.
//
// Fall-back strategy (`compute_orbiting_nfp`):
//   - Convex+convex+hole-free => try the orbital trace first; on failure
//     fall through to decomposition-based dispatch.
//   - General case => pairwise convex decomposition; for each pair try
//     orbital trace, fall back to `compute_convex_nfp` if tracing fails.
//   - Final aggregate is unioned via `merge_polygon_set`.
//
// References:
//   Burke et al. (2007). Complete and robust no-fit polygon generation
//   for the irregular stock cutting problem. EJOR.
namespace shiny::nesting::nfp {
namespace {

// Tolerances are coordinate-unit-relative; all geometry must be in a
// consistent unit (engine convention: millimeters). Adjust upward if the
// engine is ever extended to micron-scale work; the stuck-detection at
// 10× kContactTolerance assumes this scale.
constexpr double kContactTolerance = 1e-6;
constexpr double kMinTranslation = 1e-8;
constexpr std::size_t kMaxIterations = 10'000U;
constexpr std::size_t kMaxOrbitingVertexProduct = 512U;
// `trace_nfp_boundary` may temporarily fail to advance — i.e. the
// touching set yields no usable translation vectors — when contact
// classification briefly degenerates (e.g. simultaneous edge-edge with
// zero-length residual). 3 consecutive empty rounds is the empirical
// threshold above which the trace is considered terminally stuck rather
// than transiently saturated; before this point the loop simply
// continues and lets the next contact reset succeed.
constexpr std::size_t kStuckIterationLimit = 3U;
// Fraction of the COMBINED stationary+orbiting bounding-box diameter
// used as the maximum single-step slide distance. The original
// hard-coded 1000.0 silently sub-sampled long edges in non-mm units or
// large sheets; this scale-relative cap keeps the step "moderately
// long" relative to the geometry without any unit assumption.
constexpr double kMaxSlideStepDiameterFraction = 0.5;

enum class ContactType {
  vertex_edge,
  edge_vertex,
  edge_edge,
};

struct Contact {
  ContactType type{ContactType::vertex_edge};
  std::size_t stationary_index{0U};
  std::size_t orbiting_index{0U};
  geom::Point2 point{};
};

struct TouchingGroup {
  std::vector<Contact> contacts;
  geom::Point2 reference_position{};

  [[nodiscard]] auto has_contacts() const -> bool { return !contacts.empty(); }
};

struct TranslationVector {
  geom::Vector2 direction{};
  double max_distance{0.0};
};

[[nodiscard]] auto near_zero(const double value,
                             const double tolerance = kContactTolerance)
    -> bool {
  return std::abs(value) <= tolerance;
}

[[nodiscard]] auto near_same_point(const geom::Point2 &lhs,
                                   const geom::Point2 &rhs,
                                   const double tolerance = kContactTolerance)
    -> bool {
  return geom::point_distance(lhs, rhs) <= tolerance;
}

[[nodiscard]] auto ensure_ccw(const geom::Ring &ring) -> geom::Ring {
  if (geom::ring_signed_area(ring) >= 0.0) {
    return ring;
  }

  geom::Ring reversed = ring;
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

[[nodiscard]] auto edge_vector(const geom::Ring &ring, const std::size_t index)
    -> geom::Vector2 {
  const auto next = (index + 1U) % ring.size();
  return geom::vector_between(ring[index], ring[next]);
}

[[nodiscard]] auto edge_segment(const geom::Ring &ring, const std::size_t index)
    -> geom::Segment2 {
  return {.start = ring[index], .end = ring[(index + 1U) % ring.size()]};
}

[[nodiscard]] auto edges_parallel(const geom::Vector2 &lhs,
                                  const geom::Vector2 &rhs) -> bool {
  const auto lhs_length = geom::vector_length(lhs);
  const auto rhs_length = geom::vector_length(rhs);
  if (near_zero(lhs_length) || near_zero(rhs_length)) {
    return false;
  }
  return std::abs(geom::cross(lhs, rhs)) <=
         kContactTolerance * lhs_length * rhs_length;
}

[[nodiscard]] auto ring_centroid(const geom::Ring &ring) -> geom::Point2 {
  if (ring.empty()) {
    return {};
  }

  double accumulated_cross = 0.0;
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto &current = ring[index];
    const auto &next = ring[(index + 1U) % ring.size()];
    const auto edge_cross = current.x * next.y - next.x * current.y;
    accumulated_cross += edge_cross;
    centroid_x += (current.x + next.x) * edge_cross;
    centroid_y += (current.y + next.y) * edge_cross;
  }

  if (near_zero(accumulated_cross)) {
    centroid_x = 0.0;
    centroid_y = 0.0;
    for (const auto &point : ring) {
      centroid_x += point.x;
      centroid_y += point.y;
    }
    const auto scale = 1.0 / static_cast<double>(ring.size());
    return {.x = centroid_x * scale, .y = centroid_y * scale};
  }

  const auto scale = 1.0 / (3.0 * accumulated_cross);
  return {.x = centroid_x * scale, .y = centroid_y * scale};
}

[[nodiscard]] auto translate_ring(const geom::Ring &ring,
                                  const geom::Vector2 &translation)
    -> geom::Ring {
  geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(geom::point_plus_vector(point, translation));
  }
  return translated;
}

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  const auto lhs_bounds = geom::compute_bounds(lhs);
  const auto rhs_bounds = geom::compute_bounds(rhs);
  if (lhs_bounds.min.x != rhs_bounds.min.x) {
    return lhs_bounds.min.x < rhs_bounds.min.x;
  }
  if (lhs_bounds.min.y != rhs_bounds.min.y) {
    return lhs_bounds.min.y < rhs_bounds.min.y;
  }
  return geom::polygon_area(lhs) < geom::polygon_area(rhs);
}

[[nodiscard]] auto
merge_polygon_set(std::vector<geom::PolygonWithHoles> polygons)
    -> std::vector<geom::PolygonWithHoles> {
  polygons = poly::greedy_pairwise_merge(
      std::move(polygons),
      [](const geom::PolygonWithHoles &lhs, const geom::PolygonWithHoles &rhs)
          -> std::optional<geom::PolygonWithHoles> {
        auto unioned = poly::try_union_polygons(lhs, rhs);
        if (!unioned.ok()) {
          SHINY_DEBUG("orbiting_nfp: merge union failed status={} lhs_outer={} "
                      "rhs_outer={}",
                      util::status_name(unioned.status()), lhs.outer.size(),
                      rhs.outer.size());
          return std::nullopt;
        }
        if (unioned.value().size() != 1U) {
          return std::nullopt;
        }
        return unioned.value().front();
      });
  std::sort(polygons.begin(), polygons.end(), polygon_less);
  return polygons;
}

[[nodiscard]] auto has_actual_overlap(const geom::Ring &stationary,
                                      const geom::Ring &orbiting)
    -> util::StatusOr<bool> {
  auto overlap = poly::try_intersection_polygons(
      geom::PolygonWithHoles{.outer = stationary},
      geom::PolygonWithHoles{.outer = orbiting});
  if (!overlap.ok()) {
    SHINY_DEBUG("orbiting_nfp: overlap intersection failed status={}",
                util::status_name(overlap.status()));
    return overlap.status();
  }
  double overlap_area = 0.0;
  for (const auto &polygon : overlap.value()) {
    overlap_area += geom::polygon_area(polygon);
  }
  return overlap_area > kContactTolerance * kContactTolerance;
}

[[nodiscard]] auto find_contacts(const geom::Ring &stationary,
                                 const geom::Ring &orbiting)
    -> std::vector<Contact> {
  std::vector<Contact> contacts;
  contacts.reserve(stationary.size() + orbiting.size());

  for (std::size_t orbiting_index = 0; orbiting_index < orbiting.size();
       ++orbiting_index) {
    const auto &orbiting_vertex = orbiting[orbiting_index];
    for (std::size_t stationary_index = 0; stationary_index < stationary.size();
         ++stationary_index) {
      if (!geom::point_on_segment(orbiting_vertex,
                                  edge_segment(stationary, stationary_index),
                                  kContactTolerance)) {
        continue;
      }
      contacts.push_back({.type = ContactType::vertex_edge,
                          .stationary_index = stationary_index,
                          .orbiting_index = orbiting_index,
                          .point = orbiting_vertex});
    }
  }

  for (std::size_t stationary_index = 0; stationary_index < stationary.size();
       ++stationary_index) {
    const auto &stationary_vertex = stationary[stationary_index];
    for (std::size_t orbiting_index = 0; orbiting_index < orbiting.size();
         ++orbiting_index) {
      if (!geom::point_on_segment(stationary_vertex,
                                  edge_segment(orbiting, orbiting_index),
                                  kContactTolerance)) {
        continue;
      }
      contacts.push_back({.type = ContactType::edge_vertex,
                          .stationary_index = stationary_index,
                          .orbiting_index = orbiting_index,
                          .point = stationary_vertex});
    }
  }

  for (std::size_t stationary_index = 0; stationary_index < stationary.size();
       ++stationary_index) {
    const auto stationary_edge = edge_segment(stationary, stationary_index);
    const auto stationary_direction = edge_vector(stationary, stationary_index);
    for (std::size_t orbiting_index = 0; orbiting_index < orbiting.size();
         ++orbiting_index) {
      const auto orbiting_edge = edge_segment(orbiting, orbiting_index);
      const auto orbiting_direction = edge_vector(orbiting, orbiting_index);
      if (!edges_parallel(stationary_direction, orbiting_direction)) {
        continue;
      }

      const auto distance_start =
          geom::point_to_segment_distance(orbiting_edge.start, stationary_edge);
      const auto distance_end =
          geom::point_to_segment_distance(orbiting_edge.end, stationary_edge);
      if (distance_start > kContactTolerance ||
          distance_end > kContactTolerance) {
        continue;
      }

      contacts.push_back(
          {.type = ContactType::edge_edge,
           .stationary_index = stationary_index,
           .orbiting_index = orbiting_index,
           .point = {.x = (orbiting_edge.start.x + orbiting_edge.end.x) / 2.0,
                     .y =
                         (orbiting_edge.start.y + orbiting_edge.end.y) / 2.0}});
    }
  }

  return contacts;
}

[[nodiscard]] auto create_touching_group(const geom::Ring &stationary,
                                         const geom::Ring &orbiting,
                                         const geom::Point2 &reference_position)
    -> TouchingGroup {
  return {.contacts = find_contacts(stationary, orbiting),
          .reference_position = reference_position};
}

[[nodiscard]] auto compute_translation_vectors(
    const TouchingGroup &touching_group, const geom::Ring &stationary,
    const geom::Ring &orbiting) -> std::vector<TranslationVector> {
  std::vector<TranslationVector> vectors;
  vectors.reserve(touching_group.contacts.size() * 2U);

  for (const auto &contact : touching_group.contacts) {
    switch (contact.type) {
    case ContactType::vertex_edge: {
      const auto edge = edge_vector(stationary, contact.stationary_index);
      const auto direction = geom::normalize_vector(edge, kContactTolerance);
      const auto reverse = geom::scale_vector(direction, -1.0);
      const auto edge_start = stationary[contact.stationary_index];
      const auto edge_end =
          stationary[(contact.stationary_index + 1U) % stationary.size()];
      vectors.push_back(
          {.direction = direction,
           .max_distance = geom::point_distance(contact.point, edge_end)});
      vectors.push_back(
          {.direction = reverse,
           .max_distance = geom::point_distance(contact.point, edge_start)});
      break;
    }
    case ContactType::edge_vertex: {
      const auto edge = edge_vector(orbiting, contact.orbiting_index);
      const auto direction = geom::normalize_vector(edge, kContactTolerance);
      const auto reverse = geom::scale_vector(direction, -1.0);
      const auto edge_start = orbiting[contact.orbiting_index];
      const auto edge_end =
          orbiting[(contact.orbiting_index + 1U) % orbiting.size()];
      vectors.push_back(
          {.direction = reverse,
           .max_distance = geom::point_distance(contact.point, edge_end)});
      vectors.push_back(
          {.direction = direction,
           .max_distance = geom::point_distance(contact.point, edge_start)});
      break;
    }
    case ContactType::edge_edge: {
      const auto direction = geom::normalize_vector(
          edge_vector(stationary, contact.stationary_index), kContactTolerance);
      const auto reverse = geom::scale_vector(direction, -1.0);
      const auto stationary_length = geom::point_distance(
          stationary[contact.stationary_index],
          stationary[(contact.stationary_index + 1U) % stationary.size()]);
      const auto orbiting_length = geom::point_distance(
          orbiting[contact.orbiting_index],
          orbiting[(contact.orbiting_index + 1U) % orbiting.size()]);
      const auto max_distance = stationary_length + orbiting_length;
      vectors.push_back({.direction = direction, .max_distance = max_distance});
      vectors.push_back({.direction = reverse, .max_distance = max_distance});
      break;
    }
    }
  }

  return vectors;
}

[[nodiscard]] auto
handle_perfect_fit(const TouchingGroup &touching_group,
                   const geom::Ring &stationary, const geom::Ring &orbiting,
                   const std::vector<geom::Point2> &visited_positions)
    -> std::vector<TranslationVector> {
  auto vectors =
      compute_translation_vectors(touching_group, stationary, orbiting);
  vectors.erase(
      std::remove_if(vectors.begin(), vectors.end(),
                     [&](const TranslationVector &candidate) {
                       const auto step = std::min(candidate.max_distance, 1.0);
                       const auto probe = geom::point_plus_vector(
                           touching_group.reference_position,
                           geom::scale_vector(candidate.direction, step));
                       return std::any_of(
                           visited_positions.begin(), visited_positions.end(),
                           [&](const geom::Point2 &visited) {
                             return near_same_point(probe, visited,
                                                    kContactTolerance * 10.0);
                           });
                     }),
      vectors.end());
  return vectors;
}

// Pick the next slide vector. Scoring blends two preferences:
//   * Alignment with the CCW tangent at the current orbital radial
//     (so the trace winds counter-clockwise around `stationary` ⇒ outer
//     ring with positive signed area, matching geom convention).
//   * Continuity with the previously selected direction (weight 0.5),
//     which damps zig-zag oscillations on edge-edge contacts that admit
//     two collinear options.
// Vectors with effectively zero `max_distance` are heavily penalised
// (-100) so they only win if all alternatives are likewise degenerate.
// Tie-break (within 1e-9 score equality) prefers the longer slide so the
// trace makes faster progress.
[[nodiscard]] auto select_translation_vector(
    const std::vector<TranslationVector> &vectors,
    const std::optional<geom::Vector2> &previous_direction,
    const geom::Point2 &stationary_centroid,
    const geom::Point2 &orbiting_position) -> const TranslationVector * {
  if (vectors.empty()) {
    return nullptr;
  }

  const auto radial = geom::normalize_vector(
      geom::vector_between(stationary_centroid, orbiting_position),
      kContactTolerance);
  const geom::Vector2 ccw_preferred{.x = -radial.y, .y = radial.x};

  const TranslationVector *best = &vectors.front();
  double best_score = -std::numeric_limits<double>::infinity();
  for (const auto &vector : vectors) {
    double score = geom::dot(vector.direction, ccw_preferred);
    if (previous_direction.has_value()) {
      score += 0.5 * geom::dot(vector.direction, *previous_direction);
    }
    if (vector.max_distance < kMinTranslation) {
      score -= 100.0;
    }
    if (score > best_score + 1e-9 ||
        (std::abs(score - best_score) <= 1e-9 &&
         vector.max_distance > best->max_distance)) {
      best_score = score;
      best = &vector;
    }
  }
  return best;
}

struct CollisionEvent {
  double distance{0.0};
};

[[nodiscard]] auto ray_segment_intersection(const geom::Point2 &ray_origin,
                                            const geom::Vector2 &ray_direction,
                                            const geom::Segment2 &segment)
    -> std::optional<double> {
  const auto segment_direction =
      geom::vector_between(segment.start, segment.end);
  const auto denominator = geom::cross(ray_direction, segment_direction);
  if (std::abs(denominator) <= kContactTolerance) {
    return std::nullopt;
  }

  const auto diff = geom::vector_between(ray_origin, segment.start);
  const auto t = geom::cross(diff, segment_direction) / denominator;
  const auto u = geom::cross(diff, ray_direction) / denominator;
  if (t >= -kContactTolerance && u >= -kContactTolerance &&
      u <= 1.0 + kContactTolerance) {
    return std::max(t, 0.0);
  }
  return std::nullopt;
}

[[nodiscard]] auto check_translation_collision(const geom::Ring &stationary,
                                               const geom::Ring &orbiting,
                                               const geom::Vector2 &translation)
    -> std::optional<CollisionEvent> {
  const auto translation_length = geom::vector_length(translation);
  if (translation_length <= kContactTolerance) {
    return std::nullopt;
  }

  const auto direction = geom::normalize_vector(translation, kContactTolerance);
  std::optional<CollisionEvent> earliest{};

  for (std::size_t orbiting_index = 0; orbiting_index < orbiting.size();
       ++orbiting_index) {
    for (std::size_t stationary_index = 0; stationary_index < stationary.size();
         ++stationary_index) {
      const auto intersection =
          ray_segment_intersection(orbiting[orbiting_index], direction,
                                   edge_segment(stationary, stationary_index));
      if (!intersection.has_value()) {
        continue;
      }
      const auto distance = *intersection;
      if (distance <= kContactTolerance ||
          distance >= translation_length - kContactTolerance) {
        continue;
      }
      if (!earliest.has_value() || distance < earliest->distance) {
        earliest = CollisionEvent{.distance = distance};
      }
    }
  }

  const auto reverse = geom::scale_vector(direction, -1.0);
  for (std::size_t stationary_index = 0; stationary_index < stationary.size();
       ++stationary_index) {
    for (std::size_t orbiting_index = 0; orbiting_index < orbiting.size();
         ++orbiting_index) {
      const auto intersection =
          ray_segment_intersection(stationary[stationary_index], reverse,
                                   edge_segment(orbiting, orbiting_index));
      if (!intersection.has_value()) {
        continue;
      }
      const auto distance = *intersection;
      if (distance <= kContactTolerance ||
          distance >= translation_length - kContactTolerance) {
        continue;
      }
      if (!earliest.has_value() || distance < earliest->distance) {
        earliest = CollisionEvent{.distance = distance};
      }
    }
  }

  return earliest;
}

[[nodiscard]] auto recover_contact(const geom::Ring &stationary,
                                   const geom::Ring &orbiting,
                                   const geom::Point2 &current_position)
    -> std::optional<geom::Point2> {
  double minimum_distance = std::numeric_limits<double>::infinity();
  std::optional<geom::Point2> closest_point{};

  for (const auto &orbiting_vertex : orbiting) {
    for (std::size_t stationary_index = 0; stationary_index < stationary.size();
         ++stationary_index) {
      const auto candidate = geom::closest_point_on_segment(
          orbiting_vertex, edge_segment(stationary, stationary_index));
      const auto distance = geom::point_distance(orbiting_vertex, candidate);
      if (distance < minimum_distance) {
        minimum_distance = distance;
        closest_point = candidate;
      }
    }
  }

  if (!closest_point.has_value() || minimum_distance <= kContactTolerance) {
    return std::nullopt;
  }

  const auto centroid = ring_centroid(orbiting);
  const auto direction = geom::normalize_vector(
      geom::vector_between(centroid, *closest_point), kContactTolerance);
  const auto move_distance = minimum_distance - kContactTolerance * 0.5;
  if (move_distance <= kMinTranslation) {
    return std::nullopt;
  }

  return geom::point_plus_vector(current_position,
                                 geom::scale_vector(direction, move_distance));
}

[[nodiscard]] auto build_path_polygon(const std::vector<geom::Point2> &path)
    -> util::StatusOr<geom::PolygonWithHoles> {
  if (path.size() < 3U) {
    return util::Status::computation_failed;
  }

  auto simplified = poly::simplify_collinear_ring(path);
  if (simplified.size() < 3U) {
    return util::Status::computation_failed;
  }

  const auto polygon = geom::normalize_polygon(geom::PolygonWithHoles{
      .outer = std::move(simplified),
  });
  if (!geom::validate_polygon(polygon).is_valid() ||
      geom::polygon_area(polygon) <= kContactTolerance * kContactTolerance) {
    return util::Status::computation_failed;
  }

  return polygon;
}

[[nodiscard]] auto trace_nfp_boundary(const geom::Ring &stationary,
                                      const geom::Ring &orbiting,
                                      const geom::Point2 &start_position)
    -> util::StatusOr<geom::PolygonWithHoles> {
  std::vector<geom::Point2> path;
  path.reserve(128U);
  path.push_back(start_position);

  geom::Point2 current_position = start_position;
  std::optional<geom::Vector2> previous_direction{};
  const auto stationary_centroid = ring_centroid(stationary);
  std::size_t stuck_counter = 0U;

  // Scale-relative slide cap (see kMaxSlideStepDiameterFraction). Use
  // the combined bbox diameter so that the cap is comparable for both
  // tiny features and large sheets without changing units.
  const auto stationary_bounds = geom::compute_bounds(
      std::span<const geom::Point2>(stationary.data(), stationary.size()));
  const auto orbiting_bounds = geom::compute_bounds(
      std::span<const geom::Point2>(orbiting.data(), orbiting.size()));
  const auto stationary_diameter =
      geom::point_distance(stationary_bounds.min, stationary_bounds.max);
  const auto orbiting_diameter =
      geom::point_distance(orbiting_bounds.min, orbiting_bounds.max);
  const auto max_slide_distance =
      std::max(kMinTranslation, kMaxSlideStepDiameterFraction *
                                    (stationary_diameter + orbiting_diameter));

  for (std::size_t iteration = 0; iteration < kMaxIterations; ++iteration) {
    (void)iteration;
    const auto orbiting_current = translate_ring(
        orbiting, {.x = current_position.x, .y = current_position.y});
    const auto touching_group =
        create_touching_group(stationary, orbiting_current, current_position);

    if (!touching_group.has_contacts()) {
      const auto recovery =
          recover_contact(stationary, orbiting_current, current_position);
      if (!recovery.has_value()) {
        break;
      }
      current_position = *recovery;
      continue;
    }

    const auto translation_vectors =
        handle_perfect_fit(touching_group, stationary, orbiting_current, path);
    if (translation_vectors.empty()) {
      ++stuck_counter;
      if (stuck_counter > kStuckIterationLimit) {
        break;
      }
      continue;
    }
    stuck_counter = 0U;

    const auto *selected =
        select_translation_vector(translation_vectors, previous_direction,
                                  stationary_centroid, current_position);
    if (selected == nullptr) {
      break;
    }

    const auto intended_distance =
        std::min(selected->max_distance, max_slide_distance);
    if (intended_distance < kMinTranslation) {
      break;
    }

    const auto intended_translation =
        geom::scale_vector(selected->direction, intended_distance);
    const auto collision = check_translation_collision(
        stationary, orbiting_current, intended_translation);
    const auto actual_distance =
        collision.has_value()
            ? std::max(collision->distance - kContactTolerance, 0.0)
            : intended_distance;
    if (actual_distance < kMinTranslation) {
      break;
    }

    const auto new_position = geom::point_plus_vector(
        current_position,
        geom::scale_vector(selected->direction, actual_distance));
    if (path.size() > 2U && geom::point_distance(new_position, start_position) <
                                kContactTolerance * 10.0) {
      break;
    }

    if (geom::point_distance(new_position, current_position) >
        kMinTranslation) {
      path.push_back(new_position);
      previous_direction = selected->direction;
    }
    current_position = new_position;
  }

  return build_path_polygon(path);
}

// Start on the bottom-most vertex of the STATIONARY CONVEX HULL rather
// than the raw ring so the initial touch cannot be buried in a concave
// pocket. The orbiting anchor remains the top-most raw vertex.
// Find the orbital trace start position: place the orbiting reference
// such that its top-most vertex contacts the bottom-most vertex of the
// stationary convex hull.
//
// Precondition: `stationary_hull` and `orbiting` MUST be non-empty.
// Both rings are dereferenced unconditionally; callers (currently only
// `compute_simple_orbiting_nfp`) gate this with a `size() < 3` check.
[[nodiscard]] auto find_start_position(const geom::Ring &stationary_hull,
                                       const geom::Ring &orbiting)
    -> geom::Point2 {
  const auto stationary_it =
      std::min_element(stationary_hull.begin(), stationary_hull.end(),
                       [](const geom::Point2 &lhs, const geom::Point2 &rhs) {
                         if (lhs.y != rhs.y) {
                           return lhs.y < rhs.y;
                         }
                         return lhs.x < rhs.x;
                       });
  const auto orbiting_it =
      std::max_element(orbiting.begin(), orbiting.end(),
                       [](const geom::Point2 &lhs, const geom::Point2 &rhs) {
                         if (lhs.y != rhs.y) {
                           return lhs.y < rhs.y;
                         }
                         return lhs.x > rhs.x;
                       });

  return {.x = stationary_it->x - orbiting_it->x,
          .y = stationary_it->y - orbiting_it->y};
}

enum class ExtremeAxis {
  x,
  y,
};

[[nodiscard]] auto collect_extreme_vertices(const geom::Ring &ring,
                                            const ExtremeAxis axis,
                                            const bool want_min)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> extremes;
  if (ring.empty()) {
    return extremes;
  }

  auto primary = [&](const geom::Point2 &point) {
    return axis == ExtremeAxis::x ? point.x : point.y;
  };

  double extreme_value = primary(ring.front());
  for (const auto &point : ring) {
    const double value = primary(point);
    if ((want_min && value < extreme_value - kContactTolerance) ||
        (!want_min && value > extreme_value + kContactTolerance)) {
      extreme_value = value;
    }
  }

  for (const auto &point : ring) {
    if (std::abs(primary(point) - extreme_value) <= kContactTolerance) {
      extremes.push_back(point);
    }
  }

  return extremes;
}

auto append_start_positions(std::vector<geom::Point2> &start_positions,
                            const std::vector<geom::Point2> &stationary_points,
                            const std::vector<geom::Point2> &orbiting_points)
    -> void {
  for (const auto &stationary_point : stationary_points) {
    for (const auto &orbiting_point : orbiting_points) {
      const geom::Point2 candidate{.x = stationary_point.x - orbiting_point.x,
                                   .y = stationary_point.y - orbiting_point.y};
      const bool seen = std::any_of(
          start_positions.begin(), start_positions.end(),
          [&](const geom::Point2 &existing) {
            return near_same_point(existing, candidate, kContactTolerance);
          });
      if (!seen) {
        start_positions.push_back(candidate);
      }
    }
  }
}

[[nodiscard]] auto candidate_start_positions(const geom::Ring &stationary_hull,
                                             const geom::Ring &orbiting_hull)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> start_positions;
  start_positions.reserve(16U);
  start_positions.push_back(
      find_start_position(stationary_hull, orbiting_hull));

  append_start_positions(
      start_positions,
      collect_extreme_vertices(stationary_hull, ExtremeAxis::y, true),
      collect_extreme_vertices(orbiting_hull, ExtremeAxis::y, false));
  append_start_positions(
      start_positions,
      collect_extreme_vertices(stationary_hull, ExtremeAxis::x, true),
      collect_extreme_vertices(orbiting_hull, ExtremeAxis::x, false));
  append_start_positions(
      start_positions,
      collect_extreme_vertices(stationary_hull, ExtremeAxis::y, false),
      collect_extreme_vertices(orbiting_hull, ExtremeAxis::y, true));
  append_start_positions(
      start_positions,
      collect_extreme_vertices(stationary_hull, ExtremeAxis::x, false),
      collect_extreme_vertices(orbiting_hull, ExtremeAxis::x, true));
  return start_positions;
}

[[nodiscard]] auto
compute_simple_orbiting_nfp(const geom::PolygonWithHoles &fixed,
                            const geom::PolygonWithHoles &moving)
    -> util::StatusOr<geom::PolygonWithHoles> {
  if (!fixed.holes.empty() || !moving.holes.empty()) {
    return util::Status::invalid_input;
  }

  const auto stationary = ensure_ccw(fixed.outer);
  const auto stationary_hull = ensure_ccw(
      poly::compute_convex_hull(geom::Polygon{.outer = stationary}).outer);
  const auto orbiting = ensure_ccw(moving.outer);
  const auto orbiting_hull = ensure_ccw(
      poly::compute_convex_hull(geom::Polygon{.outer = orbiting}).outer);
  if (stationary.size() < 3U || stationary_hull.size() < 3U ||
      orbiting.size() < 3U || orbiting_hull.size() < 3U) {
    return util::Status::invalid_input;
  }

  for (const auto &start_position :
       candidate_start_positions(stationary_hull, orbiting_hull)) {
    const auto orbiting_at_start = translate_ring(
        orbiting, {.x = start_position.x, .y = start_position.y});
    auto overlap = has_actual_overlap(stationary, orbiting_at_start);
    if (!overlap.ok()) {
      return overlap.status();
    }
    if (overlap.value()) {
      continue;
    }

    if (auto traced = trace_nfp_boundary(stationary, orbiting, start_position);
        traced.ok()) {
      return traced;
    }
  }

  return util::Status::computation_failed;
}

} // namespace

auto compute_orbiting_nfp(const geom::PolygonWithHoles &fixed,
                          const geom::PolygonWithHoles &moving)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  const auto normalized_fixed = geom::sanitize_polygon(fixed).polygon;
  const auto normalized_moving = geom::sanitize_polygon(moving).polygon;

  if (!geom::validate_polygon(normalized_fixed).is_valid() ||
      !geom::validate_polygon(normalized_moving).is_valid()) {
    SHINY_DEBUG("orbiting_nfp: invalid input fixed_outer={} moving_outer={}",
                normalized_fixed.outer.size(), normalized_moving.outer.size());
    return util::Status::invalid_input;
  }

  const auto vertex_product =
      normalized_fixed.outer.size() * normalized_moving.outer.size();
  if (vertex_product > kMaxOrbitingVertexProduct) {
    SHINY_DEBUG(
        "orbiting_nfp: skipping high-complexity fallback fixed_outer={} "
        "moving_outer={} vertex_product={}",
        normalized_fixed.outer.size(), normalized_moving.outer.size(),
        vertex_product);
    return util::Status::computation_failed;
  }

  if (normalized_fixed.holes.empty() && normalized_moving.holes.empty() &&
      decomp::is_convex(geom::Polygon{.outer = normalized_fixed.outer}) &&
      decomp::is_convex(geom::Polygon{.outer = normalized_moving.outer})) {
    if (auto simple_nfp =
            compute_simple_orbiting_nfp(normalized_fixed, normalized_moving);
        simple_nfp.ok()) {
      return std::vector<geom::PolygonWithHoles>{std::move(simple_nfp).value()};
    }
  }

  auto fixed_parts_or = decomp::decompose_convex(normalized_fixed);
  if (!fixed_parts_or.ok()) {
    return fixed_parts_or.status();
  }
  auto moving_parts_or = decomp::decompose_convex(normalized_moving);
  if (!moving_parts_or.ok()) {
    return moving_parts_or.status();
  }

  std::vector<geom::PolygonWithHoles> aggregate;
  const auto &fixed_parts = fixed_parts_or.value();
  const auto &moving_parts = moving_parts_or.value();
  aggregate.reserve(fixed_parts.size() * moving_parts.size());

  for (const auto &fixed_part : fixed_parts) {
    for (const auto &moving_part : moving_parts) {
      auto pair_nfp = compute_simple_orbiting_nfp(
          geom::PolygonWithHoles{.outer = fixed_part.outer},
          geom::PolygonWithHoles{.outer = moving_part.outer});
      if (pair_nfp.ok()) {
        aggregate.push_back(std::move(pair_nfp).value());
        continue;
      }

      auto convex_nfp = compute_convex_nfp(fixed_part, moving_part);
      if (!convex_nfp.ok()) {
        return convex_nfp.status();
      }
      aggregate.push_back(std::move(convex_nfp).value());
    }
  }

  if (aggregate.empty()) {
    return util::Status::computation_failed;
  }

  return merge_polygon_set(std::move(aggregate));
}

} // namespace shiny::nesting::nfp
