#include "nfp/nonconvex_nfp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "decomposition/decompose.hpp"
#include "geometry/detail/point_compare.hpp"
#include "geometry/normalize.hpp"
#include "nfp/convex_nfp.hpp"
#include "polygon_ops/simplify.hpp"
#include "predicates/point_location.hpp"
#include "predicates/segment_intersection.hpp"

namespace {

using shiny::nfp::ArrangementGraph;
using shiny::nfp::ConvexNfpRequest;
using shiny::nfp::ExtractionStatus;
using shiny::nfp::GraphEdge;
using shiny::nfp::GraphEdgeKind;
using shiny::nfp::GraphVertex;
using shiny::nfp::GraphVertexKind;
using shiny::nfp::NfpFeatureKind;
using shiny::nfp::NfpLoop;
using shiny::nfp::NfpResult;
using shiny::nfp::NonconvexNfpRequest;
using shiny::nfp::NonconvexNfpResult;
using shiny::nfp::decomp::DecompositionAlgorithm;
using shiny::nfp::decomp::DecompositionResult;
using shiny::nfp::decomp::DecompositionValidity;
using shiny::nfp::geom::Point2;
using shiny::nfp::geom::Polygon;
using shiny::nfp::geom::Ring;
using shiny::nfp::geom::Segment2;

constexpr double full_turn_radians = 6.28318530717958647692;
constexpr double canonical_coordinate_scale = 1000000000.0;

using CoordinateKey = std::int64_t;

[[nodiscard]] auto interrupted(const std::function<bool()> &probe) -> bool {
  return probe && probe();
}

[[nodiscard]] auto canonicalize_coordinate(double value) -> double {
  return std::nearbyint(value * canonical_coordinate_scale) /
         canonical_coordinate_scale;
}

[[nodiscard]] auto coordinate_key(double value) -> CoordinateKey {
  return static_cast<CoordinateKey>(
      std::llround(value * canonical_coordinate_scale));
}

[[nodiscard]] auto canonicalize_point(const Point2 &point) -> Point2 {
  return {
      canonicalize_coordinate(point.x),
      canonicalize_coordinate(point.y),
  };
}

struct PointKey {
  CoordinateKey x{0};
  CoordinateKey y{0};

  auto operator==(const PointKey &other) const -> bool {
    return x == other.x && y == other.y;
  }
};

[[nodiscard]] auto point_from_key(const PointKey &key) -> Point2 {
  return {
      static_cast<double>(key.x) / canonical_coordinate_scale,
      static_cast<double>(key.y) / canonical_coordinate_scale,
  };
}

struct GeometricEdgeKey {
  PointKey first{};
  PointKey second{};

  auto operator==(const GeometricEdgeKey &other) const -> bool {
    return first == other.first && second == other.second;
  }
};

struct PointKeyHash {
  auto operator()(const PointKey &key) const noexcept -> std::size_t {
    std::size_t seed = std::hash<CoordinateKey>{}(key.x);
    seed ^= std::hash<CoordinateKey>{}(key.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct GeometricEdgeKeyHash {
  auto operator()(const GeometricEdgeKey &key) const noexcept -> std::size_t {
    const auto hash_coordinate = [](CoordinateKey value) {
      return std::hash<CoordinateKey>{}(value);
    };

    std::size_t seed = hash_coordinate(key.first.x);
    seed ^= hash_coordinate(key.first.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    seed ^= hash_coordinate(key.second.x) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    seed ^= hash_coordinate(key.second.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

[[nodiscard]] auto point_key(const Point2 &point) -> PointKey;

[[nodiscard]] auto point_equal(const Point2 &lhs, const Point2 &rhs) -> bool {
  return point_key(lhs) == point_key(rhs);
}

[[nodiscard]] auto point_key(const Point2 &point) -> PointKey {
  return {.x = coordinate_key(point.x), .y = coordinate_key(point.y)};
}

[[nodiscard]] auto midpoint(const Point2 &lhs, const Point2 &rhs) -> Point2 {
  return canonicalize_point({
      (lhs.x + rhs.x) * 0.5,
      (lhs.y + rhs.y) * 0.5,
  });
}

[[nodiscard]] auto
segment_from_edge(const GraphEdge &edge,
                  const std::unordered_map<std::uint32_t, Point2> &points_by_id)
    -> Segment2 {
  return {
      points_by_id.at(edge.from),
      points_by_id.at(edge.to),
  };
}

[[nodiscard]] auto segment_parameter(const Segment2 &segment,
                                     const Point2 &point) -> double {
  const auto dx = segment.end.x - segment.start.x;
  const auto dy = segment.end.y - segment.start.y;
  if (std::fabs(dx) >= std::fabs(dy) && dx != 0.0) {
    return (point.x - segment.start.x) / dx;
  }
  if (dy != 0.0) {
    return (point.y - segment.start.y) / dy;
  }
  return 0.0;
}

[[nodiscard]] auto normalized_geometric_edge_key(const Point2 &lhs,
                                                 const Point2 &rhs)
    -> GeometricEdgeKey {
  const auto lhs_key = point_key(lhs);
  const auto rhs_key = point_key(rhs);
  if (rhs_key.x < lhs_key.x ||
      (rhs_key.x == lhs_key.x && rhs_key.y < lhs_key.y)) {
    return {.first = rhs_key, .second = lhs_key};
  }
  return {.first = lhs_key, .second = rhs_key};
}

[[nodiscard]] auto geometric_edge_key_ignored(
    const GeometricEdgeKey &key,
    const std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash>
        &ignored_keys) -> bool {
  return ignored_keys.find(key) != ignored_keys.end();
}

[[nodiscard]] auto signed_cross(const Point2 &from, const Point2 &to,
                                const Point2 &point) -> long double;

[[nodiscard]] auto share_endpoint(const GeometricEdgeKey &lhs,
                                  const GeometricEdgeKey &rhs) -> bool {
  return lhs.first == rhs.first || lhs.first == rhs.second ||
         lhs.second == rhs.first || lhs.second == rhs.second;
}

[[nodiscard]] auto are_collinear(const GeometricEdgeKey &lhs,
                                 const GeometricEdgeKey &rhs) -> bool {
  const auto lhs_start = point_from_key(lhs.first);
  const auto lhs_end = point_from_key(lhs.second);
  return signed_cross(lhs_start, lhs_end, point_from_key(rhs.first)) == 0.0L &&
         signed_cross(lhs_start, lhs_end, point_from_key(rhs.second)) == 0.0L;
}

[[nodiscard]] auto connected_duplicate_component(
    const GeometricEdgeKey &seed,
    const std::vector<GeometricEdgeKey> &duplicate_keys)
    -> std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash> {
  std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash> component;
  std::vector<GeometricEdgeKey> pending{seed};

  while (!pending.empty()) {
    const auto current = pending.back();
    pending.pop_back();
    if (!component.insert(current).second) {
      continue;
    }

    for (const auto &candidate : duplicate_keys) {
      if (component.find(candidate) != component.end()) {
        continue;
      }
      if (share_endpoint(current, candidate) &&
          are_collinear(current, candidate)) {
        pending.push_back(candidate);
      }
    }
  }

  return component;
}

[[nodiscard]] auto signed_cross(const Point2 &from, const Point2 &to,
                                const Point2 &point) -> long double {
  return static_cast<long double>(to.x - from.x) * (point.y - from.y) -
         static_cast<long double>(to.y - from.y) * (point.x - from.x);
}

[[nodiscard]] auto vertex_kind_priority(GraphVertexKind kind) -> int {
  switch (kind) {
  case GraphVertexKind::original_vertex:
    return 0;
  case GraphVertexKind::midpoint_vertex:
    return 1;
  case GraphVertexKind::intersection_vertex:
    return 2;
  }

  return 0;
}

[[nodiscard]] auto merge_vertex_kind(GraphVertexKind lhs, GraphVertexKind rhs)
    -> GraphVertexKind {
  return vertex_kind_priority(lhs) >= vertex_kind_priority(rhs) ? lhs : rhs;
}

[[nodiscard]] auto edge_angle(const Point2 &from, const Point2 &to) -> double {
  return std::atan2(to.y - from.y, to.x - from.x);
}

[[nodiscard]] auto normalized_turn_angle(const Point2 &previous,
                                         const Point2 &current,
                                         const Point2 &next) -> double {
  auto turn = edge_angle(current, next) - edge_angle(previous, current);
  while (turn <= 0.0) {
    turn += full_turn_radians;
  }
  while (turn > full_turn_radians) {
    turn -= full_turn_radians;
  }
  return turn;
}

[[nodiscard]] auto ring_less(const Ring &lhs, const Ring &rhs) -> bool {
  const auto limit = std::min(lhs.size(), rhs.size());
  for (std::size_t index = 0; index < limit; ++index) {
    if (point_equal(lhs[index], rhs[index])) {
      continue;
    }
    return shiny::nfp::detail::point_less(lhs[index], rhs[index]);
  }
  return lhs.size() < rhs.size();
}

[[nodiscard]] auto ring_equal(const Ring &lhs, const Ring &rhs) -> bool {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!point_equal(lhs[index], rhs[index])) {
      return false;
    }
  }

  return true;
}

[[nodiscard]] auto ring_signed_area(const Ring &ring) -> long double {
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

auto rotate_to_lexicographic_min(Ring &ring) -> void {
  if (ring.empty()) {
    return;
  }

  const auto min_index = shiny::nfp::pred::lexicographic_min_vertex_index(ring);
  std::rotate(ring.begin(),
              ring.begin() + static_cast<std::ptrdiff_t>(min_index),
              ring.end());
}

[[nodiscard]] auto normalize_loop(Ring ring, NfpFeatureKind kind) -> Ring {
  if (ring.size() > 1U && point_equal(ring.front(), ring.back())) {
    ring.pop_back();
  }

  Ring cleaned;
  cleaned.reserve(ring.size());
  for (const auto &point : ring) {
    if (!cleaned.empty() && point_equal(cleaned.back(), point)) {
      continue;
    }
    cleaned.push_back(point);
  }

  if (cleaned.size() < 3U) {
    return {};
  }

  const auto area = ring_signed_area(cleaned);
  if (area == 0.0L) {
    return {};
  }

  const bool should_be_counterclockwise = kind == NfpFeatureKind::outer_loop;
  if ((area > 0.0L) != should_be_counterclockwise) {
    std::reverse(cleaned.begin(), cleaned.end());
  }
  rotate_to_lexicographic_min(cleaned);
  return cleaned;
}

[[nodiscard]] auto interior_sample_for_ring(const Ring &ring)
    -> std::optional<Point2> {
  if (ring.size() < 3U) {
    return std::nullopt;
  }

  const auto area = ring_signed_area(ring);
  if (area == 0.0L) {
    return std::nullopt;
  }

  const auto inward_sign = area > 0.0L ? 1.0 : -1.0;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    const auto &from = ring[index];
    const auto &to = ring[next_index];
    const auto dx = to.x - from.x;
    const auto dy = to.y - from.y;
    const auto length = std::hypot(dx, dy);
    if (length == 0.0) {
      continue;
    }

    const auto base = midpoint(from, to);
    const Point2 inward_normal{
        canonicalize_coordinate((-dy / length) * inward_sign),
        canonicalize_coordinate((dx / length) * inward_sign),
    };

    for (const auto scale : {1.0e-6, 1.0e-5, 1.0e-4}) {
      const auto distance = std::max(length * scale, scale);
      const auto sample =
          canonicalize_point({base.x + inward_normal.x * distance,
                              base.y + inward_normal.y * distance});
      const auto location =
          shiny::nfp::pred::locate_point_in_ring(sample, ring);
      if (location.location == shiny::nfp::pred::PointLocation::interior) {
        return sample;
      }
    }
  }

  return std::nullopt;
}

[[nodiscard]] auto find_repeated_vertex_span(const Ring &ring)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  std::unordered_map<PointKey, std::size_t, PointKeyHash> seen_vertices;
  seen_vertices.reserve(ring.size());

  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto [it, inserted] =
        seen_vertices.emplace(point_key(ring[index]), index);
    if (!inserted) {
      return std::make_pair(it->second, index);
    }
  }

  return std::nullopt;
}

[[nodiscard]] auto slice_ring(const Ring &ring, std::size_t begin,
                              std::size_t end) -> Ring {
  return {ring.begin() + static_cast<std::ptrdiff_t>(begin),
          ring.begin() + static_cast<std::ptrdiff_t>(end)};
}

[[nodiscard]] auto cyclic_remainder(const Ring &ring, std::size_t first,
                                    std::size_t second) -> Ring {
  Ring remainder;
  remainder.reserve(ring.size() - (second - first));
  for (std::size_t index = second; index < ring.size(); ++index) {
    remainder.push_back(ring[index]);
  }
  for (std::size_t index = 0; index < first; ++index) {
    remainder.push_back(ring[index]);
  }
  return remainder;
}

[[nodiscard]] auto segments_are_adjacent(std::size_t lhs_index,
                                         std::size_t rhs_index,
                                         std::size_t segment_count) -> bool {
  if (lhs_index == rhs_index) {
    return true;
  }
  if ((lhs_index + 1U) % segment_count == rhs_index) {
    return true;
  }
  if ((rhs_index + 1U) % segment_count == lhs_index) {
    return true;
  }
  return false;
}

[[nodiscard]] auto ring_has_self_intersection(const Ring &ring) -> bool {
  if (ring.size() < 3U) {
    return true;
  }

  for (std::size_t lhs_index = 0; lhs_index < ring.size(); ++lhs_index) {
    const auto lhs_next = (lhs_index + 1U) % ring.size();
    const Segment2 lhs_segment{ring[lhs_index], ring[lhs_next]};
    for (std::size_t rhs_index = lhs_index + 1U; rhs_index < ring.size();
         ++rhs_index) {
      if (segments_are_adjacent(lhs_index, rhs_index, ring.size())) {
        continue;
      }

      const auto rhs_next = (rhs_index + 1U) % ring.size();
      const Segment2 rhs_segment{ring[rhs_index], ring[rhs_next]};
      const auto contact =
          shiny::nfp::pred::classify_segment_contact(lhs_segment, rhs_segment);
      if (contact.kind != shiny::nfp::pred::SegmentContactKind::disjoint &&
          contact.kind !=
              shiny::nfp::pred::SegmentContactKind::parallel_disjoint) {
        return true;
      }
    }
  }

  return false;
}

[[nodiscard]] auto extract_simple_cycles(const Ring &ring)
    -> std::vector<Ring> {
  std::vector<Ring> pending;
  std::vector<Ring> simple_cycles;
  pending.push_back(ring);

  while (!pending.empty()) {
    auto current = std::move(pending.back());
    pending.pop_back();
    if (current.size() < 3U) {
      continue;
    }

    if (const auto repeated = find_repeated_vertex_span(current)) {
      auto repeated_cycle =
          slice_ring(current, repeated->first, repeated->second);
      auto remainder =
          cyclic_remainder(current, repeated->first, repeated->second);
      if (repeated_cycle.size() >= 3U) {
        pending.push_back(std::move(repeated_cycle));
      }
      if (remainder.size() >= 3U) {
        pending.push_back(std::move(remainder));
      }
      continue;
    }

    if (ring_has_self_intersection(current)) {
      continue;
    }

    simple_cycles.push_back(std::move(current));
  }

  return simple_cycles;
}

[[nodiscard]] auto point_on_boundary(const Point2 &point,
                                     const std::vector<NfpLoop> &loops)
    -> bool {
  for (const auto &loop : loops) {
    for (std::size_t index = 0; index < loop.vertices.size(); ++index) {
      const auto next_index = (index + 1U) % loop.vertices.size();
      const auto relation = shiny::nfp::pred::locate_point_on_segment(
          point, Segment2{loop.vertices[index], loop.vertices[next_index]});
      if (relation.relation !=
          shiny::nfp::pred::BoundaryRelation::off_boundary) {
        return true;
      }
    }
  }
  return false;
}

[[nodiscard]] auto overlap_interval_on_segment(const Segment2 &carrier,
                                               const Segment2 &candidate)
    -> std::optional<std::pair<double, double>> {
  const auto contact =
      shiny::nfp::pred::classify_segment_contact(carrier, candidate);
  if (contact.kind != shiny::nfp::pred::SegmentContactKind::collinear_overlap ||
      contact.point_count != 2U) {
    return std::nullopt;
  }

  const auto first = segment_parameter(carrier, contact.points[0]);
  const auto second = segment_parameter(carrier, contact.points[1]);
  return std::make_pair(std::min(first, second), std::max(first, second));
}

[[nodiscard]] auto
segment_fully_covered_by_loops(const Segment2 &segment,
                               const std::vector<NfpLoop> &loops) -> bool {
  std::vector<std::pair<double, double>> intervals;
  for (const auto &loop : loops) {
    if (loop.vertices.size() < 2U) {
      continue;
    }

    intervals.reserve(intervals.size() + loop.vertices.size());
    for (std::size_t index = 0; index < loop.vertices.size(); ++index) {
      const auto next_index = (index + 1U) % loop.vertices.size();
      const auto overlap = overlap_interval_on_segment(
          segment, Segment2{loop.vertices[index], loop.vertices[next_index]});
      if (overlap) {
        intervals.push_back(*overlap);
      }
    }
  }

  if (intervals.empty()) {
    return false;
  }

  std::sort(intervals.begin(), intervals.end());
  double covered_until = 0.0;
  constexpr double epsilon = 1.0e-12;
  for (const auto &[start, end] : intervals) {
    if (start > covered_until + epsilon) {
      return false;
    }
    covered_until = std::max(covered_until, end);
    if (covered_until >= 1.0 - epsilon) {
      return true;
    }
  }

  return covered_until >= 1.0 - epsilon;
}

[[nodiscard]] auto segment_on_boundary(const Segment2 &segment,
                                       const std::vector<NfpLoop> &loops)
    -> bool {
  if (loops.empty()) {
    return false;
  }

  const auto canonical_segment = Segment2{canonicalize_point(segment.start),
                                          canonicalize_point(segment.end)};
  return segment_fully_covered_by_loops(canonical_segment, loops);
}

[[nodiscard]] auto point_on_graph_boundary(
    const Point2 &point,
    const std::unordered_map<std::uint32_t, Point2> &points_by_id,
    const std::vector<GraphEdge> &edges,
    const std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash>
        &ignored_keys,
    const std::function<bool()> &interruption_requested = {}) -> bool {
  for (const auto &edge : edges) {
    if (interrupted(interruption_requested)) {
      return false;
    }

    if (edge.kind == GraphEdgeKind::pruned_edge) {
      continue;
    }
    if (geometric_edge_key_ignored(
            normalized_geometric_edge_key(points_by_id.at(edge.from),
                                          points_by_id.at(edge.to)),
            ignored_keys)) {
      continue;
    }

    const auto relation = shiny::nfp::pred::locate_point_on_segment(
        point, Segment2{points_by_id.at(edge.from), points_by_id.at(edge.to)});
    if (relation.relation != shiny::nfp::pred::BoundaryRelation::off_boundary) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto winding_count_at_point(
    const Point2 &point,
    const std::unordered_map<std::uint32_t, Point2> &points_by_id,
    const std::vector<GraphEdge> &edges,
    const std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash>
        &ignored_keys,
    const std::function<bool()> &interruption_requested = {}) -> int {
  int winding_count = 0;
  for (const auto &edge : edges) {
    if (interrupted(interruption_requested)) {
      return 0;
    }

    if (edge.kind == GraphEdgeKind::pruned_edge) {
      continue;
    }
    if (geometric_edge_key_ignored(
            normalized_geometric_edge_key(points_by_id.at(edge.from),
                                          points_by_id.at(edge.to)),
            ignored_keys)) {
      continue;
    }

    const auto &from = points_by_id.at(edge.from);
    const auto &to = points_by_id.at(edge.to);
    if (from.y <= point.y) {
      if (to.y > point.y && signed_cross(from, to, point) > 0.0L) {
        ++winding_count;
      }
      continue;
    }

    if (to.y <= point.y && signed_cross(from, to, point) < 0.0L) {
      --winding_count;
    }
  }

  return winding_count;
}

[[nodiscard]] auto sample_segment_side_occupancy(
    const Segment2 &segment,
    const std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash>
        &ignored_keys,
    const std::unordered_map<std::uint32_t, Point2> &points_by_id,
    const std::vector<GraphEdge> &edges,
    const std::function<bool()> &interruption_requested = {})
    -> std::optional<std::pair<int, int>> {
  const auto dx = segment.end.x - segment.start.x;
  const auto dy = segment.end.y - segment.start.y;
  const auto length = std::hypot(dx, dy);
  if (length == 0.0) {
    return std::nullopt;
  }

  const auto base = midpoint(segment.start, segment.end);
  const Point2 left_normal{
      canonicalize_coordinate(-dy / length),
      canonicalize_coordinate(dx / length),
  };

  for (const auto scale : {1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3}) {
    if (interrupted(interruption_requested)) {
      return std::nullopt;
    }

    const auto distance = std::max(length * scale, scale);
    const auto left_sample = canonicalize_point(
        {base.x + left_normal.x * distance, base.y + left_normal.y * distance});
    const auto right_sample = canonicalize_point(
        {base.x - left_normal.x * distance, base.y - left_normal.y * distance});
    if (point_on_graph_boundary(left_sample, points_by_id, edges, ignored_keys,
                                interruption_requested) ||
        point_on_graph_boundary(right_sample, points_by_id, edges, ignored_keys,
                                interruption_requested)) {
      continue;
    }

    const auto left_winding = winding_count_at_point(
        left_sample, points_by_id, edges, ignored_keys, interruption_requested);
    const auto right_winding =
        winding_count_at_point(right_sample, points_by_id, edges, ignored_keys,
                               interruption_requested);
    return std::make_pair(left_winding, right_winding);
  }

  return std::nullopt;
}

auto filter_degenerate_features(NfpResult &result) -> void {
  auto fit_out = result.perfect_fit_points.begin();
  for (auto point_it = result.perfect_fit_points.begin();
       point_it != result.perfect_fit_points.end(); ++point_it) {
    const auto canonical_point = canonicalize_point(*point_it);
    if (point_on_boundary(canonical_point, result.loops)) {
      *fit_out++ = canonical_point;
    }
  }
  result.perfect_fit_points.erase(fit_out, result.perfect_fit_points.end());

  auto segment_out = result.perfect_sliding_segments.begin();
  for (auto segment_it = result.perfect_sliding_segments.begin();
       segment_it != result.perfect_sliding_segments.end(); ++segment_it) {
    if (segment_on_boundary(*segment_it, result.loops)) {
      *segment_out++ = Segment2{canonicalize_point(segment_it->start),
                                canonicalize_point(segment_it->end)};
    }
  }
  result.perfect_sliding_segments.erase(segment_out,
                                        result.perfect_sliding_segments.end());
}

[[nodiscard]] auto valid_decomposition(const DecompositionResult &result)
    -> bool {
  return result.validity == DecompositionValidity::valid &&
         !result.components.empty();
}

auto append_unique_point(std::vector<Point2> &points, const Point2 &point)
    -> void {
  const auto canonical_point = canonicalize_point(point);
  if (std::find_if(points.begin(), points.end(),
                   [&canonical_point](const Point2 &candidate) {
                     return point_equal(candidate, canonical_point);
                   }) != points.end()) {
    return;
  }
  points.push_back(canonical_point);
}

auto append_unique_segment(std::vector<Segment2> &segments,
                           const Segment2 &segment) -> void {
  const auto canonical_segment = Segment2{canonicalize_point(segment.start),
                                          canonicalize_point(segment.end)};
  const auto segment_key = normalized_geometric_edge_key(
      canonical_segment.start, canonical_segment.end);
  if (std::find_if(segments.begin(), segments.end(),
                   [&segment_key](const Segment2 &candidate) {
                     return normalized_geometric_edge_key(
                                candidate.start, candidate.end) == segment_key;
                   }) != segments.end()) {
    return;
  }
  segments.push_back(canonical_segment);
}

[[nodiscard]] auto classify_loop_kind(std::size_t ring_index,
                                      const std::vector<Ring> &rings)
    -> NfpFeatureKind {
  const auto sample = interior_sample_for_ring(rings[ring_index]);
  if (!sample) {
    return NfpFeatureKind::outer_loop;
  }

  std::size_t containment_depth = 0U;
  for (std::size_t index = 0; index < rings.size(); ++index) {
    if (index == ring_index) {
      continue;
    }

    const auto location =
        shiny::nfp::pred::locate_point_in_ring(*sample, rings[index]);
    if (location.location == shiny::nfp::pred::PointLocation::interior) {
      ++containment_depth;
    }
  }

  return containment_depth % 2U == 0U ? NfpFeatureKind::outer_loop
                                      : NfpFeatureKind::hole;
}

auto append_ring_to_graph(const Ring &ring, ArrangementGraph &graph) -> void {
  if (ring.size() < 2U) {
    return;
  }

  std::vector<std::uint32_t> vertex_ids;
  vertex_ids.reserve(ring.size());
  for (const auto &point : ring) {
    const auto vertex_id = static_cast<std::uint32_t>(graph.vertices.size());
    graph.vertices.push_back(
        GraphVertex{.id = vertex_id,
                    .point = canonicalize_point(point),
                    .kind = GraphVertexKind::original_vertex});
    vertex_ids.push_back(vertex_id);
  }

  for (std::size_t index = 0; index < vertex_ids.size(); ++index) {
    graph.edges.push_back(
        GraphEdge{.from = vertex_ids[index],
                  .to = vertex_ids[(index + 1U) % vertex_ids.size()],
                  .kind = GraphEdgeKind::boundary_edge});
  }
}

[[nodiscard]] auto split_graph_edges(const ArrangementGraph &graph)
    -> ArrangementGraph {
  if (graph.edges.empty()) {
    return graph;
  }

  std::unordered_map<std::uint32_t, Point2> points_by_id;
  points_by_id.reserve(graph.vertices.size());
  for (const auto &vertex : graph.vertices) {
    points_by_id.insert_or_assign(vertex.id, vertex.point);
  }

  std::vector<std::vector<Point2>> split_points(graph.edges.size());
  for (std::size_t index = 0; index < graph.edges.size(); ++index) {
    const auto segment = segment_from_edge(graph.edges[index], points_by_id);
    split_points[index].push_back(segment.start);
    split_points[index].push_back(segment.end);
  }

  for (std::size_t lhs_index = 0; lhs_index < graph.edges.size(); ++lhs_index) {
    const auto lhs_segment =
        segment_from_edge(graph.edges[lhs_index], points_by_id);
    for (std::size_t rhs_index = lhs_index + 1U; rhs_index < graph.edges.size();
         ++rhs_index) {
      const auto rhs_segment =
          segment_from_edge(graph.edges[rhs_index], points_by_id);
      const auto contact =
          shiny::nfp::pred::classify_segment_contact(lhs_segment, rhs_segment);
      if (contact.kind == shiny::nfp::pred::SegmentContactKind::disjoint ||
          contact.kind ==
              shiny::nfp::pred::SegmentContactKind::parallel_disjoint) {
        continue;
      }

      for (std::uint8_t point_index = 0; point_index < contact.point_count;
           ++point_index) {
        split_points[lhs_index].push_back(contact.points[point_index]);
        split_points[rhs_index].push_back(contact.points[point_index]);
      }
    }
  }

  ArrangementGraph split_graph{};
  split_graph.vertices = graph.vertices;
  split_graph.perfect_fit_points = graph.perfect_fit_points;
  split_graph.perfect_sliding_segments = graph.perfect_sliding_segments;
  split_graph.pruned = false;

  for (std::size_t edge_index = 0; edge_index < graph.edges.size();
       ++edge_index) {
    const auto &edge = graph.edges[edge_index];
    const auto segment = segment_from_edge(edge, points_by_id);
    auto &edge_points = split_points[edge_index];

    std::sort(edge_points.begin(), edge_points.end(),
              [&segment](const Point2 &lhs, const Point2 &rhs) {
                return segment_parameter(segment, lhs) <
                       segment_parameter(segment, rhs);
              });
    edge_points.erase(
        std::unique(edge_points.begin(), edge_points.end(), point_equal),
        edge_points.end());

    std::vector<std::uint32_t> vertex_ids(edge_points.size(), 0U);
    for (std::size_t point_index = 0; point_index < edge_points.size();
         ++point_index) {
      const auto &point = edge_points[point_index];
      if (point_equal(point, segment.start)) {
        vertex_ids[point_index] = edge.from;
        continue;
      }
      if (point_equal(point, segment.end)) {
        vertex_ids[point_index] = edge.to;
        continue;
      }

      const auto vertex_id =
          static_cast<std::uint32_t>(split_graph.vertices.size());
      split_graph.vertices.push_back(
          GraphVertex{.id = vertex_id,
                      .point = canonicalize_point(point),
                      .kind = GraphVertexKind::intersection_vertex});
      vertex_ids[point_index] = vertex_id;
    }

    for (std::size_t point_index = 0; point_index + 1U < edge_points.size();
         ++point_index) {
      if (point_equal(edge_points[point_index],
                      edge_points[point_index + 1U])) {
        continue;
      }

      split_graph.edges.push_back(
          GraphEdge{.from = vertex_ids[point_index],
                    .to = vertex_ids[point_index + 1U],
                    .kind = GraphEdgeKind::boundary_edge});
    }
  }

  return split_graph;
}

[[nodiscard]] auto insert_midpoint_vertices(const ArrangementGraph &graph)
    -> ArrangementGraph {
  ArrangementGraph midpoint_graph{};
  midpoint_graph.vertices = graph.vertices;
  midpoint_graph.perfect_fit_points = graph.perfect_fit_points;
  midpoint_graph.perfect_sliding_segments = graph.perfect_sliding_segments;
  midpoint_graph.pruned = false;

  std::unordered_map<std::uint32_t, Point2> points_by_id;
  points_by_id.reserve(graph.vertices.size());
  for (const auto &vertex : graph.vertices) {
    points_by_id.insert_or_assign(vertex.id, vertex.point);
  }

  midpoint_graph.edges.reserve(graph.edges.size() * 2U);
  for (const auto &edge : graph.edges) {
    const auto start = points_by_id.at(edge.from);
    const auto end = points_by_id.at(edge.to);
    if (point_equal(start, end)) {
      continue;
    }

    const auto midpoint_id =
        static_cast<std::uint32_t>(midpoint_graph.vertices.size());
    midpoint_graph.vertices.push_back(
        GraphVertex{.id = midpoint_id,
                    .point = midpoint(start, end),
                    .kind = GraphVertexKind::midpoint_vertex});
    midpoint_graph.edges.push_back(
        GraphEdge{.from = edge.from, .to = midpoint_id, .kind = edge.kind});
    midpoint_graph.edges.push_back(
        GraphEdge{.from = midpoint_id, .to = edge.to, .kind = edge.kind});
  }

  return midpoint_graph;
}

[[nodiscard]] auto merge_coincident_vertices(const ArrangementGraph &graph)
    -> ArrangementGraph {
  ArrangementGraph merged_graph{};
  merged_graph.perfect_fit_points = graph.perfect_fit_points;
  merged_graph.perfect_sliding_segments = graph.perfect_sliding_segments;
  merged_graph.pruned = graph.pruned;

  std::unordered_map<PointKey, std::uint32_t, PointKeyHash> point_to_vertex_id;
  point_to_vertex_id.reserve(graph.vertices.size());
  std::unordered_map<std::uint32_t, std::uint32_t> remapped_ids;
  remapped_ids.reserve(graph.vertices.size());

  for (const auto &vertex : graph.vertices) {
    const auto key = point_key(vertex.point);
    const auto [it, inserted] = point_to_vertex_id.emplace(
        key, static_cast<std::uint32_t>(merged_graph.vertices.size()));
    if (inserted) {
      merged_graph.vertices.push_back(
          GraphVertex{.id = it->second,
                      .point = canonicalize_point(vertex.point),
                      .kind = vertex.kind});
    } else {
      auto &existing_vertex = merged_graph.vertices[it->second];
      existing_vertex.kind =
          merge_vertex_kind(existing_vertex.kind, vertex.kind);
    }

    remapped_ids.emplace(vertex.id, it->second);
  }

  merged_graph.edges.reserve(graph.edges.size());
  for (const auto &edge : graph.edges) {
    const auto from_it = remapped_ids.find(edge.from);
    const auto to_it = remapped_ids.find(edge.to);
    if (from_it == remapped_ids.end() || to_it == remapped_ids.end() ||
        from_it->second == to_it->second) {
      continue;
    }

    merged_graph.edges.push_back(GraphEdge{
        .from = from_it->second, .to = to_it->second, .kind = edge.kind});
  }

  return merged_graph;
}

[[nodiscard]] auto select_next_edge_index(
    std::uint32_t current_vertex, std::uint32_t previous_vertex,
    const std::vector<std::size_t> &candidate_indices,
    const std::vector<bool> &visited,
    const std::unordered_map<std::uint32_t, Point2> &points_by_id,
    const std::vector<GraphEdge> &edges)
    -> std::optional<std::pair<std::size_t, std::uint32_t>> {
  std::optional<std::size_t> best_index;
  std::optional<std::uint32_t> best_next_vertex;
  auto best_turn = std::numeric_limits<double>::infinity();

  const auto &previous_point = points_by_id.at(previous_vertex);
  const auto &current_point = points_by_id.at(current_vertex);

  for (const auto candidate_index : candidate_indices) {
    if (visited[candidate_index]) {
      continue;
    }

    const auto &candidate_edge = edges[candidate_index];
    const auto next_vertex = candidate_edge.from == current_vertex
                                 ? candidate_edge.to
                                 : candidate_edge.from;
    const auto &next_point = points_by_id.at(next_vertex);
    const auto turn =
        normalized_turn_angle(previous_point, current_point, next_point);

    if (!best_index || turn < best_turn ||
        (turn == best_turn && best_next_vertex &&
         shiny::nfp::detail::point_less(next_point,
                                        points_by_id.at(*best_next_vertex)))) {
      best_index = candidate_index;
      best_next_vertex = next_vertex;
      best_turn = turn;
    }
  }

  if (!best_index || !best_next_vertex) {
    return std::nullopt;
  }
  return std::make_pair(*best_index, *best_next_vertex);
}

[[nodiscard]] auto build_arrangement_graph_from_decompositions_impl(
    const NonconvexNfpRequest &request, const DecompositionResult &piece_a,
    const DecompositionResult &piece_b,
    const std::function<bool()> &interruption_requested = {})
    -> ArrangementGraph {
  ArrangementGraph graph{};
  if (!valid_decomposition(piece_a) || !valid_decomposition(piece_b)) {
    return graph;
  }

  for (const auto &component_a : piece_a.components) {
    if (interrupted(interruption_requested)) {
      return graph;
    }

    for (const auto &component_b : piece_b.components) {
      if (interrupted(interruption_requested)) {
        return graph;
      }

      const ConvexNfpRequest pair_request{
          .piece_a_id = request.piece_a_id,
          .piece_b_id = request.piece_b_id,
          .convex_a = component_a.outer,
          .convex_b = component_b.outer,
          .rotation_a = request.rotation_a,
          .rotation_b = request.rotation_b,
      };

      const auto pair_result = shiny::nfp::compute_convex_nfp(pair_request);
      for (const auto &point : pair_result.perfect_fit_points) {
        append_unique_point(graph.perfect_fit_points, point);
      }
      for (const auto &segment : pair_result.perfect_sliding_segments) {
        append_unique_segment(graph.perfect_sliding_segments, segment);
      }
      for (const auto &loop : pair_result.loops) {
        if (loop.kind != NfpFeatureKind::outer_loop ||
            loop.vertices.size() < 3U) {
          continue;
        }
        append_ring_to_graph(loop.vertices, graph);
      }
    }
  }

  return merge_coincident_vertices(
      insert_midpoint_vertices(split_graph_edges(graph)));
}

[[nodiscard]] auto status_from_graph(const ArrangementGraph &graph,
                                     const NfpResult &result)
    -> ExtractionStatus {
  if (graph.vertices.empty() || graph.edges.empty()) {
    return ExtractionStatus::empty;
  }
  if (result.loops.empty()) {
    return ExtractionStatus::invalid_graph;
  }
  return ExtractionStatus::success;
}

auto prune_leaf_edges_once(ArrangementGraph &graph) -> void {
  bool changed = true;
  while (changed) {
    changed = false;

    std::unordered_map<std::uint32_t, std::size_t> degree_by_vertex;
    degree_by_vertex.reserve(graph.vertices.size());
    for (const auto &edge : graph.edges) {
      if (edge.kind == GraphEdgeKind::pruned_edge) {
        continue;
      }
      ++degree_by_vertex[edge.from];
      ++degree_by_vertex[edge.to];
    }

    for (auto &edge : graph.edges) {
      if (edge.kind == GraphEdgeKind::pruned_edge) {
        continue;
      }
      if (degree_by_vertex[edge.from] > 1U && degree_by_vertex[edge.to] > 1U) {
        continue;
      }
      edge.kind = GraphEdgeKind::pruned_edge;
      changed = true;
    }
  }
}

[[nodiscard]] auto extract_loops_from_traversal_graph(
    const ArrangementGraph &graph,
    const std::function<bool()> &interruption_requested = {})
    -> std::vector<NfpLoop> {
  if (graph.vertices.empty() || graph.edges.empty()) {
    return {};
  }

  std::unordered_map<std::uint32_t, Point2> points_by_id;
  points_by_id.reserve(graph.vertices.size());
  for (const auto &vertex : graph.vertices) {
    points_by_id.insert_or_assign(vertex.id, vertex.point);
  }

  std::unordered_map<std::uint32_t, std::vector<std::size_t>> incident_edges;
  incident_edges.reserve(graph.vertices.size());
  for (std::size_t index = 0; index < graph.edges.size(); ++index) {
    const auto &edge = graph.edges[index];
    if (edge.kind == GraphEdgeKind::pruned_edge) {
      continue;
    }
    incident_edges[edge.from].push_back(index);
    incident_edges[edge.to].push_back(index);
  }

  std::vector<Ring> extracted_rings;
  std::vector<bool> visited(graph.edges.size(), false);
  for (std::size_t edge_index = 0; edge_index < graph.edges.size();
       ++edge_index) {
    if (interrupted(interruption_requested)) {
      return {};
    }

    const auto &start_edge = graph.edges[edge_index];
    if (start_edge.kind == GraphEdgeKind::pruned_edge) {
      continue;
    }

    if (visited[edge_index]) {
      continue;
    }

    Ring ring{};
    auto current_edge_index = edge_index;
    const auto start_vertex =
        shiny::nfp::detail::point_less(points_by_id.at(start_edge.from),
                                       points_by_id.at(start_edge.to))
            ? start_edge.from
            : start_edge.to;
    auto previous_vertex = start_vertex;
    auto current_vertex =
        start_edge.from == start_vertex ? start_edge.to : start_edge.from;

    const auto start_point_it = points_by_id.find(start_vertex);
    if (start_point_it == points_by_id.end()) {
      continue;
    }
    ring.push_back(start_point_it->second);

    while (!visited[current_edge_index]) {
      if (interrupted(interruption_requested)) {
        return {};
      }

      visited[current_edge_index] = true;
      const auto point_it = points_by_id.find(current_vertex);
      if (point_it == points_by_id.end()) {
        ring.clear();
        break;
      }
      ring.push_back(point_it->second);

      if (current_vertex == start_vertex) {
        break;
      }

      const auto incident_it = incident_edges.find(current_vertex);
      if (incident_it == incident_edges.end()) {
        ring.clear();
        break;
      }

      const auto next_edge = select_next_edge_index(
          current_vertex, previous_vertex, incident_it->second, visited,
          points_by_id, graph.edges);
      if (!next_edge) {
        ring.clear();
        break;
      }

      previous_vertex = current_vertex;
      current_edge_index = next_edge->first;
      current_vertex = next_edge->second;
    }

    if (ring.size() < 4U || !point_equal(ring.front(), ring.back())) {
      continue;
    }

    ring.pop_back();

    const auto normalized =
        shiny::nfp::geom::normalize_polygon(Polygon{.outer = ring});
    const auto simplified =
        shiny::nfp::poly::simplify_collinear_ring(normalized.outer);
    for (auto &simple_cycle : extract_simple_cycles(simplified)) {
      extracted_rings.push_back(std::move(simple_cycle));
    }
  }

  std::sort(extracted_rings.begin(), extracted_rings.end(), ring_less);
  extracted_rings.erase(
      std::unique(extracted_rings.begin(), extracted_rings.end(), ring_equal),
      extracted_rings.end());

  std::vector<NfpLoop> loops;
  loops.reserve(extracted_rings.size());
  for (std::size_t index = 0; index < extracted_rings.size(); ++index) {
    const auto kind = classify_loop_kind(index, extracted_rings);
    auto normalized_loop = normalize_loop(extracted_rings[index], kind);
    if (normalized_loop.size() < 3U) {
      continue;
    }
    loops.push_back(
        NfpLoop{.vertices = std::move(normalized_loop), .kind = kind});
  }

  std::sort(loops.begin(), loops.end(),
            [](const NfpLoop &lhs, const NfpLoop &rhs) {
              if (lhs.kind != rhs.kind) {
                return lhs.kind < rhs.kind;
              }
              return ring_less(lhs.vertices, rhs.vertices);
            });
  loops.erase(std::unique(loops.begin(), loops.end(),
                          [](const NfpLoop &lhs, const NfpLoop &rhs) {
                            return lhs.kind == rhs.kind &&
                                   ring_equal(lhs.vertices, rhs.vertices);
                          }),
              loops.end());
  return loops;
}

} // namespace

namespace shiny::nfp {

auto build_arrangement_graph(const NonconvexNfpRequest &request)
    -> ArrangementGraph {
  const decomp::DecompositionRequest request_a{
      .piece_id = request.piece_a_id,
      .polygon = request.piece_a,
      .rotation = request.rotation_a,
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };
  const decomp::DecompositionRequest request_b{
      .piece_id = request.piece_b_id,
      .polygon = request.piece_b,
      .rotation = request.rotation_b,
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto piece_a = decomp::decompose_polygon(request_a);
  const auto piece_b = decomp::decompose_polygon(request_b);
  return detail::build_arrangement_graph_from_decompositions(request, piece_a,
                                                             piece_b);
}

auto prune_arrangement_graph(ArrangementGraph graph) -> ArrangementGraph {
  return detail::prune_arrangement_graph_with_interruption(std::move(graph));
}

auto extract_nfp_from_graph(const ArrangementGraph &graph) -> NfpResult {
  return detail::extract_nfp_from_graph_with_interruption(graph);
}

auto compute_nonconvex_graph_nfp(const NonconvexNfpRequest &request)
    -> NonconvexNfpResult {
  auto pruned_graph = prune_arrangement_graph(build_arrangement_graph(request));
  auto result = extract_nfp_from_graph(pruned_graph);
  const auto status = status_from_graph(pruned_graph, result);
  return {
      .result = std::move(result),
      .graph = std::move(pruned_graph),
      .status = status,
  };
}

} // namespace shiny::nfp

namespace shiny::nfp::detail {

auto prune_arrangement_graph_with_interruption(
    ArrangementGraph graph, const std::function<bool()> &interruption_requested)
    -> ArrangementGraph {
  std::unordered_map<std::uint32_t, Point2> points_by_id;
  points_by_id.reserve(graph.vertices.size());
  for (const auto &vertex : graph.vertices) {
    points_by_id.insert_or_assign(vertex.id, vertex.point);
  }

  const auto original_edges = graph.edges;

  std::unordered_map<GeometricEdgeKey, std::vector<std::size_t>,
                     GeometricEdgeKeyHash>
      grouped_edges;
  grouped_edges.reserve(graph.edges.size());

  for (std::size_t index = 0; index < graph.edges.size(); ++index) {
    if (interrupted(interruption_requested)) {
      return graph;
    }

    const auto &edge = graph.edges[index];
    const auto key = normalized_geometric_edge_key(points_by_id.at(edge.from),
                                                   points_by_id.at(edge.to));
    grouped_edges[key].push_back(index);
  }

  std::vector<GeometricEdgeKey> duplicate_keys;
  duplicate_keys.reserve(grouped_edges.size());
  for (const auto &[key, indices] : grouped_edges) {
    if (interrupted(interruption_requested)) {
      return graph;
    }

    if (indices.size() > 1U) {
      duplicate_keys.push_back(key);
    }
  }

  for (const auto &[key, indices] : grouped_edges) {
    if (interrupted(interruption_requested)) {
      return graph;
    }

    if (indices.size() <= 1U) {
      continue;
    }

    const Segment2 carrier{point_from_key(key.first),
                           point_from_key(key.second)};
    const auto ignored_keys =
        connected_duplicate_component(key, duplicate_keys);
    const auto occupancy =
        sample_segment_side_occupancy(carrier, ignored_keys, points_by_id,
                                      original_edges, interruption_requested);
    std::optional<std::size_t> retained;
    if (occupancy && occupancy->first != occupancy->second) {
      retained = indices.front();
    }

    for (const auto index : indices) {
      if (retained && index == *retained) {
        continue;
      }
      graph.edges[index].kind = GraphEdgeKind::pruned_edge;
    }
  }

  graph.pruned = true;
  return graph;
}

auto extract_nfp_from_graph_with_interruption(
    const ArrangementGraph &graph,
    const std::function<bool()> &interruption_requested) -> NfpResult {
  NfpResult result{
      .perfect_fit_points = graph.perfect_fit_points,
      .perfect_sliding_segments = graph.perfect_sliding_segments,
      .algorithm = AlgorithmKind::nonconvex_graph_nfp,
      .normalized = true,
  };

  if (graph.vertices.empty() || graph.edges.empty()) {
    return result;
  }

  result.loops =
      extract_loops_from_traversal_graph(graph, interruption_requested);
  if (result.loops.empty()) {
    auto traversal_graph = graph;
    prune_leaf_edges_once(traversal_graph);
    if (interrupted(interruption_requested)) {
      return result;
    }
    result.loops = extract_loops_from_traversal_graph(traversal_graph,
                                                      interruption_requested);
  }
  filter_degenerate_features(result);
  return result;
}

auto build_arrangement_graph_from_decompositions(
    const NonconvexNfpRequest &request,
    const decomp::DecompositionResult &piece_a,
    const decomp::DecompositionResult &piece_b,
    const std::function<bool()> &interruption_requested) -> ArrangementGraph {
  return build_arrangement_graph_from_decompositions_impl(
      request, piece_a, piece_b, interruption_requested);
}

} // namespace shiny::nfp::detail