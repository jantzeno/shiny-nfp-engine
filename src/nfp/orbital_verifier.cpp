#include "nfp/orbital_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>

#include "decomposition/decompose.hpp"
#include "geometry/detail/point_compare.hpp"
#include "geometry/normalize.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/nonconvex_nfp.hpp"
#include "polygon_ops/simplify.hpp"
#include "predicates/segment_intersection.hpp"

namespace {

namespace bg = boost::geometry;

using shiny::nfp::AlgorithmKind;
using shiny::nfp::ArrangementGraph;
using shiny::nfp::ConvexNfpRequest;
using shiny::nfp::GraphEdge;
using shiny::nfp::GraphEdgeKind;
using shiny::nfp::GraphVertex;
using shiny::nfp::GraphVertexKind;
using shiny::nfp::NfpResult;
using shiny::nfp::NonconvexNfpRequest;
using shiny::nfp::OrbitalFeasibleTranslation;
using shiny::nfp::OrbitalToucher;
using shiny::nfp::OrbitalTouchKind;
using shiny::nfp::OrbitalVerifierResult;
using shiny::nfp::OrbitalVerifierStatus;
using shiny::nfp::decomp::DecompositionAlgorithm;
using shiny::nfp::decomp::DecompositionResult;
using shiny::nfp::decomp::DecompositionValidity;
using shiny::nfp::geom::Point2;
using shiny::nfp::geom::Polygon;
using shiny::nfp::geom::PolygonWithHoles;
using shiny::nfp::geom::Ring;
using shiny::nfp::geom::Segment2;

using BgPoint = bg::model::d2::point_xy<double>;
using BgRing = bg::model::ring<BgPoint, false, false>;
using BgPolygon = bg::model::polygon<BgPoint, false, false>;

constexpr double canonical_coordinate_scale = 1000000000.0;

using CoordinateKey = std::int64_t;

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

struct PointKeyHash {
  auto operator()(const PointKey &key) const noexcept -> std::size_t {
    std::size_t seed = std::hash<CoordinateKey>{}(key.x);
    seed ^= std::hash<CoordinateKey>{}(key.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct GeometricEdgeKey {
  PointKey first{};
  PointKey second{};

  auto operator==(const GeometricEdgeKey &other) const -> bool {
    return first == other.first && second == other.second;
  }
};

struct GeometricEdgeKeyHash {
  auto operator()(const GeometricEdgeKey &key) const noexcept -> std::size_t {
    std::size_t seed = PointKeyHash{}(key.first);
    seed ^= PointKeyHash{}(key.second) + 0x9e3779b97f4a7c15ULL + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

using CandidateEdgeSet =
    std::unordered_set<GeometricEdgeKey, GeometricEdgeKeyHash>;

[[nodiscard]] auto point_key(const Point2 &point) -> PointKey {
  return {.x = coordinate_key(point.x), .y = coordinate_key(point.y)};
}

[[nodiscard]] auto point_equal(const Point2 &lhs, const Point2 &rhs) -> bool {
  return point_key(lhs) == point_key(rhs);
}

[[nodiscard]] auto midpoint(const Point2 &lhs, const Point2 &rhs) -> Point2 {
  return canonicalize_point({
      (lhs.x + rhs.x) * 0.5,
      (lhs.y + rhs.y) * 0.5,
  });
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

[[nodiscard]] auto segment_less(const Segment2 &lhs, const Segment2 &rhs)
    -> bool {
  const auto lhs_key = normalized_geometric_edge_key(lhs.start, lhs.end);
  const auto rhs_key = normalized_geometric_edge_key(rhs.start, rhs.end);
  if (lhs_key.first.x != rhs_key.first.x) {
    return lhs_key.first.x < rhs_key.first.x;
  }
  if (lhs_key.first.y != rhs_key.first.y) {
    return lhs_key.first.y < rhs_key.first.y;
  }
  if (lhs_key.second.x != rhs_key.second.x) {
    return lhs_key.second.x < rhs_key.second.x;
  }
  return lhs_key.second.y < rhs_key.second.y;
}

[[nodiscard]] auto to_bg_point(const Point2 &point) -> BgPoint {
  return {point.x, point.y};
}

[[nodiscard]] auto to_bg_ring(const Ring &ring) -> BgRing {
  BgRing result;
  result.reserve(ring.size());
  for (const auto &point : ring) {
    result.push_back(to_bg_point(point));
  }
  return result;
}

[[nodiscard]] auto to_bg_polygon(const PolygonWithHoles &polygon) -> BgPolygon {
  const auto normalized = shiny::nfp::geom::normalize_polygon(polygon);
  BgPolygon result;
  result.outer() = to_bg_ring(normalized.outer);
  result.inners().reserve(normalized.holes.size());
  for (const auto &hole : normalized.holes) {
    result.inners().push_back(to_bg_ring(hole));
  }
  bg::correct(result);
  return result;
}

[[nodiscard]] auto translate_ring(const Ring &ring, const Point2 &translation)
    -> Ring {
  Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(
        canonicalize_point({point.x + translation.x, point.y + translation.y}));
  }
  return translated;
}

[[nodiscard]] auto translate_polygon(const PolygonWithHoles &polygon,
                                     const Point2 &translation)
    -> PolygonWithHoles {
  PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, translation);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, translation));
  }
  return shiny::nfp::geom::normalize_polygon(translated);
}

void normalize_points(std::vector<Point2> &points) {
  for (auto &point : points) {
    point = canonicalize_point(point);
  }

  std::sort(points.begin(), points.end(), shiny::nfp::detail::point_less);
  points.erase(std::unique(points.begin(), points.end(), point_equal),
               points.end());
}

void normalize_segments(std::vector<Segment2> &segments) {
  for (auto &segment : segments) {
    segment.start = canonicalize_point(segment.start);
    segment.end = canonicalize_point(segment.end);
    if (shiny::nfp::detail::point_less(segment.end, segment.start)) {
      std::swap(segment.start, segment.end);
    }
  }

  std::sort(segments.begin(), segments.end(), segment_less);
  segments.erase(std::unique(segments.begin(), segments.end(),
                             [](const Segment2 &lhs, const Segment2 &rhs) {
                               return point_equal(lhs.start, rhs.start) &&
                                      point_equal(lhs.end, rhs.end);
                             }),
                 segments.end());
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
  Segment2 canonical_segment{canonicalize_point(segment.start),
                             canonicalize_point(segment.end)};
  if (shiny::nfp::detail::point_less(canonical_segment.end,
                                     canonical_segment.start)) {
    std::swap(canonical_segment.start, canonical_segment.end);
  }

  if (std::find_if(segments.begin(), segments.end(),
                   [&canonical_segment](const Segment2 &candidate) {
                     return point_equal(candidate.start,
                                        canonical_segment.start) &&
                            point_equal(candidate.end, canonical_segment.end);
                   }) != segments.end()) {
    return;
  }

  segments.push_back(canonical_segment);
}

[[nodiscard]] auto collect_segments(const PolygonWithHoles &polygon)
    -> std::vector<Segment2> {
  std::vector<Segment2> segments;
  const auto append_ring_segments = [&segments](const Ring &ring) {
    if (ring.size() < 2U) {
      return;
    }

    for (std::size_t index = 0; index < ring.size(); ++index) {
      const auto next_index = (index + 1U) % ring.size();
      segments.push_back({ring[index], ring[next_index]});
    }
  };

  append_ring_segments(polygon.outer);
  for (const auto &hole : polygon.holes) {
    append_ring_segments(hole);
  }
  return segments;
}

[[nodiscard]] auto intersection_area(const PolygonWithHoles &lhs,
                                     const PolygonWithHoles &rhs) -> double {
  std::vector<BgPolygon> output;
  bg::intersection(to_bg_polygon(lhs), to_bg_polygon(rhs), output);

  double area = 0.0;
  for (const auto &polygon : output) {
    area += std::fabs(bg::area(polygon));
  }
  return area;
}

[[nodiscard]] auto detect_touchers_at_translation(
    const PolygonWithHoles &piece_a, const PolygonWithHoles &piece_b,
    const Point2 &translation) -> std::vector<OrbitalToucher>;

[[nodiscard]] auto
detect_touch_kind(const shiny::nfp::pred::SegmentContact &contact)
    -> OrbitalTouchKind {
  if (contact.kind == shiny::nfp::pred::SegmentContactKind::collinear_overlap) {
    return OrbitalTouchKind::edge_edge;
  }
  if (contact.a_vertex_contact && contact.b_vertex_contact) {
    return OrbitalTouchKind::vertex_vertex;
  }
  if (contact.a_vertex_contact) {
    return OrbitalTouchKind::vertex_edge;
  }
  if (contact.b_vertex_contact) {
    return OrbitalTouchKind::edge_vertex;
  }
  return OrbitalTouchKind::edge_edge;
}

[[nodiscard]] auto valid_decomposition(const DecompositionResult &result)
    -> bool {
  return result.validity == DecompositionValidity::valid &&
         !result.components.empty();
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

[[nodiscard]] auto
segment_from_edge(const GraphEdge &edge,
                  const std::unordered_map<std::uint32_t, Point2> &points_by_id)
    -> Segment2 {
  return {points_by_id.at(edge.from), points_by_id.at(edge.to)};
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

[[nodiscard]] auto
collect_candidate_edge_keys(const std::vector<Ring> &candidate_loops)
    -> CandidateEdgeSet {
  CandidateEdgeSet keys;
  for (const auto &loop : candidate_loops) {
    const auto normalized =
        shiny::nfp::geom::normalize_polygon(Polygon{.outer = loop}).outer;
    const auto simplified =
        shiny::nfp::poly::simplify_collinear_ring(normalized);
    if (simplified.size() < 2U) {
      continue;
    }

    for (std::size_t index = 0; index < simplified.size(); ++index) {
      const auto next_index = (index + 1U) % simplified.size();
      keys.insert(normalized_geometric_edge_key(simplified[index],
                                                simplified[next_index]));
    }
  }
  return keys;
}

[[nodiscard]] auto build_result_from_candidate_loops(
    const std::vector<Ring> &candidate_loops,
    std::vector<Point2> perfect_fit_points,
    std::vector<Segment2> perfect_sliding_segments) -> NfpResult {
  ArrangementGraph graph{};
  normalize_points(perfect_fit_points);
  normalize_segments(perfect_sliding_segments);
  graph.perfect_fit_points = std::move(perfect_fit_points);
  graph.perfect_sliding_segments = std::move(perfect_sliding_segments);

  for (const auto &loop : candidate_loops) {
    const auto normalized =
        shiny::nfp::geom::normalize_polygon(Polygon{.outer = loop}).outer;
    const auto simplified =
        shiny::nfp::poly::simplify_collinear_ring(normalized);
    if (simplified.size() < 3U) {
      continue;
    }
    append_ring_to_graph(simplified, graph);
  }

  auto refined_graph = merge_coincident_vertices(
      insert_midpoint_vertices(split_graph_edges(graph)));
  auto pruned_graph =
      shiny::nfp::prune_arrangement_graph(std::move(refined_graph));
  auto result = shiny::nfp::extract_nfp_from_graph(pruned_graph);
  result.algorithm = AlgorithmKind::orbital_verifier;
  return result;
}

[[nodiscard]] auto detect_touchers_at_translation(
    const PolygonWithHoles &piece_a, const PolygonWithHoles &piece_b,
    const Point2 &translation) -> std::vector<OrbitalToucher> {
  const auto translated_b = translate_polygon(piece_b, translation);
  if (intersection_area(piece_a, translated_b) > 1.0e-12) {
    return {};
  }

  std::vector<OrbitalToucher> touchers;
  const auto segments_a = collect_segments(piece_a);
  const auto segments_b = collect_segments(translated_b);
  for (const auto &segment_a : segments_a) {
    for (const auto &segment_b : segments_b) {
      const auto contact =
          shiny::nfp::pred::classify_segment_contact(segment_a, segment_b);
      if (contact.kind == shiny::nfp::pred::SegmentContactKind::disjoint ||
          contact.kind ==
              shiny::nfp::pred::SegmentContactKind::parallel_disjoint) {
        continue;
      }

      OrbitalToucher toucher{};
      toucher.translation = canonicalize_point(translation);
      toucher.kind = detect_touch_kind(contact);
      toucher.contact_points.reserve(contact.point_count);
      for (std::uint8_t point_index = 0; point_index < contact.point_count;
           ++point_index) {
        toucher.contact_points.push_back(contact.points[point_index]);
      }
      normalize_points(toucher.contact_points);

      const auto duplicate = std::find_if(
          touchers.begin(), touchers.end(),
          [&toucher](const OrbitalToucher &candidate) {
            return point_equal(candidate.translation, toucher.translation) &&
                   candidate.kind == toucher.kind &&
                   candidate.contact_points.size() ==
                       toucher.contact_points.size() &&
                   std::equal(candidate.contact_points.begin(),
                              candidate.contact_points.end(),
                              toucher.contact_points.begin(), point_equal);
          });
      if (duplicate == touchers.end()) {
        touchers.push_back(std::move(toucher));
      }
    }
  }

  std::sort(touchers.begin(), touchers.end(),
            [](const OrbitalToucher &lhs, const OrbitalToucher &rhs) {
              if (!point_equal(lhs.translation, rhs.translation)) {
                return shiny::nfp::detail::point_less(lhs.translation,
                                                      rhs.translation);
              }
              if (lhs.kind != rhs.kind) {
                return lhs.kind < rhs.kind;
              }
              if (lhs.contact_points.size() != rhs.contact_points.size()) {
                return lhs.contact_points.size() < rhs.contact_points.size();
              }
              for (std::size_t index = 0; index < lhs.contact_points.size();
                   ++index) {
                if (point_equal(lhs.contact_points[index],
                                rhs.contact_points[index])) {
                  continue;
                }
                return shiny::nfp::detail::point_less(
                    lhs.contact_points[index], rhs.contact_points[index]);
              }
              return false;
            });
  return touchers;
}

[[nodiscard]] auto collect_boundary_vertices(const NfpResult &result)
    -> std::vector<Point2> {
  std::vector<Point2> vertices;
  for (const auto &loop : result.loops) {
    for (const auto &point : loop.vertices) {
      append_unique_point(vertices, point);
    }
  }
  normalize_points(vertices);
  return vertices;
}

[[nodiscard]] auto build_feasible_translations(
    const PolygonWithHoles &piece_a, const PolygonWithHoles &piece_b,
    const NfpResult &result, const CandidateEdgeSet &candidate_edge_keys)
    -> std::vector<OrbitalFeasibleTranslation> {
  std::vector<OrbitalFeasibleTranslation> translations;
  for (const auto &loop : result.loops) {
    for (std::size_t index = 0; index < loop.vertices.size(); ++index) {
      const auto next_index = (index + 1U) % loop.vertices.size();
      const auto segment =
          Segment2{loop.vertices[index], loop.vertices[next_index]};
      const auto midpoint_point = midpoint(segment.start, segment.end);
      if (intersection_area(
              piece_a, translate_polygon(piece_b, midpoint_point)) > 1.0e-12) {
        continue;
      }
      if (detect_touchers_at_translation(piece_a, piece_b, midpoint_point)
              .empty()) {
        continue;
      }

      Segment2 canonical_segment{canonicalize_point(segment.start),
                                 canonicalize_point(segment.end)};
      if (shiny::nfp::detail::point_less(canonical_segment.end,
                                         canonical_segment.start)) {
        std::swap(canonical_segment.start, canonical_segment.end);
      }
      const auto segment_key = normalized_geometric_edge_key(
          canonical_segment.start, canonical_segment.end);
      const bool trimmed =
          candidate_edge_keys.find(segment_key) == candidate_edge_keys.end();

      const auto duplicate = std::find_if(
          translations.begin(), translations.end(),
          [&canonical_segment](const OrbitalFeasibleTranslation &candidate) {
            return point_equal(candidate.segment.start,
                               canonical_segment.start) &&
                   point_equal(candidate.segment.end, canonical_segment.end);
          });
      if (duplicate == translations.end()) {
        translations.push_back(
            {.segment = canonical_segment, .trimmed = trimmed});
      } else {
        duplicate->trimmed = duplicate->trimmed || trimmed;
      }
    }
  }

  std::sort(translations.begin(), translations.end(),
            [](const OrbitalFeasibleTranslation &lhs,
               const OrbitalFeasibleTranslation &rhs) {
              return segment_less(lhs.segment, rhs.segment);
            });
  return translations;
}

[[nodiscard]] auto status_from_result(const OrbitalVerifierResult &result)
    -> OrbitalVerifierStatus {
  if (result.result.loops.empty()) {
    return result.completed ? OrbitalVerifierStatus::invalid
                            : OrbitalVerifierStatus::empty;
  }
  return OrbitalVerifierStatus::success;
}

} // namespace

namespace shiny::nfp {

auto compute_orbital_verifier_nfp(const NonconvexNfpRequest &request)
    -> OrbitalVerifierResult {
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
  return detail::compute_orbital_verifier_from_decompositions(request, piece_a,
                                                              piece_b);
}

} // namespace shiny::nfp

namespace shiny::nfp::detail {

auto compute_orbital_verifier_from_decompositions(
    const NonconvexNfpRequest &request, const DecompositionResult &piece_a,
    const DecompositionResult &piece_b) -> OrbitalVerifierResult {
  OrbitalVerifierResult result{};
  result.result.algorithm = AlgorithmKind::orbital_verifier;
  result.result.normalized = true;

  if (!valid_decomposition(piece_a) || !valid_decomposition(piece_b)) {
    result.status = OrbitalVerifierStatus::empty;
    result.completed = false;
    return result;
  }

  std::vector<Ring> candidate_loops;
  std::vector<Point2> perfect_fit_points;
  std::vector<Segment2> perfect_sliding_segments;

  for (const auto &component_a : piece_a.components) {
    for (const auto &component_b : piece_b.components) {
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
        append_unique_point(perfect_fit_points, point);
      }
      for (const auto &segment : pair_result.perfect_sliding_segments) {
        append_unique_segment(perfect_sliding_segments, segment);
      }
      for (const auto &loop : pair_result.loops) {
        if (loop.kind != NfpFeatureKind::outer_loop ||
            loop.vertices.size() < 3U) {
          continue;
        }
        candidate_loops.push_back(loop.vertices);
      }
    }
  }

  result.result = build_result_from_candidate_loops(
      candidate_loops, std::move(perfect_fit_points),
      std::move(perfect_sliding_segments));

  const auto normalized_a =
      shiny::nfp::geom::normalize_polygon(request.piece_a);
  const auto normalized_b =
      shiny::nfp::geom::normalize_polygon(request.piece_b);
  const auto candidate_edge_keys = collect_candidate_edge_keys(candidate_loops);

  for (const auto &translation : collect_boundary_vertices(result.result)) {
    const auto touchers =
        detect_touchers_at_translation(normalized_a, normalized_b, translation);
    result.touchers.insert(result.touchers.end(), touchers.begin(),
                           touchers.end());
  }
  result.feasible_translations = build_feasible_translations(
      normalized_a, normalized_b, result.result, candidate_edge_keys);

  for (const auto &loop : result.result.loops) {
    result.traces.push_back({.start = loop.vertices.front(),
                             .kind = loop.kind,
                             .segment_count = loop.vertices.size(),
                             .completed = true});
    if (loop.kind == NfpFeatureKind::hole) {
      result.inner_loop_starts.push_back(loop.vertices.front());
    }
  }
  normalize_points(result.inner_loop_starts);
  result.completed = !result.result.loops.empty();
  result.status = status_from_result(result);
  return result;
}

} // namespace shiny::nfp::detail