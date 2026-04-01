#include "nfp/convex_nfp.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "geometry/detail/point_compare.hpp"
#include "geometry/normalize.hpp"
#include "polygon_ops/simplify.hpp"
#include "predicates/segment_intersection.hpp"

namespace shiny::nfp {
namespace {

[[nodiscard]] auto normalize_convex_ring(std::span<const geom::Point2> ring)
    -> geom::Ring {
  const auto normalized = geom::normalize_polygon(
      geom::Polygon{.outer = geom::Ring(ring.begin(), ring.end())});
  return poly::simplify_collinear_ring(normalized.outer);
}

[[nodiscard]] auto to_vector(const geom::Point2 &start, const geom::Point2 &end)
    -> geom::Vector2 {
  return {
      .x = end.x - start.x,
      .y = end.y - start.y,
  };
}

void normalize_points(std::vector<geom::Point2> &points) {
  std::sort(points.begin(), points.end(), detail::point_less);
  points.erase(std::unique(points.begin(), points.end()), points.end());
}

[[nodiscard]] auto segment_less(const geom::Segment2 &lhs,
                                const geom::Segment2 &rhs) -> bool {
  const auto lhs_start =
      detail::point_less(lhs.start, lhs.end) ? lhs.start : lhs.end;
  const auto lhs_end =
      detail::point_less(lhs.start, lhs.end) ? lhs.end : lhs.start;
  const auto rhs_start =
      detail::point_less(rhs.start, rhs.end) ? rhs.start : rhs.end;
  const auto rhs_end =
      detail::point_less(rhs.start, rhs.end) ? rhs.end : rhs.start;

  if (detail::point_less(lhs_start, rhs_start)) {
    return true;
  }
  if (detail::point_less(rhs_start, lhs_start)) {
    return false;
  }
  if (detail::point_less(lhs_end, rhs_end)) {
    return true;
  }
  if (detail::point_less(rhs_end, lhs_end)) {
    return false;
  }
  return false;
}

void normalize_segments(std::vector<geom::Segment2> &segments) {
  for (auto &segment : segments) {
    if (detail::point_less(segment.end, segment.start)) {
      std::swap(segment.start, segment.end);
    }
  }

  std::sort(segments.begin(), segments.end(), segment_less);
  segments.erase(
      std::unique(segments.begin(), segments.end(),
                  [](const geom::Segment2 &lhs, const geom::Segment2 &rhs) {
                    return lhs.start == rhs.start && lhs.end == rhs.end;
                  }),
      segments.end());
}

[[nodiscard]] auto add_vector(const geom::Point2 &point,
                              const geom::Vector2 &vector) -> geom::Point2 {
  return {
      .x = point.x + vector.x,
      .y = point.y + vector.y,
  };
}

[[nodiscard]] auto add_vectors(const geom::Vector2 &lhs,
                               const geom::Vector2 &rhs) -> geom::Vector2 {
  return {
      .x = lhs.x + rhs.x,
      .y = lhs.y + rhs.y,
  };
}

[[nodiscard]] auto cross_product(const geom::Vector2 &lhs,
                                 const geom::Vector2 &rhs) -> long double {
  return static_cast<long double>(lhs.x) * rhs.y -
         static_cast<long double>(lhs.y) * rhs.x;
}

[[nodiscard]] auto squared_length(const geom::Vector2 &vector) -> long double {
  return static_cast<long double>(vector.x) * vector.x +
         static_cast<long double>(vector.y) * vector.y;
}

[[nodiscard]] auto is_upper_half_plane(const geom::Vector2 &vector) -> bool {
  return vector.y > 0.0 || (vector.y == 0.0 && vector.x >= 0.0);
}

[[nodiscard]] auto angle_less(const geom::Vector2 &lhs,
                              const geom::Vector2 &rhs) -> bool {
  const bool lhs_upper = is_upper_half_plane(lhs);
  const bool rhs_upper = is_upper_half_plane(rhs);
  if (lhs_upper != rhs_upper) {
    return lhs_upper && !rhs_upper;
  }

  const auto cross = cross_product(lhs, rhs);
  if (cross != 0.0L) {
    return cross > 0.0L;
  }

  const auto lhs_length = squared_length(lhs);
  const auto rhs_length = squared_length(rhs);
  if (lhs_length != rhs_length) {
    return lhs_length < rhs_length;
  }

  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  return lhs.y < rhs.y;
}

[[nodiscard]] auto polar_key(const geom::Vector2 &vector) -> double {
  double angle = std::atan2(vector.y, vector.x);
  if (angle < 0.0) {
    angle += 2.0 * std::numbers::pi_v<double>;
  }
  return angle;
}

[[nodiscard]] auto reflect_ring(std::span<const geom::Point2> ring)
    -> geom::Ring {
  geom::Ring reflected;
  reflected.reserve(ring.size());
  for (const auto &point : ring) {
    reflected.push_back({-point.x, -point.y});
  }
  return reflected;
}

[[nodiscard]] auto translate_ring(const geom::Ring &ring,
                                  const geom::Point2 &translation)
    -> geom::Ring {
  geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(
        add_vector(point, {.x = translation.x, .y = translation.y}));
  }
  return translated;
}

[[nodiscard]] auto build_convex_edge_sequence_from_ring(const geom::Ring &ring)
    -> ConvexEdgeSequence {
  ConvexEdgeSequence sequence{};
  if (ring.size() < 2U) {
    return sequence;
  }

  sequence.edges.reserve(ring.size());
  sequence.source_indices.reserve(ring.size());

  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    sequence.edges.push_back(to_vector(ring[index], ring[next_index]));
    sequence.source_indices.push_back(index);
  }

  std::size_t first_index = 0;
  for (std::size_t index = 1; index < sequence.edges.size(); ++index) {
    if (angle_less(sequence.edges[index], sequence.edges[first_index])) {
      first_index = index;
    }
  }

  std::rotate(sequence.edges.begin(),
              sequence.edges.begin() + static_cast<std::ptrdiff_t>(first_index),
              sequence.edges.end());
  std::rotate(sequence.source_indices.begin(),
              sequence.source_indices.begin() +
                  static_cast<std::ptrdiff_t>(first_index),
              sequence.source_indices.end());

  return sequence;
}

[[nodiscard]] auto select_primary_loop(const NfpResult &result)
    -> const NfpLoop * {
  const auto outer = std::find_if(
      result.loops.begin(), result.loops.end(), [](const NfpLoop &loop) {
        return loop.kind == NfpFeatureKind::outer_loop;
      });
  if (outer != result.loops.end()) {
    return &*outer;
  }
  if (!result.loops.empty()) {
    return &result.loops.front();
  }
  return nullptr;
}

[[nodiscard]] auto midpoint(const geom::Point2 &lhs, const geom::Point2 &rhs)
    -> geom::Point2 {
  return {
      .x = (lhs.x + rhs.x) / 2.0,
      .y = (lhs.y + rhs.y) / 2.0,
  };
}

[[nodiscard]] auto
collect_contact_points(const geom::Ring &convex_a, const geom::Ring &convex_b,
                       const geom::Point2 &translation, bool &has_overlap)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> contact_points;
  has_overlap = false;

  const auto translated_b = translate_ring(convex_b, translation);
  for (std::size_t a_index = 0; a_index < convex_a.size(); ++a_index) {
    const auto a_next = (a_index + 1U) % convex_a.size();
    const geom::Segment2 segment_a{convex_a[a_index], convex_a[a_next]};

    for (std::size_t b_index = 0; b_index < translated_b.size(); ++b_index) {
      const auto b_next = (b_index + 1U) % translated_b.size();
      const geom::Segment2 segment_b{translated_b[b_index],
                                     translated_b[b_next]};
      const auto contact = pred::classify_segment_contact(segment_a, segment_b);

      if (contact.kind == pred::SegmentContactKind::collinear_overlap) {
        has_overlap = true;
      }

      for (std::size_t point_index = 0; point_index < contact.point_count;
           ++point_index) {
        contact_points.push_back(contact.points[point_index]);
      }
    }
  }

  normalize_points(contact_points);
  return contact_points;
}

void extract_degenerate_features(const geom::Ring &convex_a,
                                 const geom::Ring &convex_b,
                                 const geom::Ring &loop, NfpResult &result) {
  if (loop.size() < 2U) {
    return;
  }

  std::vector<bool> sliding_edge(loop.size(), false);

  for (std::size_t index = 0; index < loop.size(); ++index) {
    const auto next_index = (index + 1U) % loop.size();
    bool has_overlap = false;
    [[maybe_unused]] const auto contact_points = collect_contact_points(
        convex_a, convex_b, midpoint(loop[index], loop[next_index]),
        has_overlap);

    if (has_overlap) {
      sliding_edge[index] = true;
      result.perfect_sliding_segments.push_back(
          {loop[index], loop[next_index]});
    }
  }

  for (std::size_t index = 0; index < loop.size(); ++index) {
    const auto prev_index = (index + loop.size() - 1U) % loop.size();
    if (!sliding_edge[prev_index] && !sliding_edge[index]) {
      continue;
    }

    bool has_overlap = false;
    const auto contact_points =
        collect_contact_points(convex_a, convex_b, loop[index], has_overlap);

    if (!has_overlap && contact_points.size() == 1U) {
      result.perfect_fit_points.push_back(loop[index]);
    }
  }

  normalize_points(result.perfect_fit_points);
  normalize_segments(result.perfect_sliding_segments);
}

} // namespace

auto compute_convex_nfp(const ConvexNfpRequest &request) -> NfpResult {
  const auto convex_a = normalize_convex_ring(request.convex_a);
  const auto reflected_b =
      normalize_convex_ring(reflect_ring(request.convex_b));

  NfpResult result{};
  result.algorithm = AlgorithmKind::convex_nfp;
  result.normalized = true;

  if (convex_a.size() < 3U || reflected_b.size() < 3U) {
    return result;
  }

  const auto sequence_a = build_convex_edge_sequence_from_ring(convex_a);
  const auto sequence_b = build_convex_edge_sequence_from_ring(reflected_b);

  if (sequence_a.edges.empty() || sequence_b.edges.empty()) {
    return result;
  }

  geom::Ring loop;
  loop.reserve(sequence_a.edges.size() + sequence_b.edges.size() + 1U);

  geom::Point2 current{
      .x = convex_a[sequence_a.source_indices.front()].x +
           reflected_b[sequence_b.source_indices.front()].x,
      .y = convex_a[sequence_a.source_indices.front()].y +
           reflected_b[sequence_b.source_indices.front()].y,
  };
  loop.push_back(current);

  std::size_t index_a = 0;
  std::size_t index_b = 0;

  while (index_a < sequence_a.edges.size() &&
         index_b < sequence_b.edges.size()) {
    if (angle_less(sequence_a.edges[index_a], sequence_b.edges[index_b])) {
      current = add_vector(current, sequence_a.edges[index_a]);
      ++index_a;
    } else if (angle_less(sequence_b.edges[index_b],
                          sequence_a.edges[index_a])) {
      current = add_vector(current, sequence_b.edges[index_b]);
      ++index_b;
    } else {
      current = add_vector(current, add_vectors(sequence_a.edges[index_a],
                                                sequence_b.edges[index_b]));
      ++index_a;
      ++index_b;
    }
    loop.push_back(current);
  }

  while (index_a < sequence_a.edges.size()) {
    current = add_vector(current, sequence_a.edges[index_a]);
    ++index_a;
    loop.push_back(current);
  }

  while (index_b < sequence_b.edges.size()) {
    current = add_vector(current, sequence_b.edges[index_b]);
    ++index_b;
    loop.push_back(current);
  }

  const auto normalized_loop =
      poly::simplify_polygon(geom::Polygon{.outer = loop});
  if (!normalized_loop.outer.empty()) {
    result.loops.push_back({.vertices = normalized_loop.outer,
                            .kind = NfpFeatureKind::outer_loop});
    extract_degenerate_features(convex_a,
                                normalize_convex_ring(request.convex_b),
                                normalized_loop.outer, result);
  }

  return result;
}

auto compute_convex_nfp(
    const ConvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    cache::CacheStore<cache::PairRotationKey, NfpResult> &cache_store)
    -> NfpResult {
  const auto key = cache::make_pair_rotation_key(request, geometry_a_revision,
                                                 geometry_b_revision);
  if (const auto *cached = cache_store.get(key)) {
    return *cached;
  }

  auto result = compute_convex_nfp(request);
  cache_store.put(key, result);
  return result;
}

auto build_convex_edge_sequence(std::span<const geom::Point2> ring)
    -> ConvexEdgeSequence {
  return build_convex_edge_sequence_from_ring(normalize_convex_ring(ring));
}

auto order_convex_nfp_vertices(const NfpResult &result)
    -> std::vector<ConvexOrderingVertex> {
  const auto *primary_loop = select_primary_loop(result);
  if (primary_loop == nullptr) {
    return {};
  }

  const auto normalized_loop = normalize_convex_ring(primary_loop->vertices);
  const auto sequence = build_convex_edge_sequence_from_ring(normalized_loop);

  std::vector<ConvexOrderingVertex> ordered_vertices;
  ordered_vertices.reserve(sequence.edges.size());
  for (std::size_t index = 0; index < sequence.edges.size(); ++index) {
    ordered_vertices.push_back({
        .point = normalized_loop[sequence.source_indices[index]],
        .source_edge_index = sequence.source_indices[index],
        .polar_key = polar_key(sequence.edges[index]),
    });
  }

  return ordered_vertices;
}

} // namespace shiny::nfp