#include "nfp/convex_ifp.hpp"

#include <algorithm>
#include <span>

#include "geometry/detail/point_compare.hpp"
#include "geometry/normalize.hpp"
#include "nfp/convex_nfp.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/simplify.hpp"

namespace shiny::nfp {
namespace {

[[nodiscard]] auto normalize_convex_ring(std::span<const geom::Point2> ring)
    -> geom::Ring {
  const auto normalized = geom::normalize_polygon(
      geom::Polygon{.outer = geom::Ring(ring.begin(), ring.end())});
  return poly::simplify_collinear_ring(normalized.outer);
}

[[nodiscard]] auto cross_product(const geom::Point2 &origin,
                                 const geom::Point2 &edge_end,
                                 const geom::Point2 &point) -> long double {
  return static_cast<long double>(edge_end.x - origin.x) *
             (point.y - origin.y) -
         static_cast<long double>(edge_end.y - origin.y) * (point.x - origin.x);
}

[[nodiscard]] auto cross_vector(const geom::Point2 &vector,
                                const geom::Point2 &point) -> long double {
  return static_cast<long double>(vector.x) * point.y -
         static_cast<long double>(vector.y) * point.x;
}

[[nodiscard]] auto interpolate(const geom::Point2 &lhs, const geom::Point2 &rhs,
                               long double lhs_value, long double rhs_value)
    -> geom::Point2 {
  const auto denominator = lhs_value - rhs_value;
  if (denominator == 0.0L) {
    return lhs;
  }

  const long double t = lhs_value / denominator;
  return {
      .x = lhs.x + static_cast<double>(t * (rhs.x - lhs.x)),
      .y = lhs.y + static_cast<double>(t * (rhs.y - lhs.y)),
  };
}

[[nodiscard]] auto min_point(const geom::Ring &ring) -> geom::Point2 {
  geom::Point2 result = ring.front();
  for (const auto &point : ring) {
    result.x = std::min(result.x, point.x);
    result.y = std::min(result.y, point.y);
  }
  return result;
}

[[nodiscard]] auto max_point(const geom::Ring &ring) -> geom::Point2 {
  geom::Point2 result = ring.front();
  for (const auto &point : ring) {
    result.x = std::max(result.x, point.x);
    result.y = std::max(result.y, point.y);
  }
  return result;
}

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  if (lhs.outer.empty()) {
    return !rhs.outer.empty();
  }
  if (rhs.outer.empty()) {
    return false;
  }

  if (detail::point_less(lhs.outer.front(), rhs.outer.front())) {
    return true;
  }
  if (detail::point_less(rhs.outer.front(), lhs.outer.front())) {
    return false;
  }
  if (lhs.outer.size() != rhs.outer.size()) {
    return lhs.outer.size() < rhs.outer.size();
  }
  return lhs.holes.size() < rhs.holes.size();
}

[[nodiscard]] auto clip_against_half_plane(const geom::Ring &polygon,
                                           const geom::Point2 &edge_start,
                                           const geom::Point2 &edge_end,
                                           long double threshold)
    -> geom::Ring {
  geom::Ring clipped;
  if (polygon.empty()) {
    return clipped;
  }

  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto &current = polygon[index];
    const auto &next = polygon[(index + 1U) % polygon.size()];
    const auto current_value =
        cross_product(edge_start, edge_end, current) - threshold;
    const auto next_value =
        cross_product(edge_start, edge_end, next) - threshold;
    const bool current_inside = current_value >= 0.0L;
    const bool next_inside = next_value >= 0.0L;

    if (current_inside && next_inside) {
      clipped.push_back(next);
      continue;
    }

    if (current_inside && !next_inside) {
      clipped.push_back(interpolate(current, next, current_value, next_value));
      continue;
    }

    if (!current_inside && next_inside) {
      clipped.push_back(interpolate(current, next, current_value, next_value));
      clipped.push_back(next);
    }
  }

  return clipped;
}

[[nodiscard]] auto build_outer_ifp_loop(const geom::Ring &container,
                                        const geom::Ring &piece) -> geom::Ring {
  if (container.size() < 3U || piece.size() < 3U) {
    return {};
  }

  const auto container_min = min_point(container);
  const auto container_max = max_point(container);
  const auto piece_min = min_point(piece);
  const auto piece_max = max_point(piece);

  geom::Ring feasible{
      {container_min.x - piece_max.x, container_min.y - piece_max.y},
      {container_max.x - piece_min.x, container_min.y - piece_max.y},
      {container_max.x - piece_min.x, container_max.y - piece_min.y},
      {container_min.x - piece_max.x, container_max.y - piece_min.y}};

  for (std::size_t index = 0; index < container.size() && !feasible.empty();
       ++index) {
    const auto next_index = (index + 1U) % container.size();
    const auto edge_start = container[index];
    const auto edge_end = container[next_index];
    const geom::Point2 edge_vector{edge_end.x - edge_start.x,
                                   edge_end.y - edge_start.y};

    long double min_piece_cross = cross_vector(edge_vector, piece.front());
    for (const auto &point : piece) {
      min_piece_cross =
          std::min(min_piece_cross, cross_vector(edge_vector, point));
    }

    const auto threshold = -min_piece_cross;
    feasible =
        clip_against_half_plane(feasible, edge_start, edge_end, threshold);
  }

  if (feasible.size() < 3U) {
    return {};
  }

  return poly::simplify_polygon(geom::Polygon{.outer = feasible}).outer;
}

void append_polygon_loops(const geom::PolygonWithHoles &polygon,
                          NfpResult &result) {
  if (!polygon.outer.empty()) {
    result.loops.push_back(
        {.vertices = polygon.outer, .kind = NfpFeatureKind::outer_loop});
  }

  for (const auto &hole : polygon.holes) {
    result.loops.push_back({.vertices = hole, .kind = NfpFeatureKind::hole});
  }
}

} // namespace

auto compute_convex_ifp(const ConvexIfpRequest &request) -> NfpResult {
  const auto container = geom::normalize_polygon(request.container);
  const auto piece = geom::normalize_polygon(request.piece);

  NfpResult result{};
  result.algorithm = AlgorithmKind::convex_ifp;
  result.normalized = true;

  const auto outer_loop =
      build_outer_ifp_loop(normalize_convex_ring(container.outer),
                           normalize_convex_ring(piece.outer));
  if (outer_loop.empty()) {
    return result;
  }

  std::vector<geom::PolygonWithHoles> feasible_components{
      geom::normalize_polygon(geom::PolygonWithHoles{.outer = outer_loop})};

  for (const auto &hole : container.holes) {
    const auto hole_nfp = compute_convex_nfp({
        .piece_a_id = request.container_id,
        .piece_b_id = request.piece_id,
        .convex_a = hole,
        .convex_b = piece.outer,
        .rotation_a = request.container_rotation,
        .rotation_b = request.piece_rotation,
    });

    for (const auto &loop : hole_nfp.loops) {
      if (loop.kind != NfpFeatureKind::outer_loop || loop.vertices.empty()) {
        continue;
      }

      const auto forbidden_region = geom::normalize_polygon(
          geom::PolygonWithHoles{.outer = loop.vertices});
      std::vector<geom::PolygonWithHoles> next_feasible_components;

      for (const auto &component : feasible_components) {
        const auto remainder =
            poly::difference_polygons(component, forbidden_region);
        next_feasible_components.insert(next_feasible_components.end(),
                                        remainder.begin(), remainder.end());
      }

      feasible_components = std::move(next_feasible_components);
      if (feasible_components.empty()) {
        break;
      }
    }

    if (feasible_components.empty()) {
      break;
    }
  }

  std::sort(feasible_components.begin(), feasible_components.end(),
            polygon_less);
  for (const auto &component : feasible_components) {
    append_polygon_loops(component, result);
  }

  return result;
}

auto compute_convex_ifp(
    const ConvexIfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    cache::CacheStore<cache::PairRotationKey, NfpResult> &cache_store)
    -> NfpResult {
  const auto key = cache::make_pair_rotation_key(request, geometry_a_revision,
                                                 geometry_b_revision);
  if (const auto *cached = cache_store.get(key)) {
    return *cached;
  }

  auto result = compute_convex_ifp(request);
  cache_store.put(key, result);
  return result;
}

} // namespace shiny::nfp