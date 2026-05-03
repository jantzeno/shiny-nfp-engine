
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <numeric>
#include <span>
#include <vector>

#include <mapbox/earcut.hpp>

#include "geometry/decomposition/internal.hpp"
#include "geometry/decomposition/triangulation.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/validity.hpp"

// Triangulation for polygons with holes via earcut.hpp.
//
// Pipeline:
//   1. Normalize the polygon and feed its outer ring plus holes into earcut.
//   2. Reconstruct triangles from the returned index buffer.
//   3. Drop tiny triangles below kAreaEpsilon.
//   4. Recover triangle adjacency by matching shared undirected edges.
//   5. Sort by bbox origin then area for deterministic output.

namespace shiny::nesting::decomp {
namespace {

using EarcutPoint = std::array<double, 2>;
using EarcutRing = std::vector<EarcutPoint>;
using EarcutPolygon = std::vector<EarcutRing>;

constexpr double kAreaEpsilon = 1e-9;

struct EdgeKey {
  geom::Point2 first{};
  geom::Point2 second{};

  auto operator<=>(const EdgeKey &) const = default;
};

struct EdgeRef {
  std::int32_t triangle_index{-1};
  std::int32_t edge_index{-1};
};

[[nodiscard]] auto to_earcut_point(const geom::Point2 &point) -> EarcutPoint {
  return {point.x(), point.y()};
}

auto append_ring(const geom::Ring &ring, EarcutPolygon &earcut_polygon,
                 std::vector<geom::Point2> &flat_points) -> void {
  if (ring.size() < 3U) {
    return;
  }

  EarcutRing earcut_ring;
  earcut_ring.reserve(ring.size());
  for (const auto &point : ring) {
    earcut_ring.push_back(to_earcut_point(point));
    flat_points.push_back(point);
  }

  earcut_polygon.push_back(std::move(earcut_ring));
}

[[nodiscard]] auto build_earcut_polygon(const geom::PolygonWithHoles &polygon,
                                        std::vector<geom::Point2> &flat_points)
    -> EarcutPolygon {
  EarcutPolygon earcut_polygon;
  earcut_polygon.reserve(1U + polygon.holes().size());
  flat_points.clear();

  append_ring(polygon.outer(), earcut_polygon, flat_points);
  for (const auto &hole : polygon.holes()) {
    append_ring(hole, earcut_polygon, flat_points);
  }

  return earcut_polygon;
}

[[nodiscard]] auto make_triangle(const geom::Point2 &first,
                                 const geom::Point2 &second,
                                 const geom::Point2 &third) -> geom::Polygon {
  return geom::normalize_polygon(
      geom::Polygon(geom::Ring{first, second, third}));
}

[[nodiscard]] auto canonical_edge(const geom::Point2 &first,
                                  const geom::Point2 &second) -> EdgeKey {
  if (second < first) {
    return EdgeKey{second, first};
  }
  return EdgeKey{first, second};
}

auto populate_adjacency(const std::vector<geom::Polygon> &triangles,
                        std::vector<std::array<std::int32_t, 3>> &neighbours)
    -> void {
  std::map<EdgeKey, EdgeRef> edge_refs;

  for (std::size_t triangle_index = 0; triangle_index < triangles.size();
       ++triangle_index) {
    const auto &ring = triangles[triangle_index].outer();
    if (ring.size() != 3U) {
      continue;
    }

    for (std::int32_t edge_index = 0; edge_index < 3; ++edge_index) {
      const auto next_index = (edge_index + 1) % 3;
      const auto key =
          canonical_edge(ring[static_cast<std::size_t>(edge_index)],
                         ring[static_cast<std::size_t>(next_index)]);

      const auto [it, inserted] = edge_refs.emplace(
          key, EdgeRef{static_cast<std::int32_t>(triangle_index), edge_index});
      if (!inserted) {
        neighbours[triangle_index][static_cast<std::size_t>(edge_index)] =
            it->second.triangle_index;
        neighbours[static_cast<std::size_t>(it->second.triangle_index)]
                  [static_cast<std::size_t>(it->second.edge_index)] =
                      static_cast<std::int32_t>(triangle_index);
      }
    }
  }
}

[[nodiscard]] auto triangle_less(const geom::Polygon &lhs,
                                 const geom::Polygon &rhs) -> bool {
  return detail::polygon_origin_less(lhs, rhs);
}

} // namespace

auto triangulate_polygon(const geom::Polygon &polygon)
    -> std::expected<TriangulationResult, util::Status> {
  return triangulate_polygon(geom::PolygonWithHoles(polygon.outer()));
}

auto triangulate_polygon(const geom::PolygonWithHoles &polygon)
    -> std::expected<TriangulationResult, util::Status> {
  const auto normalized = geom::normalize_polygon(polygon);
  if (!geom::validate_polygon(normalized).is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }

  std::vector<geom::Point2> flat_points;
  const auto earcut_polygon = build_earcut_polygon(normalized, flat_points);
  if (earcut_polygon.empty() || flat_points.size() < 3U) {
    return std::unexpected(util::Status::computation_failed);
  }

  const auto indices = mapbox::earcut<std::uint32_t>(earcut_polygon);
  if (indices.size() < 3U || indices.size() % 3U != 0U) {
    return std::unexpected(util::Status::computation_failed);
  }

  std::vector<geom::Polygon> triangles;
  triangles.reserve(indices.size() / 3U);
  for (std::size_t index = 0; index < indices.size(); index += 3U) {
    const auto first_index = static_cast<std::size_t>(indices[index]);
    const auto second_index = static_cast<std::size_t>(indices[index + 1U]);
    const auto third_index = static_cast<std::size_t>(indices[index + 2U]);
    if (first_index >= flat_points.size() ||
        second_index >= flat_points.size() ||
        third_index >= flat_points.size()) {
      continue;
    }

    auto triangle =
        make_triangle(flat_points[first_index], flat_points[second_index],
                      flat_points[third_index]);
    if (geom::polygon_area(triangle) > kAreaEpsilon) {
      triangles.push_back(std::move(triangle));
    }
  }

  if (triangles.empty()) {
    return std::unexpected(util::Status::computation_failed);
  }

  std::vector<std::array<std::int32_t, 3>> neighbours(triangles.size(),
                                                      {-1, -1, -1});
  populate_adjacency(triangles, neighbours);

  std::vector<std::int32_t> order(triangles.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](const std::int32_t lhs, const std::int32_t rhs) {
              return triangle_less(triangles[static_cast<std::size_t>(lhs)],
                                   triangles[static_cast<std::size_t>(rhs)]);
            });

  std::vector<std::int32_t> old_to_new(triangles.size(), -1);
  for (std::size_t new_index = 0; new_index < order.size(); ++new_index) {
    old_to_new[static_cast<std::size_t>(order[new_index])] =
        static_cast<std::int32_t>(new_index);
  }

  TriangulationResult result{};
  result.triangles.reserve(order.size());
  result.neighbours.reserve(order.size());
  for (const auto old_index : order) {
    const auto source_index = static_cast<std::size_t>(old_index);
    result.triangles.push_back(std::move(triangles[source_index]));

    auto remapped = neighbours[source_index];
    for (auto &neighbour_index : remapped) {
      if (neighbour_index >= 0) {
        neighbour_index = old_to_new[static_cast<std::size_t>(neighbour_index)];
      }
    }
    result.neighbours.push_back(remapped);
  }

  return result;
}

} // namespace shiny::nesting::decomp