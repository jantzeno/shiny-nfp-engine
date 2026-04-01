#include "decomposition/decompose.hpp"

#include <algorithm>
#include <cmath>
#include <list>
#include <span>
#include <vector>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/exceptions.h>
#include <CGAL/partition_2.h>

#include "geometry/detail/point_compare.hpp"
#include "geometry/normalize.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/merge_region.hpp"
#include "polygon_ops/simplify.hpp"
#include "predicates/orientation.hpp"

namespace shiny::nfp::decomp {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Traits = CGAL::Partition_traits_2<Kernel>;
using CgalPoint = Kernel::Point_2;
using CgalPolygon = Traits::Polygon_2;

struct FaceInfo {
  int nesting_level{-1};

  [[nodiscard]] auto in_domain() const -> bool {
    return nesting_level % 2 == 1;
  }
};

using VertexBase = CGAL::Triangulation_vertex_base_2<Kernel>;
using FaceBaseInfo =
    CGAL::Triangulation_face_base_with_info_2<FaceInfo, Kernel>;
using FaceBase =
    CGAL::Constrained_triangulation_face_base_2<Kernel, FaceBaseInfo>;
using TriangulationDataStructure =
    CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
using Cdt = CGAL::Constrained_Delaunay_triangulation_2<
    Kernel, TriangulationDataStructure, CGAL::Exact_intersections_tag>;

constexpr auto area_epsilon = 1.0e-9;

[[nodiscard]] auto ring_signed_area(const geom::Ring &ring) -> long double {
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

[[nodiscard]] auto polygon_signed_area(const geom::PolygonWithHoles &polygon)
    -> long double {
  long double area = ring_signed_area(polygon.outer);
  for (const auto &hole : polygon.holes) {
    area += ring_signed_area(hole);
  }
  return area;
}

[[nodiscard]] auto to_cgal_point(const geom::Point2 &point) -> CgalPoint {
  return {point.x, point.y};
}

[[nodiscard]] auto from_cgal_point(const CgalPoint &point) -> geom::Point2 {
  return {CGAL::to_double(point.x()), CGAL::to_double(point.y())};
}

[[nodiscard]] auto normalize_component_ring(const geom::Ring &ring)
    -> geom::Ring {
  return poly::simplify_polygon(geom::Polygon{.outer = ring}).outer;
}

void insert_ring_constraints(Cdt &cdt, const geom::Ring &ring) {
  if (ring.size() < 2U) {
    return;
  }

  std::vector<CgalPoint> points;
  points.reserve(ring.size());
  for (const auto &point : ring) {
    points.push_back(to_cgal_point(point));
  }

  cdt.insert_constraint(points.begin(), points.end(), true);
}

template <class Triangulation>
void mark_domains(Triangulation &triangulation,
                  typename Triangulation::Face_handle start_face, int index,
                  std::list<typename Triangulation::Edge> &border) {
  if (start_face->info().nesting_level != -1) {
    return;
  }

  std::list<typename Triangulation::Face_handle> queue;
  queue.push_back(start_face);

  while (!queue.empty()) {
    const auto face = queue.front();
    queue.pop_front();
    if (face->info().nesting_level != -1) {
      continue;
    }

    face->info().nesting_level = index;
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      const typename Triangulation::Edge edge(face, edge_index);
      const auto neighbor = face->neighbor(edge_index);
      if (neighbor->info().nesting_level != -1) {
        continue;
      }

      if (triangulation.is_constrained(edge)) {
        border.push_back(edge);
      } else {
        queue.push_back(neighbor);
      }
    }
  }
}

template <class Triangulation> void mark_nested_domains(Triangulation &cdt) {
  for (auto face = cdt.all_faces_begin(); face != cdt.all_faces_end(); ++face) {
    face->info().nesting_level = -1;
  }

  std::list<typename Triangulation::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while (!border.empty()) {
    const auto edge = border.front();
    border.pop_front();
    const auto neighbor = edge.first->neighbor(edge.second);
    if (neighbor->info().nesting_level == -1) {
      mark_domains(cdt, neighbor, edge.first->info().nesting_level + 1, border);
    }
  }
}

[[nodiscard]] auto
triangulate_polygon_with_holes(const geom::PolygonWithHoles &source)
    -> std::vector<ConvexComponent> {
  Cdt triangulation;
  insert_ring_constraints(triangulation, source.outer);
  for (const auto &hole : source.holes) {
    insert_ring_constraints(triangulation, hole);
  }

  mark_nested_domains(triangulation);

  std::vector<ConvexComponent> components;
  std::size_t component_index = 0;
  for (auto face = triangulation.finite_faces_begin();
       face != triangulation.finite_faces_end(); ++face) {
    if (!face->info().in_domain()) {
      continue;
    }

    geom::Ring outer;
    outer.reserve(3U);
    for (int vertex_index = 0; vertex_index < 3; ++vertex_index) {
      outer.push_back(from_cgal_point(face->vertex(vertex_index)->point()));
    }

    outer = normalize_component_ring(outer);
    if (outer.size() < 3U) {
      continue;
    }

    components.push_back({
        .outer = std::move(outer),
        .source_component_index = component_index,
        .normalized = true,
    });
    ++component_index;
  }

  return components;
}

[[nodiscard]] auto component_less(const ConvexComponent &lhs,
                                  const ConvexComponent &rhs) -> bool {
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
  return lhs.source_component_index < rhs.source_component_index;
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

[[nodiscard]] auto normalize_source(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  return poly::simplify_polygon(geom::normalize_polygon(polygon));
}

[[nodiscard]] auto polygon_equal(const geom::PolygonWithHoles &lhs,
                                 const geom::PolygonWithHoles &rhs) -> bool {
  return poly::difference_polygons(lhs, rhs).empty() &&
         poly::difference_polygons(rhs, lhs).empty();
}

[[nodiscard]] auto union_components(const DecompositionResult &result)
    -> std::vector<geom::PolygonWithHoles> {
  if (result.components.empty()) {
    return {};
  }

  auto merged_region =
      poly::make_merged_region({.outer = result.components.front().outer});

  for (std::size_t index = 1; index < result.components.size(); ++index) {
    merged_region = poly::merge_polygon_into_region(
        merged_region,
        geom::PolygonWithHoles{.outer = result.components[index].outer});
  }

  return merged_region.regions;
}

template <class OutputIterator>
void run_partition(const std::vector<CgalPoint> &vertices,
                   DecompositionAlgorithm algorithm, OutputIterator output) {
  Traits traits{};
  switch (algorithm) {
  case DecompositionAlgorithm::cgal_optimal_convex_partition:
    CGAL::optimal_convex_partition_2(vertices.begin(), vertices.end(), output,
                                     traits);
    return;
  case DecompositionAlgorithm::cgal_approx_convex_partition:
    CGAL::approx_convex_partition_2(vertices.begin(), vertices.end(), output,
                                    traits);
    return;
  }
}

[[nodiscard]] auto partition_components(const geom::PolygonWithHoles &source,
                                        DecompositionAlgorithm algorithm)
    -> std::optional<std::vector<ConvexComponent>> {
  std::vector<CgalPoint> vertices;
  vertices.reserve(source.outer.size());
  for (const auto &point : source.outer) {
    vertices.push_back(to_cgal_point(point));
  }

  std::list<CgalPolygon> partition;
  try {
    run_partition(vertices, algorithm, std::back_inserter(partition));
  } catch (const CGAL::Failure_exception &) {
    return std::nullopt;
  }

  std::vector<ConvexComponent> components;
  std::size_t component_index = 0;
  for (const auto &polygon : partition) {
    geom::Ring outer;
    outer.reserve(polygon.size());
    for (auto vertex = polygon.vertices_begin();
         vertex != polygon.vertices_end(); ++vertex) {
      outer.push_back(from_cgal_point(*vertex));
    }

    outer = normalize_component_ring(outer);
    if (outer.size() < 3U) {
      ++component_index;
      continue;
    }

    components.push_back({
        .outer = std::move(outer),
        .source_component_index = component_index,
        .normalized = true,
    });
    ++component_index;
  }

  return components;
}

} // namespace

auto decompose_polygon(const DecompositionRequest &request)
    -> DecompositionResult {
  const auto source = normalize_source(request.polygon);

  DecompositionResult result{};
  result.signed_area = static_cast<double>(polygon_signed_area(source));

  if (source.outer.size() < 3U) {
    result.validity = DecompositionValidity::invalid_topology;
    return result;
  }

  if (source.holes.empty() && is_ring_convex(source.outer)) {
    result.components.push_back({
        .outer = source.outer,
        .source_component_index = 0,
        .normalized = true,
    });
    result.validity = validate_decomposition(source, result);
    return result;
  }

  if (!source.holes.empty()) {
    result.components = triangulate_polygon_with_holes(source);
    std::sort(result.components.begin(), result.components.end(),
              component_less);
    result.validity = validate_decomposition(source, result);
    return result;
  }

  if (const auto partitioned =
          partition_components(source, request.algorithm)) {
    result.components = *partitioned;
    std::sort(result.components.begin(), result.components.end(),
              component_less);
    result.validity = validate_decomposition(source, result);
    if (result.validity == DecompositionValidity::valid) {
      return result;
    }
  }

  result.components = triangulate_polygon_with_holes(source);
  std::sort(result.components.begin(), result.components.end(), component_less);
  result.validity = validate_decomposition(source, result);
  return result;
}

auto decompose_polygon(const DecompositionRequest &request,
                       cache::GeometryRevision geometry_revision,
                       cache::CacheStore<cache::PieceRotationKey,
                                         DecompositionResult> &cache_store)
    -> DecompositionResult {
  const auto key = cache::make_piece_rotation_key(
      request.piece_id, request.rotation, geometry_revision);
  if (const auto *cached = cache_store.get(key)) {
    auto result = *cached;
    result.cached = true;
    return result;
  }

  auto result = decompose_polygon(request);
  result.cached = false;
  cache_store.put(key, result);
  return result;
}

auto validate_decomposition(const geom::PolygonWithHoles &source,
                            const DecompositionResult &result)
    -> DecompositionValidity {
  const auto normalized_source = normalize_source(source);

  if (normalized_source.outer.size() < 3U) {
    return DecompositionValidity::invalid_topology;
  }

  if (result.components.empty()) {
    return DecompositionValidity::invalid_topology;
  }

  long double component_area_sum = 0.0L;
  for (const auto &component : result.components) {
    if (!component.normalized || component.outer.size() < 3U) {
      return DecompositionValidity::invalid_topology;
    }

    const auto area = ring_signed_area(component.outer);
    if (area <= 0.0L) {
      return DecompositionValidity::invalid_orientation;
    }

    if (!is_ring_convex(component.outer)) {
      return DecompositionValidity::nonconvex_component;
    }

    component_area_sum += area;
  }

  const auto expected_area = polygon_signed_area(normalized_source);
  if (std::fabs(static_cast<double>(component_area_sum - expected_area)) >
      area_epsilon) {
    return DecompositionValidity::area_mismatch;
  }

  const auto merged_components = union_components(result);
  if (merged_components.size() != 1U) {
    return DecompositionValidity::invalid_topology;
  }

  const auto merged = normalize_source(merged_components.front());
  if (!polygon_equal(merged, normalized_source)) {
    return DecompositionValidity::invalid_topology;
  }

  return DecompositionValidity::valid;
}

auto DecompositionEngine::decompose_polygon(
    const DecompositionRequest &request,
    cache::GeometryRevision geometry_revision) -> DecompositionResult {
  return shiny::nfp::decomp::decompose_polygon(request, geometry_revision,
                                               cache_);
}

auto DecompositionEngine::clear_cache() -> void { cache_.clear(); }

auto DecompositionEngine::cache_size() const -> std::size_t {
  return cache_.size();
}

} // namespace shiny::nfp::decomp