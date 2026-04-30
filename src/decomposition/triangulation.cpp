#include "decomposition/triangulation.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <list>
#include <numeric>
#include <span>
#include <vector>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include "decomposition/internal.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/validity.hpp"

// Constrained Delaunay Triangulation (CDT) for polygons with holes.
//
// Pipeline:
//   1. Insert every input ring as a *constrained* edge sequence into a CGAL
//      CDT. Outer ring + holes are constrained identically; the kernel
//      handles intersection-free polygons only — callers must normalize
//      first via geom::normalize_polygon().
//   2. Run mark_domains() — a flood-fill on the CDT face graph that assigns
//      each face a "nesting level" (parity == in/out of the polygon
//      domain). The infinite face starts at level 0 (outside); crossing a
//      constrained edge bumps the level. Even => outside, odd => inside.
//      This is the standard CGAL recipe documented in
//      `CGAL_2D_Triangulations/Constrained_triangulation_2.html` (search
//      "mark_domains").
//   3. Emit only finite faces with `in_domain() == true`, sorted by bbox
//      origin then area for deterministic output.
//
// Precision notes: uses Exact_predicates_inexact_constructions_kernel.
// Predicates (orientation/in-circle) are exact; constructions (segment
// intersection) are double precision. Sliver triangles below kAreaEpsilon
// are dropped to keep downstream NFP/decomposition robust.

namespace shiny::nesting::decomp {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using VertexBase = CGAL::Triangulation_vertex_base_2<Kernel>;

struct FaceInfo {
  int nesting_level{-1};
  std::int32_t triangle_index{-1};

  [[nodiscard]] auto in_domain() const -> bool {
    return nesting_level % 2 == 1;
  }
};

using FaceBaseInfo =
    CGAL::Triangulation_face_base_with_info_2<FaceInfo, Kernel>;
using FaceBase =
    CGAL::Constrained_triangulation_face_base_2<Kernel, FaceBaseInfo>;
using TriangulationData =
    CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
using ConstrainedTriangulation =
    CGAL::Constrained_Delaunay_triangulation_2<Kernel, TriangulationData>;

constexpr double kAreaEpsilon = 1e-9;

[[nodiscard]] auto to_cgal_point(const geom::Point2 &point) -> Kernel::Point_2 {
  return {point.x, point.y};
}

auto insert_ring_constraints(ConstrainedTriangulation &triangulation,
                             std::span<const geom::Point2> ring) -> void {
  if (ring.size() < 2U) {
    return;
  }

  auto previous = triangulation.insert(to_cgal_point(ring.front()));
  const auto first = previous;

  for (std::size_t index = 1; index < ring.size(); ++index) {
    auto current = triangulation.insert(to_cgal_point(ring[index]));
    triangulation.insert_constraint(previous, current);
    previous = current;
  }

  triangulation.insert_constraint(previous, first);
}

template <typename Triangulation>
auto mark_domain_faces(Triangulation &triangulation,
                       typename Triangulation::Face_handle seed_face,
                       const int nesting_level,
                       std::list<typename Triangulation::Edge> &border)
    -> void {
  if (seed_face->info().nesting_level != -1) {
    return;
  }

  std::list<typename Triangulation::Face_handle> queue;
  queue.push_back(seed_face);

  while (!queue.empty()) {
    const auto face = queue.front();
    queue.pop_front();

    if (face->info().nesting_level != -1) {
      continue;
    }

    face->info().nesting_level = nesting_level;
    for (int index = 0; index < 3; ++index) {
      const typename Triangulation::Edge edge(face, index);
      const auto neighbor = face->neighbor(index);

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

// Two-pass nesting-level assignment:
//   Pass 1: flood from the (single) infinite face at level 0, stopping at
//           constrained edges; every face reachable without crossing a
//           constraint is "outside" (level 0).
//   Pass 2: drain `border` (constrained edges encountered during pass 1).
//           Each constrained edge separates the just-marked region from a
//           deeper region whose level = current + 1. Recurse until every
//           face has a level. After this, face.in_domain() == (level % 2).
auto mark_domains(ConstrainedTriangulation &triangulation) -> void {
  for (auto face = triangulation.all_faces_begin();
       face != triangulation.all_faces_end(); ++face) {
    face->info().nesting_level = -1;
    face->info().triangle_index = -1;
  }

  std::list<ConstrainedTriangulation::Edge> border;
  mark_domain_faces(triangulation, triangulation.infinite_face(), 0, border);

  while (!border.empty()) {
    const auto edge = border.front();
    border.pop_front();
    const auto face = edge.first;
    const auto neighbor = face->neighbor(edge.second);
    if (neighbor->info().nesting_level == -1) {
      mark_domain_faces(triangulation, neighbor, face->info().nesting_level + 1,
                        border);
    }
  }
}

[[nodiscard]] auto triangle_less(const geom::Polygon &lhs,
                                 const geom::Polygon &rhs) -> bool {
  return detail::polygon_origin_less(lhs, rhs);
}

} // namespace

// Convenience overload: triangulate a hole-free outer ring.
// `geom::Polygon` does not carry holes (single `outer` Ring member). A
// `static_assert` guards that invariant: if `Polygon` ever grows a `holes`
// member, this overload silently dropping them becomes a correctness bug,
// so the build must fail and force the author to revisit.
auto triangulate_polygon(const geom::Polygon &polygon)
    -> util::StatusOr<TriangulationResult> {
  static_assert(sizeof(geom::Polygon) == sizeof(geom::Ring),
                "geom::Polygon gained a member beyond `outer`. This overload "
                "would silently drop the new data — promote callers to the "
                "PolygonWithHoles overload or update this wrapper.");
  return triangulate_polygon(geom::PolygonWithHoles{.outer = polygon.outer});
}

auto triangulate_polygon(const geom::PolygonWithHoles &polygon)
    -> util::StatusOr<TriangulationResult> {
  const auto normalized = geom::normalize_polygon(polygon);
  if (!geom::validate_polygon(normalized).is_valid()) {
    return util::Status::invalid_input;
  }

  ConstrainedTriangulation triangulation;
  insert_ring_constraints(triangulation, normalized.outer);
  for (const auto &hole : normalized.holes) {
    insert_ring_constraints(triangulation, hole);
  }
  mark_domains(triangulation);

  std::vector<geom::Polygon> triangles;
  std::vector<std::array<std::int32_t, 3>> neighbours;
  std::vector<ConstrainedTriangulation::Face_handle> kept_faces;
  for (auto face = triangulation.finite_faces_begin();
       face != triangulation.finite_faces_end(); ++face) {
    if (!face->info().in_domain()) {
      continue;
    }

    geom::Polygon triangle{.outer = {
                               {.x = face->vertex(0)->point().x(),
                                .y = face->vertex(0)->point().y()},
                               {.x = face->vertex(1)->point().x(),
                                .y = face->vertex(1)->point().y()},
                               {.x = face->vertex(2)->point().x(),
                                .y = face->vertex(2)->point().y()},
                           }};
    triangle = geom::normalize_polygon(triangle);
    if (geom::polygon_area(triangle) > kAreaEpsilon) {
      face->info().triangle_index = static_cast<std::int32_t>(triangles.size());
      triangles.push_back(std::move(triangle));
      neighbours.push_back({-1, -1, -1});
      kept_faces.push_back(face);
    }
  }

  if (triangles.empty()) {
    return util::Status::computation_failed;
  }

  for (std::size_t index = 0; index < kept_faces.size(); ++index) {
    const auto face = kept_faces[index];
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      const ConstrainedTriangulation::Edge edge(face, edge_index);
      if (triangulation.is_constrained(edge)) {
        continue;
      }

      const auto neighbour = face->neighbor(edge_index);
      const auto neighbour_index = neighbour->info().triangle_index;
      if (neighbour_index >= 0 && neighbour->info().in_domain()) {
        neighbours[index][edge_index] = neighbour_index;
      }
    }
  }

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
