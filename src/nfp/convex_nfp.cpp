#include "nfp/convex_nfp.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>

#include "decomposition/convex_decomposition.hpp"
#include "geometry/normalize.hpp"
#include "geometry/validity.hpp"

namespace shiny::nesting::nfp {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPolygon = CGAL::Polygon_2<Kernel>;
using CgalPolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

[[nodiscard]] auto to_cgal_polygon(const geom::Polygon &polygon) -> CgalPolygon {
  const auto normalized = geom::normalize_polygon(polygon);
  CgalPolygon result;
  for (const auto &point : normalized.outer) {
    result.push_back({point.x, point.y});
  }
  if (result.orientation() == CGAL::CLOCKWISE) {
    result.reverse_orientation();
  }
  return result;
}

[[nodiscard]] auto reflect_polygon(const geom::Polygon &polygon) -> geom::Polygon {
  geom::Polygon reflected;
  reflected.outer.reserve(polygon.outer.size());
  for (const auto &point : polygon.outer) {
    reflected.outer.push_back({.x = -point.x, .y = -point.y});
  }
  return geom::normalize_polygon(reflected);
}

[[nodiscard]] auto from_cgal_polygon(const CgalPolygon &polygon)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles result;
  result.outer.reserve(polygon.size());
  for (auto vertex = polygon.vertices_begin(); vertex != polygon.vertices_end();
       ++vertex) {
    result.outer.push_back({.x = vertex->x(), .y = vertex->y()});
  }
  return geom::normalize_polygon(result);
}

[[nodiscard]] auto from_cgal_polygon_with_holes(const CgalPolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles result = from_cgal_polygon(polygon.outer_boundary());
  for (auto hole = polygon.holes_begin(); hole != polygon.holes_end(); ++hole) {
    geom::Ring converted_hole;
    converted_hole.reserve(hole->size());
    for (auto vertex = hole->vertices_begin(); vertex != hole->vertices_end();
         ++vertex) {
      converted_hole.push_back({.x = vertex->x(), .y = vertex->y()});
    }
    result.holes.push_back(std::move(converted_hole));
  }
  return geom::normalize_polygon(result);
}

} // namespace

// No-Fit Polygon for two convex pieces.
//
// Identity:  NFP(A, B) = A ⊕ (-B)   (Minkowski sum of A with reflection of B)
// where -B = { -p : p ∈ B }. Reflecting B around the origin and convolving
// with A yields the locus of B's reference point such that A and B touch
// (boundary) or overlap (interior) — exactly the NFP. CGAL's
// minkowski_sum_2 requires both operands to be CCW-oriented simple polygons
// (preconditions enforced by `to_cgal_polygon` and `is_convex` checks
// below).
//
// Returns a single connected polygon (with holes possible only when the
// input pieces themselves were not convex — guarded against above).
auto compute_convex_nfp(const geom::Polygon &fixed, const geom::Polygon &moving)
    -> util::StatusOr<geom::PolygonWithHoles> {
  const auto normalized_fixed = geom::normalize_polygon(fixed);
  const auto normalized_moving = geom::normalize_polygon(moving);

  if (!geom::validate_polygon(normalized_fixed).is_valid() ||
      !geom::validate_polygon(normalized_moving).is_valid()) {
    return util::Status::invalid_input;
  }
  if (!decomp::is_convex(normalized_fixed) || !decomp::is_convex(normalized_moving)) {
    return util::Status::invalid_input;
  }

  const auto reflected_moving = reflect_polygon(normalized_moving);
  const auto minkowski =
      CGAL::minkowski_sum_2(to_cgal_polygon(normalized_fixed),
                            to_cgal_polygon(reflected_moving));
  return from_cgal_polygon_with_holes(minkowski);
}

} // namespace shiny::nesting::nfp
