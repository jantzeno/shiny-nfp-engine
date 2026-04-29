#include "nfp/convex_nfp.hpp"

#include <cmath>
#include <exception>

#include <algorithm>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/minkowski_sum_2.h>

#include "decomposition/convex_decomposition.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/validity.hpp"
#include "logging/shiny_log.hpp"
#include "polygon_ops/simplify.hpp"

namespace shiny::nesting::nfp {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPolygon = CGAL::Polygon_2<Kernel>;
using CgalPolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

[[nodiscard]] auto to_cgal_polygon(const geom::Polygon &polygon)
    -> CgalPolygon {
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

[[nodiscard]] auto reflect_polygon(const geom::Polygon &polygon)
    -> geom::Polygon {
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

[[nodiscard]] auto
from_cgal_polygon_with_holes(const CgalPolygonWithHoles &polygon)
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

[[nodiscard]] auto lowest_leftmost_index(const geom::Ring &ring)
    -> std::size_t {
  return static_cast<std::size_t>(std::distance(
      ring.begin(),
      std::min_element(ring.begin(), ring.end(),
                       [](const geom::Point2 &lhs, const geom::Point2 &rhs) {
                         if (lhs.y != rhs.y) {
                           return lhs.y < rhs.y;
                         }
                         return lhs.x < rhs.x;
                       })));
}

[[nodiscard]] auto edge_vector(const geom::Ring &ring, const std::size_t index)
    -> geom::Vector2 {
  const auto next = (index + 1U) % ring.size();
  return {.x = ring[next].x - ring[index].x, .y = ring[next].y - ring[index].y};
}

[[nodiscard]] auto add_point_vector(const geom::Point2 &point,
                                    const geom::Vector2 &vector)
    -> geom::Point2 {
  return {.x = point.x + vector.x, .y = point.y + vector.y};
}

[[nodiscard]] auto add_vectors(const geom::Vector2 &lhs,
                               const geom::Vector2 &rhs) -> geom::Vector2 {
  return {.x = lhs.x + rhs.x, .y = lhs.y + rhs.y};
}

[[nodiscard]] auto cross(const geom::Vector2 &lhs, const geom::Vector2 &rhs)
    -> double {
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

[[nodiscard]] auto manual_convex_nfp(const geom::Polygon &fixed,
                                     const geom::Polygon &moving)
    -> util::StatusOr<geom::PolygonWithHoles> {
  constexpr double kParallelTolerance = 1e-9;

  const auto reflected_moving = reflect_polygon(moving);
  const auto &fixed_ring = fixed.outer;
  const auto &moving_ring = reflected_moving.outer;
  if (fixed_ring.size() < 3U || moving_ring.size() < 3U) {
    return util::Status::invalid_input;
  }

  const auto fixed_start = lowest_leftmost_index(fixed_ring);
  const auto moving_start = lowest_leftmost_index(moving_ring);
  const auto fixed_count = fixed_ring.size();
  const auto moving_count = moving_ring.size();

  std::vector<geom::Point2> sum_ring;
  sum_ring.reserve(fixed_count + moving_count + 1U);
  geom::Point2 current{
      .x = fixed_ring[fixed_start].x + moving_ring[moving_start].x,
      .y = fixed_ring[fixed_start].y + moving_ring[moving_start].y};
  sum_ring.push_back(current);

  std::size_t fixed_steps = 0U;
  std::size_t moving_steps = 0U;
  while (fixed_steps < fixed_count || moving_steps < moving_count) {
    geom::Vector2 step{};
    const bool can_take_fixed = fixed_steps < fixed_count;
    const bool can_take_moving = moving_steps < moving_count;
    const auto fixed_index = (fixed_start + fixed_steps) % fixed_count;
    const auto moving_index = (moving_start + moving_steps) % moving_count;

    if (can_take_fixed && can_take_moving) {
      const auto fixed_edge = edge_vector(fixed_ring, fixed_index);
      const auto moving_edge = edge_vector(moving_ring, moving_index);
      const double edge_cross = cross(fixed_edge, moving_edge);
      if (edge_cross > kParallelTolerance) {
        step = fixed_edge;
        ++fixed_steps;
      } else if (edge_cross < -kParallelTolerance) {
        step = moving_edge;
        ++moving_steps;
      } else {
        step = add_vectors(fixed_edge, moving_edge);
        ++fixed_steps;
        ++moving_steps;
      }
    } else if (can_take_fixed) {
      step = edge_vector(fixed_ring, fixed_index);
      ++fixed_steps;
    } else {
      step = edge_vector(moving_ring, moving_index);
      ++moving_steps;
    }

    current = add_point_vector(current, step);
    sum_ring.push_back(current);
  }

  if (sum_ring.size() > 1U &&
      geom::point_distance(sum_ring.front(), sum_ring.back()) <=
          kParallelTolerance) {
    sum_ring.pop_back();
  }

  auto simplified = poly::simplify_collinear_ring(sum_ring);
  if (simplified.size() < 3U) {
    return util::Status::computation_failed;
  }

  const auto polygon = geom::normalize_polygon(
      geom::PolygonWithHoles{.outer = std::move(simplified)});
  if (!geom::validate_polygon(polygon).is_valid() ||
      geom::polygon_area(polygon) <= 0.0) {
    return util::Status::computation_failed;
  }

  return polygon;
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
  if (!decomp::is_convex(normalized_fixed) ||
      !decomp::is_convex(normalized_moving)) {
    return util::Status::invalid_input;
  }

  const auto reflected_moving = reflect_polygon(normalized_moving);
  if (!geom::validate_polygon(reflected_moving).is_valid()) {
    return util::Status::invalid_input;
  }

  try {
    const auto minkowski = CGAL::minkowski_sum_2(
        to_cgal_polygon(normalized_fixed), to_cgal_polygon(reflected_moving));
    return from_cgal_polygon_with_holes(minkowski);
  } catch (const std::exception &) {
    SHINY_DEBUG("nfp: convex CGAL minkowski_sum_2 failed fixed_outer={} "
                "moving_outer={}",
                normalized_fixed.outer.size(), normalized_moving.outer.size());
  } catch (...) {
    SHINY_DEBUG("nfp: convex CGAL minkowski_sum_2 failed fixed_outer={} "
                "moving_outer={} with unknown exception",
                normalized_fixed.outer.size(), normalized_moving.outer.size());
  }

  auto manual_nfp = manual_convex_nfp(normalized_fixed, normalized_moving);
  if (manual_nfp.ok()) {
    SHINY_DEBUG(
        "nfp: convex manual fallback succeeded fixed_outer={} moving_outer={}",
        normalized_fixed.outer.size(), normalized_moving.outer.size());
    return manual_nfp;
  }
  return util::Status::computation_failed;
}

} // namespace shiny::nesting::nfp
