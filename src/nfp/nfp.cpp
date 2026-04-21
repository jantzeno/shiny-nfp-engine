#include "nfp/nfp.hpp"

#include <algorithm>
#include <optional>
#include <vector>

#include "decomposition/convex_decomposition.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/validity.hpp"
#include "nfp/convex_nfp.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/greedy_merge.hpp"

namespace shiny::nesting::nfp {
namespace {

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

// Iteratively unions any pair of polygons whose union is a single connected
// region. Used to merge the per-pair convex NFPs produced by decomposition
// into the final aggregate NFP (which is the union of all pair NFPs by
// definition of decomposed Minkowski sum). A union returning >1 polygon
// means the pair is disjoint (skip); returning 1 polygon means they
// overlap or share an edge (accept). Same O(P³) shape as the greedy merge
// in `decomposition/convex_decomposition.cpp` — see DRY note in review
// report.
[[nodiscard]] auto merge_polygon_set(std::vector<geom::PolygonWithHoles> polygons)
    -> std::vector<geom::PolygonWithHoles> {
  polygons = poly::greedy_pairwise_merge(
      std::move(polygons),
      [](const geom::PolygonWithHoles &lhs, const geom::PolygonWithHoles &rhs)
          -> std::optional<geom::PolygonWithHoles> {
        const auto unioned = poly::union_polygons(lhs, rhs);
        if (unioned.size() != 1U) {
          return std::nullopt;
        }
        return unioned.front();
      });
  std::sort(polygons.begin(), polygons.end(), polygon_less);
  return polygons;
}

} // namespace

// General NFP dispatcher.
//
// Two paths:
//   1. Fast path — both inputs hole-free convex: delegate to
//      `compute_convex_nfp` (single CGAL minkowski_sum_2 call).
//   2. General path — decompose each polygon into convex pieces, compute
//      pairwise convex NFP for every (fixed_part, moving_part) pair, and
//      union the results. The union is correct because for A = ∪Aᵢ and
//      B = ∪Bⱼ we have NFP(A,B) = ∪ᵢⱼ NFP(Aᵢ, Bⱼ) when the decompositions
//      cover the originals (see Bennell & Oliveira, 2008, §3.3).
//
// Output is a vector of polygons because the union may be disconnected
// when A and B are non-convex with sufficiently complex reflex pockets.
auto compute_nfp(const geom::PolygonWithHoles &fixed,
                 const geom::PolygonWithHoles &moving)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  const auto normalized_fixed = geom::normalize_polygon(fixed);
  const auto normalized_moving = geom::normalize_polygon(moving);

  if (!geom::validate_polygon(normalized_fixed).is_valid() ||
      !geom::validate_polygon(normalized_moving).is_valid()) {
    return util::Status::invalid_input;
  }

  if (normalized_fixed.holes.empty() && normalized_moving.holes.empty() &&
      decomp::is_convex(geom::Polygon{.outer = normalized_fixed.outer}) &&
      decomp::is_convex(geom::Polygon{.outer = normalized_moving.outer})) {
    auto convex_nfp = compute_convex_nfp(geom::Polygon{.outer = normalized_fixed.outer},
                                         geom::Polygon{.outer = normalized_moving.outer});
    if (!convex_nfp.ok()) {
      return convex_nfp.status();
    }
    return std::vector<geom::PolygonWithHoles>{std::move(convex_nfp).value()};
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
      auto pair_nfp = compute_convex_nfp(fixed_part, moving_part);
      if (!pair_nfp.ok()) {
        return pair_nfp.status();
      }
      aggregate.push_back(std::move(pair_nfp).value());
    }
  }

  if (aggregate.empty()) {
    return util::Status::computation_failed;
  }

  return merge_polygon_set(std::move(aggregate));
}

} // namespace shiny::nesting::nfp
