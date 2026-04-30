#include "nfp/nfp.hpp"

#include <algorithm>
#include <optional>
#include <vector>

#include "geometry/decomposition/convex_decomposition.hpp"
#include "geometry/operations/boolean_ops.hpp"
#include "geometry/operations/greedy_merge.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/sanitize.hpp"
#include "geometry/queries/validity.hpp"
#include "logging/shiny_log.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/orbiting_nfp.hpp"

namespace shiny::nesting::nfp {
namespace {

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  const auto lhs_bounds = geom::compute_bounds(lhs);
  const auto rhs_bounds = geom::compute_bounds(rhs);

  if (lhs_bounds.min.x() != rhs_bounds.min.x()) {
    return lhs_bounds.min.x() < rhs_bounds.min.x();
  }
  if (lhs_bounds.min.y() != rhs_bounds.min.y()) {
    return lhs_bounds.min.y() < rhs_bounds.min.y();
  }
  return geom::polygon_area(lhs) < geom::polygon_area(rhs);
}

[[nodiscard]] auto bounds_may_touch_or_overlap(const geom::Box2 &lhs,
                                               const geom::Box2 &rhs) -> bool {
  return !(lhs.max.x() < rhs.min.x() || rhs.max.x() < lhs.min.x() ||
           lhs.max.y() < rhs.min.y() || rhs.max.y() < lhs.min.y());
}

[[nodiscard]] constexpr auto
validity_issue_name(const geom::PolygonValidityIssue issue) -> const char * {
  switch (issue) {
  case geom::PolygonValidityIssue::ok:
    return "ok";
  case geom::PolygonValidityIssue::non_finite_coordinate:
    return "non_finite_coordinate";
  case geom::PolygonValidityIssue::too_few_vertices:
    return "too_few_vertices";
  case geom::PolygonValidityIssue::zero_area:
    return "zero_area";
  case geom::PolygonValidityIssue::self_intersection:
    return "self_intersection";
  case geom::PolygonValidityIssue::hole_outside_outer:
    return "hole_outside_outer";
  case geom::PolygonValidityIssue::hole_intersection:
    return "hole_intersection";
  }
  return "unknown";
}

auto log_invalid_input(const char *label,
                       const geom::PolygonSanitization &sanitized,
                       const geom::PolygonValidity &validity) -> void {
  SHINY_DEBUG(
      "nfp: invalid {} polygon issue={} outer={} holes={} duplicates={} "
      "zero_edges={} slivers={}",
      label, validity_issue_name(validity.issue),
      sanitized.polygon.outer().size(), sanitized.polygon.holes().size(),
      sanitized.duplicate_vertices, sanitized.zero_length_edges,
      sanitized.sliver_rings);
}

auto log_orbiting_fallback(const char *reason, const util::Status status)
    -> void {
  SHINY_DEBUG("nfp: orbiting fallback after {} returned {}", reason,
              util::status_name(status));
}

// Iteratively unions any pair of polygons whose union is a single connected
// region. Used to merge the per-pair convex NFPs produced by decomposition
// into the final aggregate NFP (which is the union of all pair NFPs by
// definition of decomposed Minkowski sum). A union returning >1 polygon
// means the pair is disjoint (skip); returning 1 polygon means they
// overlap or share an edge (accept). Same O(P³) shape as the greedy merge
// in `decomposition/convex_decomposition.cpp` — see DRY note in review
// report.
[[nodiscard]] auto
merge_polygon_set(std::vector<geom::PolygonWithHoles> polygons)
    -> std::vector<geom::PolygonWithHoles> {
  polygons = geom::greedy_pairwise_merge(
      std::move(polygons),
      [](const geom::PolygonWithHoles &lhs, const geom::PolygonWithHoles &rhs)
          -> std::optional<geom::PolygonWithHoles> {
        if (!bounds_may_touch_or_overlap(geom::compute_bounds(lhs),
                                         geom::compute_bounds(rhs))) {
          return std::nullopt;
        }

        auto unioned = geom::try_union_polygons(lhs, rhs);
        if (!unioned.ok()) {
          SHINY_DEBUG(
              "nfp: merge_polygon_set union failed status={} lhs_outer={} "
              "rhs_outer={}",
              util::status_name(unioned.status()), lhs.outer().size(),
              rhs.outer().size());
          return std::nullopt;
        }
        if (unioned.value().size() != 1U) {
          return std::nullopt;
        }
        return unioned.value().front();
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
  const auto sanitized_fixed = geom::sanitize_polygon(fixed);
  const auto sanitized_moving = geom::sanitize_polygon(moving);
  const auto &normalized_fixed = sanitized_fixed.polygon;
  const auto &normalized_moving = sanitized_moving.polygon;
  const auto fixed_validity = geom::validate_polygon(normalized_fixed);
  const auto moving_validity = geom::validate_polygon(normalized_moving);

  if (!fixed_validity.is_valid() || !moving_validity.is_valid()) {
    if (!fixed_validity.is_valid()) {
      log_invalid_input("fixed", sanitized_fixed, fixed_validity);
    }
    if (!moving_validity.is_valid()) {
      log_invalid_input("moving", sanitized_moving, moving_validity);
    }
    return util::Status::invalid_input;
  }

  if (normalized_fixed.holes().empty() && normalized_moving.holes().empty() &&
      geom::polygon_is_convex(geom::Polygon(normalized_fixed.outer())) &&
      geom::polygon_is_convex(geom::Polygon(normalized_moving.outer()))) {
    auto convex_nfp =
        compute_convex_nfp(geom::Polygon(normalized_fixed.outer()),
                           geom::Polygon(normalized_moving.outer()));
    if (!convex_nfp.ok()) {
      SHINY_DEBUG("nfp: convex fast path failed status={}",
                  util::status_name(convex_nfp.status()));
      auto orbiting = compute_orbiting_nfp(normalized_fixed, normalized_moving);
      log_orbiting_fallback("convex_fast_path_failure", orbiting.status());
      if (orbiting.ok()) {
        return orbiting;
      }
      return convex_nfp.status();
    }
    return std::vector<geom::PolygonWithHoles>{std::move(convex_nfp).value()};
  }

  auto fixed_parts_or = decomp::decompose_convex(normalized_fixed);
  if (!fixed_parts_or.ok()) {
    SHINY_DEBUG("nfp: fixed decomposition failed status={}",
                util::status_name(fixed_parts_or.status()));
    auto orbiting = compute_orbiting_nfp(normalized_fixed, normalized_moving);
    log_orbiting_fallback("fixed_decomposition_failure", orbiting.status());
    if (orbiting.ok()) {
      return orbiting;
    }
    return fixed_parts_or.status();
  }
  auto moving_parts_or = decomp::decompose_convex(normalized_moving);
  if (!moving_parts_or.ok()) {
    SHINY_DEBUG("nfp: moving decomposition failed status={}",
                util::status_name(moving_parts_or.status()));
    auto orbiting = compute_orbiting_nfp(normalized_fixed, normalized_moving);
    log_orbiting_fallback("moving_decomposition_failure", orbiting.status());
    if (orbiting.ok()) {
      return orbiting;
    }
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
        SHINY_DEBUG("nfp: convex pair failed status={} fixed_outer={} "
                    "moving_outer={}",
                    util::status_name(pair_nfp.status()),
                    fixed_part.outer().size(), moving_part.outer().size());
        auto pair_orbiting =
            compute_orbiting_nfp(geom::PolygonWithHoles(fixed_part.outer()),
                                 geom::PolygonWithHoles(moving_part.outer()));
        log_orbiting_fallback("convex_pair_failure_local",
                              pair_orbiting.status());
        if (pair_orbiting.ok()) {
          auto &pair_polygons = pair_orbiting.value();
          aggregate.insert(aggregate.end(),
                           std::make_move_iterator(pair_polygons.begin()),
                           std::make_move_iterator(pair_polygons.end()));
          continue;
        }

        auto orbiting =
            compute_orbiting_nfp(normalized_fixed, normalized_moving);
        log_orbiting_fallback("convex_pair_failure_global", orbiting.status());
        if (orbiting.ok()) {
          return orbiting;
        }
        return pair_nfp.status();
      }
      aggregate.push_back(std::move(pair_nfp).value());
    }
  }

  if (aggregate.empty()) {
    auto orbiting = compute_orbiting_nfp(normalized_fixed, normalized_moving);
    log_orbiting_fallback("empty_aggregate", orbiting.status());
    if (orbiting.ok()) {
      return orbiting;
    }
    return util::Status::computation_failed;
  }

  return merge_polygon_set(std::move(aggregate));
}

} // namespace shiny::nesting::nfp
