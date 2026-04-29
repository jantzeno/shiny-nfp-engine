#include "packing/irregular/candidate_generation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "geometry/polygon.hpp"
#include "logging/shiny_log.hpp"
#include "nfp/ifp.hpp"
#include "packing/common.hpp"
#include "packing/irregular/blocked_regions.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "predicates/point_location.hpp"
#include "predicates/segment_intersection.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kCandidateEpsilon = 1e-8;
constexpr std::size_t kProtectedCandidatePrefixDivisor = 2U;
constexpr std::size_t kCandidateStrategyCount =
    static_cast<std::size_t>(CandidateStrategy::count);

struct CandidateAccumulator {
  std::vector<GeneratedCandidatePoint> points;

  struct KeyHash {
    auto operator()(const std::tuple<std::int64_t, std::int64_t, int> &key)
        const noexcept -> std::size_t {
      const auto h1 = static_cast<std::size_t>(std::get<0>(key));
      const auto h2 = static_cast<std::size_t>(std::get<1>(key));
      const auto h3 = static_cast<std::size_t>(std::get<2>(key));
      return runtime::hash::combine(runtime::hash::combine(h1, h2), h3);
    }
  };

  std::unordered_map<std::tuple<std::int64_t, std::int64_t, int>, std::size_t,
                     KeyHash>
      seen;

  auto try_emplace(const geom::Point2 &translation,
                   const place::PlacementCandidateSource source,
                   const cache::NfpCacheAccuracy nfp_accuracy) -> void {
    const auto qx = static_cast<std::int64_t>(
        std::llround(translation.x / kCandidateEpsilon));
    const auto qy = static_cast<std::int64_t>(
        std::llround(translation.y / kCandidateEpsilon));
    const auto key = std::make_tuple(qx, qy, static_cast<int>(source));
    if (const auto it = seen.find(key); it != seen.end()) {
      auto &existing = points[it->second];
      if (existing.nfp_accuracy != cache::NfpCacheAccuracy::exact &&
          nfp_accuracy == cache::NfpCacheAccuracy::exact) {
        existing.nfp_accuracy = nfp_accuracy;
      }
      return;
    }

    seen.emplace(key, points.size());
    points.push_back({
        .translation = translation,
        .source = source,
        .nfp_accuracy = nfp_accuracy,
    });
  }
};

struct WeightedTailCandidate {
  std::size_t index{0};
  double key{0.0};
};

[[nodiscard]] constexpr auto strategy_index(const CandidateStrategy strategy)
    -> std::size_t {
  return static_cast<std::size_t>(strategy);
}

[[nodiscard]] auto
candidate_point_priority(const geom::Point2 &translation,
                         const place::PlacementStartCorner start_corner)
    -> std::pair<double, double> {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
    return {translation.y, translation.x};
  case place::PlacementStartCorner::bottom_right:
    return {translation.y, -translation.x};
  case place::PlacementStartCorner::top_left:
    return {-translation.y, translation.x};
  case place::PlacementStartCorner::top_right:
    return {-translation.y, -translation.x};
  }
  return {translation.y, translation.x};
}

[[nodiscard]] auto half_normal_weight(const std::size_t rank_offset,
                                      const double sigma_points) -> double {
  const double normalized =
      static_cast<double>(rank_offset) /
      std::max(sigma_points, std::numeric_limits<double>::min());
  return std::exp(-0.5 * normalized * normalized);
}

[[nodiscard]] auto weighted_tail_sample_indices(
    const std::size_t protected_prefix, const std::size_t point_count,
    const std::size_t selected_count, const double sigma_points,
    runtime::DeterministicRng &rng) -> std::vector<std::size_t> {
  std::vector<WeightedTailCandidate> ranked_tail;
  ranked_tail.reserve(point_count - protected_prefix);
  for (std::size_t index = protected_prefix; index < point_count; ++index) {
    const double weight =
        std::max(half_normal_weight(index - protected_prefix, sigma_points),
                 std::numeric_limits<double>::min());
    const double probe =
        std::max(rng.uniform_real(), std::numeric_limits<double>::min());
    ranked_tail.push_back({
        .index = index,
        .key = -std::log(probe) / weight,
    });
  }

  const auto take_count = std::min(selected_count, ranked_tail.size());
  std::partial_sort(
      ranked_tail.begin(), ranked_tail.begin() + take_count, ranked_tail.end(),
      [](const WeightedTailCandidate &lhs, const WeightedTailCandidate &rhs) {
        if (!almost_equal(lhs.key, rhs.key)) {
          return lhs.key < rhs.key;
        }
        return lhs.index < rhs.index;
      });

  std::vector<std::size_t> selected;
  selected.reserve(take_count);
  for (std::size_t index = 0; index < take_count; ++index) {
    selected.push_back(ranked_tail[index].index);
  }
  std::sort(selected.begin(), selected.end());
  return selected;
}

auto append_unique_translation(
    CandidateAccumulator &accumulator, const geom::Point2 &translation,
    const place::PlacementCandidateSource source,
    const cache::NfpCacheAccuracy nfp_accuracy = cache::NfpCacheAccuracy::exact)
    -> void {
  accumulator.try_emplace(translation, source, nfp_accuracy);
}

template <typename Fn>
auto for_each_polygon_vertex(const geom::PolygonWithHoles &polygon, Fn &&fn)
    -> void {
  geom::for_each_ring(polygon, [&](std::span<const geom::Point2> ring) {
    for (const auto &vertex : ring) {
      fn(vertex);
    }
  });
}

auto append_polygon_vertices(CandidateAccumulator &accumulator,
                             const geom::PolygonWithHoles &polygon,
                             const place::PlacementCandidateSource source,
                             const cache::NfpCacheAccuracy nfp_accuracy)
    -> void {
  for_each_polygon_vertex(polygon, [&](const geom::Point2 &vertex) {
    append_unique_translation(accumulator, vertex, source, nfp_accuracy);
  });
}

[[nodiscard]] auto
build_base_domain(const geom::PolygonWithHoles &container,
                  std::span<const geom::PolygonWithHoles> exclusion_regions,
                  const geom::PolygonWithHoles &moving_piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  auto ifp = nfp::compute_ifp(container, moving_piece);
  if (ifp.ok()) {
    return ifp.value();
  }

  SHINY_DEBUG("candidate_generation: build_base_domain compute_ifp returned {}",
              util::status_name(ifp.status()));

  std::vector<geom::PolygonWithHoles> domain{container};
  for (const auto &region : exclusion_regions) {
    const auto subtract_status = poly::try_subtract_region_set(domain, region);
    if (subtract_status != util::Status::ok) {
      SHINY_DEBUG(
          "candidate_generation: exclusion subtraction failed status={} "
          "domain_outer={} obstacle_outer={}",
          util::status_name(subtract_status), container.outer.size(),
          region.outer.size());
      return subtract_status;
    }
  }
  return domain;
}

[[nodiscard]] auto ring_edges(std::span<const geom::Point2> ring)
    -> std::vector<geom::Segment2> {
  std::vector<geom::Segment2> edges;
  if (ring.size() < 2U) {
    return edges;
  }

  edges.reserve(ring.size());
  for (std::size_t index = 0; index < ring.size(); ++index) {
    edges.push_back({ring[index], ring[(index + 1U) % ring.size()]});
  }
  return edges;
}

auto append_arrangement_intersections(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked,
    const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  std::vector<geom::Segment2> domain_edges;
  for (const auto &polygon : domain) {
    const auto outer_edges = ring_edges(polygon.outer);
    domain_edges.insert(domain_edges.end(), outer_edges.begin(),
                        outer_edges.end());
    for (const auto &hole : polygon.holes) {
      const auto hole_edges = ring_edges(hole);
      domain_edges.insert(domain_edges.end(), hole_edges.begin(),
                          hole_edges.end());
    }
  }

  for (const auto &blocked_polygon : blocked) {
    std::vector<geom::Segment2> blocked_edges =
        ring_edges(blocked_polygon.outer);
    for (const auto &hole : blocked_polygon.holes) {
      const auto hole_edges = ring_edges(hole);
      blocked_edges.insert(blocked_edges.end(), hole_edges.begin(),
                           hole_edges.end());
    }

    for (const auto &domain_edge : domain_edges) {
      for (const auto &blocked_edge : blocked_edges) {
        const auto contact =
            pred::classify_segment_contact(domain_edge, blocked_edge);
        if (contact.kind == pred::SegmentContactKind::disjoint ||
            contact.kind == pred::SegmentContactKind::parallel_disjoint) {
          continue;
        }
        for (std::size_t index = 0; index < contact.point_count; ++index) {
          append_unique_translation(
              accumulator, contact.points[index],
              place::PlacementCandidateSource::perfect_slide, nfp_accuracy);
        }
      }
    }
  }
  return util::Status::ok;
}

auto append_points_inside_domain(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked,
    const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  for (const auto &blocked_polygon : blocked) {
    for (const auto &vertex : blocked_polygon.outer) {
      const auto inside_domain =
          std::any_of(domain.begin(), domain.end(), [&](const auto &region) {
            return pred::locate_point_in_polygon(vertex, region).location !=
                   pred::PointLocation::exterior;
          });
      if (inside_domain) {
        append_unique_translation(
            accumulator, vertex, place::PlacementCandidateSource::perfect_slide,
            nfp_accuracy);
      }
    }
  }
  return util::Status::ok;
}

[[nodiscard]] auto
point_is_feasible(const geom::Point2 &point,
                  const std::vector<geom::PolygonWithHoles> &domain,
                  const std::vector<geom::PolygonWithHoles> &blocked) -> bool {
  const bool in_domain =
      std::any_of(domain.begin(), domain.end(), [&](const auto &region) {
        return pred::locate_point_in_polygon(point, region).location !=
               pred::PointLocation::exterior;
      });
  if (!in_domain) {
    return false;
  }

  return std::none_of(blocked.begin(), blocked.end(), [&](const auto &polygon) {
    return pred::locate_point_in_polygon(point, polygon).location ==
           pred::PointLocation::interior;
  });
}

auto append_boundary_feasible_vertices(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked,
    const place::PlacementCandidateSource source,
    const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  for (const auto &region : domain) {
    for_each_polygon_vertex(region, [&](const geom::Point2 &vertex) {
      if (point_is_feasible(vertex, domain, blocked)) {
        append_unique_translation(accumulator, vertex, source, nfp_accuracy);
      }
    });
  }
  return util::Status::ok;
}

auto append_feasible_region_vertices(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked,
    const place::PlacementCandidateSource source,
    const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  if (nfp_accuracy == cache::NfpCacheAccuracy::conservative_bbox_fallback) {
    for (const auto &region : domain) {
      append_polygon_vertices(accumulator, region, source, nfp_accuracy);
    }
    const auto inside_status =
        append_points_inside_domain(accumulator, domain, blocked, nfp_accuracy);
    if (inside_status != util::Status::ok) {
      return inside_status;
    }
    return append_boundary_feasible_vertices(accumulator, domain, {}, source,
                                             nfp_accuracy);
  }

  std::vector<geom::PolygonWithHoles> feasible = domain;
  for (const auto &blocked_polygon : blocked) {
    const auto subtract_status =
        poly::try_subtract_region_set(feasible, blocked_polygon);
    if (subtract_status != util::Status::ok) {
      return subtract_status;
    }
  }
  for (const auto &region : feasible) {
    append_polygon_vertices(accumulator, region, source, nfp_accuracy);
  }
  return append_boundary_feasible_vertices(accumulator, domain, blocked, source,
                                           nfp_accuracy);
}

auto append_anchor_strategy(CandidateAccumulator &accumulator,
                            const std::vector<geom::PolygonWithHoles> &domain,
                            const std::vector<geom::PolygonWithHoles> &blocked)
    -> util::Status {
  return append_feasible_region_vertices(
      accumulator, domain, blocked,
      place::PlacementCandidateSource::constructive_boundary,
      cache::NfpCacheAccuracy::exact);
}

auto append_arrangement_strategy(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked,
    const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  if (nfp_accuracy == cache::NfpCacheAccuracy::conservative_bbox_fallback) {
    const auto perfect_status = append_feasible_region_vertices(
        accumulator, domain, blocked,
        place::PlacementCandidateSource::perfect_slide, nfp_accuracy);
    if (perfect_status != util::Status::ok) {
      return perfect_status;
    }
    return append_arrangement_intersections(accumulator, domain, blocked,
                                            nfp_accuracy);
  }

  for (const auto &region : domain) {
    append_polygon_vertices(accumulator, region,
                            place::PlacementCandidateSource::perfect_slide,
                            nfp_accuracy);
  }
  const auto inside_status =
      append_points_inside_domain(accumulator, domain, blocked, nfp_accuracy);
  if (inside_status != util::Status::ok) {
    return inside_status;
  }
  return append_arrangement_intersections(accumulator, domain, blocked,
                                          nfp_accuracy);
}

auto append_perfect_strategy(CandidateAccumulator &accumulator,
                             const std::vector<geom::PolygonWithHoles> &domain,
                             const std::vector<geom::PolygonWithHoles> &blocked,
                             const cache::NfpCacheAccuracy nfp_accuracy)
    -> util::Status {
  return append_feasible_region_vertices(
      accumulator, domain, blocked,
      place::PlacementCandidateSource::perfect_fit, nfp_accuracy);
}

auto append_hybrid_strategy(CandidateAccumulator &accumulator,
                            const std::vector<geom::PolygonWithHoles> &domain,
                            const std::vector<geom::PolygonWithHoles> &blocked,
                            const cache::NfpCacheAccuracy nfp_accuracy)
    -> util::Status {
  const auto arrangement_status =
      append_arrangement_strategy(accumulator, domain, blocked, nfp_accuracy);
  if (arrangement_status != util::Status::ok) {
    return arrangement_status;
  }
  return append_perfect_strategy(accumulator, domain, blocked, nfp_accuracy);
}

auto run_strategy(CandidateAccumulator &accumulator,
                  const CandidateStrategy strategy,
                  const std::vector<geom::PolygonWithHoles> &domain,
                  const std::vector<geom::PolygonWithHoles> &blocked,
                  const cache::NfpCacheAccuracy nfp_accuracy) -> util::Status {
  switch (strategy) {
  case CandidateStrategy::anchor_vertex:
    return append_anchor_strategy(accumulator, domain, blocked);
  case CandidateStrategy::nfp_perfect:
    return append_perfect_strategy(accumulator, domain, blocked, nfp_accuracy);
  case CandidateStrategy::nfp_arrangement:
    return append_arrangement_strategy(accumulator, domain, blocked,
                                       nfp_accuracy);
  case CandidateStrategy::nfp_hybrid:
    return append_hybrid_strategy(accumulator, domain, blocked, nfp_accuracy);
  case CandidateStrategy::count:
    break;
  }
  return util::Status::invalid_input;
}

auto populate_candidate_generation_diagnostics(
    const std::vector<GeneratedCandidatePoint> &points,
    CandidateGenerationDiagnostics &diagnostics) -> void {
  for (const auto &point : points) {
    switch (point.source) {
    case place::PlacementCandidateSource::perfect_fit:
      ++diagnostics.perfect_fit_candidates;
      break;
    case place::PlacementCandidateSource::perfect_slide:
      ++diagnostics.perfect_slide_candidates;
      break;
    case place::PlacementCandidateSource::constructive_boundary:
    case place::PlacementCandidateSource::bin_boundary:
    case place::PlacementCandidateSource::hole_boundary:
    case place::PlacementCandidateSource::concave_boundary:
      ++diagnostics.constructive_boundary_candidates;
      break;
    }

    if (point.nfp_accuracy ==
        cache::NfpCacheAccuracy::conservative_bbox_fallback) {
      ++diagnostics.conservative_bbox_fallback_candidates;
    }
  }
}

} // namespace

auto limit_candidate_points(std::vector<GeneratedCandidatePoint> &points,
                            const ExecutionPolicy &execution,
                            const place::PlacementStartCorner start_corner,
                            runtime::DeterministicRng *rng) -> void {
  std::sort(points.begin(), points.end(),
            [start_corner](const GeneratedCandidatePoint &lhs,
                           const GeneratedCandidatePoint &rhs) {
              const auto lhs_priority =
                  candidate_point_priority(lhs.translation, start_corner);
              const auto rhs_priority =
                  candidate_point_priority(rhs.translation, start_corner);
              if (!almost_equal(lhs_priority.first, rhs_priority.first)) {
                return lhs_priority.first < rhs_priority.first;
              }
              if (!almost_equal(lhs_priority.second, rhs_priority.second)) {
                return lhs_priority.second < rhs_priority.second;
              }
              if (lhs.nfp_accuracy != rhs.nfp_accuracy) {
                return static_cast<int>(lhs.nfp_accuracy) <
                       static_cast<int>(rhs.nfp_accuracy);
              }
              return static_cast<int>(lhs.source) <
                     static_cast<int>(rhs.source);
            });

  points.erase(
      std::unique(points.begin(), points.end(),
                  [](const GeneratedCandidatePoint &lhs,
                     const GeneratedCandidatePoint &rhs) {
                    return almost_equal(lhs.translation.x, rhs.translation.x) &&
                           almost_equal(lhs.translation.y, rhs.translation.y) &&
                           lhs.source == rhs.source;
                  }),
      points.end());

  const auto max_points = execution.irregular.max_candidate_points;
  if (points.size() <= max_points) {
    return;
  }

  if (rng == nullptr) {
    points.resize(max_points);
    return;
  }

  const std::size_t protected_prefix =
      std::min(max_points / kProtectedCandidatePrefixDivisor, points.size());
  const std::size_t remaining_count = max_points - protected_prefix;

  std::vector<GeneratedCandidatePoint> limited;
  limited.reserve(max_points);
  limited.insert(limited.end(), points.begin(),
                 points.begin() + protected_prefix);

  const double sigma_points = execution.irregular.candidate_gaussian_sigma *
                              static_cast<double>(max_points);
  for (const auto index :
       weighted_tail_sample_indices(protected_prefix, points.size(),
                                    remaining_count, sigma_points, *rng)) {
    limited.push_back(points[index]);
  }

  points = std::move(limited);
}

// NFP-driven candidate point generation for piece placement.
//
// Candidate strategies deliberately separate "cheap constructive anchors" from
// "obstacle-aware NFP geometry":
//
//   * anchor_vertex  — uses container/exclusion feasible-domain vertices only
//     and skips obstacle NFP generation. Placement search can still add legacy
//     anchors and skyline points on top of this domain-only seed set.
//
//   * nfp_perfect    — uses feasible-region vertices after subtracting exact
//     obstacle NFPs when available. If exact NFP fails, conservative bbox
//     fallback geometry is cached and labelled explicitly as fallback.
//
//   * nfp_arrangement — extends `nfp_perfect` with blocked-vertex and boundary
//     intersection candidates to capture single-edge slide placements.
//
//   * nfp_hybrid     — unions the robust constructive anchors used by the
//     packer with the exact/fallback-labelled NFP candidates above.
//
// Exact NFP cache entries and conservative bbox fallback entries use separate
// cache keys so a fallback value is never returned as if it were exact.
auto generate_nfp_candidate_points(
    const geom::PolygonWithHoles &container,
    const std::span<const geom::PolygonWithHoles> exclusion_regions,
    const std::span<const CandidateGenerationObstacle> obstacles,
    const geom::PolygonWithHoles &moving_piece,
    const std::uint64_t moving_piece_revision,
    const geom::ResolvedRotation moving_rotation,
    const CandidateStrategy strategy, cache::NfpCache *cache,
    CandidateGenerationDiagnostics *diagnostics)
    -> util::StatusOr<std::vector<GeneratedCandidatePoint>> {
  try {
    const auto strategy_id = strategy_index(strategy);
    if (strategy_id >= kCandidateStrategyCount) {
      return util::Status::invalid_input;
    }
    if (diagnostics != nullptr) {
      *diagnostics = {};
      diagnostics->strategy = strategy;
    }

    auto domain = build_base_domain(container, exclusion_regions, moving_piece);
    if (!domain.ok()) {
      return domain.status();
    }

    BlockedRegions blocked;
    if (strategy != CandidateStrategy::anchor_vertex) {
      auto blocked_or =
          build_blocked_regions(obstacles, moving_piece, moving_piece_revision,
                                moving_rotation, cache, diagnostics);
      if (!blocked_or.ok()) {
        return blocked_or.status();
      }
      blocked = std::move(blocked_or).value();
    }

    CandidateAccumulator accumulator;
    const auto strategy_status =
        run_strategy(accumulator, strategy, domain.value(), blocked.polygons,
                     blocked.accuracy);
    if (strategy_status != util::Status::ok) {
      SHINY_DEBUG("candidate_generation: strategy execution failed status={} "
                  "strategy={} domain={} blocked={}",
                  util::status_name(strategy_status),
                  static_cast<int>(strategy), domain.value().size(),
                  blocked.polygons.size());
      return strategy_status;
    }

    if (diagnostics != nullptr) {
      populate_candidate_generation_diagnostics(accumulator.points,
                                                *diagnostics);
    }

    return std::move(accumulator.points);
  } catch (const std::exception &ex) {
    SHINY_DEBUG("candidate_generation: generate_nfp_candidate_points threw "
                "strategy={} moving_rev={} obstacle_count={} error={}",
                static_cast<int>(strategy), moving_piece_revision,
                obstacles.size(), ex.what());
    return util::Status::computation_failed;
  } catch (...) {
    SHINY_DEBUG("candidate_generation: generate_nfp_candidate_points threw "
                "strategy={} moving_rev={} obstacle_count={} with unknown "
                "exception",
                static_cast<int>(strategy), moving_piece_revision,
                obstacles.size());
    return util::Status::computation_failed;
  }
}

} // namespace shiny::nesting::pack
