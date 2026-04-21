#include "packing/candidate_generation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <unordered_set>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "nfp/ifp.hpp"
#include "nfp/nfp.hpp"
#include "packing/common.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "predicates/point_location.hpp"
#include "predicates/segment_intersection.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kCandidateEpsilon = 1e-8;
constexpr std::size_t kProtectedCandidatePrefixDivisor = 3U;
constexpr std::size_t kCandidateStrategyCount =
    static_cast<std::size_t>(CandidateStrategy::count);

struct CandidateAccumulator;

using CandidateStrategyExecutor = void (*)(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked);

// O(1) duplicate-point detection for candidate accumulation.
//
// Replaces the previous O(N²) std::find_if scan over the points vector.
// Coordinates are quantized to an integer lattice with cell width
// kCandidateEpsilon; two candidates that map to the same (qx, qy, source)
// triple are treated as duplicates. A point that lies within
// epsilon/2 of a lattice boundary may fall into an adjacent cell (and
// therefore not be deduped against a near-equal neighbour), but for NFP
// outputs — which are deterministic — this is accurate enough and
// preserves the historical "exact-match modulo float noise" semantics.
struct CandidateAccumulator {
  std::vector<GeneratedCandidatePoint> points;
  struct KeyHash {
    auto operator()(const std::tuple<std::int64_t, std::int64_t, int> &key) const
        noexcept -> std::size_t {
      const auto h1 = static_cast<std::size_t>(std::get<0>(key));
      const auto h2 = static_cast<std::size_t>(std::get<1>(key));
      const auto h3 = static_cast<std::size_t>(std::get<2>(key));
      return runtime::hash::combine(runtime::hash::combine(h1, h2), h3);
    }
  };
  std::unordered_set<std::tuple<std::int64_t, std::int64_t, int>, KeyHash> seen;

  auto try_emplace(const geom::Point2 &translation,
                   const place::PlacementCandidateSource source) -> void {
    const auto qx = static_cast<std::int64_t>(std::llround(translation.x / kCandidateEpsilon));
    const auto qy = static_cast<std::int64_t>(std::llround(translation.y / kCandidateEpsilon));
    if (seen.emplace(qx, qy, static_cast<int>(source)).second) {
      points.push_back({.translation = translation, .source = source});
    }
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

[[nodiscard]] auto candidate_point_priority(
    const geom::Point2 &translation,
    const place::PlacementStartCorner start_corner) -> std::pair<double, double> {
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
    const double weight = std::max(
        half_normal_weight(index - protected_prefix, sigma_points),
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

auto append_unique_translation(CandidateAccumulator &accumulator,
                               const geom::Point2 &translation,
                               const place::PlacementCandidateSource source) -> void {
  accumulator.try_emplace(translation, source);
}

[[maybe_unused]] auto append_ring_vertices(CandidateAccumulator &accumulator,
                          std::span<const geom::Point2> ring,
                          const place::PlacementCandidateSource source) -> void {
  for (const auto &point : ring) {
    append_unique_translation(accumulator, point, source);
  }
}

// Iterate every vertex (outer ring + each hole ring) of `polygon`, in
// the order produced by `geom::for_each_ring`. Centralises the
// "for each ring -> for each vertex" pattern shared by polygon-level
// candidate accumulators.
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
                             const place::PlacementCandidateSource source) -> void {
  for_each_polygon_vertex(polygon, [&](const geom::Point2 &vertex) {
    append_unique_translation(accumulator, vertex, source);
  });
}

// Build the feasible domain (IFP) for the moving piece.
//   - Fast path: `compute_ifp` succeeds when container is an axis-aligned
//     rectangle and returns the closed-form Inner-Fit Rectangle.
//   - General path: `compute_ifp` returns one or more feasible regions for
//     non-rectangular containers by subtracting obstacle NFPs from the
//     container-bbox translation domain.
//   - Final subtraction: remove the already-blocked regions contributed by
//     placed obstacles.
[[nodiscard]] auto build_base_domain(
    const geom::PolygonWithHoles &container,
    std::span<const geom::PolygonWithHoles> exclusion_regions,
    const geom::PolygonWithHoles &moving_piece)
    -> std::vector<geom::PolygonWithHoles> {
  auto ifp = nfp::compute_ifp(container, moving_piece);
  if (ifp.ok()) {
    return ifp.value();
  }

  std::vector<geom::PolygonWithHoles> domain{container};
  for (const auto &region : exclusion_regions) {
    poly::subtract_region_set(domain, region);
  }
  return domain;
}

[[nodiscard]] auto obstacle_base_polygon(const CandidateGenerationObstacle &obstacle)
    -> geom::PolygonWithHoles {
  return geom::translate(
      obstacle.polygon,
      {.x = -obstacle.translation.x, .y = -obstacle.translation.y});
}

// For each placed piece (obstacle), retrieve or compute the NFP relative
// to the moving piece, then translate it into world coordinates.
//
// IMPORTANT: NFP geometry is cached *un-translated* (in obstacle-local
// frame) so the cache key only depends on geometry revisions and
// rotations, not on world position. Multiple instances of the same piece
// at different positions all share one cache entry.
[[nodiscard]] auto build_blocked_regions(
    std::span<const CandidateGenerationObstacle> obstacles,
    const geom::PolygonWithHoles &moving_piece,
    const std::uint64_t moving_piece_revision,
    const geom::ResolvedRotation moving_rotation, cache::NfpCache *cache_ptr)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  std::vector<geom::PolygonWithHoles> blocked;

  for (const auto &obstacle : obstacles) {
    const auto fixed_polygon = obstacle_base_polygon(obstacle);
    const auto fixed_revision =
        obstacle.geometry_revision != 0U
            ? obstacle.geometry_revision
            : geom::polygon_revision(fixed_polygon);
    const auto key = cache::make_nfp_cache_key(
        fixed_revision, moving_piece_revision, obstacle.rotation.degrees,
        moving_rotation.degrees);

    std::shared_ptr<const cache::NfpCacheValue> cached;
    if (cache_ptr != nullptr) {
      cached = cache_ptr->get(key);
    }

    const cache::NfpCacheValue *base_nfp_ptr = cached.get();
    cache::NfpCacheValue computed_storage;
    if (base_nfp_ptr == nullptr) {
      auto computed = nfp::compute_nfp(fixed_polygon, moving_piece);
      if (!computed.ok()) {
        return computed.status();
      }
      computed_storage = std::move(computed).value();
      if (cache_ptr != nullptr) {
        cache_ptr->put(key, computed_storage);
      }
      base_nfp_ptr = &computed_storage;
    }

    for (const auto &polygon : *base_nfp_ptr) {
      blocked.push_back(geom::translate(
          polygon, {.x = obstacle.translation.x, .y = obstacle.translation.y}));
    }
  }

  return blocked;
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
    const std::vector<geom::PolygonWithHoles> &blocked) -> void {
  std::vector<geom::Segment2> domain_edges;
  for (const auto &polygon : domain) {
    const auto outer_edges = ring_edges(polygon.outer);
    domain_edges.insert(domain_edges.end(), outer_edges.begin(), outer_edges.end());
    for (const auto &hole : polygon.holes) {
      const auto hole_edges = ring_edges(hole);
      domain_edges.insert(domain_edges.end(), hole_edges.begin(), hole_edges.end());
    }
  }

  for (const auto &blocked_polygon : blocked) {
    std::vector<geom::Segment2> blocked_edges = ring_edges(blocked_polygon.outer);
    for (const auto &hole : blocked_polygon.holes) {
      const auto hole_edges = ring_edges(hole);
      blocked_edges.insert(blocked_edges.end(), hole_edges.begin(), hole_edges.end());
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
          append_unique_translation(accumulator, contact.points[index],
                                    place::PlacementCandidateSource::perfect_slide);
        }
      }
    }
  }
}

auto append_points_inside_domain(CandidateAccumulator &accumulator,
                                 const std::vector<geom::PolygonWithHoles> &domain,
                                 const std::vector<geom::PolygonWithHoles> &blocked)
    -> void {
  for (const auto &blocked_polygon : blocked) {
    for (const auto &vertex : blocked_polygon.outer) {
      const auto inside_domain = std::any_of(
          domain.begin(), domain.end(), [&](const auto &region) {
            return pred::locate_point_in_polygon(vertex, region).location !=
                   pred::PointLocation::exterior;
          });
      if (inside_domain) {
        append_unique_translation(accumulator, vertex,
                                  place::PlacementCandidateSource::perfect_slide);
      }
    }
  }
}

[[nodiscard]] auto point_is_feasible(
    const geom::Point2 &point, const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked) -> bool {
  const bool in_domain = std::any_of(domain.begin(), domain.end(), [&](const auto &region) {
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
    const std::vector<geom::PolygonWithHoles> &blocked) -> void {
  for (const auto &region : domain) {
    for_each_polygon_vertex(region, [&](const geom::Point2 &vertex) {
      if (point_is_feasible(vertex, domain, blocked)) {
        append_unique_translation(accumulator, vertex,
                                  place::PlacementCandidateSource::perfect_fit);
      }
    });
  }
}

auto append_anchor_strategy(CandidateAccumulator &,
                            const std::vector<geom::PolygonWithHoles> &,
                            const std::vector<geom::PolygonWithHoles> &) -> void {}

auto append_arrangement_strategy(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked) -> void {
  for (const auto &region : domain) {
    append_polygon_vertices(accumulator, region,
                            place::PlacementCandidateSource::perfect_slide);
  }
  append_points_inside_domain(accumulator, domain, blocked);
  append_arrangement_intersections(accumulator, domain, blocked);
}

auto append_perfect_strategy(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked) -> void {
  std::vector<geom::PolygonWithHoles> feasible = domain;
  for (const auto &blocked_polygon : blocked) {
    poly::subtract_region_set(feasible, blocked_polygon);
  }
  for (const auto &region : feasible) {
    append_polygon_vertices(accumulator, region,
                            place::PlacementCandidateSource::perfect_fit);
  }
  append_boundary_feasible_vertices(accumulator, domain, blocked);
}

auto append_hybrid_strategy(
    CandidateAccumulator &accumulator,
    const std::vector<geom::PolygonWithHoles> &domain,
    const std::vector<geom::PolygonWithHoles> &blocked) -> void {
  append_arrangement_strategy(accumulator, domain, blocked);
  append_perfect_strategy(accumulator, domain, blocked);
}

const std::array<CandidateStrategyExecutor, kCandidateStrategyCount>
    kCandidateStrategyExecutors{
        &append_anchor_strategy,
        &append_perfect_strategy,
        &append_arrangement_strategy,
        &append_hybrid_strategy,
    };

static_assert(kCandidateStrategyCount == 4U);
static_assert(strategy_index(CandidateStrategy::anchor_vertex) == 0U);
static_assert(strategy_index(CandidateStrategy::nfp_perfect) == 1U);
static_assert(strategy_index(CandidateStrategy::nfp_arrangement) == 2U);
static_assert(strategy_index(CandidateStrategy::nfp_hybrid) == 3U);

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
              return static_cast<int>(lhs.source) < static_cast<int>(rhs.source);
            });

  points.erase(std::unique(points.begin(), points.end(),
                           [](const GeneratedCandidatePoint &lhs,
                              const GeneratedCandidatePoint &rhs) {
                             return almost_equal(lhs.translation.x,
                                                 rhs.translation.x) &&
                                    almost_equal(lhs.translation.y,
                                                 rhs.translation.y) &&
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
  limited.insert(limited.end(), points.begin(), points.begin() + protected_prefix);

  const double sigma_points =
      execution.irregular.candidate_gaussian_sigma *
      static_cast<double>(max_points);
  for (const auto index : weighted_tail_sample_indices(
           protected_prefix, points.size(), remaining_count, sigma_points, *rng)) {
    limited.push_back(points[index]);
  }

  points = std::move(limited);
}

// NFP-driven candidate point generation for piece placement.
//
// Two strategies (and their hybrid) implement the candidate-point
// approaches from the literature:
//
//   * nfp_perfect    — "perfect-fit" candidates: vertices of the feasible
//     region (IFP minus union of NFPs of placed pieces). Each such vertex
//     is a position where the moving piece touches AT LEAST TWO existing
//     constraints (container boundary and/or other pieces) — i.e. a
//     locally-tight placement. Reference: 2DNesting `perfectCandidate
//     Points()`.
//
//   * nfp_arrangement — "arrangement" candidates: every vertex of an NFP
//     plus every intersection point between an NFP boundary and the IFP
//     boundary. Larger candidate set, includes single-edge slides as
//     well as perfect fits. Reference: 2DNesting `arrangement
//     CandidatePoints()`.
//
//   * nfp_hybrid     — union of the above (with vertex-position-only
//     dedup; same coordinate from different sources is intentionally
//     kept twice so downstream evaluators can prefer perfect-fit).
//
//   * anchor_vertex  — sentinel for "use the legacy anchor heuristic"
//     (no NFP candidates). Returns an empty vector; the constructive
//     packer then falls back to its own anchor enumeration.
// The constructive packer later orders these candidates and caps them via
// `limit_candidate_points`, which preserves a protected high-priority prefix
// and applies a half-normal weighted tail sample when truncation is needed.
auto generate_nfp_candidate_points(
    const geom::PolygonWithHoles &container,
    const std::span<const geom::PolygonWithHoles> exclusion_regions,
    const std::span<const CandidateGenerationObstacle> obstacles,
    const geom::PolygonWithHoles &moving_piece,
    const std::uint64_t moving_piece_revision,
    const geom::ResolvedRotation moving_rotation, const CandidateStrategy strategy,
    cache::NfpCache *cache) -> util::StatusOr<std::vector<GeneratedCandidatePoint>> {
  const auto strategy_id = strategy_index(strategy);
  if (strategy_id >= kCandidateStrategyExecutors.size()) {
    return util::Status::invalid_input;
  }

  const auto domain = build_base_domain(container, exclusion_regions, moving_piece);
  auto blocked_or = build_blocked_regions(obstacles, moving_piece, moving_piece_revision,
                                          moving_rotation, cache);
  if (!blocked_or.ok()) {
    return blocked_or.status();
  }
  const auto &blocked = blocked_or.value();

  CandidateAccumulator accumulator;
  kCandidateStrategyExecutors[strategy_id](accumulator, domain, blocked);

  return std::move(accumulator.points);
}

} // namespace shiny::nesting::pack
