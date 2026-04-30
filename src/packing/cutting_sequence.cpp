#include "packing/cutting_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_map>

#include "geometry/polygon.hpp"
#include "geometry/rtree_index.hpp"
#include "logging/shiny_log.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack {
namespace {

[[nodiscard]] auto box_center(const geom::Box2 &bounds) -> geom::Point2 {
  return geom::Point2((bounds.min.x() + bounds.max.x()) / 2.0,
                      (bounds.min.y() + bounds.max.y()) / 2.0);
}

// 2-opt local search operating on indices into a precomputed centers
// vector. We work on indices instead of CutContour values so the caller
// can map the final order back to its source-node array in O(1) without
// vector-equality scans (cf. finding #72).
auto two_opt_indices(std::span<const geom::Point2> centers,
                     std::vector<std::size_t> &order) -> void {
  if (order.size() < 4U) {
    return;
  }

  const auto dist = [&](const std::size_t a, const std::size_t b) {
    return geom::point_distance(centers[a], centers[b]);
  };

  bool improved = true;
  while (improved) {
    improved = false;
    for (std::size_t i = 1; i + 2 < order.size(); ++i) {
      for (std::size_t k = i + 1; k + 1 < order.size(); ++k) {
        const auto before =
            dist(order[i - 1], order[i]) + dist(order[k], order[k + 1]);
        const auto after =
            dist(order[i - 1], order[k]) + dist(order[i], order[k + 1]);
        if (after + 1e-9 < before) {
          std::reverse(order.begin() + static_cast<std::ptrdiff_t>(i),
                       order.begin() + static_cast<std::ptrdiff_t>(k + 1U));
          improved = true;
        }
      }
    }
  }
}

[[nodiscard]] auto
nearest_neighbor_indices(std::span<const geom::Point2> centers,
                         std::vector<std::size_t> pool)
    -> std::vector<std::size_t> {
  if (pool.size() < 2U) {
    return pool;
  }

  std::vector<std::size_t> ordered;
  ordered.reserve(pool.size());
  ordered.push_back(pool.front());
  pool.erase(pool.begin());

  while (!pool.empty()) {
    const auto best = std::min_element(
        pool.begin(), pool.end(),
        [&](const std::size_t lhs, const std::size_t rhs) {
          return geom::point_distance(centers[ordered.back()], centers[lhs]) <
                 geom::point_distance(centers[ordered.back()], centers[rhs]);
        });
    ordered.push_back(*best);
    pool.erase(best);
  }
  two_opt_indices(centers, ordered);
  return ordered;
}

struct ContourNode {
  CutContour contour{};
  geom::PolygonWithHoles owner_polygon{};
  geom::Box2 bounds{};
};

[[nodiscard]] auto is_contained_in_outer_ring(const CutContour &candidate,
                                              const ContourNode &container)
    -> bool {
  if (candidate.bin_id != container.contour.bin_id ||
      candidate.piece_id == container.contour.piece_id) {
    return false;
  }

  const geom::PolygonWithHoles outer_only(container.owner_polygon.outer());
  bool saw_strict_interior = false;
  for (const auto &point : candidate.ring) {
    const auto location =
        pred::locate_point_in_polygon(point, outer_only).location;
    if (location == pred::PointLocation::exterior) {
      return false;
    }
    saw_strict_interior |= (location == pred::PointLocation::interior);
  }
  return saw_strict_interior;
}

// Returns the scheduling order of the ready set as indices into the
// caller's `centers`/`pool` arrays. The first emitted index is the
// node closest to `previous_center` (greedy NN seed); the rest follow
// from a 2-opt-refined nearest-neighbour walk. Operating on indices
// avoids the O(K · N · V) ring-equality scan that the old
// CutContour-valued variant required to map results back to nodes.
[[nodiscard]] auto
order_ready_indices(std::span<const geom::Point2> centers,
                    std::vector<std::size_t> ready,
                    const std::optional<geom::Point2> previous_center)
    -> std::vector<std::size_t> {
  if (ready.size() < 2U) {
    return ready;
  }

  if (previous_center.has_value()) {
    const auto best = std::min_element(
        ready.begin(), ready.end(),
        [&](const std::size_t lhs, const std::size_t rhs) {
          return geom::point_distance(centers[lhs], *previous_center) <
                 geom::point_distance(centers[rhs], *previous_center);
        });
    std::iter_swap(ready.begin(), best);
  }

  return nearest_neighbor_indices(centers, std::move(ready));
}

} // namespace

// Builds the global cut order across every bin of the final layout
// (Plan §12.1).
//
// Per-bin algorithm:
//   1. Enumerate contours: every hole ring (cut first) and every outer
//      ring (cut last) of every placed piece. Each contour is wrapped
//      in a ContourNode that remembers its owner polygon for
//      containment tests.
//   2. Build a precedence DAG with two kinds of edges:
//        - hole→outer of the same piece (you must cut interior holes
//          before separating the piece from the sheet).
//        - inner-piece-outer→outer-piece-outer when one piece sits
//          inside another piece''s outer ring (interior pieces must be
//          freed before their host).
//   3. Topologically schedule the DAG. At each iteration:
//        - Collect all nodes with indegree zero (the "ready set").
//        - Order the ready set by a TSP-like heuristic
//          (`order_ready_contours`): nearest-neighbour seeded from
//          the previously-emitted contour''s centre, refined by
//          a local 2-opt swap pass.
//        - Emit the ordered ready set, decrement dependents'' indegrees,
//          continue.
// COMPLEXITY: DAG construction builds one R-tree over outer-ring
// bounding boxes, then only runs exact point-in-polygon containment on
// bbox-containing candidates. Worst-case remains O(N²) when every
// outer-ring bbox overlaps, but dense bins no longer pay the full
// all-pairs point-in-polygon cost. The topological pass is O(N²) due
// to the ready-set ordering, and 2-opt is O(K²) per ready set.
auto build_cutting_sequence(const Layout &layout) -> std::vector<CutContour> {
  std::vector<CutContour> ordered;

  for (const auto &bin : layout.bins) {
    std::vector<ContourNode> nodes;
    for (const auto &piece : bin.placements) {
      for (const auto &hole : piece.polygon.holes()) {
        nodes.push_back({
            .contour =
                {
                    .bin_id = bin.bin_id,
                    .piece_id = piece.placement.piece_id,
                    .from_hole = true,
                    .ring = hole,
                },
            .owner_polygon = piece.polygon,
            .bounds = geom::compute_bounds(
                std::span<const geom::Point2>(hole.data(), hole.size())),
        });
      }
      nodes.push_back({
          .contour =
              {
                  .bin_id = bin.bin_id,
                  .piece_id = piece.placement.piece_id,
                  .from_hole = false,
                  .ring = piece.polygon.outer(),
              },
          .owner_polygon = piece.polygon,
          .bounds = geom::compute_bounds(std::span<const geom::Point2>(
              piece.polygon.outer().data(), piece.polygon.outer().size())),
      });
    }

    std::vector<std::vector<std::size_t>> edges(nodes.size());
    std::vector<std::size_t> indegree(nodes.size(), 0U);

    std::unordered_map<std::uint32_t, std::size_t> outer_node_by_piece;
    geom::RTreeIndex outer_index;
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if (nodes[index].contour.from_hole) {
        continue;
      }
      outer_node_by_piece.emplace(nodes[index].contour.piece_id, index);
      outer_index.insert(static_cast<std::uint32_t>(index),
                         nodes[index].bounds);
    }

    const auto add_dependency = [&](const std::size_t from,
                                    const std::size_t to) {
      edges[from].push_back(to);
      ++indegree[to];
    };

    for (std::size_t lhs = 0; lhs < nodes.size(); ++lhs) {
      if (nodes[lhs].contour.from_hole) {
        if (const auto outer_it =
                outer_node_by_piece.find(nodes[lhs].contour.piece_id);
            outer_it != outer_node_by_piece.end()) {
          add_dependency(lhs, outer_it->second);
        }
        continue;
      }

      for (const auto candidate_id : outer_index.query(nodes[lhs].bounds)) {
        const auto rhs = static_cast<std::size_t>(candidate_id);
        if (rhs == lhs ||
            !geom::box_contains(nodes[rhs].bounds, nodes[lhs].bounds)) {
          continue;
        }

        if (is_contained_in_outer_ring(nodes[lhs].contour, nodes[rhs])) {
          add_dependency(lhs, rhs);
        }
      }
    }

    // Pre-compute the centroid of every contour once. The scheduling
    // loop indexes into this vector instead of re-deriving centres
    // from rings (avoids quadratic recomputation; required by the
    // index-based 2-opt path).
    std::vector<geom::Point2> centers;
    centers.reserve(nodes.size());
    for (const auto &node : nodes) {
      centers.push_back(box_center(node.bounds));
    }

    // Bootstrap the ready set with all currently-zero-indegree nodes.
    // From here on we maintain the set incrementally: when a
    // dependent's indegree hits zero we push it onto `ready_pool`
    // (finding #71 — replaces the per-iteration linear rescan).
    std::vector<bool> scheduled(nodes.size(), false);
    std::vector<std::size_t> ready_pool;
    ready_pool.reserve(nodes.size());
    for (std::size_t index = 0; index < nodes.size(); ++index) {
      if (indegree[index] == 0U) {
        ready_pool.push_back(index);
      }
    }

    std::optional<geom::Point2> previous_center;
    std::size_t scheduled_count = 0;
    while (scheduled_count < nodes.size()) {
      if (ready_pool.empty()) {
        // Cycle in the dependency graph (or ill-formed input). Rather
        // than silently dropping the remaining contours — which would
        // emit a tool path that skips legitimate cuts — log a warning
        // and emit the unscheduled remainder in a deterministic
        // order (lowest current indegree first, then by piece_id /
        // from_hole / ring index). This guarantees every contour is
        // cut while making the cycle visible to the operator.
        SHINY_WARN("cutting_sequence: dependency cycle detected; emitting "
                   "{} unscheduled contour(s) in fallback order",
                   nodes.size() - scheduled_count);
        std::vector<std::size_t> remaining;
        remaining.reserve(nodes.size() - scheduled_count);
        for (std::size_t index = 0; index < nodes.size(); ++index) {
          if (!scheduled[index]) {
            remaining.push_back(index);
          }
        }
        std::sort(remaining.begin(), remaining.end(),
                  [&](const std::size_t lhs, const std::size_t rhs) {
                    if (indegree[lhs] != indegree[rhs]) {
                      return indegree[lhs] < indegree[rhs];
                    }
                    const auto &lc = nodes[lhs].contour;
                    const auto &rc = nodes[rhs].contour;
                    if (lc.piece_id != rc.piece_id) {
                      return lc.piece_id < rc.piece_id;
                    }
                    if (lc.from_hole != rc.from_hole) {
                      return static_cast<int>(lc.from_hole) <
                             static_cast<int>(rc.from_hole);
                    }
                    return lhs < rhs;
                  });
        for (const auto index : remaining) {
          scheduled[index] = true;
          ++scheduled_count;
          ordered.push_back(nodes[index].contour);
          previous_center = centers[index];
        }
        break;
      }

      // Order the current ready set with NN+2-opt seeded from the
      // previously-emitted contour, then drain it before recomputing.
      // The drain unblocks dependents into the next ready_pool window.
      std::vector<std::size_t> ready_window;
      ready_window.swap(ready_pool);
      const auto schedule_order = order_ready_indices(
          centers, std::move(ready_window), previous_center);
      for (const auto node_index : schedule_order) {
        scheduled[node_index] = true;
        ++scheduled_count;
        ordered.push_back(nodes[node_index].contour);
        previous_center = centers[node_index];
        for (const auto dependent : edges[node_index]) {
          if (indegree[dependent] > 0U) {
            --indegree[dependent];
            if (indegree[dependent] == 0U) {
              ready_pool.push_back(dependent);
            }
          }
        }
      }
    }
  }
  return ordered;
}

} // namespace shiny::nesting::pack
