#include "decomposition/convex_decomposition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "decomposition/internal.hpp"
#include "decomposition/triangulation.hpp"
#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/validity.hpp"
#include "polygon_ops/boolean_ops.hpp"

// Convex decomposition via triangulate-then-adjacency-merge.
//
// Strategy: triangulate via CDT, retain the unconstrained face-adjacency graph,
// then repeatedly fuse only adjacent pieces whose union remains a single convex
// polygon with preserved area. This keeps the merge search on the triangulation
// graph rather than restarting an all-pairs scan after each successful union.
//
// Convexity check uses cross-product turn-direction consistency over the
// normalized ring; degenerate (near-collinear) turns within kCrossEpsilon
// are skipped so collinear vertices on a merged edge are not misread as a
// reflex turn.
//
// Determinism: pieces are sorted by bbox-origin then area both before and
// after merging so identical inputs yield identical outputs (important for
// reproducible NFP cache keys downstream).

namespace shiny::nesting::decomp {
namespace {

constexpr double kCrossEpsilon = 1e-9;
constexpr double kAreaEpsilon = 1e-8;

[[nodiscard]] auto cross(const geom::Point2 &origin, const geom::Point2 &lhs,
                         const geom::Point2 &rhs) -> double {
  const auto ax = lhs.x() - origin.x();
  const auto ay = lhs.y() - origin.y();
  const auto bx = rhs.x() - origin.x();
  const auto by = rhs.y() - origin.y();
  return ax * by - ay * bx;
}

[[nodiscard]] auto polygon_less(const geom::Polygon &lhs,
                                const geom::Polygon &rhs) -> bool {
  return detail::polygon_origin_less(lhs, rhs);
}

struct PieceState {
  geom::Polygon polygon{};
  std::vector<std::int32_t> neighbours{};
  bool active{true};
};

// Attempt to fuse two convex pieces into a single convex piece.
//
// Preconditions:
//   - `lhs` and `rhs` are valid simple polygons (outer ring only; holes
//     are not considered) with rings of size >= 3.
// Postconditions:
//   - On success returns a normalized convex polygon equal in area to
//     `lhs ∪ rhs` within `kAreaEpsilon`.
//   - On failure returns `std::nullopt`; inputs are unchanged.
//
// The union must (a) be a single connected region with no induced hole,
// (b) preserve total area within kAreaEpsilon (catches non-adjacent pairs
// whose union is two disjoint regions reported as one MultiPolygon by some
// boolean back-ends, and catches numerical artifacts), and (c) remain
// convex. Returns std::nullopt on any failure.
[[nodiscard]] auto try_merge_polygons(const geom::Polygon &lhs,
                                      const geom::Polygon &rhs)
    -> std::optional<geom::Polygon> {
  const auto merged = poly::union_polygons(geom::PolygonWithHoles{lhs.outer()},
                                           geom::PolygonWithHoles{rhs.outer()});
  if (merged.size() != 1U || !merged.front().holes().empty()) {
    return std::nullopt;
  }

  geom::Polygon candidate{merged.front().outer()};
  candidate = geom::normalize_polygon(candidate);
  const auto candidate_area = geom::polygon_area(candidate);
  const auto expected_area = geom::polygon_area(lhs) + geom::polygon_area(rhs);
  if (std::fabs(candidate_area - expected_area) > kAreaEpsilon) {
    return std::nullopt;
  }
  if (!geom::polygon_is_convex(candidate)) {
    return std::nullopt;
  }
  return candidate;
}

auto add_unique_neighbour(std::vector<std::int32_t> &neighbours,
                          const std::int32_t neighbour) -> void {
  if (neighbour < 0) {
    return;
  }

  if (std::find(neighbours.begin(), neighbours.end(), neighbour) ==
      neighbours.end()) {
    neighbours.push_back(neighbour);
  }
}

auto remove_neighbour(std::vector<std::int32_t> &neighbours,
                      const std::int32_t neighbour) -> void {
  neighbours.erase(std::remove(neighbours.begin(), neighbours.end(), neighbour),
                   neighbours.end());
}

// Append every active neighbour of `piece_index` (excluding self and any
// already-merged-away pieces) into `out`. Used both to materialise a
// fresh adjacency view in `active_neighbours()` and to fold lhs/rhs
// adjacency lists in `merge_pieces`.
auto adjacent_active_pieces(const std::vector<PieceState> &pieces,
                            const std::int32_t piece_index,
                            const std::vector<std::int32_t> &raw_neighbours,
                            const std::int32_t exclude_index,
                            std::vector<std::int32_t> &out) -> void {
  for (const auto neighbour : raw_neighbours) {
    if (neighbour < 0 || neighbour == piece_index ||
        neighbour == exclude_index) {
      continue;
    }
    if (!pieces[static_cast<std::size_t>(neighbour)].active) {
      continue;
    }
    add_unique_neighbour(out, neighbour);
  }
}

[[nodiscard]] auto active_neighbours(const std::vector<PieceState> &pieces,
                                     const std::int32_t piece_index)
    -> std::vector<std::int32_t> {
  std::vector<std::int32_t> result;
  if (piece_index < 0) {
    return result;
  }

  const auto &piece = pieces[static_cast<std::size_t>(piece_index)];
  result.reserve(piece.neighbours.size());
  adjacent_active_pieces(pieces, piece_index, piece.neighbours,
                         /*exclude_index=*/-1, result);

  std::sort(result.begin(), result.end());
  return result;
}

// Merge `rhs` into `lhs` and rewrite the adjacency graph.
//
// Preconditions:
//   - `lhs_index` and `rhs_index` reference distinct active pieces.
//   - `merged_polygon` is a valid convex polygon equal to `lhs ∪ rhs`
//     (typically the value returned by `try_merge_polygons`).
// Postconditions:
//   - `lhs` owns `merged_polygon` and its `neighbours` is the union of
//     the active neighbours of the two source pieces (excluding self
//     and `rhs`).
//   - `rhs` is marked `active = false`, neighbour list cleared, and
//     polygon reset; outer scans must skip inactive pieces.
//   - Every other piece that previously referenced `rhs` now references
//     `lhs` exactly once.
auto merge_pieces(std::vector<PieceState> &pieces, const std::int32_t lhs_index,
                  const std::int32_t rhs_index, geom::Polygon merged_polygon)
    -> void {
  auto &lhs = pieces[static_cast<std::size_t>(lhs_index)];
  auto &rhs = pieces[static_cast<std::size_t>(rhs_index)];

  lhs.polygon = std::move(merged_polygon);

  std::vector<std::int32_t> merged_neighbours;
  merged_neighbours.reserve(lhs.neighbours.size() + rhs.neighbours.size());
  adjacent_active_pieces(pieces, lhs_index, lhs.neighbours, rhs_index,
                         merged_neighbours);
  adjacent_active_pieces(pieces, lhs_index, rhs.neighbours, rhs_index,
                         merged_neighbours);

  for (const auto neighbour : merged_neighbours) {
    auto &adjacent = pieces[static_cast<std::size_t>(neighbour)].neighbours;
    remove_neighbour(adjacent, rhs_index);
    add_unique_neighbour(adjacent, lhs_index);
  }

  lhs.neighbours = std::move(merged_neighbours);
  rhs.active = false;
  rhs.neighbours.clear();
  rhs.polygon = {};
}

} // namespace

auto is_convex(const geom::Polygon &polygon) -> bool {
  const auto normalized = geom::normalize_polygon(polygon);
  const auto &ring = normalized.outer();
  if (ring.size() < 4U) {
    return true;
  }

  int turn_sign = 0;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto &previous = ring[(index + ring.size() - 1U) % ring.size()];
    const auto &current = ring[index];
    const auto &next = ring[(index + 1U) % ring.size()];
    const auto turn = cross(previous, current, next);
    if (std::fabs(turn) <= kCrossEpsilon) {
      continue;
    }

    const int current_sign = turn > 0.0 ? 1 : -1;
    if (turn_sign == 0) {
      turn_sign = current_sign;
      continue;
    }
    if (turn_sign != current_sign) {
      return false;
    }
  }

  return true;
}

auto decompose_convex(const geom::Polygon &polygon)
    -> util::StatusOr<std::vector<geom::Polygon>> {
  return decompose_convex(geom::PolygonWithHoles{polygon.outer()});
}

auto decompose_convex(const geom::PolygonWithHoles &polygon)
    -> util::StatusOr<std::vector<geom::Polygon>> {
  const auto normalized = geom::normalize_polygon(polygon);
  if (!geom::validate_polygon(normalized).is_valid()) {
    return util::Status::invalid_input;
  }

  if (normalized.holes().empty() &&
      is_convex(geom::Polygon{normalized.outer()})) {
    return std::vector<geom::Polygon>{geom::Polygon{normalized.outer()}};
  }

  auto triangulation_or = triangulate_polygon(normalized);
  if (!triangulation_or.ok()) {
    return triangulation_or.status();
  }

  auto triangulation = std::move(triangulation_or).value();
  std::vector<PieceState> piece_states;
  piece_states.reserve(triangulation.triangles.size());
  for (std::size_t index = 0; index < triangulation.triangles.size(); ++index) {
    PieceState piece{};
    piece.polygon = std::move(triangulation.triangles[index]);
    for (const auto neighbour : triangulation.neighbours[index]) {
      add_unique_neighbour(piece.neighbours, neighbour);
    }
    piece_states.push_back(std::move(piece));
  }

  bool merged_any = true;
  while (merged_any) {
    merged_any = false;

    // After a successful merge, the absorbed `rhs` is left active=false
    // with an empty neighbour list (`merge_pieces`). We do not delete it
    // in place — instead the outer pass simply skips inactive entries
    // on the next iteration, which preserves indices for adjacency
    // bookkeeping during this sweep.
    for (std::int32_t lhs_index = 0;
         lhs_index < static_cast<std::int32_t>(piece_states.size());
         ++lhs_index) {
      auto &lhs = piece_states[static_cast<std::size_t>(lhs_index)];
      if (!lhs.active) {
        continue;
      }

      std::size_t neighbour_offset = 0;
      while (true) {
        const auto neighbours = active_neighbours(piece_states, lhs_index);
        if (neighbour_offset >= neighbours.size()) {
          break;
        }

        const auto rhs_index = neighbours[neighbour_offset];
        ++neighbour_offset;
        if (rhs_index <= lhs_index) {
          continue;
        }

        auto merged = try_merge_polygons(
            lhs.polygon,
            piece_states[static_cast<std::size_t>(rhs_index)].polygon);
        if (!merged.has_value()) {
          continue;
        }

        merge_pieces(piece_states, lhs_index, rhs_index, std::move(*merged));
        merged_any = true;
        neighbour_offset = 0;
      }
    }
  }

  std::vector<geom::Polygon> pieces;
  pieces.reserve(piece_states.size());
  for (auto &piece : piece_states) {
    if (piece.active) {
      pieces.push_back(std::move(piece.polygon));
    }
  }

  std::sort(pieces.begin(), pieces.end(), polygon_less);
  return pieces;
}

} // namespace shiny::nesting::decomp
