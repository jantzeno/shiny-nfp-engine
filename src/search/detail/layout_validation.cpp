#include "search/detail/layout_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <unordered_set>

#include "packing/common.hpp"
#include "polygon_ops/boolean_ops.hpp"

namespace shiny::nesting::search::detail {
namespace {

constexpr double kConstructiveOverlapAreaEpsilon = 1e-6;
constexpr double kConstructiveContainmentAreaEpsilon = 1e-6;
constexpr double kRotationAngleEpsilon = 1e-9;

[[nodiscard]] auto exact_overlap_area(const geom::PolygonWithHoles &lhs,
                                      const geom::PolygonWithHoles &rhs)
    -> double {
  return geom::polygon_area_sum(poly::intersection_polygons(lhs, rhs));
}

[[nodiscard]] auto exact_area_outside(const geom::PolygonWithHoles &container,
                                      const geom::PolygonWithHoles &piece)
    -> double {
  const auto piece_area = geom::polygon_area(piece);
  const auto contained_area =
      geom::polygon_area_sum(poly::intersection_polygons(container, piece));
  return piece_area - contained_area;
}

[[nodiscard]] auto
source_piece_for_expanded_piece(const NormalizedRequest &request,
                                const std::uint32_t expanded_piece_id)
    -> const PieceRequest * {
  const auto expanded_it = std::find_if(
      request.expanded_pieces.begin(), request.expanded_pieces.end(),
      [expanded_piece_id](const ExpandedPieceInstance &expanded) {
        return expanded.expanded_piece_id == expanded_piece_id;
      });
  if (expanded_it == request.expanded_pieces.end()) {
    return nullptr;
  }

  const auto piece_it =
      std::find_if(request.request.pieces.begin(), request.request.pieces.end(),
                   [source_piece_id = expanded_it->source_piece_id](
                       const PieceRequest &piece) {
                     return piece.piece_id == source_piece_id;
                   });
  return piece_it == request.request.pieces.end() ? nullptr : &*piece_it;
}

[[nodiscard]] auto
source_bin_id_for_expanded_bin(const NormalizedRequest &request,
                               const std::uint32_t expanded_bin_id)
    -> std::optional<std::uint32_t> {
  for (const auto &expanded_bin : request.expanded_bins) {
    if (expanded_bin.expanded_bin_id == expanded_bin_id) {
      return expanded_bin.source_bin_id;
    }
  }
  return std::nullopt;
}

[[nodiscard]] auto
source_bin_for_expanded_bin(const NormalizedRequest &request,
                            const std::uint32_t expanded_bin_id)
    -> const BinRequest * {
  const auto source_bin_id =
      source_bin_id_for_expanded_bin(request, expanded_bin_id);
  if (!source_bin_id.has_value()) {
    return nullptr;
  }

  const auto &bins = request.request.bins;
  const auto it = std::find_if(bins.begin(), bins.end(),
                               [source_bin_id](const BinRequest &bin) {
                                 return bin.bin_id == *source_bin_id;
                               });
  return it == bins.end() ? nullptr : &*it;
}

[[nodiscard]] auto
piece_allows_expanded_bin(const NormalizedRequest &request,
                          const PieceRequest &piece,
                          const std::uint32_t expanded_bin_id) -> bool {
  if (piece.allowed_bin_ids.empty()) {
    return true;
  }
  const auto source_bin_id =
      source_bin_id_for_expanded_bin(request, expanded_bin_id);
  if (!source_bin_id.has_value()) {
    return false;
  }
  return std::find(piece.allowed_bin_ids.begin(), piece.allowed_bin_ids.end(),
                   *source_bin_id) != piece.allowed_bin_ids.end();
}

[[nodiscard]] auto piece_allows_rotation(const NormalizedRequest &request,
                                         const PieceRequest &piece,
                                         const pack::PlacedPiece &placed)
    -> bool {
  const auto &rotations = piece.allowed_rotations.has_value()
                              ? *piece.allowed_rotations
                              : request.request.execution.default_rotations;
  const auto resolved =
      geom::resolve_rotation(placed.placement.rotation_index, rotations);
  if (!resolved.has_value()) {
    return false;
  }
  return std::fabs(
             geom::normalize_angle_degrees(resolved->degrees) -
             geom::normalize_angle_degrees(placed.resolved_rotation.degrees)) <=
         kRotationAngleEpsilon;
}

[[nodiscard]] auto
layout_has_conservation_violation(const NormalizedRequest &request,
                                  const NestingResult &result) -> bool {
  std::unordered_set<std::uint32_t> expected_ids;
  expected_ids.reserve(request.expanded_pieces.size());
  for (const auto &expanded : request.expanded_pieces) {
    expected_ids.insert(expanded.expanded_piece_id);
  }

  std::unordered_set<std::uint32_t> observed_ids;
  observed_ids.reserve(expected_ids.size());
  for (const auto &bin : result.layout.bins) {
    for (const auto &placed : bin.placements) {
      const auto piece_id = placed.placement.piece_id;
      if (!expected_ids.contains(piece_id) ||
          !observed_ids.insert(piece_id).second) {
        return true;
      }
    }
  }

  for (const auto piece_id : result.layout.unplaced_piece_ids) {
    if (!expected_ids.contains(piece_id) ||
        !observed_ids.insert(piece_id).second) {
      return true;
    }
  }

  return observed_ids.size() != expected_ids.size();
}

[[nodiscard]] auto
overlaps_configured_exclusion_zone(const NormalizedRequest &request,
                                   const std::uint32_t expanded_bin_id,
                                   const pack::PlacedPiece &placed) -> bool {
  const auto *source_bin =
      source_bin_for_expanded_bin(request, expanded_bin_id);
  if (source_bin == nullptr || source_bin->exclusion_zones.empty()) {
    return false;
  }

  const auto placed_bounds = pack::compute_bounds(placed.polygon);
  for (const auto &zone : source_bin->exclusion_zones) {
    if (pack::overlaps_exclusion_zone(placed.polygon, placed_bounds, zone)) {
      return true;
    }
  }
  return false;
}

} // namespace

auto constructive_layout_has_geometry_violation(
    const NormalizedRequest &request, const NestingResult &result) -> bool {
  if (layout_has_conservation_violation(request, result)) {
    return true;
  }

  for (const auto &bin : result.layout.bins) {
    if (source_bin_for_expanded_bin(request, bin.bin_id) == nullptr) {
      return true;
    }

    for (const auto &placed : bin.placements) {
      const auto *source_piece =
          source_piece_for_expanded_piece(request, placed.placement.piece_id);
      if (source_piece == nullptr || placed.placement.bin_id != bin.bin_id ||
          (placed.placement.mirrored && !source_piece->allow_mirror) ||
          !piece_allows_expanded_bin(request, *source_piece, bin.bin_id) ||
          !piece_allows_rotation(request, *source_piece, placed) ||
          exact_area_outside(bin.container, placed.polygon) >
              kConstructiveContainmentAreaEpsilon) {
        return true;
      }

      if (overlaps_configured_exclusion_zone(request, bin.bin_id, placed)) {
        return true;
      }
    }

    for (std::size_t lhs_index = 0; lhs_index < bin.placements.size();
         ++lhs_index) {
      const auto &lhs = bin.placements[lhs_index];
      const auto lhs_bounds = pack::compute_bounds(lhs.polygon);
      for (std::size_t rhs_index = lhs_index + 1;
           rhs_index < bin.placements.size(); ++rhs_index) {
        const auto &rhs = bin.placements[rhs_index];
        const auto rhs_bounds = pack::compute_bounds(rhs.polygon);
        const bool part_in_part_pair =
            request.request.execution.enable_part_in_part_placement &&
            (lhs.inside_hole || rhs.inside_hole);
        if (!part_in_part_pair && pack::boxes_violate_spacing(
                                      lhs_bounds, rhs_bounds,
                                      request.request.execution.part_spacing)) {
          return true;
        }
        if (!pack::boxes_overlap(lhs_bounds, rhs_bounds)) {
          continue;
        }
        if (exact_overlap_area(lhs.polygon, rhs.polygon) >
            kConstructiveOverlapAreaEpsilon) {
          return true;
        }
      }
    }
  }
  return false;
}

} // namespace shiny::nesting::search::detail
