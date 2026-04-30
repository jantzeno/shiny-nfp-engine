#include "validation/layout_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>

#include "geometry/operations/boolean_ops.hpp"
#include "packing/common.hpp"

namespace shiny::nesting::validation {
namespace {

[[nodiscard]] auto exact_overlap_area(const geom::PolygonWithHoles &lhs,
                                      const geom::PolygonWithHoles &rhs)
    -> double {
  return geom::polygon_area_sum(geom::intersection_polygons(lhs, rhs));
}

[[nodiscard]] auto exact_area_outside(const geom::PolygonWithHoles &container,
                                      const geom::PolygonWithHoles &piece)
    -> double {
  const auto piece_area = geom::polygon_area(piece);
  const auto contained_area =
      geom::polygon_area_sum(geom::intersection_polygons(container, piece));
  return piece_area - contained_area;
}

auto add_issue(LayoutValidationReport &report,
               const LayoutValidationIssueKind issue_kind,
               const std::optional<std::uint32_t> expanded_piece_id = {},
               const std::optional<std::uint32_t> expanded_bin_id = {},
               const std::optional<std::uint32_t> other_piece_id = {},
               const double measured_value = 0.0, const double tolerance = 0.0)
    -> void {
  report.valid = false;
  report.issues.push_back({
      .issue_kind = issue_kind,
      .expanded_piece_id = expanded_piece_id,
      .expanded_bin_id = expanded_bin_id,
      .other_piece_id = other_piece_id,
      .measured_value = measured_value,
      .tolerance = tolerance,
  });
}

auto add_repair(LayoutValidationReport &report,
                const std::uint32_t expanded_piece_id) -> void {
  report.issues.push_back({
      .issue_kind = LayoutValidationIssueKind::conservation_repair,
      .expanded_piece_id = expanded_piece_id,
  });
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
source_request_bin_id_for_layout_bin(const NormalizedRequest &request,
                                     const pack::LayoutBin &bin)
    -> std::optional<std::uint32_t> {
  if (const auto source_bin_id =
          source_bin_id_for_expanded_bin(request, bin.bin_id);
      source_bin_id.has_value()) {
    return source_bin_id;
  }
  if (bin.identity.source_request_bin_id != 0U) {
    return bin.identity.source_request_bin_id;
  }
  return std::nullopt;
}

[[nodiscard]] auto source_bin_for_layout_bin(const NormalizedRequest &request,
                                             const pack::LayoutBin &bin)
    -> const BinRequest * {
  const auto source_bin_id = source_request_bin_id_for_layout_bin(request, bin);
  if (!source_bin_id.has_value()) {
    return nullptr;
  }

  const auto &bins = request.request.bins;
  const auto it = std::find_if(bins.begin(), bins.end(),
                               [source_bin_id](const BinRequest &candidate) {
                                 return candidate.bin_id == *source_bin_id;
                               });
  return it == bins.end() ? nullptr : &*it;
}

[[nodiscard]] auto piece_allows_rotation(const NormalizedRequest &request,
                                         const PieceRequest &piece,
                                         const pack::PlacedPiece &placed,
                                         const double angle_tolerance) -> bool {
  const auto &rotations = piece.allowed_rotations.has_value()
                              ? *piece.allowed_rotations
                              : request.request.execution.default_rotations;
  const auto resolved =
      geom::resolve_rotation(placed.placement.rotation_index, rotations);
  if (!resolved.has_value()) {
    return false;
  }
  return std::fabs(geom::normalize_angle_degrees(resolved->degrees) -
                   geom::normalize_angle_degrees(
                       placed.resolved_rotation.degrees)) <= angle_tolerance;
}

[[nodiscard]] auto
overlaps_configured_exclusion_zone(const NormalizedRequest &request,
                                   const pack::LayoutBin &bin,
                                   const pack::PlacedPiece &placed) -> bool {
  const auto *source_bin = source_bin_for_layout_bin(request, bin);
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

auto validate_layout(const NormalizedRequest &request,
                     const NestingResult &result,
                     const LayoutValidationOptions options)
    -> LayoutValidationReport {
  LayoutValidationReport report{.valid = true};

  std::unordered_set<std::uint32_t> expected_ids;
  expected_ids.reserve(request.expanded_pieces.size());
  for (const auto &expanded : request.expanded_pieces) {
    expected_ids.insert(expanded.expanded_piece_id);
  }

  std::unordered_set<std::uint32_t> observed_ids;
  observed_ids.reserve(expected_ids.size());

  for (const auto &bin : result.layout.bins) {
    if (source_bin_for_layout_bin(request, bin) == nullptr) {
      add_issue(report, LayoutValidationIssueKind::unknown_bin, std::nullopt,
                bin.bin_id);
    }

    for (const auto &placed : bin.placements) {
      const auto piece_id = placed.placement.piece_id;
      const auto *source_piece =
          source_piece_for_expanded_piece(request, piece_id);
      if (source_piece == nullptr) {
        add_issue(report, LayoutValidationIssueKind::unknown_piece, piece_id,
                  bin.bin_id);
      } else if (!observed_ids.insert(piece_id).second) {
        add_issue(report, LayoutValidationIssueKind::duplicate_piece, piece_id,
                  bin.bin_id);
      }

      if (placed.placement.bin_id != bin.bin_id) {
        add_issue(report, LayoutValidationIssueKind::bin_mismatch, piece_id,
                  bin.bin_id);
      }

      if (source_piece == nullptr) {
        continue;
      }

      if (placed.placement.mirrored && !source_piece->allow_mirror) {
        add_issue(report, LayoutValidationIssueKind::disallowed_mirror,
                  piece_id, bin.bin_id);
      }
      const auto source_request_bin_id =
          source_request_bin_id_for_layout_bin(request, bin);
      const bool bin_allowed = source_piece->allowed_bin_ids.empty() ||
                               (source_request_bin_id.has_value() &&
                                std::find(source_piece->allowed_bin_ids.begin(),
                                          source_piece->allowed_bin_ids.end(),
                                          *source_request_bin_id) !=
                                    source_piece->allowed_bin_ids.end());
      if (!bin_allowed) {
        add_issue(report, LayoutValidationIssueKind::disallowed_bin, piece_id,
                  bin.bin_id);
      }
      if (!piece_allows_rotation(request, *source_piece, placed,
                                 options.rotation_angle_tolerance)) {
        add_issue(report, LayoutValidationIssueKind::disallowed_rotation,
                  piece_id, bin.bin_id);
      }

      const auto outside_area =
          exact_area_outside(bin.container, placed.polygon);
      if (outside_area > options.containment_area_tolerance) {
        add_issue(report, LayoutValidationIssueKind::outside_container,
                  piece_id, bin.bin_id, std::nullopt, outside_area,
                  options.containment_area_tolerance);
      }

      if (overlaps_configured_exclusion_zone(request, bin, placed)) {
        add_issue(report, LayoutValidationIssueKind::inside_exclusion_zone,
                  piece_id, bin.bin_id);
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

        if (!part_in_part_pair &&
            request.request.execution.part_spacing > 0.0 &&
            pack::boxes_violate_spacing(
                lhs_bounds, rhs_bounds,
                request.request.execution.part_spacing)) {
          const auto distance =
              geom::polygon_distance(lhs.polygon, rhs.polygon);
          if (distance + options.spacing_tolerance <
              request.request.execution.part_spacing) {
            add_issue(report, LayoutValidationIssueKind::spacing_violation,
                      lhs.placement.piece_id, bin.bin_id,
                      rhs.placement.piece_id, distance,
                      options.spacing_tolerance);
          }
        }

        if (!pack::boxes_overlap(lhs_bounds, rhs_bounds)) {
          continue;
        }
        const auto overlap_area = exact_overlap_area(lhs.polygon, rhs.polygon);
        if (overlap_area > options.overlap_area_tolerance) {
          add_issue(report, LayoutValidationIssueKind::piece_overlap,
                    lhs.placement.piece_id, bin.bin_id, rhs.placement.piece_id,
                    overlap_area, options.overlap_area_tolerance);
        }
      }
    }
  }

  for (const auto piece_id : result.layout.unplaced_piece_ids) {
    if (!expected_ids.contains(piece_id)) {
      add_issue(report, LayoutValidationIssueKind::unknown_piece, piece_id);
      continue;
    }
    if (!observed_ids.insert(piece_id).second) {
      add_issue(report, LayoutValidationIssueKind::duplicate_piece, piece_id);
    }
  }

  for (const auto &expanded : request.expanded_pieces) {
    if (!observed_ids.contains(expanded.expanded_piece_id)) {
      add_issue(report, LayoutValidationIssueKind::missing_piece,
                expanded.expanded_piece_id);
    }
  }

  return report;
}

auto layout_has_geometry_violation(const NormalizedRequest &request,
                                   const NestingResult &result,
                                   const LayoutValidationOptions options)
    -> bool {
  return !validate_layout(request, result, options).valid;
}

auto finalize_layout_conservation(const NormalizedRequest &request,
                                  NestingResult &result) -> void {
  std::unordered_set<std::uint32_t> placed_ids;
  placed_ids.reserve(request.expanded_pieces.size());
  for (const auto &bin : result.layout.bins) {
    for (const auto &placed : bin.placements) {
      placed_ids.insert(placed.placement.piece_id);
    }
  }

  std::unordered_set<std::uint32_t> unplaced_ids;
  unplaced_ids.reserve(result.layout.unplaced_piece_ids.size());
  for (const auto piece_id : result.layout.unplaced_piece_ids) {
    unplaced_ids.insert(piece_id);
  }

  for (const auto &expanded : request.expanded_pieces) {
    if (!placed_ids.contains(expanded.expanded_piece_id) &&
        !unplaced_ids.contains(expanded.expanded_piece_id)) {
      result.layout.unplaced_piece_ids.push_back(expanded.expanded_piece_id);
      unplaced_ids.insert(expanded.expanded_piece_id);
    }
  }
}

auto finalize_result(const NormalizedRequest &request, NestingResult &result,
                     const LayoutValidationOptions options) -> void {
  const auto before_repair = validate_layout(request, result, options);
  finalize_layout_conservation(request, result);
  result.validation = validate_layout(request, result, options);
  for (const auto &issue : before_repair.issues) {
    if (issue.issue_kind == LayoutValidationIssueKind::missing_piece &&
        issue.expanded_piece_id.has_value()) {
      add_repair(result.validation, *issue.expanded_piece_id);
    }
  }
}

} // namespace shiny::nesting::validation
