#include "packing/bounding_box/free_rectangle_packer.hpp"

#include <cmath>
#include <optional>

#include "packing/bounding_box/common.hpp"
#include "packing/common.hpp"

namespace shiny::nesting::pack {

namespace {

[[nodiscard]] auto free_rectangle_better(const BinPackingState &state,
                                         const geom::Box2 &lhs_bounds,
                                         const geom::Box2 &rhs_bounds,
                                         const geom::Box2 &lhs_free_rectangle,
                                         const geom::Box2 &rhs_free_rectangle,
                                         place::PlacementPolicy policy)
    -> bool {
  const bool lhs_policy_better = better_candidate(
      state, PlacementCandidate{.translated_bounds = lhs_bounds},
      PlacementCandidate{.translated_bounds = rhs_bounds}, policy);
  const bool rhs_policy_better = better_candidate(
      state, PlacementCandidate{.translated_bounds = rhs_bounds},
      PlacementCandidate{.translated_bounds = lhs_bounds}, policy);
  if (lhs_policy_better != rhs_policy_better) {
    return lhs_policy_better;
  }

  const double lhs_short_side_fit = std::min(
      std::abs(box_width(lhs_free_rectangle) - box_width(lhs_bounds)),
      std::abs(box_height(lhs_free_rectangle) - box_height(lhs_bounds)));
  const double rhs_short_side_fit = std::min(
      std::abs(box_width(rhs_free_rectangle) - box_width(rhs_bounds)),
      std::abs(box_height(rhs_free_rectangle) - box_height(rhs_bounds)));
  if (!almost_equal(lhs_short_side_fit, rhs_short_side_fit)) {
    return lhs_short_side_fit < rhs_short_side_fit;
  }

  const double lhs_area_fit =
      box_width(lhs_free_rectangle) * box_height(lhs_free_rectangle) -
      box_width(lhs_bounds) * box_height(lhs_bounds);
  const double rhs_area_fit =
      box_width(rhs_free_rectangle) * box_height(rhs_free_rectangle) -
      box_width(rhs_bounds) * box_height(rhs_bounds);
  if (!almost_equal(lhs_area_fit, rhs_area_fit)) {
    return lhs_area_fit < rhs_area_fit;
  }

  return false;
}

} // namespace

auto find_best_free_rectangle_candidate(
    const BinPackingState &state, const PieceInput &piece,
    const DecoderRequest &request, const geom::PolygonWithHoles &rotated_piece,
    const geom::Box2 &rotated_bounds,
    const geom::ResolvedRotation &resolved_rotation,
    const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  std::optional<geom::Box2> best_free_rectangle;
  const double width = box_width(rotated_bounds);
  const double height = box_height(rotated_bounds);

  for (const geom::Box2 &free_rectangle : state.free_rectangles) {
    const geom::Box2 canonical_free_rectangle = box_for_start_corner(
        free_rectangle, state.container_bounds, state.bin_state.start_corner);
    if (box_width(canonical_free_rectangle) + kCoordinateSnap < width ||
        box_height(canonical_free_rectangle) + kCoordinateSnap < height) {
      continue;
    }

    const geom::Box2 canonical_candidate_bounds{
        .min = canonical_free_rectangle.min,
        .max = {.x = canonical_free_rectangle.min.x + width,
                .y = canonical_free_rectangle.min.y + height},
    };
    const geom::Box2 actual_candidate_bounds =
        box_for_start_corner(canonical_candidate_bounds, state.container_bounds,
                             state.bin_state.start_corner);
    if (!contains_box(state.container_bounds, actual_candidate_bounds) ||
        overlaps_any_occupied_bounds(state.occupied_bounds,
                                     actual_candidate_bounds)) {
      continue;
    }

    const geom::Point2 translation{
        .x = actual_candidate_bounds.min.x - rotated_bounds.min.x,
        .y = actual_candidate_bounds.min.y - rotated_bounds.min.y,
    };
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_exclusion_zone(translated_piece, actual_candidate_bounds,
                                    request.config.placement.exclusion_zones,
                                    state.bin_state.bin_id)) {
      continue;
    }

    const auto envelope_area =
        resulting_envelope_area(state, actual_candidate_bounds);
    const auto piece_area = polygon_area(rotated_piece);
    const auto utilization =
        envelope_area > kCoordinateSnap
            ? (state.occupied_area + piece_area) / envelope_area
            : 0.0;
    PlacementCandidate candidate{
        .shelf_index = 0,
        .starts_new_shelf = false,
        .placement =
            {
                .piece_id = piece.piece_id,
                .bin_id = state.bin_state.bin_id,
                .rotation_index = rotation_index,
                .translation = translation,
            },
        .resolved_rotation = resolved_rotation,
        .rotated_piece = rotated_piece,
        .translated_bounds = actual_candidate_bounds,
        .resulting_utilization = utilization,
    };

    if (!best.has_value() ||
        free_rectangle_better(state, candidate.translated_bounds,
                              best->translated_bounds, canonical_free_rectangle,
                              *best_free_rectangle, request.policy)) {
      best = std::move(candidate);
      best_free_rectangle = canonical_free_rectangle;
    }
  }

  return best;
}

} // namespace shiny::nesting::pack
