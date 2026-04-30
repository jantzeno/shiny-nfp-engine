#include "packing/bounding_box/skyline_packer.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "packing/bounding_box/common.hpp"
#include "packing/common.hpp"

namespace shiny::nesting::pack {

namespace {

[[nodiscard]] auto
skyline_candidate_min_xs(const geom::Box2 &container_bounds,
                         std::span<const geom::Box2> occupied_bounds,
                         double width) -> std::vector<double> {
  std::vector<double> candidates{
      container_bounds.min.x,
      container_bounds.max.x - width,
  };
  candidates.reserve(2U + occupied_bounds.size() * 4U);
  for (const geom::Box2 &occupied_bounds_entry : occupied_bounds) {
    candidates.push_back(occupied_bounds_entry.min.x);
    candidates.push_back(occupied_bounds_entry.max.x);
    candidates.push_back(occupied_bounds_entry.min.x - width);
    candidates.push_back(occupied_bounds_entry.max.x - width);
  }

  std::vector<double> unique = unique_sorted_values(std::move(candidates));
  unique.erase(std::remove_if(unique.begin(), unique.end(),
                              [&](double value) {
                                return value < container_bounds.min.x -
                                                   kCoordinateSnap ||
                                       value + width > container_bounds.max.x +
                                                           kCoordinateSnap;
                              }),
               unique.end());
  return unique;
}

} // namespace

auto find_best_skyline_candidate(
    const BinPackingState &state, const PieceInput &piece,
    const DecoderRequest &request, const geom::PolygonWithHoles &rotated_piece,
    const geom::Box2 &rotated_bounds,
    const geom::ResolvedRotation &resolved_rotation,
    const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  const double clearance = request.config.placement.part_clearance;
  const geom::Box2 canonical_container =
      box_for_start_corner(state.container_bounds, state.container_bounds,
                           state.bin_state.start_corner);
  std::vector<geom::Box2> canonical_occupied_bounds;
  canonical_occupied_bounds.reserve(state.occupied_bounds.size());
  for (const geom::Box2 &occupied_bounds_entry : state.occupied_bounds) {
    canonical_occupied_bounds.push_back(box_for_start_corner(
        spacing_reservation_bounds(occupied_bounds_entry, clearance),
        state.container_bounds, state.bin_state.start_corner));
  }

  const geom::Box2 canonical_rotated_bounds = box_for_start_corner(
      rotated_bounds, rotated_bounds, place::PlacementStartCorner::bottom_left);
  const double width = box_width(canonical_rotated_bounds);
  const double height = box_height(canonical_rotated_bounds);

  for (const double candidate_min_x : skyline_candidate_min_xs(
           canonical_container, canonical_occupied_bounds, width)) {
    double candidate_min_y = canonical_container.min.y;
    for (const geom::Box2 &occupied_bounds_entry : canonical_occupied_bounds) {
      if (!intervals_overlap_interior(candidate_min_x, candidate_min_x + width,
                                      occupied_bounds_entry.min.x,
                                      occupied_bounds_entry.max.x)) {
        continue;
      }
      candidate_min_y = std::max(candidate_min_y, occupied_bounds_entry.max.y);
    }

    const geom::Box2 canonical_candidate_bounds{
        .min = {.x = candidate_min_x, .y = candidate_min_y},
        .max = {.x = candidate_min_x + width, .y = candidate_min_y + height},
    };
    const geom::Box2 actual_candidate_bounds =
        box_for_start_corner(canonical_candidate_bounds, state.container_bounds,
                             state.bin_state.start_corner);
    if (!contains_box(state.container_bounds, actual_candidate_bounds)) {
      continue;
    }

    const geom::Point2 translation{
        .x = actual_candidate_bounds.min.x - rotated_bounds.min.x,
        .y = actual_candidate_bounds.min.y - rotated_bounds.min.y,
    };
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_occupied_bounds(state.occupied_bounds,
                                     actual_candidate_bounds, clearance) ||
        overlaps_any_exclusion_zone(translated_piece, actual_candidate_bounds,
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
        better_candidate(state, candidate, *best, request.policy)) {
      best = std::move(candidate);
    }
  }

  return best;
}

} // namespace shiny::nesting::pack
