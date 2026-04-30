#include "packing/bounding_box/shelf_packer.hpp"

#include <cstddef>
#include <optional>

#include "packing/bounding_box/common.hpp"
#include "packing/common.hpp"

namespace shiny::nesting::pack {

namespace {

[[nodiscard]] auto fits_on_existing_shelf(const ShelfState &shelf,
                                          const geom::Box2 &rotated_bounds)
    -> bool {
  const auto width = rotated_bounds.max.x() - rotated_bounds.min.x();
  const auto height = rotated_bounds.max.y() - rotated_bounds.min.y();
  return height <= shelf.height + kCoordinateSnap && width >= 0.0 &&
         height >= 0.0;
}

} // namespace

auto find_best_shelf_candidate(const BinPackingState &state,
                               const PieceInput &piece,
                               const DecoderRequest &request,
                               const geom::PolygonWithHoles &rotated_piece,
                               const geom::Box2 &rotated_bounds,
                               const geom::ResolvedRotation &resolved_rotation,
                               const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  const double clearance = request.config.placement.part_clearance;
  const double width = box_width(rotated_bounds);
  const double height = box_height(rotated_bounds);

  const auto consider_candidate = [&](double target_min_x, double target_min_y,
                                      std::size_t shelf_index,
                                      bool starts_new_shelf) {
    const geom::Box2 translated_bounds{
        geom::Point2{target_min_x, target_min_y},
        geom::Point2{target_min_x + width, target_min_y + height},
    };
    if (!contains_box(state.container_bounds, translated_bounds) ||
        overlaps_any_occupied_bounds(state.occupied_bounds, translated_bounds,
                                     clearance)) {
      return;
    }

    const geom::Point2 translation{target_min_x - rotated_bounds.min.x(),
                                   target_min_y - rotated_bounds.min.y()};
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_exclusion_zone(translated_piece, translated_bounds,
                                    request.config.placement.exclusion_zones,
                                    state.bin_state.bin_id)) {
      return;
    }

    const auto envelope_area =
        resulting_envelope_area(state, translated_bounds);
    const auto piece_area = polygon_area(rotated_piece);
    const auto utilization =
        envelope_area > kCoordinateSnap
            ? (state.occupied_area + piece_area) / envelope_area
            : 0.0;
    PlacementCandidate candidate{
        .shelf_index = shelf_index,
        .starts_new_shelf = starts_new_shelf,
        .placement =
            {
                .piece_id = piece.piece_id,
                .bin_id = state.bin_state.bin_id,
                .rotation_index = rotation_index,
                .translation = translation,
            },
        .resolved_rotation = resolved_rotation,
        .rotated_piece = rotated_piece,
        .translated_bounds = translated_bounds,
        .resulting_utilization = utilization,
    };

    if (!best.has_value() ||
        better_candidate(state, candidate, *best, request.policy)) {
      best = std::move(candidate);
    }
  };

  for (std::size_t shelf_index = 0; shelf_index < state.shelves.size();
       ++shelf_index) {
    const ShelfState &shelf = state.shelves[shelf_index];
    if (!fits_on_existing_shelf(shelf, rotated_bounds)) {
      continue;
    }
    const double target_min_x =
        start_corner_on_right(state.bin_state.start_corner)
            ? shelf.next_x - width
            : shelf.next_x;
    consider_candidate(target_min_x, shelf.y, shelf_index, false);
  }

  const double next_shelf_y =
      state.shelves.empty()
          ? (start_corner_on_top(state.bin_state.start_corner)
                 ? state.container_bounds.max.y() - height
                 : state.container_bounds.min.y())
          : (start_corner_on_top(state.bin_state.start_corner)
                 ? state.shelves.back().y - height - clearance
                 : state.shelves.back().y + state.shelves.back().height +
                       clearance);
  const double new_shelf_min_x =
      start_corner_on_right(state.bin_state.start_corner)
          ? state.container_bounds.max.x() - width
          : state.container_bounds.min.x();
  consider_candidate(new_shelf_min_x, next_shelf_y, state.shelves.size(), true);
  return best;
}

} // namespace shiny::nesting::pack
