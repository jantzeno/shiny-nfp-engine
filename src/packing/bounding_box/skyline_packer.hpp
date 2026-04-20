#pragma once

#include <optional>

#include "geometry/types.hpp"
#include "packing/bounding_box/types.hpp"
#include "packing/decoder.hpp"

namespace shiny::nesting::pack {

[[nodiscard]] auto find_best_skyline_candidate(
    const BinPackingState &state, const PieceInput &piece,
    const DecoderRequest &request, const geom::PolygonWithHoles &rotated_piece,
    const geom::Box2 &rotated_bounds,
    const geom::ResolvedRotation &resolved_rotation,
    const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate>;

} // namespace shiny::nesting::pack
