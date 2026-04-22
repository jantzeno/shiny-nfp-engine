#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "geometry/types.hpp"
#include "packing/bounding_box/types.hpp"
#include "packing/decoder.hpp"
#include "placement/config.hpp"

namespace shiny::nesting::pack {

[[nodiscard]] auto
point_for_start_corner(const geom::Point2 &point, const geom::Box2 &container,
                       place::PlacementStartCorner start_corner)
    -> geom::Point2;

[[nodiscard]] auto
box_for_start_corner(const geom::Box2 &box, const geom::Box2 &container,
                     place::PlacementStartCorner start_corner) -> geom::Box2;

[[nodiscard]] auto unique_sorted_values(std::vector<double> values)
    -> std::vector<double>;

[[nodiscard]] auto
overlaps_any_occupied_bounds(std::span<const geom::Box2> occupied_bounds,
                              const geom::Box2 &candidate_bounds) -> bool;

[[nodiscard]] auto
overlaps_any_occupied_bounds(std::span<const geom::Box2> occupied_bounds,
                             const geom::Box2 &candidate_bounds,
                             double spacing) -> bool;

auto split_free_rectangles(std::vector<geom::Box2> &free_rectangles,
                           const geom::Box2 &used_bounds) -> void;

[[nodiscard]] auto total_container_area(const DecoderResult &result) -> double;

[[nodiscard]] auto total_occupied_area(const DecoderResult &result) -> double;

[[nodiscard]] auto overall_utilization(const DecoderResult &result) -> double;

[[nodiscard]] auto decode_result_better(const DecoderResult &lhs,
                                        const DecoderResult &rhs) -> bool;

[[nodiscard]] auto resulting_envelope_area(const BinPackingState &state,
                                           const geom::Box2 &candidate_bounds)
    -> double;

[[nodiscard]] auto
start_corner_on_right(place::PlacementStartCorner start_corner) -> bool;

[[nodiscard]] auto start_corner_on_top(place::PlacementStartCorner start_corner)
    -> bool;

[[nodiscard]] auto
primary_edge_distance(const geom::Box2 &container_bounds,
                      const geom::Box2 &piece_bounds,
                      place::PlacementStartCorner start_corner) -> double;

[[nodiscard]] auto
secondary_edge_distance(const geom::Box2 &container_bounds,
                        const geom::Box2 &piece_bounds,
                        place::PlacementStartCorner start_corner) -> double;

[[nodiscard]] auto better_candidate(const BinPackingState &state,
                                    const PlacementCandidate &lhs,
                                    const PlacementCandidate &rhs,
                                    place::PlacementPolicy policy) -> bool;

[[nodiscard]] auto make_empty_bin(const BinInput &bin) -> BinPackingState;

} // namespace shiny::nesting::pack
