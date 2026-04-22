#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "geometry/types.hpp"
#include "packing/decoder.hpp"
#include "placement/config.hpp"

namespace shiny::nesting::pack {

inline constexpr double kAreaEpsilon = 1e-9;
inline constexpr double kCoordinateSnap = 1e-10;
inline constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] auto snap_coordinate(double value) -> double;

[[nodiscard]] auto almost_equal(double lhs, double rhs) -> bool;

[[nodiscard]] auto translate_point(const geom::Point2 &point,
                                   const geom::Point2 &translation)
    -> geom::Point2;

[[nodiscard]] auto translate_ring(const geom::Ring &ring,
                                  const geom::Point2 &translation)
    -> geom::Ring;

[[nodiscard]] auto translate_polygon(const geom::PolygonWithHoles &polygon,
                                     const geom::Point2 &translation)
    -> geom::PolygonWithHoles;

[[nodiscard]] auto rotate_point(const geom::Point2 &point, double degrees)
    -> geom::Point2;

[[nodiscard]] auto rotate_ring(const geom::Ring &ring, double degrees)
    -> geom::Ring;

[[nodiscard]] auto rotate_polygon(const geom::PolygonWithHoles &polygon,
                                  double degrees) -> geom::PolygonWithHoles;

[[nodiscard]] auto signed_area(const geom::Ring &ring) -> long double;

[[nodiscard]] auto polygon_area(const geom::PolygonWithHoles &polygon)
    -> double;

[[nodiscard]] auto
total_polygon_area(std::span<const geom::PolygonWithHoles> polygons) -> double;

[[nodiscard]] auto compute_bounds(const geom::PolygonWithHoles &polygon)
    -> geom::Box2;

[[nodiscard]] auto box_width(const geom::Box2 &box) -> double;

[[nodiscard]] auto box_height(const geom::Box2 &box) -> double;

[[nodiscard]] auto box_has_area(const geom::Box2 &box) -> bool;

[[nodiscard]] auto normalize_box(const geom::Point2 &first,
                                 const geom::Point2 &second) -> geom::Box2;

[[nodiscard]] auto contains_box(const geom::Box2 &container,
                                const geom::Box2 &candidate) -> bool;

[[nodiscard]] auto expand_box(const geom::Box2 &box, double clearance)
    -> geom::Box2;

[[nodiscard]] auto spacing_reservation_bounds(const geom::Box2 &box,
                                              double spacing) -> geom::Box2;

[[nodiscard]] auto boxes_violate_spacing(const geom::Box2 &lhs,
                                         const geom::Box2 &rhs,
                                         double spacing) -> bool;

[[nodiscard]] auto boxes_overlap(const geom::Box2 &lhs, const geom::Box2 &rhs)
    -> bool;

[[nodiscard]] auto intervals_overlap_interior(double lhs_min, double lhs_max,
                                              double rhs_min, double rhs_max)
    -> bool;

[[nodiscard]] auto boxes_overlap_interior(const geom::Box2 &lhs,
                                          const geom::Box2 &rhs) -> bool;

[[nodiscard]] auto interrupted(const InterruptionProbe &interruption_requested)
    -> bool;

[[nodiscard]] auto as_polygon_with_holes(const place::BedExclusionZone &zone)
    -> geom::PolygonWithHoles;

[[nodiscard]] auto overlaps_exclusion_zone(const geom::PolygonWithHoles &piece,
                                           const geom::Box2 &piece_bounds,
                                           const place::BedExclusionZone &zone)
    -> bool;

[[nodiscard]] auto overlaps_any_exclusion_zone(
    const geom::PolygonWithHoles &piece, const geom::Box2 &piece_bounds,
    std::span<const place::BedExclusionZone> exclusion_zones,
    std::uint32_t bin_id) -> bool;

[[nodiscard]] auto piece_allows_bin(const PieceInput &piece,
                                    std::uint32_t bin_id) -> bool;
[[nodiscard]] auto allowed_rotations_for(
    const PieceInput &piece,
    const place::PlacementConfig &config) -> const geom::DiscreteRotationSet &;

auto mark_remaining_unplaced(std::span<const PieceInput> pieces,
                             std::size_t start_index,
                             std::vector<std::uint32_t> &unplaced_piece_ids)
    -> void;

} // namespace shiny::nesting::pack
