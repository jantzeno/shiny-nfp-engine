#include "packing/common.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "placement/config.hpp"
#include "polygon_ops/boolean_ops.hpp"

namespace shiny::nesting::pack {

auto snap_coordinate(double value) -> double {
  if (std::fabs(value) < kCoordinateSnap) {
    return 0.0;
  }
  return value;
}

auto almost_equal(double lhs, double rhs) -> bool {
  return std::fabs(lhs - rhs) <= kCoordinateSnap;
}

auto translate_point(const geom::Point2 &point, const geom::Point2 &translation)
    -> geom::Point2 {
  return geom::translate(point,
                         geom::Vector2{translation.x(), translation.y()});
}

auto translate_ring(const geom::Ring &ring, const geom::Point2 &translation)
    -> geom::Ring {
  return geom::translate(ring, geom::Vector2{translation.x(), translation.y()});
}

auto translate_polygon(const geom::PolygonWithHoles &polygon,
                       const geom::Point2 &translation)
    -> geom::PolygonWithHoles {
  return geom::translate(polygon,
                         geom::Vector2{translation.x(), translation.y()});
}

auto rotate_point(const geom::Point2 &point, double degrees) -> geom::Point2 {
  return geom::rotate(point, geom::ResolvedRotation{.degrees = degrees});
}

auto rotate_ring(const geom::Ring &ring, double degrees) -> geom::Ring {
  return geom::rotate(ring, geom::ResolvedRotation{.degrees = degrees});
}

auto rotate_polygon(const geom::PolygonWithHoles &polygon, double degrees)
    -> geom::PolygonWithHoles {
  return geom::rotate(polygon, geom::ResolvedRotation{.degrees = degrees});
}

auto signed_area(const geom::Ring &ring) -> long double {
  return static_cast<long double>(geom::ring_signed_area(ring));
}

auto polygon_area(const geom::PolygonWithHoles &polygon) -> double {
  return geom::polygon_area(polygon);
}

auto total_polygon_area(std::span<const geom::PolygonWithHoles> polygons)
    -> double {
  return geom::polygon_area_sum(polygons);
}

auto compute_bounds(const geom::PolygonWithHoles &polygon) -> geom::Box2 {
  return geom::compute_bounds(polygon);
}

auto box_width(const geom::Box2 &box) -> double { return geom::box_width(box); }

auto box_height(const geom::Box2 &box) -> double {
  return geom::box_height(box);
}

auto box_has_area(const geom::Box2 &box) -> bool {
  return geom::box_width(box) > kCoordinateSnap &&
         geom::box_height(box) > kCoordinateSnap;
}

auto normalize_box(const geom::Point2 &first, const geom::Point2 &second)
    -> geom::Box2 {
  return {geom::Point2{std::min(first.x(), second.x()),
                       std::min(first.y(), second.y())},
          geom::Point2{std::max(first.x(), second.x()),
                       std::max(first.y(), second.y())}};
}

auto contains_box(const geom::Box2 &container, const geom::Box2 &candidate)
    -> bool {
  return geom::box_contains(container, candidate);
}

auto expand_box(const geom::Box2 &box, double clearance) -> geom::Box2 {
  if (clearance <= 0.0) {
    return box;
  }

  return {geom::Point2{box.min.x() - clearance, box.min.y() - clearance},
          geom::Point2{box.max.x() + clearance, box.max.y() + clearance}};
}

auto spacing_reservation_bounds(const geom::Box2 &box, double spacing)
    -> geom::Box2 {
  return expand_box(box, spacing);
}

auto boxes_violate_spacing(const geom::Box2 &lhs, const geom::Box2 &rhs,
                           double spacing) -> bool {
  return boxes_overlap_interior(lhs, spacing_reservation_bounds(rhs, spacing));
}

auto boxes_overlap(const geom::Box2 &lhs, const geom::Box2 &rhs) -> bool {
  return geom::boxes_overlap(lhs, rhs);
}

auto intervals_overlap_interior(double lhs_min, double lhs_max, double rhs_min,
                                double rhs_max) -> bool {
  return lhs_max > rhs_min + kCoordinateSnap &&
         rhs_max > lhs_min + kCoordinateSnap;
}

auto boxes_overlap_interior(const geom::Box2 &lhs, const geom::Box2 &rhs)
    -> bool {
  return intervals_overlap_interior(lhs.min.x(), lhs.max.x(), rhs.min.x(),
                                    rhs.max.x()) &&
         intervals_overlap_interior(lhs.min.y(), lhs.max.y(), rhs.min.y(),
                                    rhs.max.y());
}

auto interrupted(const InterruptionProbe &interruption_requested) -> bool {
  return interruption_requested && interruption_requested();
}

auto as_polygon_with_holes(const place::BedExclusionZone &zone)
    -> geom::PolygonWithHoles {
  return {zone.region.outer()};
}

auto overlaps_exclusion_zone(const geom::PolygonWithHoles &piece,
                             const geom::Box2 &piece_bounds,
                             const place::BedExclusionZone &zone) -> bool {
  const auto zone_polygon = as_polygon_with_holes(zone);
  if (piece.outer().empty() || zone_polygon.outer().empty() ||
      !geom::boxes_overlap(piece_bounds, geom::compute_bounds(zone_polygon))) {
    return false;
  }

  const auto remaining = poly::difference_polygons(piece, zone_polygon);
  return total_polygon_area(remaining) + kAreaEpsilon <
         geom::polygon_area(piece);
}

auto overlaps_any_exclusion_zone(
    const geom::PolygonWithHoles &piece, const geom::Box2 &piece_bounds,
    std::span<const place::BedExclusionZone> exclusion_zones,
    std::uint32_t bin_id) -> bool {
  for (const auto &zone : exclusion_zones) {
    if (!place::exclusion_zone_applies_to_bin(zone, bin_id)) {
      continue;
    }
    if (overlaps_exclusion_zone(piece, piece_bounds, zone)) {
      return true;
    }
  }
  return false;
}

auto piece_allows_bin(const PieceInput &piece, std::uint32_t bin_id) -> bool {
  return (!piece.restricted_to_allowed_bins && piece.allowed_bin_ids.empty()) ||
         std::find(piece.allowed_bin_ids.begin(), piece.allowed_bin_ids.end(),
                   bin_id) != piece.allowed_bin_ids.end();
}

auto allowed_rotations_for(const PieceInput &piece,
                           const place::PlacementConfig &config)
    -> const geom::DiscreteRotationSet & {
  return piece.allowed_rotations.has_value() ? *piece.allowed_rotations
                                             : config.allowed_rotations;
}

auto mark_remaining_unplaced(std::span<const PieceInput> pieces,
                             std::size_t start_index,
                             std::vector<std::uint32_t> &unplaced_piece_ids)
    -> void {
  for (std::size_t index = start_index; index < pieces.size(); ++index) {
    unplaced_piece_ids.push_back(pieces[index].piece_id);
  }
}

} // namespace shiny::nesting::pack
