#include "packing/common.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "geometry/normalize.hpp"
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
  return {
      .x = point.x + translation.x,
      .y = point.y + translation.y,
  };
}

auto translate_ring(const geom::Ring &ring, const geom::Point2 &translation)
    -> geom::Ring {
  geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(translate_point(point, translation));
  }
  return translated;
}

auto translate_polygon(const geom::PolygonWithHoles &polygon,
                       const geom::Point2 &translation)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, translation);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, translation));
  }
  return translated;
}

auto rotate_point(const geom::Point2 &point, double degrees) -> geom::Point2 {
  const auto radians = degrees * kPi / 180.0;
  auto cosine = std::cos(radians);
  auto sine = std::sin(radians);

  cosine = snap_coordinate(cosine);
  sine = snap_coordinate(sine);

  return {
      .x = snap_coordinate(point.x * cosine - point.y * sine),
      .y = snap_coordinate(point.x * sine + point.y * cosine),
  };
}

auto rotate_ring(const geom::Ring &ring, double degrees) -> geom::Ring {
  geom::Ring rotated;
  rotated.reserve(ring.size());
  for (const auto &point : ring) {
    rotated.push_back(rotate_point(point, degrees));
  }
  return rotated;
}

auto rotate_polygon(const geom::PolygonWithHoles &polygon, double degrees)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles rotated{};
  rotated.outer = rotate_ring(polygon.outer, degrees);
  rotated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    rotated.holes.push_back(rotate_ring(hole, degrees));
  }
  return geom::normalize_polygon(rotated);
}

auto signed_area(const geom::Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

auto polygon_area(const geom::PolygonWithHoles &polygon) -> double {
  long double area = std::abs(signed_area(polygon.outer));
  for (const auto &hole : polygon.holes) {
    area -= std::abs(signed_area(hole));
  }
  return static_cast<double>(area);
}

auto total_polygon_area(std::span<const geom::PolygonWithHoles> polygons)
    -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += polygon_area(polygon);
  }
  return total;
}

auto compute_bounds(const geom::PolygonWithHoles &polygon) -> geom::Box2 {
  geom::Box2 bounds{};
  bool initialized = false;

  const auto include_ring = [&bounds, &initialized](const geom::Ring &ring) {
    for (const auto &point : ring) {
      if (!initialized) {
        bounds.min = point;
        bounds.max = point;
        initialized = true;
        continue;
      }

      bounds.min.x = std::min(bounds.min.x, point.x);
      bounds.min.y = std::min(bounds.min.y, point.y);
      bounds.max.x = std::max(bounds.max.x, point.x);
      bounds.max.y = std::max(bounds.max.y, point.y);
    }
  };

  include_ring(polygon.outer);
  for (const auto &hole : polygon.holes) {
    include_ring(hole);
  }

  return bounds;
}

auto box_width(const geom::Box2 &box) -> double {
  return box.max.x - box.min.x;
}

auto box_height(const geom::Box2 &box) -> double {
  return box.max.y - box.min.y;
}

auto box_has_area(const geom::Box2 &box) -> bool {
  return box_width(box) > kCoordinateSnap && box_height(box) > kCoordinateSnap;
}

auto normalize_box(const geom::Point2 &first, const geom::Point2 &second)
    -> geom::Box2 {
  return {
      .min = {.x = std::min(first.x, second.x),
              .y = std::min(first.y, second.y)},
      .max = {.x = std::max(first.x, second.x),
              .y = std::max(first.y, second.y)},
  };
}

auto contains_box(const geom::Box2 &container, const geom::Box2 &candidate)
    -> bool {
  return candidate.min.x >= container.min.x - kCoordinateSnap &&
         candidate.min.y >= container.min.y - kCoordinateSnap &&
         candidate.max.x <= container.max.x + kCoordinateSnap &&
         candidate.max.y <= container.max.y + kCoordinateSnap;
}

auto expand_box(const geom::Box2 &box, double clearance) -> geom::Box2 {
  if (clearance <= 0.0) {
    return box;
  }

  return {
      .min = {.x = box.min.x - clearance, .y = box.min.y - clearance},
      .max = {.x = box.max.x + clearance, .y = box.max.y + clearance},
  };
}

auto boxes_overlap(const geom::Box2 &lhs, const geom::Box2 &rhs) -> bool {
  return !(lhs.max.x < rhs.min.x - kCoordinateSnap ||
           rhs.max.x < lhs.min.x - kCoordinateSnap ||
           lhs.max.y < rhs.min.y - kCoordinateSnap ||
           rhs.max.y < lhs.min.y - kCoordinateSnap);
}

auto intervals_overlap_interior(double lhs_min, double lhs_max, double rhs_min,
                                double rhs_max) -> bool {
  return lhs_max > rhs_min + kCoordinateSnap &&
         rhs_max > lhs_min + kCoordinateSnap;
}

auto boxes_overlap_interior(const geom::Box2 &lhs, const geom::Box2 &rhs)
    -> bool {
  return intervals_overlap_interior(lhs.min.x, lhs.max.x, rhs.min.x,
                                    rhs.max.x) &&
         intervals_overlap_interior(lhs.min.y, lhs.max.y, rhs.min.y, rhs.max.y);
}

auto interrupted(const InterruptionProbe &interruption_requested) -> bool {
  return interruption_requested && interruption_requested();
}

auto as_polygon_with_holes(const place::BedExclusionZone &zone)
    -> geom::PolygonWithHoles {
  return {.outer = zone.region.outer};
}

auto overlaps_exclusion_zone(const geom::PolygonWithHoles &piece,
                             const geom::Box2 &piece_bounds,
                             const place::BedExclusionZone &zone) -> bool {
  const auto zone_polygon = as_polygon_with_holes(zone);
  if (piece.outer.empty() || zone_polygon.outer.empty() ||
      !boxes_overlap(piece_bounds, compute_bounds(zone_polygon))) {
    return false;
  }

  const auto remaining = poly::difference_polygons(piece, zone_polygon);
  return total_polygon_area(remaining) + kAreaEpsilon < polygon_area(piece);
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
  return piece.allowed_bin_ids.empty() ||
         std::find(piece.allowed_bin_ids.begin(), piece.allowed_bin_ids.end(),
                   bin_id) != piece.allowed_bin_ids.end();
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
