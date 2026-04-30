#include "packing/irregular/core.hpp"

#include <algorithm>

#include "polygon_ops/boolean_ops.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack::detail {
namespace {

[[nodiscard]] auto primary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
  case place::PlacementStartCorner::bottom_right:
    return piece_bounds.min.y() - container_bounds.min.y();
  case place::PlacementStartCorner::top_left:
  case place::PlacementStartCorner::top_right:
    return container_bounds.max.y() - piece_bounds.max.y();
  }
  return piece_bounds.min.y() - container_bounds.min.y();
}

[[nodiscard]] auto secondary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
  case place::PlacementStartCorner::top_left:
    return piece_bounds.min.x() - container_bounds.min.x();
  case place::PlacementStartCorner::bottom_right:
  case place::PlacementStartCorner::top_right:
    return container_bounds.max.x() - piece_bounds.max.x();
  }
  return piece_bounds.min.x() - container_bounds.min.x();
}

[[nodiscard]] auto
translation_priority(const geom::Point2 &translation,
                     const place::PlacementStartCorner start_corner)
    -> std::pair<double, double> {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
    return {translation.y(), translation.x()};
  case place::PlacementStartCorner::bottom_right:
    return {translation.y(), -translation.x()};
  case place::PlacementStartCorner::top_left:
    return {-translation.y(), translation.x()};
  case place::PlacementStartCorner::top_right:
    return {-translation.y(), -translation.x()};
  }
  return {translation.y(), translation.x()};
}

[[nodiscard]] auto envelope_bounds_with_candidate(
    const WorkingBin &bin, const geom::Box2 &candidate_bounds) -> geom::Box2 {
  geom::Box2 envelope = candidate_bounds;
  for (const auto &bounds : bin.placement_bounds) {
    envelope.min.set_x(std::min(envelope.min.x(), bounds.min.x()));
    envelope.min.set_y(std::min(envelope.min.y(), bounds.min.y()));
    envelope.max.set_x(std::max(envelope.max.x(), bounds.max.x()));
    envelope.max.set_y(std::max(envelope.max.y(), bounds.max.y()));
  }
  return envelope;
}

[[nodiscard]] auto
candidate_utilization(const WorkingBin &bin,
                      const geom::PolygonWithHoles &piece_polygon,
                      const geom::Box2 &candidate_bounds) -> double {
  const auto envelope = envelope_bounds_with_candidate(bin, candidate_bounds);
  const auto envelope_area = std::max(0.0, geom::box_width(envelope)) *
                             std::max(0.0, geom::box_height(envelope));
  const auto occupied_area =
      bin.state.utilization.occupied_area + geom::polygon_area(piece_polygon);
  return envelope_area > kAreaEpsilon ? occupied_area / envelope_area : 0.0;
}

[[nodiscard]] auto candidate_strip_length(const WorkingBin &bin,
                                          const geom::Box2 &candidate_bounds)
    -> double {
  return geom::box_width(envelope_bounds_with_candidate(bin, candidate_bounds));
}

[[nodiscard]] auto fits_region(const geom::PolygonWithHoles &piece,
                               const geom::Box2 &piece_bbox,
                               const geom::PolygonWithHoles &region,
                               const geom::Box2 &region_bbox) -> bool {
  if (!geom::box_contains(region_bbox, piece_bbox)) {
    return false;
  }
  bool all_interior = true;
  for (const auto &vertex : piece.outer()) {
    const auto loc = pred::locate_point_in_polygon(vertex, region);
    if (loc.location == pred::PointLocation::exterior) {
      return false;
    }
    if (loc.location != pred::PointLocation::interior) {
      all_interior = false;
    }
  }
  if (all_interior && piece.holes().empty()) {
    return true;
  }
  const auto remainder = poly::difference_polygons(piece, region);
  return geom::polygon_area_sum(remainder) <= kAreaEpsilon;
}

} // namespace

auto better_candidate(const WorkingBin &bin,
                      const place::PlacementPolicy policy,
                      const CandidatePlacement &lhs,
                      const CandidatePlacement &rhs) -> bool {
  const auto container_bounds = geom::compute_bounds(bin.state.container);
  const auto lhs_primary = primary_edge_distance(container_bounds, lhs.bounds,
                                                 bin.state.start_corner);
  const auto rhs_primary = primary_edge_distance(container_bounds, rhs.bounds,
                                                 bin.state.start_corner);
  const auto lhs_secondary = secondary_edge_distance(
      container_bounds, lhs.bounds, bin.state.start_corner);
  const auto rhs_secondary = secondary_edge_distance(
      container_bounds, rhs.bounds, bin.state.start_corner);

  switch (policy) {
  case place::PlacementPolicy::bottom_left:
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::minimum_length:
    if (!almost_equal(lhs.score, rhs.score)) {
      return lhs.score < rhs.score;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::maximum_utilization:
    if (!almost_equal(lhs.score, rhs.score)) {
      return lhs.score > rhs.score;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  }

  if (lhs.inside_hole != rhs.inside_hole) {
    return lhs.inside_hole && !rhs.inside_hole;
  }
  if (lhs.placement.rotation_index.value !=
      rhs.placement.rotation_index.value) {
    return lhs.placement.rotation_index.value <
           rhs.placement.rotation_index.value;
  }
  const auto lhs_translation_priority =
      translation_priority(lhs.placement.translation, bin.state.start_corner);
  const auto rhs_translation_priority =
      translation_priority(rhs.placement.translation, bin.state.start_corner);
  if (!almost_equal(lhs_translation_priority.first,
                    rhs_translation_priority.first)) {
    return lhs_translation_priority.first < rhs_translation_priority.first;
  }
  if (!almost_equal(lhs_translation_priority.second,
                    rhs_translation_priority.second)) {
    return lhs_translation_priority.second < rhs_translation_priority.second;
  }
  return lhs.placement.translation.x() < rhs.placement.translation.x();
}

auto fits_any_region(const geom::PolygonWithHoles &piece,
                     const geom::Box2 &piece_bbox,
                     const std::span<const geom::PolygonWithHoles> regions,
                     const std::span<const geom::Box2> region_bboxes) -> bool {
  for (std::size_t index = 0; index < regions.size(); ++index) {
    if (fits_region(piece, piece_bbox, regions[index], region_bboxes[index])) {
      return true;
    }
  }
  return false;
}

auto fits_bin_direct(const geom::PolygonWithHoles &piece,
                     const geom::Box2 &piece_bbox, const WorkingBin &bin,
                     const ExecutionPolicy &execution) -> bool {
  const auto container_bounds = geom::compute_bounds(bin.state.container);
  if (!geom::box_contains(container_bounds, piece_bbox)) {
    return false;
  }
  if (geom::polygon_area_sum(poly::difference_polygons(
          piece, bin.state.container)) > kAreaEpsilon) {
    return false;
  }

  for (const auto &zone : bin.exclusion_regions) {
    if (geom::polygon_area_sum(poly::intersection_polygons(piece, zone)) >
        kAreaEpsilon) {
      return false;
    }
  }
  for (const auto &placement : bin.state.placements) {
    const auto obstacle = execution.enable_part_in_part_placement
                              ? placement.polygon
                              : fill_polygon_holes(placement.polygon);
    if (geom::polygon_area_sum(poly::intersection_polygons(piece, obstacle)) >
        kAreaEpsilon) {
      return false;
    }
  }
  return true;
}

auto hole_index_for_candidate(
    const geom::PolygonWithHoles &piece, const geom::Box2 &piece_bbox,
    const std::span<const geom::PolygonWithHoles> holes) -> std::int32_t {
  for (std::size_t index = 0; index < holes.size(); ++index) {
    const auto hole_bbox = geom::compute_bounds(holes[index]);
    if (fits_region(piece, piece_bbox, holes[index], hole_bbox)) {
      return static_cast<std::int32_t>(index);
    }
  }
  return -1;
}

auto respects_spacing(const geom::PolygonWithHoles &piece,
                      const WorkingBin &bin, const ExecutionPolicy &execution)
    -> bool {
  if (execution.part_spacing <= 0.0) {
    return true;
  }

  for (const auto &zone : bin.exclusion_regions) {
    if (poly::polygon_distance(piece, zone) + kDistanceEpsilon <
        execution.part_spacing) {
      return false;
    }
  }
  for (const auto &placement : bin.state.placements) {
    if (poly::polygon_distance(piece, placement.polygon) + kDistanceEpsilon <
        execution.part_spacing) {
      return false;
    }
  }
  return true;
}

auto compute_candidate_score(const WorkingBin &bin,
                             const ExecutionPolicy &execution,
                             const geom::PolygonWithHoles &piece_polygon,
                             const geom::Box2 &candidate_bounds) -> double {
  switch (execution.placement_policy) {
  case place::PlacementPolicy::bottom_left:
    return 0.0;
  case place::PlacementPolicy::minimum_length:
    return candidate_strip_length(bin, candidate_bounds);
  case place::PlacementPolicy::maximum_utilization:
    return candidate_utilization(bin, piece_polygon, candidate_bounds);
  }
  return 0.0;
}

} // namespace shiny::nesting::pack::detail
