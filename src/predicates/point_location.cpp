#include "predicates/point_location.hpp"

#include <cmath>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

namespace shiny::nesting::pred {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

[[nodiscard]] auto to_cgal_point(const geom::Point2 &point) -> Kernel::Point_2 {
  return {point.x(), point.y()};
}

[[nodiscard]] auto ring_to_cgal_points(std::span<const geom::Point2> ring)
    -> std::vector<Kernel::Point_2> {
  std::vector<Kernel::Point_2> result;
  result.reserve(ring.size());
  for (const auto &point : ring) {
    result.push_back(to_cgal_point(point));
  }
  return result;
}

[[nodiscard]] auto compute_parametric_t(const geom::Point2 &point,
                                        const geom::Segment2 &segment)
    -> double {
  const double dx = segment.end.x() - segment.start.x();
  const double dy = segment.end.y() - segment.start.y();

  if (dx == 0.0 && dy == 0.0) {
    return 0.0;
  }

  if (std::abs(dx) >= std::abs(dy)) {
    return (point.x() - segment.start.x()) / dx;
  }

  return (point.y() - segment.start.y()) / dy;
}

} // namespace

auto locate_point_on_segment(const geom::Point2 &point,
                             const geom::Segment2 &segment)
    -> PointOnSegmentResult {
  if (segment.start == segment.end) {
    PointOnSegmentResult result{};
    if (point == segment.start) {
      result.relation = BoundaryRelation::on_vertex;
    }
    return result;
  }

  const auto cgal_start = to_cgal_point(segment.start);
  const auto cgal_end = to_cgal_point(segment.end);
  const auto cgal_point = to_cgal_point(point);

  if (CGAL::orientation(cgal_start, cgal_end, cgal_point) != CGAL::COLLINEAR) {
    return {};
  }

  if (!CGAL::collinear_are_ordered_along_line(cgal_start, cgal_point,
                                              cgal_end)) {
    return {};
  }

  PointOnSegmentResult result{};
  result.parametric_t = compute_parametric_t(point, segment);
  result.relation = point == segment.start || point == segment.end
                        ? BoundaryRelation::on_vertex
                        : BoundaryRelation::on_edge_interior;
  return result;
}

auto locate_point_in_ring(const geom::Point2 &point,
                          std::span<const geom::Point2> ring)
    -> RingLocationResult {
  RingLocationResult result{};

  if (ring.empty()) {
    return result;
  }

  const auto edge_count = ring.size();
  for (std::size_t index = 0; index < edge_count; ++index) {
    const std::size_t next_index = (index + 1U) % edge_count;
    const geom::Segment2 edge{ring[index], ring[next_index]};
    const auto on_segment = locate_point_on_segment(point, edge);

    if (on_segment.relation == BoundaryRelation::off_boundary) {
      continue;
    }

    result.location = PointLocation::boundary;
    result.edge_index = static_cast<std::int32_t>(index);
    if (on_segment.relation == BoundaryRelation::on_vertex) {
      result.vertex_index = point == ring[index]
                                ? static_cast<std::int32_t>(index)
                                : static_cast<std::int32_t>(next_index);
    }
    return result;
  }

  if (ring.size() < 3U) {
    return result;
  }

  const auto cgal_ring = ring_to_cgal_points(ring);
  const auto bounded_side = CGAL::bounded_side_2(
      cgal_ring.begin(), cgal_ring.end(), to_cgal_point(point), Kernel{});

  if (bounded_side == CGAL::ON_BOUNDED_SIDE) {
    result.location = PointLocation::interior;
  } else if (bounded_side == CGAL::ON_BOUNDARY) {
    result.location = PointLocation::boundary;
  } else {
    result.location = PointLocation::exterior;
  }
  return result;
}

auto locate_point_in_polygon(const geom::Point2 &point,
                             const geom::PolygonWithHoles &polygon)
    -> PolygonLocationResult {
  PolygonLocationResult result{};
  const auto outer_location = locate_point_in_ring(point, polygon.outer());

  result.location = outer_location.location;
  result.boundary_edge_index = outer_location.edge_index;
  result.boundary_vertex_index = outer_location.vertex_index;

  if (outer_location.location != PointLocation::interior) {
    return result;
  }

  result.boundary_edge_index = -1;
  result.boundary_vertex_index = -1;

  for (std::size_t hole_index = 0; hole_index < polygon.holes().size();
       ++hole_index) {
    const auto hole_location = locate_point_in_ring(
        point, std::span<const geom::Point2>(polygon.holes()[hole_index]));

    if (hole_location.location == PointLocation::exterior) {
      continue;
    }

    result.location = hole_location.location == PointLocation::interior
                          ? PointLocation::exterior
                          : PointLocation::boundary;
    result.inside_hole = true;
    result.hole_index = static_cast<std::int32_t>(hole_index);
    result.boundary_edge_index = hole_location.edge_index;
    result.boundary_vertex_index = hole_location.vertex_index;
    return result;
  }

  result.location = PointLocation::interior;
  return result;
}

} // namespace shiny::nesting::pred