#include "predicates/point_location.hpp"

#include <cmath>

#include "predicates/detail/point_compare.hpp"
#include "predicates/orientation.hpp"

namespace shiny::nesting::pred {
namespace {

constexpr double kBoundaryEpsilon = 1e-12;

[[nodiscard]] auto cross(const geom::Point2 &origin, const geom::Point2 &lhs,
                         const geom::Point2 &rhs) -> double {
  const auto lhs_x = lhs.x() - origin.x();
  const auto lhs_y = lhs.y() - origin.y();
  const auto rhs_x = rhs.x() - origin.x();
  const auto rhs_y = rhs.y() - origin.y();
  return lhs_x * rhs_y - lhs_y * rhs_x;
}

[[nodiscard]] auto boundary_tolerance(const geom::Segment2 &segment,
                                      const geom::Point2 &point) -> double {
  const auto dx = std::max({1.0, std::abs(segment.end.x() - segment.start.x()),
                            std::abs(point.x() - segment.start.x())});
  const auto dy = std::max({1.0, std::abs(segment.end.y() - segment.start.y()),
                            std::abs(point.y() - segment.start.y())});
  return kBoundaryEpsilon * dx * dy;
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

[[nodiscard]] auto point_in_ring_parity(const geom::Point2 &point,
                                        std::span<const geom::Point2> ring)
    -> bool {
  bool inside = false;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto &current = ring[index];
    const auto &next = ring[(index + 1U) % ring.size()];

    const bool straddles = (current.y() > point.y()) != (next.y() > point.y());
    if (!straddles) {
      continue;
    }

    const auto y_delta = next.y() - current.y();
    if (y_delta == 0.0) {
      continue;
    }

    const auto x_intersection = current.x() + (point.y() - current.y()) *
                                                  (next.x() - current.x()) /
                                                  y_delta;
    if (x_intersection > point.x()) {
      inside = !inside;
    }
  }

  return inside;
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

  if (std::abs(cross(segment.start, segment.end, point)) >
      boundary_tolerance(segment, point)) {
    return {};
  }

  const auto parametric_t = compute_parametric_t(point, segment);
  if (parametric_t < -kBoundaryEpsilon ||
      parametric_t > 1.0 + kBoundaryEpsilon) {
    return {};
  }

  PointOnSegmentResult result{};
  result.parametric_t = detail::canonicalize_coordinate(parametric_t);
  result.relation = detail::near_same_point(point, segment.start) ||
                            detail::near_same_point(point, segment.end)
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

  if (point_in_ring_parity(point, ring)) {
    result.location = PointLocation::interior;
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