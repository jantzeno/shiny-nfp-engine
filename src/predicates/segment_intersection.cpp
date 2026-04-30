#include "predicates/segment_intersection.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry/detail/point_compare.hpp"
#include "predicates/orientation.hpp"

namespace shiny::nesting::pred {
namespace {

constexpr double kIntersectionEpsilon = 1e-12;

[[nodiscard]] auto cross(const geom::Vector2 &lhs, const geom::Vector2 &rhs)
    -> double {
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

[[nodiscard]] auto subtract(const geom::Point2 &lhs, const geom::Point2 &rhs)
    -> geom::Vector2 {
  return {lhs.x() - rhs.x(), lhs.y() - rhs.y()};
}

[[nodiscard]] auto point_from_parametric(const geom::Segment2 &segment,
                                         const double t) -> geom::Point2 {
  return {detail::canonicalize_coordinate(
              segment.start.x() + (segment.end.x() - segment.start.x()) * t),
          detail::canonicalize_coordinate(
              segment.start.y() + (segment.end.y() - segment.start.y()) * t)};
}

[[nodiscard]] auto point_is_segment_vertex(const geom::Point2 &point,
                                           const geom::Segment2 &segment)
    -> bool {
  return detail::near_same_point(point, segment.start) ||
         detail::near_same_point(point, segment.end);
}

void normalize_contact_points(std::vector<geom::Point2> &points) {
  std::sort(points.begin(), points.end(), detail::point_less);
  points.erase(
      std::unique(points.begin(), points.end(), detail::near_same_point),
      points.end());
}

[[nodiscard]] auto orientation_tolerance(const geom::Vector2 &lhs,
                                         const geom::Vector2 &rhs) -> double {
  const auto scale = std::max({1.0, std::abs(lhs.x()), std::abs(lhs.y()),
                               std::abs(rhs.x()), std::abs(rhs.y())});
  return kIntersectionEpsilon * scale * scale;
}

[[nodiscard]] auto is_parallel(const geom::Vector2 &lhs,
                               const geom::Vector2 &rhs) -> bool {
  return std::abs(cross(lhs, rhs)) <= orientation_tolerance(lhs, rhs);
}

[[nodiscard]] auto build_contact(const SegmentContactKind kind,
                                 const std::vector<geom::Point2> &points,
                                 const geom::Segment2 &lhs,
                                 const geom::Segment2 &rhs) -> SegmentContact {
  SegmentContact result{};
  result.kind = kind;
  result.point_count = static_cast<std::uint8_t>(points.size());

  for (std::size_t index = 0;
       index < points.size() && index < result.points.size(); ++index) {
    result.points[index] = points[index];
    result.a_vertex_contact =
        result.a_vertex_contact || point_is_segment_vertex(points[index], lhs);
    result.b_vertex_contact =
        result.b_vertex_contact || point_is_segment_vertex(points[index], rhs);
  }

  return result;
}

[[nodiscard]] auto build_intersection_contact(const geom::Point2 &point,
                                              const geom::Segment2 &lhs,
                                              const geom::Segment2 &rhs)
    -> SegmentContact {
  const bool a_vertex_contact = point_is_segment_vertex(point, lhs);
  const bool b_vertex_contact = point_is_segment_vertex(point, rhs);
  const auto kind = (a_vertex_contact || b_vertex_contact)
                        ? SegmentContactKind::endpoint_touch
                        : SegmentContactKind::proper_intersection;
  return build_contact(kind, {point}, lhs, rhs);
}

[[nodiscard]] auto overlap_from_collinear_segments(const geom::Segment2 &lhs,
                                                   const geom::Segment2 &rhs)
    -> SegmentContact {
  const auto direction = subtract(lhs.end, lhs.start);
  const bool use_x = std::abs(direction.x()) >= std::abs(direction.y());
  const double lhs_extent = use_x ? direction.x() : direction.y();
  if (lhs_extent == 0.0) {
    if (detail::near_same_point(lhs.start, rhs.start) ||
        detail::near_same_point(lhs.start, rhs.end)) {
      return build_contact(SegmentContactKind::endpoint_touch, {lhs.start}, lhs,
                           rhs);
    }
    return build_contact(SegmentContactKind::parallel_disjoint, {}, lhs, rhs);
  }

  const auto project = [&](const geom::Point2 &point) {
    return use_x ? (point.x() - lhs.start.x()) / lhs_extent
                 : (point.y() - lhs.start.y()) / lhs_extent;
  };

  const auto first_t = project(rhs.start);
  const auto second_t = project(rhs.end);
  const auto overlap_start = std::max(0.0, std::min(first_t, second_t));
  const auto overlap_end = std::min(1.0, std::max(first_t, second_t));
  if (overlap_start > overlap_end + kIntersectionEpsilon) {
    return build_contact(SegmentContactKind::parallel_disjoint, {}, lhs, rhs);
  }

  std::vector<geom::Point2> overlap_points{
      point_from_parametric(lhs, overlap_start),
      point_from_parametric(lhs, overlap_end)};
  normalize_contact_points(overlap_points);
  if (overlap_points.empty()) {
    return build_contact(SegmentContactKind::parallel_disjoint, {}, lhs, rhs);
  }
  if (overlap_points.size() == 1U) {
    return build_contact(SegmentContactKind::endpoint_touch, overlap_points,
                         lhs, rhs);
  }
  return build_contact(SegmentContactKind::collinear_overlap, overlap_points,
                       lhs, rhs);
}

} // namespace

auto classify_segment_contact(const geom::Segment2 &lhs,
                              const geom::Segment2 &rhs) -> SegmentContact {
  const auto lhs_direction = subtract(lhs.end, lhs.start);
  const auto rhs_direction = subtract(rhs.end, rhs.start);
  const auto delta = subtract(rhs.start, lhs.start);
  const auto denominator = cross(lhs_direction, rhs_direction);

  if (is_parallel(lhs_direction, rhs_direction)) {
    if (std::abs(cross(delta, lhs_direction)) <=
        orientation_tolerance(delta, lhs_direction)) {
      return overlap_from_collinear_segments(lhs, rhs);
    }
    return build_contact(SegmentContactKind::parallel_disjoint, {}, lhs, rhs);
  }

  const auto lhs_t = cross(delta, rhs_direction) / denominator;
  const auto rhs_t = cross(delta, lhs_direction) / denominator;
  if (lhs_t < -kIntersectionEpsilon || lhs_t > 1.0 + kIntersectionEpsilon ||
      rhs_t < -kIntersectionEpsilon || rhs_t > 1.0 + kIntersectionEpsilon) {
    return build_contact(SegmentContactKind::disjoint, {}, lhs, rhs);
  }

  return build_intersection_contact(point_from_parametric(lhs, lhs_t), lhs,
                                    rhs);
}

} // namespace shiny::nesting::pred