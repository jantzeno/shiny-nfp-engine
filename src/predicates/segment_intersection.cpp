#include "predicates/segment_intersection.hpp"

#include <algorithm>
#include <variant>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersection_2.h>

#include "geometry/detail/point_compare.hpp"
#include "predicates/orientation.hpp"

namespace shiny::nesting::pred {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

[[nodiscard]] auto to_cgal_point(const geom::Point2 &point) -> Kernel::Point_2 {
  return {point.x(), point.y()};
}

[[nodiscard]] auto to_cgal_segment(const geom::Segment2 &segment)
    -> Kernel::Segment_2 {
  return {to_cgal_point(segment.start), to_cgal_point(segment.end)};
}

[[nodiscard]] auto from_cgal_point(const Kernel::Point_2 &point)
    -> geom::Point2 {
  return {detail::canonicalize_coordinate(CGAL::to_double(point.x())),
          detail::canonicalize_coordinate(CGAL::to_double(point.y()))};
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

[[nodiscard]] auto is_parallel(const geom::Segment2 &lhs,
                               const geom::Segment2 &rhs) -> bool {
  return orient({{0.0, 0.0},
                 {lhs.end.x() - lhs.start.x(), lhs.end.y() - lhs.start.y()},
                 {rhs.end.x() - rhs.start.x(), rhs.end.y() - rhs.start.y()}}) ==
         Orientation::collinear;
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

} // namespace

auto classify_segment_contact(const geom::Segment2 &lhs,
                              const geom::Segment2 &rhs) -> SegmentContact {
  const auto intersection =
      CGAL::intersection(to_cgal_segment(lhs), to_cgal_segment(rhs));

  if (intersection) {
    if (const auto *point = std::get_if<Kernel::Point_2>(&*intersection)) {
      return build_intersection_contact(from_cgal_point(*point), lhs, rhs);
    }

    const auto *segment = std::get_if<Kernel::Segment_2>(&*intersection);
    std::vector<geom::Point2> overlap_points{
        from_cgal_point(segment->source()),
        from_cgal_point(segment->target()),
    };
    normalize_contact_points(overlap_points);

    if (overlap_points.size() == 1U) {
      return build_contact(SegmentContactKind::endpoint_touch, overlap_points,
                           lhs, rhs);
    }

    return build_contact(SegmentContactKind::collinear_overlap, overlap_points,
                         lhs, rhs);
  }

  if (is_parallel(lhs, rhs)) {
    return build_contact(SegmentContactKind::parallel_disjoint, {}, lhs, rhs);
  }

  return build_contact(SegmentContactKind::disjoint, {}, lhs, rhs);
}

} // namespace shiny::nesting::pred