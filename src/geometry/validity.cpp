#include "geometry/validity.hpp"

#include <algorithm>

#include <boost/geometry.hpp>
#include <cmath>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/types.hpp"
#include "predicates/segment_intersection.hpp"

namespace shiny::nesting::geom {
namespace detail {

namespace bg = boost::geometry;

constexpr double kAreaEpsilon = 1e-9;

[[nodiscard]] auto point_is_finite(const Point2 &point) -> bool {
  return std::isfinite(point.x()) && std::isfinite(point.y());
}

[[nodiscard]] auto edges_are_adjacent(const std::size_t lhs,
                                      const std::size_t rhs,
                                      const std::size_t edge_count) -> bool {
  return lhs == rhs || (lhs + 1U) % edge_count == rhs ||
         (rhs + 1U) % edge_count == lhs;
}

[[nodiscard]] auto ring_contact_is_invalid(const pred::SegmentContact &contact,
                                           const bool allow_shared_endpoint)
    -> bool {
  switch (contact.kind) {
  case pred::SegmentContactKind::disjoint:
  case pred::SegmentContactKind::parallel_disjoint:
    return false;
  case pred::SegmentContactKind::endpoint_touch:
    return !allow_shared_endpoint;
  case pred::SegmentContactKind::proper_intersection:
  case pred::SegmentContactKind::collinear_overlap:
    return true;
  }

  return true;
}

[[nodiscard]] auto ring_as_polygon(const Ring &ring) -> PolygonWithHoles {
  PolygonWithHoles polygon{ring};
  bg::correct(polygon);
  return polygon;
}

[[nodiscard]] auto validate_ring_simple(const Ring &ring,
                                        const std::int32_t ring_index)
    -> PolygonValidity {
  if (ring.size() < 3U) {
    return {.issue = PolygonValidityIssue::too_few_vertices,
            .ring_index = ring_index};
  }
  if (!std::all_of(ring.begin(), ring.end(), point_is_finite)) {
    return {.issue = PolygonValidityIssue::non_finite_coordinate,
            .ring_index = ring_index};
  }

  const auto edge_count = ring.size();
  for (std::size_t lhs = 0; lhs < edge_count; ++lhs) {
    const Segment2 lhs_edge{ring[lhs], ring[(lhs + 1U) % edge_count]};
    for (std::size_t rhs = lhs + 1U; rhs < edge_count; ++rhs) {
      if (edges_are_adjacent(lhs, rhs, edge_count)) {
        continue;
      }

      const Segment2 rhs_edge{ring[rhs], ring[(rhs + 1U) % edge_count]};
      const auto contact = pred::classify_segment_contact(lhs_edge, rhs_edge);
      if (ring_contact_is_invalid(contact, false)) {
        return {.issue = PolygonValidityIssue::self_intersection,
                .ring_index = ring_index,
                .edge_index = static_cast<std::int32_t>(lhs)};
      }
    }
  }

  bg::validity_failure_type failure = bg::no_failure;
  if (!bg::is_valid(ring_as_polygon(ring), failure)) {
    switch (failure) {
    case bg::failure_invalid_coordinate:
      return {.issue = PolygonValidityIssue::non_finite_coordinate,
              .ring_index = ring_index};
    case bg::failure_few_points:
      return {.issue = PolygonValidityIssue::too_few_vertices,
              .ring_index = ring_index};
    case bg::failure_wrong_topological_dimension:
      return {.issue = PolygonValidityIssue::zero_area,
              .ring_index = ring_index};
    case bg::failure_self_intersections:
    case bg::failure_spikes:
    case bg::failure_duplicate_points:
      return {.issue = PolygonValidityIssue::self_intersection,
              .ring_index = ring_index};
    default:
      break;
    }
  }

  if (std::abs(ring_signed_area(ring)) <= kAreaEpsilon) {
    return {.issue = PolygonValidityIssue::zero_area, .ring_index = ring_index};
  }

  return {};
}

[[nodiscard]] auto rings_intersect(const Ring &lhs, const Ring &rhs) -> bool {
  if (lhs.empty() || rhs.empty()) {
    return false;
  }

  for (std::size_t lhs_index = 0; lhs_index < lhs.size(); ++lhs_index) {
    const Segment2 lhs_edge{lhs[lhs_index], lhs[(lhs_index + 1U) % lhs.size()]};
    for (std::size_t rhs_index = 0; rhs_index < rhs.size(); ++rhs_index) {
      const Segment2 rhs_edge{rhs[rhs_index],
                              rhs[(rhs_index + 1U) % rhs.size()]};
      const auto contact = pred::classify_segment_contact(lhs_edge, rhs_edge);
      if (ring_contact_is_invalid(contact, false)) {
        return true;
      }
      if (contact.kind == pred::SegmentContactKind::endpoint_touch) {
        return true;
      }
    }
  }

  return false;
}

} // namespace detail

auto validate_polygon(const Polygon &polygon) -> PolygonValidity {
  const auto normalized = normalize_polygon(polygon);
  return detail::validate_ring_simple(normalized.outer(), -1);
}

auto validate_polygon(const PolygonWithHoles &polygon) -> PolygonValidity {
  const auto normalized = normalize_polygon(polygon);
  const auto outer_validity =
      detail::validate_ring_simple(normalized.outer(), -1);
  if (!outer_validity.is_valid()) {
    return outer_validity;
  }

  for (std::size_t hole_index = 0; hole_index < normalized.holes().size();
       ++hole_index) {
    const auto hole_validity = detail::validate_ring_simple(
        normalized.holes()[hole_index], static_cast<std::int32_t>(hole_index));
    if (!hole_validity.is_valid()) {
      return hole_validity;
    }

    if (!detail::bg::within(
            detail::ring_as_polygon(normalized.holes()[hole_index]),
            detail::ring_as_polygon(normalized.outer()))) {
      return {.issue = PolygonValidityIssue::hole_outside_outer,
              .ring_index = static_cast<std::int32_t>(hole_index)};
    }
  }

  for (std::size_t lhs = 0; lhs < normalized.holes().size(); ++lhs) {
    for (std::size_t rhs = lhs + 1U; rhs < normalized.holes().size(); ++rhs) {
      if (detail::bg::intersects(
              detail::ring_as_polygon(normalized.holes()[lhs]),
              detail::ring_as_polygon(normalized.holes()[rhs]))) {
        return {.issue = PolygonValidityIssue::hole_intersection,
                .ring_index = static_cast<std::int32_t>(lhs)};
      }
    }
  }

  return {};
}

} // namespace shiny::nesting::geom
