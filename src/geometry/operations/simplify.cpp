#include "geometry/operations/simplify.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/types.hpp"
#include "geometry/vector_ops.hpp"
#include "predicates/orientation.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::geom {
namespace detail {

[[nodiscard]] auto distance_to_segment(const Point2 &point,
                                       const Segment2 &segment) -> double {
  const auto edge = vector_between(segment.start, segment.end);
  const auto length_squared = dot(edge, edge);
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return point_distance(point, segment.start);
  }
  return point_to_segment_distance(point, segment);
}

void simplify_open_chain_douglas_peucker(std::span<const Point2> chain,
                                         double epsilon,
                                         std::vector<Point2> &output) {
  if (chain.empty()) {
    return;
  }
  if (chain.size() <= 2U) {
    output.insert(output.end(), chain.begin(), chain.end());
    return;
  }

  const Segment2 baseline{.start = chain.front(), .end = chain.back()};
  double max_distance = -1.0;
  std::size_t split_index = 0U;
  for (std::size_t index = 1U; index + 1U < chain.size(); ++index) {
    const double distance = distance_to_segment(chain[index], baseline);
    if (distance > max_distance) {
      max_distance = distance;
      split_index = index;
    }
  }

  if (max_distance > epsilon) {
    simplify_open_chain_douglas_peucker(chain.subspan(0U, split_index + 1U),
                                        epsilon, output);
    output.pop_back();
    simplify_open_chain_douglas_peucker(chain.subspan(split_index), epsilon,
                                        output);
    return;
  }

  output.push_back(chain.front());
  output.push_back(chain.back());
}

[[nodiscard]] auto simplify_normalized_ring(const Ring &ring) -> Ring {
  if (ring.size() < 4U) {
    return ring;
  }

  Ring simplified = ring;

  while (simplified.size() >= 4U) {
    std::vector<bool> remove_vertex(simplified.size(), false);
    std::size_t removed_count = 0;

    for (std::size_t index = 0; index < simplified.size(); ++index) {
      const auto prev_index =
          (index + simplified.size() - 1U) % simplified.size();
      const auto next_index = (index + 1U) % simplified.size();

      const auto orientation = pred::orient(
          {simplified[prev_index], simplified[index], simplified[next_index]});
      if (orientation != pred::Orientation::collinear) {
        continue;
      }

      const auto on_segment = pred::locate_point_on_segment(
          simplified[index],
          Segment2{simplified[prev_index], simplified[next_index]});
      if (on_segment.relation == pred::BoundaryRelation::off_boundary) {
        continue;
      }

      remove_vertex[index] = true;
      ++removed_count;
    }

    if (removed_count == 0U || simplified.size() - removed_count < 3U) {
      break;
    }

    Ring next_ring;
    next_ring.reserve(simplified.size() - removed_count);
    for (std::size_t index = 0; index < simplified.size(); ++index) {
      if (!remove_vertex[index]) {
        next_ring.push_back(simplified[index]);
      }
    }

    simplified = std::move(next_ring);
  }

  return simplified;
}

[[nodiscard]] auto simplify_ring_douglas_peucker(const Ring &ring,
                                                 double epsilon) -> Ring {
  if (ring.size() <= 3U || epsilon <= 0.0) {
    return simplify_normalized_ring(ring);
  }

  Ring closed_ring = ring;
  closed_ring.push_back(ring.front());

  Ring simplified;
  simplified.reserve(closed_ring.size());
  simplify_open_chain_douglas_peucker(closed_ring, epsilon, simplified);

  if (!simplified.empty() && simplified.front() == simplified.back()) {
    simplified.pop_back();
  }

  if (simplified.size() < 3U) {
    return simplify_normalized_ring(ring);
  }

  return simplify_normalized_ring(simplified);
}

} // namespace detail

auto simplify_collinear_ring(std::span<const Point2> ring) -> Ring {
  const auto normalized =
      normalize_polygon(Polygon(Ring(ring.begin(), ring.end())));
  const auto simplified = detail::simplify_normalized_ring(normalized.outer());
  return normalize_polygon(Polygon(simplified)).outer();
}

auto simplify_polygon(const Polygon &polygon) -> Polygon {
  const auto normalized = normalize_polygon(polygon);
  return normalize_polygon(
      Polygon(detail::simplify_normalized_ring(normalized.outer())));
}

auto simplify_polygon_douglas_peucker(const Polygon &polygon, double epsilon)
    -> Polygon {
  const auto normalized = normalize_polygon(polygon);
  return normalize_polygon(Polygon(
      detail::simplify_ring_douglas_peucker(normalized.outer(), epsilon)));
}

auto simplify_polygon(const PolygonWithHoles &polygon) -> PolygonWithHoles {
  const auto normalized = normalize_polygon(polygon);

  PolygonWithHoles simplified{};
  simplified.outer() = detail::simplify_normalized_ring(normalized.outer());
  simplified.holes().reserve(normalized.holes().size());

  for (const auto &hole : normalized.holes()) {
    simplified.holes().push_back(detail::simplify_normalized_ring(hole));
  }

  return normalize_polygon(simplified);
}

auto simplify_polygon_douglas_peucker(const PolygonWithHoles &polygon,
                                      double epsilon) -> PolygonWithHoles {
  const auto normalized = normalize_polygon(polygon);

  PolygonWithHoles simplified{};
  simplified.outer() =
      detail::simplify_ring_douglas_peucker(normalized.outer(), epsilon);
  simplified.holes().reserve(normalized.holes().size());

  for (const auto &hole : normalized.holes()) {
    simplified.holes().push_back(
        detail::simplify_ring_douglas_peucker(hole, epsilon));
  }

  return normalize_polygon(simplified);
}

} // namespace shiny::nesting::geom