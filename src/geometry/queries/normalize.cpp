#include "geometry/queries/normalize.hpp"

#include <algorithm>

#include "predicates/classify.hpp"

namespace shiny::nesting::geom {
namespace {

enum class DesiredWinding {
  clockwise,
  counterclockwise,
};

[[nodiscard]] auto signed_area(const Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area +=
        static_cast<long double>(ring[index].x()) * ring[next_index].y() -
        static_cast<long double>(ring[next_index].x()) * ring[index].y();
  }

  return twice_area / 2.0L;
}

void rotate_to_lexicographic_min(Ring &ring) {
  if (ring.empty()) {
    return;
  }

  const auto min_index = pred::lexicographic_min_vertex_index(ring);
  std::rotate(ring.begin(),
              ring.begin() + static_cast<std::ptrdiff_t>(min_index),
              ring.end());
}

[[nodiscard]] auto remove_duplicate_closing_vertex(Ring ring) -> Ring {
  if (ring.size() > 1U && ring.front() == ring.back()) {
    ring.pop_back();
  }
  return ring;
}

[[nodiscard]] auto remove_consecutive_duplicates(Ring ring) -> Ring {
  Ring cleaned;
  cleaned.reserve(ring.size());

  for (const auto &point : ring) {
    if (!cleaned.empty() && cleaned.back() == point) {
      continue;
    }
    cleaned.push_back(point);
  }

  if (cleaned.size() > 1U && cleaned.front() == cleaned.back()) {
    cleaned.pop_back();
  }

  return cleaned;
}

void enforce_winding(Ring &ring, DesiredWinding desired_winding) {
  if (ring.size() < 3U) {
    return;
  }

  const auto area = signed_area(ring);
  if (area == 0.0L) {
    return;
  }

  const bool is_counterclockwise = area > 0.0L;
  const bool should_be_counterclockwise =
      desired_winding == DesiredWinding::counterclockwise;

  if (is_counterclockwise != should_be_counterclockwise) {
    std::reverse(ring.begin(), ring.end());
  }
}

[[nodiscard]] auto normalize_ring(Ring ring, DesiredWinding desired_winding)
    -> Ring {
  ring = remove_duplicate_closing_vertex(std::move(ring));
  ring = remove_consecutive_duplicates(std::move(ring));
  enforce_winding(ring, desired_winding);
  rotate_to_lexicographic_min(ring);
  return ring;
}

} // namespace

auto normalize_polygon(const Polygon &polygon) -> Polygon {
  return Polygon{
      normalize_ring(polygon.outer(), DesiredWinding::counterclockwise)};
}

auto normalize_polygon(const PolygonWithHoles &polygon) -> PolygonWithHoles {
  PolygonWithHoles normalized{};
  normalized.outer() =
      normalize_ring(polygon.outer(), DesiredWinding::counterclockwise);
  normalized.holes().reserve(polygon.holes().size());

  for (const auto &hole : polygon.holes()) {
    normalized.holes().push_back(
        normalize_ring(hole, DesiredWinding::clockwise));
  }

  return normalized;
}

} // namespace shiny::nesting::geom