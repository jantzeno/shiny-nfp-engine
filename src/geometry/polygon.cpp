#include "geometry/polygon.hpp"

#include <algorithm>
#include <cmath>

#include "geometry/normalize.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::geom {
namespace detail {

constexpr double kBoundsEpsilon = 1e-9;
constexpr double kHashScale = 1'000'000.0;

template <typename T> auto hash_value(std::uint64_t &hash, const T &value) -> void {
  runtime::hash::fnv1a_mix_value(hash, value);
}

[[nodiscard]] auto quantize_coordinate(const double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * kHashScale));
}

auto hash_ring(std::uint64_t &hash, std::span<const Point2> ring) -> void {
  const auto ring_size = static_cast<std::uint64_t>(ring.size());
  hash_value(hash, ring_size);
  for (const auto &point : ring) {
    const auto x = quantize_coordinate(point.x);
    const auto y = quantize_coordinate(point.y);
    hash_value(hash, x);
    hash_value(hash, y);
  }
}

} // namespace detail

auto ring_signed_area(std::span<const Point2> ring) -> double {
  if (ring.size() < 3U) {
    return 0.0;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }

  return static_cast<double>(twice_area / 2.0L);
}

auto polygon_area(const Polygon &polygon) -> double {
  return std::abs(ring_signed_area(polygon.outer));
}

auto polygon_area(const PolygonWithHoles &polygon) -> double {
  double area = std::abs(ring_signed_area(polygon.outer));
  for (const auto &hole : polygon.holes) {
    area -= std::abs(ring_signed_area(hole));
  }
  return area;
}

auto polygon_area_sum(std::span<const PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += polygon_area(polygon);
  }
  return total;
}

auto point_distance(const Point2 &lhs, const Point2 &rhs) -> double {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

auto squared_distance(const Point2 &lhs, const Point2 &rhs) -> double {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

auto compute_bounds(std::span<const Point2> ring) -> Box2 {
  Box2 bounds{};
  if (ring.empty()) {
    return bounds;
  }

  bounds.min = ring.front();
  bounds.max = ring.front();
  for (const auto &point : ring) {
    bounds.min.x = std::min(bounds.min.x, point.x);
    bounds.min.y = std::min(bounds.min.y, point.y);
    bounds.max.x = std::max(bounds.max.x, point.x);
    bounds.max.y = std::max(bounds.max.y, point.y);
  }
  return bounds;
}

auto compute_bounds(const Polygon &polygon) -> Box2 {
  return compute_bounds(polygon.outer);
}

auto compute_bounds(const PolygonWithHoles &polygon) -> Box2 {
  Box2 bounds = compute_bounds(polygon.outer);
  if (polygon.outer.empty()) {
    return bounds;
  }

  for (const auto &hole : polygon.holes) {
    const auto hole_bounds = compute_bounds(hole);
    bounds.min.x = std::min(bounds.min.x, hole_bounds.min.x);
    bounds.min.y = std::min(bounds.min.y, hole_bounds.min.y);
    bounds.max.x = std::max(bounds.max.x, hole_bounds.max.x);
    bounds.max.y = std::max(bounds.max.y, hole_bounds.max.y);
  }
  return bounds;
}

auto box_width(const Box2 &box) -> double { return box.max.x - box.min.x; }

auto box_height(const Box2 &box) -> double { return box.max.y - box.min.y; }

auto box_to_polygon(const Box2 &box) -> PolygonWithHoles {
  return {
      .outer = {
          {box.min.x, box.min.y},
          {box.max.x, box.min.y},
          {box.max.x, box.max.y},
          {box.min.x, box.max.y},
      },
  };
}

auto box_to_polygon_clamped(const Box2 &box, const double max_width)
    -> PolygonWithHoles {
  const auto width = std::max(0.0, std::min(max_width, box_width(box)));
  return {
      .outer = {
          {box.min.x, box.min.y},
          {box.min.x + width, box.min.y},
          {box.min.x + width, box.max.y},
          {box.min.x, box.max.y},
      },
  };
}

auto boxes_overlap(const Box2 &lhs, const Box2 &rhs) -> bool {
  return !(lhs.max.x < rhs.min.x - detail::kBoundsEpsilon ||
           rhs.max.x < lhs.min.x - detail::kBoundsEpsilon ||
           lhs.max.y < rhs.min.y - detail::kBoundsEpsilon ||
           rhs.max.y < lhs.min.y - detail::kBoundsEpsilon);
}

auto box_contains(const Box2 &container, const Box2 &candidate) -> bool {
  return candidate.min.x >= container.min.x - detail::kBoundsEpsilon &&
         candidate.min.y >= container.min.y - detail::kBoundsEpsilon &&
         candidate.max.x <= container.max.x + detail::kBoundsEpsilon &&
         candidate.max.y <= container.max.y + detail::kBoundsEpsilon;
}

auto polygon_revision(const Polygon &polygon) -> std::uint64_t {
  const auto normalized = normalize_polygon(polygon);
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  detail::hash_ring(hash, normalized.outer);
  return hash;
}

auto polygon_revision(const PolygonWithHoles &polygon) -> std::uint64_t {
  const auto normalized = normalize_polygon(polygon);
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  detail::hash_ring(hash, normalized.outer);
  const auto hole_count = static_cast<std::uint64_t>(normalized.holes.size());
  detail::hash_value(hash, hole_count);
  for (const auto &hole : normalized.holes) {
    detail::hash_ring(hash, hole);
  }
  return hash;
}

} // namespace shiny::nesting::geom
