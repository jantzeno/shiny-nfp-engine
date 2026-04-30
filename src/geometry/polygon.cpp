#include "geometry/polygon.hpp"

#include <algorithm>
#include <boost/geometry.hpp>
#include <cmath>

#include "geometry/queries/normalize.hpp"
#include "runtime/hash.hpp"

namespace shiny::nesting::geom {
namespace detail {

namespace bg = boost::geometry;

constexpr double kBoundsEpsilon = 1e-9;
constexpr double kHashScale = 1'000'000.0;

template <typename T>
auto hash_value(std::uint64_t &hash, const T &value) -> void {
  runtime::hash::fnv1a_mix_value(hash, value);
}

[[nodiscard]] auto quantize_coordinate(const double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * kHashScale));
}

auto hash_ring(std::uint64_t &hash, std::span<const Point2> ring) -> void {
  const auto ring_size = static_cast<std::uint64_t>(ring.size());
  hash_value(hash, ring_size);
  for (const auto &point : ring) {
    const auto x = quantize_coordinate(point.x());
    const auto y = quantize_coordinate(point.y());
    hash_value(hash, x);
    hash_value(hash, y);
  }
}

[[nodiscard]] auto expand_box(const Box2 &box, const double padding) -> Box2 {
  return Box2{Point2{box.min.x() - padding, box.min.y() - padding},
              Point2{box.max.x() + padding, box.max.y() + padding}};
}

} // namespace detail

auto ring_signed_area(std::span<const Point2> ring) -> double {
  if (ring.size() < 3U) {
    return 0.0;
  }

  const Ring boost_ring(ring.begin(), ring.end());
  return detail::bg::area(boost_ring);
}

auto polygon_area(const Polygon &polygon) -> double {
  return polygon_area(PolygonWithHoles{polygon.outer()});
}

auto polygon_area(const PolygonWithHoles &polygon) -> double {
  if (polygon.outer().empty()) {
    return 0.0;
  }

  auto corrected = normalize_polygon(polygon);
  detail::bg::correct(corrected);
  return std::abs(detail::bg::area(corrected));
}

auto polygon_area_sum(std::span<const PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += polygon_area(polygon);
  }
  return total;
}

auto point_distance(const Point2 &lhs, const Point2 &rhs) -> double {
  return detail::bg::distance(lhs, rhs);
}

auto squared_distance(const Point2 &lhs, const Point2 &rhs) -> double {
  return detail::bg::comparable_distance(lhs, rhs);
}

auto compute_bounds(std::span<const Point2> ring) -> Box2 {
  if (ring.empty()) {
    return {};
  }

  const Ring boost_ring(ring.begin(), ring.end());
  Box2 bounds{};
  detail::bg::envelope(boost_ring, bounds);
  return bounds;
}

auto compute_bounds(const Polygon &polygon) -> Box2 {
  return compute_bounds(PolygonWithHoles{polygon.outer()});
}

auto compute_bounds(const PolygonWithHoles &polygon) -> Box2 {
  if (polygon.outer().empty()) {
    return {};
  }

  Box2 bounds{};
  detail::bg::envelope(polygon, bounds);
  return bounds;
}

auto polygon_is_convex(const Polygon &polygon) -> bool {
  if (polygon.outer().empty()) {
    return true;
  }

  const auto normalized = normalize_polygon(polygon);
  return detail::bg::is_convex(normalized.outer());
}

auto box_width(const Box2 &box) -> double { return box.max.x() - box.min.x(); }

auto box_height(const Box2 &box) -> double { return box.max.y() - box.min.y(); }

auto box_to_polygon(const Box2 &box) -> PolygonWithHoles {
  return PolygonWithHoles{Ring{{box.min.x(), box.min.y()},
                               {box.max.x(), box.min.y()},
                               {box.max.x(), box.max.y()},
                               {box.min.x(), box.max.y()}}};
}

auto box_to_polygon_clamped(const Box2 &box, const double max_width)
    -> PolygonWithHoles {
  const auto width = std::max(0.0, std::min(max_width, box_width(box)));
  return PolygonWithHoles{Ring{{box.min.x(), box.min.y()},
                               {box.min.x() + width, box.min.y()},
                               {box.min.x() + width, box.max.y()},
                               {box.min.x(), box.max.y()}}};
}

auto boxes_overlap(const Box2 &lhs, const Box2 &rhs) -> bool {
  const auto lhs_padded = detail::expand_box(lhs, detail::kBoundsEpsilon * 0.5);
  const auto rhs_padded = detail::expand_box(rhs, detail::kBoundsEpsilon * 0.5);
  return detail::bg::intersects(lhs_padded, rhs_padded);
}

auto box_contains(const Box2 &container, const Box2 &candidate) -> bool {
  return detail::bg::covered_by(
      candidate, detail::expand_box(container, detail::kBoundsEpsilon));
}

auto polygon_revision(const Polygon &polygon) -> std::uint64_t {
  const auto normalized = normalize_polygon(polygon);
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  detail::hash_ring(hash, normalized.outer());
  return hash;
}

auto polygon_revision(const PolygonWithHoles &polygon) -> std::uint64_t {
  const auto normalized = normalize_polygon(polygon);
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  detail::hash_ring(hash, normalized.outer());
  const auto hole_count = static_cast<std::uint64_t>(normalized.holes().size());
  detail::hash_value(hash, hole_count);
  for (const auto &hole : normalized.holes()) {
    detail::hash_ring(hash, hole);
  }
  return hash;
}

} // namespace shiny::nesting::geom
