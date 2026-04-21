#include "polygon_ops/boolean_ops.hpp"

#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

#include "geometry/normalize.hpp"
#include "polygon_ops/simplify.hpp"

namespace shiny::nesting::poly {
namespace {

namespace bg = boost::geometry;

using BgPoint = bg::model::d2::point_xy<double>;
using BgRing = bg::model::ring<BgPoint, false, false>;
using BgPolygon = bg::model::polygon<BgPoint, false, false>;

[[nodiscard]] auto to_bg_point(const geom::Point2 &point) -> BgPoint {
  return {point.x, point.y};
}

[[nodiscard]] auto from_bg_point(const BgPoint &point) -> geom::Point2 {
  return {bg::get<0>(point), bg::get<1>(point)};
}

[[nodiscard]] auto to_bg_ring(const geom::Ring &ring) -> BgRing {
  BgRing result;
  result.reserve(ring.size());
  for (const auto &point : ring) {
    result.push_back(to_bg_point(point));
  }
  return result;
}

[[nodiscard]] auto from_bg_ring(const BgRing &ring) -> geom::Ring {
  geom::Ring result;
  result.reserve(ring.size());
  for (const auto &point : ring) {
    result.push_back(from_bg_point(point));
  }
  return result;
}

[[nodiscard]] auto to_bg_polygon(const geom::PolygonWithHoles &polygon)
    -> BgPolygon {
  const auto normalized = geom::normalize_polygon(polygon);

  BgPolygon result;
  result.outer() = to_bg_ring(normalized.outer);
  result.inners().reserve(normalized.holes.size());
  for (const auto &hole : normalized.holes) {
    result.inners().push_back(to_bg_ring(hole));
  }
  bg::correct(result);
  return result;
}

[[nodiscard]] auto from_bg_polygon(const BgPolygon &polygon)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles result{};
  result.outer = from_bg_ring(polygon.outer());
  result.holes.reserve(polygon.inners().size());
  for (const auto &hole : polygon.inners()) {
    result.holes.push_back(from_bg_ring(hole));
  }
  return simplify_polygon(result);
}

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  if (lhs.outer.empty()) {
    return !rhs.outer.empty();
  }
  if (rhs.outer.empty()) {
    return false;
  }

  if (lhs.outer.front().x != rhs.outer.front().x) {
    return lhs.outer.front().x < rhs.outer.front().x;
  }
  if (lhs.outer.front().y != rhs.outer.front().y) {
    return lhs.outer.front().y < rhs.outer.front().y;
  }
  if (lhs.outer.size() != rhs.outer.size()) {
    return lhs.outer.size() < rhs.outer.size();
  }
  return lhs.holes.size() < rhs.holes.size();
}

[[nodiscard]] auto normalize_output(const std::vector<BgPolygon> &output)
    -> std::vector<geom::PolygonWithHoles> {
  std::vector<geom::PolygonWithHoles> normalized_output;
  normalized_output.reserve(output.size());
  for (const auto &polygon : output) {
    normalized_output.push_back(from_bg_polygon(polygon));
  }

  std::sort(normalized_output.begin(), normalized_output.end(), polygon_less);
  return normalized_output;
}

} // namespace

auto union_polygons(const geom::PolygonWithHoles &lhs,
                    const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer.empty() && rhs.outer.empty()) {
    return {};
  }
  if (lhs.outer.empty()) {
    return {geom::normalize_polygon(rhs)};
  }
  if (rhs.outer.empty()) {
    return {geom::normalize_polygon(lhs)};
  }

  std::vector<BgPolygon> output;
  bg::union_(to_bg_polygon(lhs), to_bg_polygon(rhs), output);
  return normalize_output(output);
}

auto intersection_polygons(const geom::PolygonWithHoles &lhs,
                           const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer.empty() || rhs.outer.empty()) {
    return {};
  }

  std::vector<BgPolygon> output;
  bg::intersection(to_bg_polygon(lhs), to_bg_polygon(rhs), output);
  return normalize_output(output);
}

auto difference_polygons(const geom::PolygonWithHoles &lhs,
                         const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer.empty()) {
    return {};
  }
  if (rhs.outer.empty()) {
    return {geom::normalize_polygon(lhs)};
  }

  std::vector<BgPolygon> output;
  bg::difference(to_bg_polygon(lhs), to_bg_polygon(rhs), output);
  return normalize_output(output);
}

auto polygon_distance(const geom::PolygonWithHoles &lhs,
                      const geom::PolygonWithHoles &rhs) -> double {
  if (lhs.outer.empty() || rhs.outer.empty()) {
    return 0.0;
  }

  return bg::distance(to_bg_polygon(lhs), to_bg_polygon(rhs));
}

auto buffer_polygon(const geom::PolygonWithHoles &polygon, double distance)
    -> std::vector<geom::PolygonWithHoles> {
  if (polygon.outer.empty() || distance == 0.0) {
    if (polygon.outer.empty()) {
      return {};
    }
    return {geom::normalize_polygon(polygon)};
  }

  using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

  bg::strategy::buffer::distance_symmetric<double> dist_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_square point_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  BgMultiPolygon buffered;
  bg::buffer(to_bg_polygon(polygon), buffered, dist_strategy, side_strategy,
             join_strategy, end_strategy, point_strategy);

  std::vector<geom::PolygonWithHoles> result;
  result.reserve(buffered.size());
  for (const auto &poly : buffered) {
    auto converted = from_bg_polygon(poly);
    if (!converted.outer.empty()) {
      result.push_back(std::move(converted));
    }
  }

  std::sort(result.begin(), result.end(), polygon_less);
  return result;
}

auto subtract_region_set(std::vector<geom::PolygonWithHoles> &regions,
                         const geom::PolygonWithHoles &obstacle) -> void {
  std::vector<geom::PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    const auto difference = difference_polygons(region, obstacle);
    next_regions.insert(next_regions.end(), difference.begin(), difference.end());
  }
  regions = std::move(next_regions);
}

} // namespace shiny::nesting::poly
