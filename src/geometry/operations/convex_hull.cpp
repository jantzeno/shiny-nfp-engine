#include "geometry/operations/convex_hull.hpp"

#include <utility>

#include <boost/geometry.hpp>

#include "geometry/queries/normalize.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::geom {
namespace {

namespace bg = boost::geometry;

using BgMultiPoint = bg::model::multi_point<Point2>;

[[nodiscard]] auto collect_polygon_points(const PolygonWithHoles &polygon)
    -> BgMultiPoint {
  const auto normalized = normalize_polygon(polygon);

  BgMultiPoint points;
  for (const auto &point : normalized.outer()) {
    bg::append(points, point);
  }
  for (const auto &hole : normalized.holes()) {
    for (const auto &point : hole) {
      bg::append(points, point);
    }
  }

  return points;
}

} // namespace

auto compute_convex_hull(std::span<const Point2> points) -> Polygon {
  if (points.empty()) {
    return {};
  }

  if (points.size() < 3U) {
    return normalize_polygon(Polygon{Ring(points.begin(), points.end())});
  }

  BgMultiPoint multipoint;
  for (const auto &point : points) {
    bg::append(multipoint, point);
  }

  PolygonWithHoles hull;
  bg::convex_hull(multipoint, hull);
  return normalize_polygon(Polygon{std::move(hull.outer())});
}

auto compute_convex_hull(const Polygon &polygon) -> Polygon {
  return compute_convex_hull(
      std::span<const Point2>(polygon.outer().data(), polygon.outer().size()));
}

auto compute_convex_hull(const PolygonWithHoles &polygon) -> Polygon {
  const auto points = collect_polygon_points(polygon);
  if (points.empty()) {
    return {};
  }

  PolygonWithHoles hull;
  bg::convex_hull(points, hull);
  return normalize_polygon(Polygon{std::move(hull.outer())});
}

} // namespace shiny::nesting::geom