#include "polygon_ops/convex_hull.hpp"

#include <utility>

#include <boost/geometry.hpp>

#include "geometry/normalize.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::poly {
namespace {

namespace bg = boost::geometry;

using BgMultiPoint = bg::model::multi_point<geom::Point2>;

[[nodiscard]] auto collect_polygon_points(const geom::PolygonWithHoles &polygon)
    -> BgMultiPoint {
  const auto normalized = geom::normalize_polygon(polygon);

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

auto compute_convex_hull(std::span<const geom::Point2> points)
    -> geom::Polygon {
  if (points.empty()) {
    return {};
  }

  if (points.size() < 3U) {
    return geom::normalize_polygon(
        geom::Polygon{geom::Ring(points.begin(), points.end())});
  }

  BgMultiPoint multipoint;
  for (const auto &point : points) {
    bg::append(multipoint, point);
  }

  geom::PolygonWithHoles hull;
  bg::convex_hull(multipoint, hull);
  return geom::normalize_polygon(geom::Polygon{std::move(hull.outer())});
}

auto compute_convex_hull(const geom::Polygon &polygon) -> geom::Polygon {
  return compute_convex_hull(std::span<const geom::Point2>(
      polygon.outer().data(), polygon.outer().size()));
}

auto compute_convex_hull(const geom::PolygonWithHoles &polygon)
    -> geom::Polygon {
  const auto points = collect_polygon_points(polygon);
  if (points.empty()) {
    return {};
  }

  geom::PolygonWithHoles hull;
  bg::convex_hull(points, hull);
  return geom::normalize_polygon(geom::Polygon{std::move(hull.outer())});
}

} // namespace shiny::nesting::poly