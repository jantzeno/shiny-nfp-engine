#include "polygon_ops/convex_hull.hpp"

#include <algorithm>

#include <boost/geometry.hpp>

#include "geometry/normalize.hpp"

namespace shiny::nfp::poly {
namespace {

namespace bg = boost::geometry;

using BgPoint = bg::model::d2::point_xy<double>;
using BgMultiPoint = bg::model::multi_point<BgPoint>;
using BgRing = bg::model::ring<BgPoint, false, false>;
using BgPolygon = bg::model::polygon<BgPoint, false, false>;

[[nodiscard]] auto to_bg_point(const geom::Point2 &point) -> BgPoint {
  return {point.x, point.y};
}

[[nodiscard]] auto from_bg_point(const BgPoint &point) -> geom::Point2 {
  return {bg::get<0>(point), bg::get<1>(point)};
}

[[nodiscard]] auto from_bg_ring(const BgRing &ring) -> geom::Ring {
  geom::Ring result;
  result.reserve(ring.size());
  for (const auto &point : ring) {
    result.push_back(from_bg_point(point));
  }
  return result;
}

[[nodiscard]] auto collect_polygon_points(const geom::PolygonWithHoles &polygon)
    -> BgMultiPoint {
  const auto normalized = geom::normalize_polygon(polygon);

  BgMultiPoint points;
  for (const auto &point : normalized.outer) {
    bg::append(points, to_bg_point(point));
  }
  for (const auto &hole : normalized.holes) {
    for (const auto &point : hole) {
      bg::append(points, to_bg_point(point));
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
        geom::Polygon{.outer = geom::Ring(points.begin(), points.end())});
  }

  BgMultiPoint multipoint;
  for (const auto &point : points) {
    bg::append(multipoint, to_bg_point(point));
  }

  BgPolygon hull;
  bg::convex_hull(multipoint, hull);
  return geom::normalize_polygon(
      geom::Polygon{.outer = from_bg_ring(hull.outer())});
}

auto compute_convex_hull(const geom::Polygon &polygon) -> geom::Polygon {
  return compute_convex_hull(std::span<const geom::Point2>(
      polygon.outer.data(), polygon.outer.size()));
}

auto compute_convex_hull(const geom::PolygonWithHoles &polygon)
    -> geom::Polygon {
  const auto points = collect_polygon_points(polygon);
  if (points.empty()) {
    return {};
  }

  BgPolygon hull;
  bg::convex_hull(points, hull);
  return geom::normalize_polygon(
      geom::Polygon{.outer = from_bg_ring(hull.outer())});
}

} // namespace shiny::nfp::poly