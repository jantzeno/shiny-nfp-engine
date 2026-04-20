#include "geometry/transform.hpp"

#include <cmath>

#include "geometry/normalize.hpp"

namespace shiny::nesting::geom {
namespace detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSnapEpsilon = 1e-10;

[[nodiscard]] auto snap_coordinate(const double value) -> double {
  return std::fabs(value) <= kSnapEpsilon ? 0.0 : value;
}

} // namespace detail

auto normalize_angle_degrees(double degrees) -> double {
  degrees = std::fmod(degrees, 360.0);
  if (degrees < 0.0) {
    degrees += 360.0;
  }
  if (std::fabs(degrees - 360.0) <= detail::kSnapEpsilon) {
    return 0.0;
  }
  return degrees;
}

auto translate_point(const Point2 &point, const Vector2 translation) -> Point2 {
  return {
      .x = point.x + translation.x,
      .y = point.y + translation.y,
  };
}

auto translate_ring(std::span<const Point2> ring, const Vector2 translation)
    -> Ring {
  Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(translate_point(point, translation));
  }
  return translated;
}

auto translate_polygon(const Polygon &polygon, const Vector2 translation)
    -> Polygon {
  return {.outer = translate_ring(polygon.outer, translation)};
}

auto translate_polygon(const PolygonWithHoles &polygon, const Vector2 translation)
    -> PolygonWithHoles {
  PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, translation);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, translation));
  }
  return translated;
}

auto rotate_point(const Point2 &point, const ResolvedRotation rotation) -> Point2 {
  const auto radians = normalize_angle_degrees(rotation.degrees) * detail::kPi /
                       180.0;
  auto cosine = std::cos(radians);
  auto sine = std::sin(radians);
  cosine = detail::snap_coordinate(cosine);
  sine = detail::snap_coordinate(sine);

  return {
      .x = detail::snap_coordinate(point.x * cosine - point.y * sine),
      .y = detail::snap_coordinate(point.x * sine + point.y * cosine),
  };
}

auto rotate_ring(std::span<const Point2> ring, const ResolvedRotation rotation)
    -> Ring {
  Ring rotated;
  rotated.reserve(ring.size());
  for (const auto &point : ring) {
    rotated.push_back(rotate_point(point, rotation));
  }
  return rotated;
}

auto rotate_polygon(const Polygon &polygon, const ResolvedRotation rotation)
    -> Polygon {
  return normalize_polygon(Polygon{.outer = rotate_ring(polygon.outer, rotation)});
}

auto rotate_polygon(const PolygonWithHoles &polygon,
                    const ResolvedRotation rotation) -> PolygonWithHoles {
  PolygonWithHoles rotated{};
  rotated.outer = rotate_ring(polygon.outer, rotation);
  rotated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    rotated.holes.push_back(rotate_ring(hole, rotation));
  }
  return normalize_polygon(rotated);
}

auto apply_transform(const Point2 &point, const ResolvedRotation rotation,
                     const Vector2 translation) -> Point2 {
  return translate_point(rotate_point(point, rotation), translation);
}

auto apply_transform(const Polygon &polygon, const ResolvedRotation rotation,
                     const Vector2 translation) -> Polygon {
  return translate_polygon(rotate_polygon(polygon, rotation), translation);
}

auto apply_transform(const PolygonWithHoles &polygon,
                     const ResolvedRotation rotation, const Vector2 translation)
    -> PolygonWithHoles {
  return translate_polygon(rotate_polygon(polygon, rotation), translation);
}

auto resolve_rotation(const RotationIndex rotation_index,
                      const DiscreteRotationSet &rotations)
    -> std::optional<ResolvedRotation> {
  if (rotation_index.value >= rotations.angles_degrees.size()) {
    return std::nullopt;
  }
  return ResolvedRotation{
      .degrees = normalize_angle_degrees(rotations.angles_degrees[rotation_index.value]),
  };
}

auto apply_transform(const Point2 &point, const Transform2 &transform,
                     const DiscreteRotationSet &rotations)
    -> std::optional<Point2> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  return apply_transform(point, *rotation, transform.translation);
}

auto apply_transform(const Polygon &polygon, const Transform2 &transform,
                     const DiscreteRotationSet &rotations)
    -> std::optional<Polygon> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  return apply_transform(polygon, *rotation, transform.translation);
}

auto apply_transform(const PolygonWithHoles &polygon, const Transform2 &transform,
                     const DiscreteRotationSet &rotations)
    -> std::optional<PolygonWithHoles> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  return apply_transform(polygon, *rotation, transform.translation);
}

} // namespace shiny::nesting::geom
