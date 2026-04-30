#include "geometry/transforms/transform.hpp"

#include <cmath>
#include <vector>

#include "geometry/queries/normalize.hpp"

namespace shiny::nesting::geom {
namespace detail {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSnapEpsilon = 1e-10;
constexpr std::size_t kMaxMaterializedRotations = 100'000;

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

auto materialize_rotations(const DiscreteRotationSet &rotations)
    -> std::vector<double> {
  if (!rotations.angles_degrees.empty()) {
    std::vector<double> angles;
    angles.reserve(rotations.angles_degrees.size());
    for (const auto angle : rotations.angles_degrees) {
      angles.push_back(normalize_angle_degrees(angle));
    }
    return angles;
  }

  if (!rotations.range_degrees.has_value()) {
    return {};
  }

  const auto &range = *rotations.range_degrees;
  if (!std::isfinite(range.min_degrees) || !std::isfinite(range.max_degrees) ||
      !std::isfinite(range.step_degrees) || range.step_degrees <= 0.0 ||
      range.max_degrees < range.min_degrees) {
    return {};
  }

  const long double span = static_cast<long double>(range.max_degrees) -
                           static_cast<long double>(range.min_degrees);
  const long double predicted_count =
      span / static_cast<long double>(range.step_degrees) + 1.0L;
  if (!(predicted_count >= 0.0L) ||
      predicted_count >
          static_cast<long double>(detail::kMaxMaterializedRotations)) {
    return {};
  }

  std::vector<double> angles;
  angles.reserve(static_cast<std::size_t>(predicted_count));
  for (double angle = range.min_degrees; angle <= range.max_degrees + 1e-9;
       angle += range.step_degrees) {
    if (angles.size() >= detail::kMaxMaterializedRotations) {
      break;
    }
    angles.push_back(normalize_angle_degrees(angle));
  }
  return angles;
}

auto rotation_count(const DiscreteRotationSet &rotations) -> std::size_t {
  return materialize_rotations(rotations).size();
}

auto detail::translate_geometry(const Point2 &point, const Vector2 translation)
    -> Point2 {
  return Point2{point.x() + translation.x(), point.y() + translation.y()};
}

auto detail::translate_geometry(const Ring &ring, const Vector2 translation)
    -> Ring {
  Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(translate(point, translation));
  }
  return translated;
}

auto detail::translate_geometry(const Polygon &polygon,
                                const Vector2 translation) -> Polygon {
  return Polygon{translate(polygon.outer(), translation)};
}

auto detail::translate_geometry(const PolygonWithHoles &polygon,
                                const Vector2 translation) -> PolygonWithHoles {
  PolygonWithHoles translated{};
  translated.outer() = translate(polygon.outer(), translation);
  translated.holes().reserve(polygon.holes().size());
  for (const auto &hole : polygon.holes()) {
    translated.holes().push_back(translate(hole, translation));
  }
  return translated;
}

auto detail::mirror_geometry(const Point2 &point) -> Point2 {
  return Point2{detail::snap_coordinate(-point.x()), point.y()};
}

auto detail::mirror_geometry(const Ring &ring) -> Ring {
  Ring mirrored;
  mirrored.reserve(ring.size());
  for (const auto &point : ring) {
    mirrored.push_back(mirror(point));
  }
  return mirrored;
}

auto detail::mirror_geometry(const Polygon &polygon) -> Polygon {
  return normalize_polygon(Polygon{mirror(polygon.outer())});
}

auto detail::mirror_geometry(const PolygonWithHoles &polygon)
    -> PolygonWithHoles {
  PolygonWithHoles mirrored{};
  mirrored.outer() = mirror(polygon.outer());
  mirrored.holes().reserve(polygon.holes().size());
  for (const auto &hole : polygon.holes()) {
    mirrored.holes().push_back(mirror(hole));
  }
  return normalize_polygon(mirrored);
}

auto detail::rotate_geometry(const Point2 &point,
                             const ResolvedRotation rotation) -> Point2 {
  const auto radians =
      normalize_angle_degrees(rotation.degrees) * detail::kPi / 180.0;
  auto cosine = std::cos(radians);
  auto sine = std::sin(radians);
  cosine = detail::snap_coordinate(cosine);
  sine = detail::snap_coordinate(sine);

  return Point2{detail::snap_coordinate(point.x() * cosine - point.y() * sine),
                detail::snap_coordinate(point.x() * sine + point.y() * cosine)};
}

auto detail::rotate_geometry(const Ring &ring, const ResolvedRotation rotation)
    -> Ring {
  Ring rotated;
  rotated.reserve(ring.size());
  for (const auto &point : ring) {
    rotated.push_back(rotate(point, rotation));
  }
  return rotated;
}

auto detail::rotate_geometry(const Polygon &polygon,
                             const ResolvedRotation rotation) -> Polygon {
  return normalize_polygon(Polygon{rotate(polygon.outer(), rotation)});
}

auto detail::rotate_geometry(const PolygonWithHoles &polygon,
                             const ResolvedRotation rotation)
    -> PolygonWithHoles {
  PolygonWithHoles rotated{};
  rotated.outer() = rotate(polygon.outer(), rotation);
  rotated.holes().reserve(polygon.holes().size());
  for (const auto &hole : polygon.holes()) {
    rotated.holes().push_back(rotate(hole, rotation));
  }
  return normalize_polygon(rotated);
}

auto detail::apply_transform_geometry(const Point2 &point,
                                      const ResolvedRotation rotation,
                                      const Vector2 translation) -> Point2 {
  return translate(rotate(point, rotation), translation);
}

auto detail::apply_transform_geometry(const Polygon &polygon,
                                      const ResolvedRotation rotation,
                                      const Vector2 translation) -> Polygon {
  return translate(rotate(polygon, rotation), translation);
}

auto detail::apply_transform_geometry(const PolygonWithHoles &polygon,
                                      const ResolvedRotation rotation,
                                      const Vector2 translation)
    -> PolygonWithHoles {
  return translate(rotate(polygon, rotation), translation);
}

auto resolve_rotation(const RotationIndex rotation_index,
                      const DiscreteRotationSet &rotations)
    -> std::optional<ResolvedRotation> {
  const auto angles = materialize_rotations(rotations);
  if (rotation_index.value >= angles.size()) {
    return std::nullopt;
  }
  return ResolvedRotation{
      .degrees = normalize_angle_degrees(angles[rotation_index.value]),
  };
}

auto detail::apply_transform_geometry(const Point2 &point,
                                      const Transform2 &transform,
                                      const DiscreteRotationSet &rotations)
    -> std::optional<Point2> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  const auto transformed_point = transform.mirrored ? mirror(point) : point;
  return translate(rotate(transformed_point, *rotation), transform.translation);
}

auto detail::apply_transform_geometry(const Polygon &polygon,
                                      const Transform2 &transform,
                                      const DiscreteRotationSet &rotations)
    -> std::optional<Polygon> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  const auto transformed =
      rotate(transform.mirrored ? mirror(polygon) : polygon, *rotation);
  return translate(transformed, transform.translation);
}

auto detail::apply_transform_geometry(const PolygonWithHoles &polygon,
                                      const Transform2 &transform,
                                      const DiscreteRotationSet &rotations)
    -> std::optional<PolygonWithHoles> {
  const auto rotation = resolve_rotation(transform.rotation_index, rotations);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  const auto transformed =
      rotate(transform.mirrored ? mirror(polygon) : polygon, *rotation);
  return translate(transformed, transform.translation);
}

} // namespace shiny::nesting::geom