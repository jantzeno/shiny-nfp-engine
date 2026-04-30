#pragma once

#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <vector>

#include "geometry/concepts.hpp"
#include "types.hpp"

namespace shiny::nesting::geom {

/**
 * @brief Index into a discrete rotation set.
 */
struct RotationIndex {
  std::uint16_t value{};

  auto operator<=>(const RotationIndex &) const = default;
};

/**
 * @brief Concrete resolved rotation angle in degrees.
 */
struct ResolvedRotation {
  double degrees{};

  auto operator<=>(const ResolvedRotation &) const = default;
};

/**
 * @brief Discrete rigid transform used by placement and packing.
 */
struct Transform2 {
  RotationIndex rotation_index{};
  Vector2 translation{};
  bool mirrored{false};
};

struct RotationRange {
  double min_degrees{0.0};
  double max_degrees{0.0};
  double step_degrees{0.0};
};

/**
 * @brief Configured set of allowed piece rotations.
 */
struct DiscreteRotationSet {
  std::vector<double> angles_degrees{};
  std::optional<RotationRange> range_degrees{};
};

[[nodiscard]] auto normalize_angle_degrees(double degrees) -> double;

[[nodiscard]] auto materialize_rotations(const DiscreteRotationSet &rotations)
    -> std::vector<double>;

[[nodiscard]] auto rotation_count(const DiscreteRotationSet &rotations)
    -> std::size_t;

namespace detail {

[[nodiscard]] auto translate_geometry(const Point2 &point, Vector2 translation)
    -> Point2;

[[nodiscard]] auto translate_geometry(const Ring &ring, Vector2 translation)
    -> Ring;

[[nodiscard]] auto translate_geometry(const Polygon &polygon,
                                      Vector2 translation) -> Polygon;

[[nodiscard]] auto translate_geometry(const PolygonWithHoles &polygon,
                                      Vector2 translation) -> PolygonWithHoles;

[[nodiscard]] auto mirror_geometry(const Point2 &point) -> Point2;

[[nodiscard]] auto mirror_geometry(const Ring &ring) -> Ring;

[[nodiscard]] auto mirror_geometry(const Polygon &polygon) -> Polygon;

[[nodiscard]] auto mirror_geometry(const PolygonWithHoles &polygon)
    -> PolygonWithHoles;

[[nodiscard]] auto rotate_geometry(const Point2 &point,
                                   ResolvedRotation rotation) -> Point2;

[[nodiscard]] auto rotate_geometry(const Ring &ring, ResolvedRotation rotation)
    -> Ring;

[[nodiscard]] auto rotate_geometry(const Polygon &polygon,
                                   ResolvedRotation rotation) -> Polygon;

[[nodiscard]] auto rotate_geometry(const PolygonWithHoles &polygon,
                                   ResolvedRotation rotation)
    -> PolygonWithHoles;

[[nodiscard]] auto apply_transform_geometry(const Point2 &point,
                                            ResolvedRotation rotation,
                                            Vector2 translation) -> Point2;

[[nodiscard]] auto apply_transform_geometry(const Polygon &polygon,
                                            ResolvedRotation rotation,
                                            Vector2 translation) -> Polygon;

[[nodiscard]] auto apply_transform_geometry(const PolygonWithHoles &polygon,
                                            ResolvedRotation rotation,
                                            Vector2 translation)
    -> PolygonWithHoles;

[[nodiscard]] auto
apply_transform_geometry(const Point2 &point, const Transform2 &transform,
                         const DiscreteRotationSet &rotations)
    -> std::optional<Point2>;

[[nodiscard]] auto
apply_transform_geometry(const Polygon &polygon, const Transform2 &transform,
                         const DiscreteRotationSet &rotations)
    -> std::optional<Polygon>;

[[nodiscard]] auto apply_transform_geometry(
    const PolygonWithHoles &polygon, const Transform2 &transform,
    const DiscreteRotationSet &rotations) -> std::optional<PolygonWithHoles>;

} // namespace detail

[[nodiscard]] auto resolve_rotation(RotationIndex rotation_index,
                                    const DiscreteRotationSet &rotations)
    -> std::optional<ResolvedRotation>;

template <TransformGeometry Geometry>
[[nodiscard]] inline auto translate(const Geometry &geometry,
                                    const Vector2 translation)
    -> std::remove_cvref_t<Geometry> {
  return detail::translate_geometry(geometry, translation);
}

template <TransformGeometry Geometry>
[[nodiscard]] inline auto mirror(const Geometry &geometry)
    -> std::remove_cvref_t<Geometry> {
  return detail::mirror_geometry(geometry);
}

template <TransformGeometry Geometry>
[[nodiscard]] inline auto rotate(const Geometry &geometry,
                                 const ResolvedRotation rotation)
    -> std::remove_cvref_t<Geometry> {
  return detail::rotate_geometry(geometry, rotation);
}

template <PlaceableGeometry Geometry>
[[nodiscard]] inline auto apply_transform(const Geometry &geometry,
                                          const ResolvedRotation rotation,
                                          const Vector2 translation)
    -> std::remove_cvref_t<Geometry> {
  return detail::apply_transform_geometry(geometry, rotation, translation);
}

template <PlaceableGeometry Geometry>
[[nodiscard]] inline auto apply_transform(const Geometry &geometry,
                                          const Transform2 &transform,
                                          const DiscreteRotationSet &rotations)
    -> std::optional<std::remove_cvref_t<Geometry>> {
  return detail::apply_transform_geometry(geometry, transform, rotations);
}

} // namespace shiny::nesting::geom
