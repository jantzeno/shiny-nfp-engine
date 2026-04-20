#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

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
};

/**
 * @brief Configured set of allowed piece rotations.
 */
struct DiscreteRotationSet {
  std::vector<double> angles_degrees{};
};

[[nodiscard]] auto normalize_angle_degrees(double degrees) -> double;

[[nodiscard]] auto translate_point(const Point2 &point, Vector2 translation)
    -> Point2;

[[nodiscard]] auto translate_ring(std::span<const Point2> ring,
                                  Vector2 translation) -> Ring;

[[nodiscard]] auto translate_polygon(const Polygon &polygon,
                                     Vector2 translation) -> Polygon;

[[nodiscard]] auto translate_polygon(const PolygonWithHoles &polygon,
                                     Vector2 translation) -> PolygonWithHoles;

[[nodiscard]] auto rotate_point(const Point2 &point, ResolvedRotation rotation)
    -> Point2;

[[nodiscard]] auto rotate_ring(std::span<const Point2> ring,
                               ResolvedRotation rotation) -> Ring;

[[nodiscard]] auto rotate_polygon(const Polygon &polygon,
                                  ResolvedRotation rotation) -> Polygon;

[[nodiscard]] auto rotate_polygon(const PolygonWithHoles &polygon,
                                  ResolvedRotation rotation)
    -> PolygonWithHoles;

[[nodiscard]] auto apply_transform(const Point2 &point, ResolvedRotation rotation,
                                   Vector2 translation) -> Point2;

[[nodiscard]] auto apply_transform(const Polygon &polygon,
                                   ResolvedRotation rotation,
                                   Vector2 translation) -> Polygon;

[[nodiscard]] auto apply_transform(const PolygonWithHoles &polygon,
                                   ResolvedRotation rotation,
                                   Vector2 translation)
    -> PolygonWithHoles;

[[nodiscard]] auto resolve_rotation(RotationIndex rotation_index,
                                    const DiscreteRotationSet &rotations)
    -> std::optional<ResolvedRotation>;

[[nodiscard]] auto apply_transform(const Point2 &point, const Transform2 &transform,
                                   const DiscreteRotationSet &rotations)
    -> std::optional<Point2>;

[[nodiscard]] auto apply_transform(const Polygon &polygon,
                                   const Transform2 &transform,
                                   const DiscreteRotationSet &rotations)
    -> std::optional<Polygon>;

[[nodiscard]] auto apply_transform(const PolygonWithHoles &polygon,
                                   const Transform2 &transform,
                                   const DiscreteRotationSet &rotations)
    -> std::optional<PolygonWithHoles>;

} // namespace shiny::nesting::geom
