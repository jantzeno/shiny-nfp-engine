#pragma once

#include <concepts>
#include <type_traits>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

template <typename T>
concept PointGeometry = std::same_as<std::remove_cvref_t<T>, Point2>;

template <typename T>
concept RingGeometry = std::same_as<std::remove_cvref_t<T>, Ring>;

template <typename T>
concept PolygonGeometry = std::same_as<std::remove_cvref_t<T>, Polygon>;

template <typename T>
concept PolygonWithHolesGeometry =
    std::same_as<std::remove_cvref_t<T>, PolygonWithHoles>;

template <typename T>
concept TransformGeometry = PointGeometry<T> || RingGeometry<T> ||
                            PolygonGeometry<T> || PolygonWithHolesGeometry<T>;

template <typename T>
concept PlaceableGeometry =
    PointGeometry<T> || PolygonGeometry<T> || PolygonWithHolesGeometry<T>;

} // namespace shiny::nesting::geom
