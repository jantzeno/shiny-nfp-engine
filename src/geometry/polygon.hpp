#pragma once

#include <cstdint>
#include <span>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

[[nodiscard]] auto ring_signed_area(std::span<const Point2> ring) -> double;

[[nodiscard]] auto polygon_area(const Polygon &polygon) -> double;

[[nodiscard]] auto polygon_area(const PolygonWithHoles &polygon) -> double;

[[nodiscard]] auto compute_bounds(std::span<const Point2> ring) -> Box2;

[[nodiscard]] auto compute_bounds(const Polygon &polygon) -> Box2;

[[nodiscard]] auto compute_bounds(const PolygonWithHoles &polygon) -> Box2;

[[nodiscard]] auto box_width(const Box2 &box) -> double;

[[nodiscard]] auto box_height(const Box2 &box) -> double;

[[nodiscard]] auto boxes_overlap(const Box2 &lhs, const Box2 &rhs) -> bool;

[[nodiscard]] auto box_contains(const Box2 &container,
                                const Box2 &candidate) -> bool;

[[nodiscard]] auto polygon_revision(const Polygon &polygon) -> std::uint64_t;

[[nodiscard]] auto polygon_revision(const PolygonWithHoles &polygon)
    -> std::uint64_t;

} // namespace shiny::nesting::geom
