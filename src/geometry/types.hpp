#pragma once

#include <compare>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/segment.hpp>
#include <boost/mpl/int.hpp>

namespace shiny::nesting::geom {

namespace detail {

namespace bg = boost::geometry;

using BoostPoint2 = bg::model::d2::point_xy<double>;

} // namespace detail

/**
 * @brief Cartesian point in model-space coordinates.
 *
 * Stores one polygon vertex or translation anchor in the library's geometric
 * kernel. Ordering and equality are value-based so normalized rings and cache
 * keys can compare points directly.
 *
 * @par Invariants
 * - Coordinates are interpreted in the repository's normalized 2D plane.
 *
 * @par Performance Notes
 * - Trivially copyable aggregate used heavily in rings, segments, and caches.
 */
struct Point2 {
  using backing_type = detail::BoostPoint2;

  backing_type storage{};

  Point2() = default;

  Point2(const double x_value, const double y_value)
      : storage(x_value, y_value) {}

  explicit Point2(backing_type value) : storage(std::move(value)) {}

  [[nodiscard]] auto x() const -> double { return storage.x(); }

  auto set_x(const double value) -> void { storage.x(value); }

  [[nodiscard]] auto y() const -> double { return storage.y(); }

  auto set_y(const double value) -> void { storage.y(value); }

  [[nodiscard]] auto backing() -> backing_type & { return storage; }

  [[nodiscard]] auto backing() const -> const backing_type & { return storage; }

  [[nodiscard]] auto operator==(const Point2 &other) const -> bool {
    return x() == other.x() && y() == other.y();
  }

  auto operator<=>(const Point2 &other) const {
    if (const auto x_compare = x() <=> other.x(); x_compare != 0) {
      return x_compare;
    }
    return y() <=> other.y();
  }
};

/**
 * @brief 2D displacement vector.
 *
 * Represents translation offsets and rotated edge deltas without introducing a
 * separate storage format from Point2.
 *
 * @par Invariants
 * - Components are interpreted as a displacement, not an absolute position.
 *
 * @par Performance Notes
 * - Stored directly inside transforms and placement records.
 */
struct Vector2 {
  using backing_type = detail::BoostPoint2;

  backing_type storage{};

  Vector2() = default;

  Vector2(const double x_value, const double y_value)
      : storage(x_value, y_value) {}

  explicit Vector2(backing_type value) : storage(std::move(value)) {}

  [[nodiscard]] auto x() const -> double { return storage.x(); }

  auto set_x(const double value) -> void { storage.x(value); }

  [[nodiscard]] auto y() const -> double { return storage.y(); }

  auto set_y(const double value) -> void { storage.y(value); }

  [[nodiscard]] auto backing() -> backing_type & { return storage; }

  [[nodiscard]] auto backing() const -> const backing_type & { return storage; }

  [[nodiscard]] auto operator==(const Vector2 &other) const -> bool {
    return x() == other.x() && y() == other.y();
  }

  auto operator<=>(const Vector2 &other) const {
    if (const auto x_compare = x() <=> other.x(); x_compare != 0) {
      return x_compare;
    }
    return y() <=> other.y();
  }
};

} // namespace shiny::nesting::geom

namespace boost::geometry::traits {

template <> struct tag<shiny::nesting::geom::Point2> {
  using type = point_tag;
};

template <> struct coordinate_type<shiny::nesting::geom::Point2> {
  using type = double;
};

template <> struct coordinate_system<shiny::nesting::geom::Point2> {
  using type = cs::cartesian;
};

template <>
struct dimension<shiny::nesting::geom::Point2> : boost::mpl::int_<2> {};

template <> struct access<shiny::nesting::geom::Point2, 0> {
  static auto get(const shiny::nesting::geom::Point2 &point) -> double {
    return point.x();
  }

  static auto set(shiny::nesting::geom::Point2 &point, const double value)
      -> void {
    point.set_x(value);
  }
};

template <> struct access<shiny::nesting::geom::Point2, 1> {
  static auto get(const shiny::nesting::geom::Point2 &point) -> double {
    return point.y();
  }

  static auto set(shiny::nesting::geom::Point2 &point, const double value)
      -> void {
    point.set_y(value);
  }
};

template <> struct tag<shiny::nesting::geom::Vector2> {
  using type = point_tag;
};

template <> struct coordinate_type<shiny::nesting::geom::Vector2> {
  using type = double;
};

template <> struct coordinate_system<shiny::nesting::geom::Vector2> {
  using type = cs::cartesian;
};

template <>
struct dimension<shiny::nesting::geom::Vector2> : boost::mpl::int_<2> {};

template <> struct access<shiny::nesting::geom::Vector2, 0> {
  static auto get(const shiny::nesting::geom::Vector2 &vector) -> double {
    return vector.x();
  }

  static auto set(shiny::nesting::geom::Vector2 &vector, const double value)
      -> void {
    vector.set_x(value);
  }
};

template <> struct access<shiny::nesting::geom::Vector2, 1> {
  static auto get(const shiny::nesting::geom::Vector2 &vector) -> double {
    return vector.y();
  }

  static auto set(shiny::nesting::geom::Vector2 &vector, const double value)
      -> void {
    vector.set_y(value);
  }
};

} // namespace boost::geometry::traits

namespace shiny::nesting::geom {

/**
 * @brief Axis-aligned bounding box over model-space coordinates.
 *
 * Captures coarse spatial extent for polygons, placements, and intermediate
 * geometric queries.
 *
 * @par Invariants
 * - `min` and `max` are expected to bound the same region.
 *
 * @par Performance Notes
 * - Used as a low-cost broad-phase summary before exact geometry checks.
 */
struct Box2 {
  Point2 min{};
  Point2 max{};

  auto operator<=>(const Box2 &) const = default;
};

/**
 * @brief Directed segment between two points.
 *
 * Used for boundary queries, contact extraction, and cut-plan generation.
 *
 * @par Invariants
 * - Endpoints are interpreted as an ordered pair unless a caller explicitly
 *   canonicalizes them.
 *
 * @par Performance Notes
 * - Value comparisons support segment deduplication after canonical ordering.
 */
struct Segment2 {
  Point2 start{};
  Point2 end{};

  auto operator<=>(const Segment2 &) const = default;
};

using Ring = boost::geometry::model::ring<Point2, false, false>;

namespace detail {

using BoostPolygon = bg::model::polygon<Point2, false, false>;

} // namespace detail

/**
 * @brief Simple polygon represented by one outer ring.
 *
 * This is the base representation for convex geometry and for intermediate
 * operations that do not need explicit cavities.
 *
 * @par Invariants
 * - Callers are expected to normalize winding and duplicate closure rules when
 *   required by an algorithm.
 *
 * @par Performance Notes
 * - Keeps storage minimal when holes are not needed.
 */
struct Polygon {
  using backing_type = detail::BoostPolygon;

  backing_type storage{};

  Polygon() = default;

  explicit Polygon(backing_type value) : storage(std::move(value)) {}

  explicit Polygon(Ring outer_ring) { storage.outer() = std::move(outer_ring); }

  [[nodiscard]] auto outer() -> Ring & { return storage.outer(); }

  [[nodiscard]] auto outer() const -> const Ring & { return storage.outer(); }

  [[nodiscard]] auto backing() -> backing_type & { return storage; }

  [[nodiscard]] auto backing() const -> const backing_type & { return storage; }

  [[nodiscard]] auto operator==(const Polygon &other) const -> bool {
    return outer() == other.outer() &&
           backing().inners() == other.backing().inners();
  }
};

/**
 * @brief Polygon with one outer boundary and zero or more hole rings.
 *
 * This is the repository's main shape representation for bins, pieces, and NFP
 * outputs.
 *
 * @par Invariants
 * - `outer` defines the exterior boundary and `holes` define excluded regions.
 *
 * @par Performance Notes
 * - Shared across decomposition, boolean ops, placement, and packing layers.
 */
struct PolygonWithHoles {
  using backing_type = detail::BoostPolygon;

  backing_type storage{};

  PolygonWithHoles() = default;

  explicit PolygonWithHoles(backing_type value) : storage(std::move(value)) {}

  PolygonWithHoles(Ring outer_ring, std::vector<Ring> hole_rings = {}) {
    storage.outer() = std::move(outer_ring);
    storage.inners() = std::move(hole_rings);
  }

  [[nodiscard]] auto outer() -> Ring & { return storage.outer(); }

  [[nodiscard]] auto outer() const -> const Ring & { return storage.outer(); }

  [[nodiscard]] auto holes() -> std::vector<Ring> & { return storage.inners(); }

  [[nodiscard]] auto holes() const -> const std::vector<Ring> & {
    return storage.inners();
  }

  [[nodiscard]] auto backing() -> backing_type & { return storage; }

  [[nodiscard]] auto backing() const -> const backing_type & { return storage; }

  [[nodiscard]] auto operator==(const PolygonWithHoles &other) const -> bool {
    return outer() == other.outer() && holes() == other.holes();
  }
};

} // namespace shiny::nesting::geom

BOOST_GEOMETRY_REGISTER_BOX(shiny::nesting::geom::Box2,
                            shiny::nesting::geom::Point2, min, max)
BOOST_GEOMETRY_REGISTER_SEGMENT(shiny::nesting::geom::Segment2,
                                shiny::nesting::geom::Point2, start, end)

namespace boost::geometry::traits {

template <>
struct point_order<shiny::nesting::geom::Ring>
    : std::integral_constant<order_selector, counterclockwise> {};

template <>
struct closure<shiny::nesting::geom::Ring>
    : std::integral_constant<closure_selector, open> {};

template <> struct tag<shiny::nesting::geom::Polygon> {
  using type = polygon_tag;
};

template <> struct ring_const_type<shiny::nesting::geom::Polygon> {
  using type = const shiny::nesting::geom::Ring &;
};

template <> struct ring_mutable_type<shiny::nesting::geom::Polygon> {
  using type = shiny::nesting::geom::Ring &;
};

template <> struct interior_const_type<shiny::nesting::geom::Polygon> {
  using type = const std::vector<shiny::nesting::geom::Ring> &;
};

template <> struct interior_mutable_type<shiny::nesting::geom::Polygon> {
  using type = std::vector<shiny::nesting::geom::Ring> &;
};

template <> struct exterior_ring<shiny::nesting::geom::Polygon> {
  static auto get(shiny::nesting::geom::Polygon &polygon)
      -> shiny::nesting::geom::Ring & {
    return polygon.outer();
  }

  static auto get(const shiny::nesting::geom::Polygon &polygon)
      -> const shiny::nesting::geom::Ring & {
    return polygon.outer();
  }
};

template <> struct interior_rings<shiny::nesting::geom::Polygon> {
  static auto get(shiny::nesting::geom::Polygon &polygon)
      -> std::vector<shiny::nesting::geom::Ring> & {
    return polygon.backing().inners();
  }

  static auto get(const shiny::nesting::geom::Polygon &polygon)
      -> const std::vector<shiny::nesting::geom::Ring> & {
    return polygon.backing().inners();
  }
};

template <>
struct point_order<shiny::nesting::geom::Polygon>
    : std::integral_constant<order_selector, counterclockwise> {};

template <>
struct closure<shiny::nesting::geom::Polygon>
    : std::integral_constant<closure_selector, open> {};

template <> struct tag<shiny::nesting::geom::PolygonWithHoles> {
  using type = polygon_tag;
};

template <> struct ring_const_type<shiny::nesting::geom::PolygonWithHoles> {
  using type = const shiny::nesting::geom::Ring &;
};

template <> struct ring_mutable_type<shiny::nesting::geom::PolygonWithHoles> {
  using type = shiny::nesting::geom::Ring &;
};

template <> struct interior_const_type<shiny::nesting::geom::PolygonWithHoles> {
  using type = const std::vector<shiny::nesting::geom::Ring> &;
};

template <>
struct interior_mutable_type<shiny::nesting::geom::PolygonWithHoles> {
  using type = std::vector<shiny::nesting::geom::Ring> &;
};

template <> struct exterior_ring<shiny::nesting::geom::PolygonWithHoles> {
  static auto get(shiny::nesting::geom::PolygonWithHoles &polygon)
      -> shiny::nesting::geom::Ring & {
    return polygon.outer();
  }

  static auto get(const shiny::nesting::geom::PolygonWithHoles &polygon)
      -> const shiny::nesting::geom::Ring & {
    return polygon.outer();
  }
};

template <> struct interior_rings<shiny::nesting::geom::PolygonWithHoles> {
  static auto get(shiny::nesting::geom::PolygonWithHoles &polygon)
      -> std::vector<shiny::nesting::geom::Ring> & {
    return polygon.holes();
  }

  static auto get(const shiny::nesting::geom::PolygonWithHoles &polygon)
      -> const std::vector<shiny::nesting::geom::Ring> & {
    return polygon.holes();
  }
};

template <>
struct point_order<shiny::nesting::geom::PolygonWithHoles>
    : std::integral_constant<order_selector, counterclockwise> {};

template <>
struct closure<shiny::nesting::geom::PolygonWithHoles>
    : std::integral_constant<closure_selector, open> {};

} // namespace boost::geometry::traits

namespace shiny::nesting::geom {}