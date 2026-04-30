#include "nfp/ifp.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/sanitize.hpp"
#include "geometry/validity.hpp"
#include "logging/shiny_log.hpp"
#include "nfp/nfp.hpp"
#include "polygon_ops/boolean_ops.hpp"

namespace shiny::nesting::nfp {
namespace {

constexpr double kAxisEpsilon = 1e-9;
constexpr double kMinimumRegionArea = 1e-12;

[[nodiscard]] auto nearly_equal(const double lhs, const double rhs) -> bool {
  return std::fabs(lhs - rhs) <= kAxisEpsilon;
}

[[nodiscard]] auto
is_axis_aligned_rectangle(const geom::PolygonWithHoles &polygon)
    -> std::optional<geom::Box2> {
  if (!polygon.holes().empty() || polygon.outer().size() != 4U) {
    return std::nullopt;
  }

  const auto bounds = geom::compute_bounds(polygon);
  for (std::size_t index = 0; index < polygon.outer().size(); ++index) {
    const auto &current = polygon.outer()[index];
    const auto &next = polygon.outer()[(index + 1U) % polygon.outer().size()];

    const bool x_is_corner = nearly_equal(current.x(), bounds.min.x()) ||
                             nearly_equal(current.x(), bounds.max.x());
    const bool y_is_corner = nearly_equal(current.y(), bounds.min.y()) ||
                             nearly_equal(current.y(), bounds.max.y());
    if (!x_is_corner || !y_is_corner) {
      return std::nullopt;
    }

    const bool vertical = nearly_equal(current.x(), next.x());
    const bool horizontal = nearly_equal(current.y(), next.y());
    if (vertical == horizontal) {
      return std::nullopt;
    }
  }

  return bounds;
}

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  const auto lhs_bounds = geom::compute_bounds(lhs);
  const auto rhs_bounds = geom::compute_bounds(rhs);
  if (lhs_bounds.min.x() != rhs_bounds.min.x()) {
    return lhs_bounds.min.x() < rhs_bounds.min.x();
  }
  if (lhs_bounds.min.y() != rhs_bounds.min.y()) {
    return lhs_bounds.min.y() < rhs_bounds.min.y();
  }
  return geom::polygon_area(lhs) < geom::polygon_area(rhs);
}

[[nodiscard]] auto
normalize_regions(std::vector<geom::PolygonWithHoles> regions)
    -> std::vector<geom::PolygonWithHoles> {
  std::vector<geom::PolygonWithHoles> normalized;
  normalized.reserve(regions.size());
  for (auto &region : regions) {
    region = geom::normalize_polygon(region);
    if (!geom::validate_polygon(region).is_valid()) {
      continue;
    }
    if (geom::polygon_area(region) <= kMinimumRegionArea) {
      continue;
    }
    normalized.push_back(std::move(region));
  }
  std::sort(normalized.begin(), normalized.end(), polygon_less);
  return normalized;
}

[[nodiscard]] auto compute_general_ifp(const geom::PolygonWithHoles &container,
                                       const geom::PolygonWithHoles &piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  const auto container_bounds = geom::compute_bounds(container);
  const auto piece_bounds = geom::compute_bounds(piece);
  const auto move_bounds =
      inner_fit_rectangle_bounds(container_bounds, piece_bounds);
  if (move_bounds.max.x() < move_bounds.min.x() - kAxisEpsilon ||
      move_bounds.max.y() < move_bounds.min.y() - kAxisEpsilon) {
    return std::vector<geom::PolygonWithHoles>{};
  }

  std::vector<geom::PolygonWithHoles> feasible_regions{
      geom::normalize_polygon(geom::box_to_polygon(move_bounds))};
  const auto container_bbox =
      geom::normalize_polygon(geom::box_to_polygon(container_bounds));
  auto obstacles = poly::try_difference_polygons(container_bbox, container);
  if (!obstacles.ok()) {
    SHINY_DEBUG("ifp: container obstacle extraction failed status={}",
                util::status_name(obstacles.status()));
    return obstacles.status();
  }
  for (const auto &obstacle : obstacles.value()) {
    auto blocked = compute_nfp(obstacle, piece);
    if (!blocked.ok()) {
      SHINY_DEBUG("ifp: blocked-region NFP failed status={}",
                  util::status_name(blocked.status()));
      return blocked.status();
    }
    for (const auto &blocked_region : blocked.value()) {
      const auto subtract_status =
          poly::try_subtract_region_set(feasible_regions, blocked_region);
      if (subtract_status != util::Status::ok) {
        SHINY_DEBUG("ifp: subtract blocked region failed status={}",
                    util::status_name(subtract_status));
        return subtract_status;
      }
      if (feasible_regions.empty()) {
        return feasible_regions;
      }
    }
  }

  return normalize_regions(std::move(feasible_regions));
}

} // namespace

// Inner-Fit Polygon (IFP) for an axis-aligned-rectangle container only.
//
// For a rectangular container C = [cx_min, cx_max] × [cy_min, cy_max] and
// a piece P with bounding box [px_min, px_max] × [py_min, py_max], the set
// of translations t such that P + t ⊆ C is exactly the rectangle
//    [cx_min - px_min, cx_max - px_max] × [cy_min - py_min, cy_max - py_max]
// (this is the Inner-Fit Rectangle, IFR — a closed-form Minkowski
// difference for the axis-aligned case).
//
// Returns:
//   - non-empty single rectangle when the piece can fit somewhere in C;
//   - empty vector when the piece is too large for the container;
auto inner_fit_rectangle_bounds(const geom::Box2 &container_bounds,
                                const geom::Box2 &piece_bounds) -> geom::Box2 {
  return {
      .min = geom::Point2(container_bounds.min.x() - piece_bounds.min.x(),
                          container_bounds.min.y() - piece_bounds.min.y()),
      .max = geom::Point2(container_bounds.max.x() - piece_bounds.max.x(),
                          container_bounds.max.y() - piece_bounds.max.y()),
  };
}

auto compute_inner_fit_polygon(const geom::PolygonWithHoles &container,
                               const geom::PolygonWithHoles &piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  const auto normalized_container = geom::sanitize_polygon(container).polygon;
  const auto normalized_piece = geom::sanitize_polygon(piece).polygon;

  if (!geom::validate_polygon(normalized_container).is_valid() ||
      !geom::validate_polygon(normalized_piece).is_valid()) {
    SHINY_DEBUG("ifp: invalid input container_outer={} piece_outer={}",
                normalized_container.outer().size(),
                normalized_piece.outer().size());
    return util::Status::invalid_input;
  }

  const auto container_bounds = is_axis_aligned_rectangle(normalized_container);
  if (!container_bounds.has_value()) {
    return compute_general_ifp(normalized_container, normalized_piece);
  }

  const auto piece_bounds = geom::compute_bounds(normalized_piece);
  const auto ifr = inner_fit_rectangle_bounds(*container_bounds, piece_bounds);

  if (ifr.max.x() < ifr.min.x() - kAxisEpsilon ||
      ifr.max.y() < ifr.min.y() - kAxisEpsilon) {
    return std::vector<geom::PolygonWithHoles>{};
  }

  return std::vector<geom::PolygonWithHoles>{
      geom::normalize_polygon(geom::PolygonWithHoles(geom::Ring{
          ifr.min,
          geom::Point2(ifr.max.x(), ifr.min.y()),
          ifr.max,
          geom::Point2(ifr.min.x(), ifr.max.y()),
      }))};
}

auto compute_ifp(const geom::PolygonWithHoles &container,
                 const geom::PolygonWithHoles &piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  return compute_inner_fit_polygon(container, piece);
}

} // namespace shiny::nesting::nfp
