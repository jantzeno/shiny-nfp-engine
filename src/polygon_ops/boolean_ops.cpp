#include "polygon_ops/boolean_ops.hpp"

#include <algorithm>
#include <string_view>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

#include "geometry/normalize.hpp"
#include "geometry/sanitize.hpp"
#include "geometry/types.hpp"
#include "logging/shiny_log.hpp"
#include "polygon_ops/simplify.hpp"

namespace shiny::nesting::poly {
namespace {

namespace bg = boost::geometry;

[[nodiscard]] auto prepare_boolean_input(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  const auto sanitized = geom::sanitize_polygon(polygon);
  auto prepared = sanitized.polygon;
  bg::correct(prepared);
  return prepared;
}

[[nodiscard]] auto polygon_less(const geom::PolygonWithHoles &lhs,
                                const geom::PolygonWithHoles &rhs) -> bool {
  if (lhs.outer().empty()) {
    return !rhs.outer().empty();
  }
  if (rhs.outer().empty()) {
    return false;
  }

  if (lhs.outer().front().x() != rhs.outer().front().x()) {
    return lhs.outer().front().x() < rhs.outer().front().x();
  }
  if (lhs.outer().front().y() != rhs.outer().front().y()) {
    return lhs.outer().front().y() < rhs.outer().front().y();
  }
  if (lhs.outer().size() != rhs.outer().size()) {
    return lhs.outer().size() < rhs.outer().size();
  }
  return lhs.holes().size() < rhs.holes().size();
}

[[nodiscard]] auto
normalize_output(const std::vector<geom::PolygonWithHoles> &output)
    -> std::vector<geom::PolygonWithHoles> {
  std::vector<geom::PolygonWithHoles> normalized_output;
  normalized_output.reserve(output.size());
  for (const auto &polygon : output) {
    normalized_output.push_back(simplify_polygon(polygon));
  }

  std::sort(normalized_output.begin(), normalized_output.end(), polygon_less);
  return normalized_output;
}

auto log_boolean_failure(const std::string_view operation,
                         const geom::PolygonWithHoles &lhs,
                         const geom::PolygonWithHoles &rhs,
                         const std::exception *ex) -> void {
  if (ex != nullptr) {
    SHINY_DEBUG("boolean_ops: {} failed lhs_outer={} lhs_holes={} rhs_outer={} "
                "rhs_holes={} error={}",
                operation, lhs.outer().size(), lhs.holes().size(),
                rhs.outer().size(), rhs.holes().size(), ex->what());
    return;
  }

  SHINY_DEBUG("boolean_ops: {} failed lhs_outer={} lhs_holes={} rhs_outer={} "
              "rhs_holes={} with unknown exception",
              operation, lhs.outer().size(), lhs.holes().size(),
              rhs.outer().size(), rhs.holes().size());
}

template <typename Fn>
auto try_binary_boolean(const std::string_view operation,
                        const geom::PolygonWithHoles &lhs,
                        const geom::PolygonWithHoles &rhs, Fn &&fn)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  try {
    return fn();
  } catch (const std::exception &ex) {
    log_boolean_failure(operation, lhs, rhs, &ex);
    return util::Status::computation_failed;
  } catch (...) {
    log_boolean_failure(operation, lhs, rhs, nullptr);
    return util::Status::computation_failed;
  }
}

} // namespace

auto union_polygons(const geom::PolygonWithHoles &lhs,
                    const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer().empty() && rhs.outer().empty()) {
    return {};
  }
  if (lhs.outer().empty()) {
    return {geom::normalize_polygon(rhs)};
  }
  if (rhs.outer().empty()) {
    return {geom::normalize_polygon(lhs)};
  }

  std::vector<geom::PolygonWithHoles> output;
  bg::union_(prepare_boolean_input(lhs), prepare_boolean_input(rhs), output);
  return normalize_output(output);
}

auto try_union_polygons(const geom::PolygonWithHoles &lhs,
                        const geom::PolygonWithHoles &rhs)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  return try_binary_boolean("union", lhs, rhs,
                            [&]() { return union_polygons(lhs, rhs); });
}

auto intersection_polygons(const geom::PolygonWithHoles &lhs,
                           const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer().empty() || rhs.outer().empty()) {
    return {};
  }

  std::vector<geom::PolygonWithHoles> output;
  bg::intersection(prepare_boolean_input(lhs), prepare_boolean_input(rhs),
                   output);
  return normalize_output(output);
}

auto try_intersection_polygons(const geom::PolygonWithHoles &lhs,
                               const geom::PolygonWithHoles &rhs)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  return try_binary_boolean("intersection", lhs, rhs,
                            [&]() { return intersection_polygons(lhs, rhs); });
}

auto difference_polygons(const geom::PolygonWithHoles &lhs,
                         const geom::PolygonWithHoles &rhs)
    -> std::vector<geom::PolygonWithHoles> {
  if (lhs.outer().empty()) {
    return {};
  }
  if (rhs.outer().empty()) {
    return {geom::normalize_polygon(lhs)};
  }

  std::vector<geom::PolygonWithHoles> output;
  bg::difference(prepare_boolean_input(lhs), prepare_boolean_input(rhs),
                 output);
  return normalize_output(output);
}

auto try_difference_polygons(const geom::PolygonWithHoles &lhs,
                             const geom::PolygonWithHoles &rhs)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  return try_binary_boolean("difference", lhs, rhs,
                            [&]() { return difference_polygons(lhs, rhs); });
}

auto polygon_distance(const geom::PolygonWithHoles &lhs,
                      const geom::PolygonWithHoles &rhs) -> double {
  if (lhs.outer().empty() || rhs.outer().empty()) {
    return 0.0;
  }

  return bg::distance(prepare_boolean_input(lhs), prepare_boolean_input(rhs));
}

auto buffer_polygon(const geom::PolygonWithHoles &polygon, double distance)
    -> std::vector<geom::PolygonWithHoles> {
  if (polygon.outer().empty() || distance == 0.0) {
    if (polygon.outer().empty()) {
      return {};
    }
    return {geom::normalize_polygon(polygon)};
  }

  using BgMultiPolygon = bg::model::multi_polygon<geom::PolygonWithHoles>;

  bg::strategy::buffer::distance_symmetric<double> dist_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_square point_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  BgMultiPolygon buffered;
  bg::buffer(prepare_boolean_input(polygon), buffered, dist_strategy,
             side_strategy, join_strategy, end_strategy, point_strategy);

  std::vector<geom::PolygonWithHoles> result;
  result.reserve(buffered.size());
  for (const auto &poly : buffered) {
    auto converted = simplify_polygon(poly);
    if (!converted.outer().empty()) {
      result.push_back(std::move(converted));
    }
  }

  std::sort(result.begin(), result.end(), polygon_less);
  return result;
}

auto subtract_region_set(std::vector<geom::PolygonWithHoles> &regions,
                         const geom::PolygonWithHoles &obstacle) -> void {
  std::vector<geom::PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    const auto difference = difference_polygons(region, obstacle);
    next_regions.insert(next_regions.end(), difference.begin(),
                        difference.end());
  }
  regions = std::move(next_regions);
}

auto try_subtract_region_set(std::vector<geom::PolygonWithHoles> &regions,
                             const geom::PolygonWithHoles &obstacle)
    -> util::Status {
  std::vector<geom::PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    auto difference = try_difference_polygons(region, obstacle);
    if (!difference.ok()) {
      return difference.status();
    }
    const auto &difference_regions = difference.value();
    next_regions.insert(next_regions.end(), difference_regions.begin(),
                        difference_regions.end());
  }
  regions = std::move(next_regions);
  return util::Status::ok;
}

} // namespace shiny::nesting::poly
