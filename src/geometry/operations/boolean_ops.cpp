#include "geometry/operations/boolean_ops.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

#include "geometry/operations/simplify.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/sanitize.hpp"
#include "geometry/types.hpp"
#include "logging/shiny_log.hpp"

namespace shiny::nesting::geom {
namespace {

namespace bg = boost::geometry;

[[nodiscard]] auto prepare_boolean_input(const PolygonWithHoles &polygon)
    -> PolygonWithHoles {
  const auto sanitized = sanitize_polygon(polygon);
  auto prepared = sanitized.polygon;
  bg::correct(prepared);
  return prepared;
}

[[nodiscard]] auto polygon_less(const PolygonWithHoles &lhs,
                                const PolygonWithHoles &rhs) -> bool {
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

// Boost geometry's boolean operations sometimes produce near-duplicate
// consecutive vertices (x/y coordinates differing by ~1e-14) when input
// polygons share an edge. These near-duplicates are not caught by the exact
// equality check in normalize_polygon, which causes simplify_normalized_ring
// to receive a degenerate ring and incorrectly remove valid vertices. This
// function removes consecutive vertices that are within epsilon of each other,
// collapsing the degenerate near-duplicate pairs before normalization.
[[nodiscard]] auto remove_near_consecutive_duplicates_ring(const Ring &ring)
    -> Ring {
  constexpr double kNearDuplicateEpsilon = 1e-10;
  Ring cleaned;
  cleaned.reserve(ring.size());
  for (const auto &pt : ring) {
    if (!cleaned.empty()) {
      const auto dx = pt.x() - cleaned.back().x();
      const auto dy = pt.y() - cleaned.back().y();
      if (std::sqrt(dx * dx + dy * dy) <= kNearDuplicateEpsilon) {
        continue;
      }
    }
    cleaned.push_back(pt);
  }
  if (cleaned.size() > 1U) {
    const auto dx = cleaned.front().x() - cleaned.back().x();
    const auto dy = cleaned.front().y() - cleaned.back().y();
    if (std::sqrt(dx * dx + dy * dy) <= kNearDuplicateEpsilon) {
      cleaned.pop_back();
    }
  }
  return cleaned;
}

[[nodiscard]] auto clean_boolean_output_polygon(const PolygonWithHoles &polygon)
    -> PolygonWithHoles {
  PolygonWithHoles cleaned{};
  cleaned.outer() = remove_near_consecutive_duplicates_ring(polygon.outer());
  cleaned.holes().reserve(polygon.holes().size());
  for (const auto &hole : polygon.holes()) {
    cleaned.holes().push_back(remove_near_consecutive_duplicates_ring(hole));
  }
  return cleaned;
}

[[nodiscard]] auto normalize_output(const std::vector<PolygonWithHoles> &output)
    -> std::vector<PolygonWithHoles> {
  std::vector<PolygonWithHoles> normalized_output;
  normalized_output.reserve(output.size());
  for (const auto &polygon : output) {
    normalized_output.push_back(
        simplify_polygon(clean_boolean_output_polygon(polygon)));
  }
  std::sort(normalized_output.begin(), normalized_output.end(), polygon_less);
  return normalized_output;
}

auto log_boolean_failure(const std::string_view operation,
                         const PolygonWithHoles &lhs,
                         const PolygonWithHoles &rhs, const std::exception *ex)
    -> void {
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
                        const PolygonWithHoles &lhs,
                        const PolygonWithHoles &rhs, Fn &&fn)
    -> std::expected<std::vector<PolygonWithHoles>, util::Status> {
  try {
    return fn();
  } catch (const std::exception &ex) {
    log_boolean_failure(operation, lhs, rhs, &ex);
    return std::unexpected(util::Status::computation_failed);
  } catch (...) {
    log_boolean_failure(operation, lhs, rhs, nullptr);
    return std::unexpected(util::Status::computation_failed);
  }
}

} // namespace

auto union_polygons(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs)
    -> std::vector<PolygonWithHoles> {
  if (lhs.outer().empty() && rhs.outer().empty()) {
    return {};
  }
  if (lhs.outer().empty()) {
    return {normalize_polygon(rhs)};
  }
  if (rhs.outer().empty()) {
    return {normalize_polygon(lhs)};
  }

  std::vector<PolygonWithHoles> output;
  bg::union_(prepare_boolean_input(lhs), prepare_boolean_input(rhs), output);
  return normalize_output(output);
}

auto try_union_polygons(const PolygonWithHoles &lhs,
                        const PolygonWithHoles &rhs)
    -> std::expected<std::vector<PolygonWithHoles>, util::Status> {
  return try_binary_boolean("union", lhs, rhs,
                            [&]() { return union_polygons(lhs, rhs); });
}

auto intersection_polygons(const PolygonWithHoles &lhs,
                           const PolygonWithHoles &rhs)
    -> std::vector<PolygonWithHoles> {
  if (lhs.outer().empty() || rhs.outer().empty()) {
    return {};
  }

  const auto lhs_prepared = prepare_boolean_input(lhs);
  const auto rhs_prepared = prepare_boolean_input(rhs);

  std::vector<PolygonWithHoles> output;
  bg::intersection(lhs_prepared, rhs_prepared, output);
  return normalize_output(output);
}

auto try_intersection_polygons(const PolygonWithHoles &lhs,
                               const PolygonWithHoles &rhs)
    -> std::expected<std::vector<PolygonWithHoles>, util::Status> {
  return try_binary_boolean("intersection", lhs, rhs,
                            [&]() { return intersection_polygons(lhs, rhs); });
}

auto difference_polygons(const PolygonWithHoles &lhs,
                         const PolygonWithHoles &rhs)
    -> std::vector<PolygonWithHoles> {
  if (lhs.outer().empty()) {
    return {};
  }
  if (rhs.outer().empty()) {
    return {normalize_polygon(lhs)};
  }

  std::vector<PolygonWithHoles> output;
  bg::difference(prepare_boolean_input(lhs), prepare_boolean_input(rhs),
                 output);
  return normalize_output(output);
}

auto try_difference_polygons(const PolygonWithHoles &lhs,
                             const PolygonWithHoles &rhs)
    -> std::expected<std::vector<PolygonWithHoles>, util::Status> {
  return try_binary_boolean("difference", lhs, rhs,
                            [&]() { return difference_polygons(lhs, rhs); });
}

auto polygon_distance(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs)
    -> double {
  if (lhs.outer().empty() || rhs.outer().empty()) {
    return 0.0;
  }

  return bg::distance(prepare_boolean_input(lhs), prepare_boolean_input(rhs));
}

auto buffer_polygon(const PolygonWithHoles &polygon, double distance)
    -> std::vector<PolygonWithHoles> {
  if (polygon.outer().empty() || distance == 0.0) {
    if (polygon.outer().empty()) {
      return {};
    }
    return {normalize_polygon(polygon)};
  }

  using BgMultiPolygon = bg::model::multi_polygon<PolygonWithHoles>;

  bg::strategy::buffer::distance_symmetric<double> dist_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_square point_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  BgMultiPolygon buffered;
  bg::buffer(prepare_boolean_input(polygon), buffered, dist_strategy,
             side_strategy, join_strategy, end_strategy, point_strategy);

  std::vector<PolygonWithHoles> result;
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

auto subtract_region_set(std::vector<PolygonWithHoles> &regions,
                         const PolygonWithHoles &obstacle) -> void {
  std::vector<PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    const auto difference = difference_polygons(region, obstacle);
    next_regions.insert(next_regions.end(), difference.begin(),
                        difference.end());
  }
  regions = std::move(next_regions);
}

auto try_subtract_region_set(std::vector<PolygonWithHoles> &regions,
                             const PolygonWithHoles &obstacle) -> util::Status {
  std::vector<PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    auto difference = try_difference_polygons(region, obstacle);
    if (!difference.has_value()) {
      return difference.error();
    }
    const auto &difference_regions = difference.value();
    next_regions.insert(next_regions.end(), difference_regions.begin(),
                        difference_regions.end());
  }
  regions = std::move(next_regions);
  return util::Status::ok;
}

} // namespace shiny::nesting::geom