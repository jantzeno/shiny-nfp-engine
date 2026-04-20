#include "packing/bounding_box_packer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "geometry/normalize.hpp"
#include "logging/shiny_log.hpp"
#include "placement/config.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "polygon_ops/merge_region.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kAreaEpsilon = 1e-9;
constexpr double kCoordinateSnap = 1e-10;
constexpr double kPi = 3.14159265358979323846;

struct ShelfState {
  double y{0.0};
  double height{0.0};
  double next_x{0.0};
  double used_min_x{0.0};
  double used_max_x{0.0};
};

struct BinPackingState {
  BinState bin_state{};
  geom::Box2 container_bounds{};
  std::vector<ShelfState> shelves{};
  std::vector<geom::Box2> occupied_bounds{};
  std::vector<geom::Box2> free_rectangles{};
  double occupied_area{0.0};
};

struct PlacementCandidate {
  std::size_t shelf_index{0};
  bool starts_new_shelf{false};
  place::Placement placement{};
  geom::ResolvedRotation resolved_rotation{};
  geom::PolygonWithHoles rotated_piece{};
  geom::Box2 translated_bounds{};
  double resulting_utilization{0.0};
};

struct PieceOrderingMetrics {
  std::size_t original_index{0};
  double width{0.0};
  double height{0.0};
  double area{0.0};
  double max_dimension{0.0};
};

[[nodiscard]] auto as_polygon_with_holes(const place::BedExclusionZone &zone)
    -> geom::PolygonWithHoles {
  return {.outer = zone.region.outer};
}

[[nodiscard]] auto interrupted(const InterruptionProbe &interruption_requested)
    -> bool {
  return interruption_requested && interruption_requested();
}

[[nodiscard]] auto snap_coordinate(double value) -> double {
  if (std::fabs(value) < kCoordinateSnap) {
    return 0.0;
  }
  return value;
}

[[nodiscard]] auto almost_equal(double lhs, double rhs) -> bool {
  return std::fabs(lhs - rhs) <= kCoordinateSnap;
}

[[nodiscard]] auto translate_point(const geom::Point2 &point,
                                   const geom::Point2 &translation)
    -> geom::Point2 {
  return {
      .x = point.x + translation.x,
      .y = point.y + translation.y,
  };
}

[[nodiscard]] auto translate_ring(const geom::Ring &ring,
                                  const geom::Point2 &translation)
    -> geom::Ring {
  geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(translate_point(point, translation));
  }
  return translated;
}

[[nodiscard]] auto translate_polygon(const geom::PolygonWithHoles &polygon,
                                     const geom::Point2 &translation)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, translation);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, translation));
  }
  return translated;
}

[[nodiscard]] auto rotate_point(const geom::Point2 &point, double degrees)
    -> geom::Point2 {
  const auto radians = degrees * kPi / 180.0;
  auto cosine = std::cos(radians);
  auto sine = std::sin(radians);

  cosine = snap_coordinate(cosine);
  sine = snap_coordinate(sine);

  return {
      .x = snap_coordinate(point.x * cosine - point.y * sine),
      .y = snap_coordinate(point.x * sine + point.y * cosine),
  };
}

[[nodiscard]] auto rotate_ring(const geom::Ring &ring, double degrees)
    -> geom::Ring {
  geom::Ring rotated;
  rotated.reserve(ring.size());
  for (const auto &point : ring) {
    rotated.push_back(rotate_point(point, degrees));
  }
  return rotated;
}

[[nodiscard]] auto rotate_polygon(const geom::PolygonWithHoles &polygon,
                                  double degrees) -> geom::PolygonWithHoles {
  geom::PolygonWithHoles rotated{};
  rotated.outer = rotate_ring(polygon.outer, degrees);
  rotated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    rotated.holes.push_back(rotate_ring(hole, degrees));
  }
  return geom::normalize_polygon(rotated);
}

[[nodiscard]] auto signed_area(const geom::Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

[[nodiscard]] auto polygon_area(const geom::PolygonWithHoles &polygon)
    -> double {
  long double area = std::abs(signed_area(polygon.outer));
  for (const auto &hole : polygon.holes) {
    area -= std::abs(signed_area(hole));
  }
  return static_cast<double>(area);
}

[[nodiscard]] auto compute_bounds(const geom::PolygonWithHoles &polygon)
    -> geom::Box2 {
  geom::Box2 bounds{};
  bool initialized = false;

  const auto include_ring = [&bounds, &initialized](const geom::Ring &ring) {
    for (const auto &point : ring) {
      if (!initialized) {
        bounds.min = point;
        bounds.max = point;
        initialized = true;
        continue;
      }

      bounds.min.x = std::min(bounds.min.x, point.x);
      bounds.min.y = std::min(bounds.min.y, point.y);
      bounds.max.x = std::max(bounds.max.x, point.x);
      bounds.max.y = std::max(bounds.max.y, point.y);
    }
  };

  include_ring(polygon.outer);
  for (const auto &hole : polygon.holes) {
    include_ring(hole);
  }

  return bounds;
}

[[nodiscard]] auto boxes_overlap(const geom::Box2 &lhs, const geom::Box2 &rhs)
    -> bool {
  return !(lhs.max.x < rhs.min.x - kCoordinateSnap ||
           rhs.max.x < lhs.min.x - kCoordinateSnap ||
           lhs.max.y < rhs.min.y - kCoordinateSnap ||
           rhs.max.y < lhs.min.y - kCoordinateSnap);
}

[[nodiscard]] auto intervals_overlap_interior(double lhs_min, double lhs_max,
                                              double rhs_min, double rhs_max)
    -> bool {
  return lhs_max > rhs_min + kCoordinateSnap &&
         rhs_max > lhs_min + kCoordinateSnap;
}

[[nodiscard]] auto boxes_overlap_interior(const geom::Box2 &lhs,
                                          const geom::Box2 &rhs) -> bool {
  return intervals_overlap_interior(lhs.min.x, lhs.max.x, rhs.min.x,
                                    rhs.max.x) &&
         intervals_overlap_interior(lhs.min.y, lhs.max.y, rhs.min.y, rhs.max.y);
}

[[nodiscard]] auto box_width(const geom::Box2 &box) -> double {
  return box.max.x - box.min.x;
}

[[nodiscard]] auto box_height(const geom::Box2 &box) -> double {
  return box.max.y - box.min.y;
}

[[nodiscard]] auto box_has_area(const geom::Box2 &box) -> bool {
  return box_width(box) > kCoordinateSnap && box_height(box) > kCoordinateSnap;
}

[[nodiscard]] auto normalize_box(const geom::Point2 &first,
                                 const geom::Point2 &second) -> geom::Box2 {
  return {
      .min = {.x = std::min(first.x, second.x),
              .y = std::min(first.y, second.y)},
      .max = {.x = std::max(first.x, second.x),
              .y = std::max(first.y, second.y)},
  };
}

[[nodiscard]] auto contains_box(const geom::Box2 &container,
                                const geom::Box2 &candidate) -> bool {
  return candidate.min.x >= container.min.x - kCoordinateSnap &&
         candidate.min.y >= container.min.y - kCoordinateSnap &&
         candidate.max.x <= container.max.x + kCoordinateSnap &&
         candidate.max.y <= container.max.y + kCoordinateSnap;
}

[[nodiscard]] auto
point_for_start_corner(const geom::Point2 &point, const geom::Box2 &container,
                       const place::PlacementStartCorner start_corner)
    -> geom::Point2 {
  const bool on_right =
      start_corner == place::PlacementStartCorner::bottom_right ||
      start_corner == place::PlacementStartCorner::top_right;
  const bool on_top = start_corner == place::PlacementStartCorner::top_left ||
                      start_corner == place::PlacementStartCorner::top_right;
  return {
      .x = on_right ? container.min.x + (container.max.x - point.x) : point.x,
      .y = on_top ? container.min.y + (container.max.y - point.y) : point.y,
  };
}

[[nodiscard]] auto
box_for_start_corner(const geom::Box2 &box, const geom::Box2 &container,
                     const place::PlacementStartCorner start_corner)
    -> geom::Box2 {
  return normalize_box(
      point_for_start_corner(box.min, container, start_corner),
      point_for_start_corner(box.max, container, start_corner));
}

[[nodiscard]] auto unique_sorted_values(std::vector<double> values)
    -> std::vector<double> {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end(),
                           [](double lhs, double rhs) {
                             return almost_equal(lhs, rhs);
                           }),
               values.end());
  return values;
}

[[nodiscard]] auto
overlaps_any_occupied_bounds(std::span<const geom::Box2> occupied_bounds,
                             const geom::Box2 &candidate_bounds) -> bool {
  return std::any_of(occupied_bounds.begin(), occupied_bounds.end(),
                     [&](const geom::Box2 &occupied_bounds_entry) {
                       return boxes_overlap_interior(candidate_bounds,
                                                     occupied_bounds_entry);
                     });
}

auto split_free_rectangles(std::vector<geom::Box2> &free_rectangles,
                           const geom::Box2 &used_bounds) -> void {
  std::vector<geom::Box2> updated;
  updated.reserve(free_rectangles.size() * 4U + 1U);
  for (const geom::Box2 &free_rectangle : free_rectangles) {
    if (!boxes_overlap_interior(free_rectangle, used_bounds)) {
      updated.push_back(free_rectangle);
      continue;
    }

    const geom::Box2 intersection{
        .min = {.x = std::max(free_rectangle.min.x, used_bounds.min.x),
                .y = std::max(free_rectangle.min.y, used_bounds.min.y)},
        .max = {.x = std::min(free_rectangle.max.x, used_bounds.max.x),
                .y = std::min(free_rectangle.max.y, used_bounds.max.y)},
    };

    const geom::Box2 left = {
        .min = free_rectangle.min,
        .max = {.x = intersection.min.x, .y = free_rectangle.max.y},
    };
    const geom::Box2 right = {
        .min = {.x = intersection.max.x, .y = free_rectangle.min.y},
        .max = free_rectangle.max,
    };
    const geom::Box2 bottom = {
        .min = {.x = intersection.min.x, .y = free_rectangle.min.y},
        .max = {.x = intersection.max.x, .y = intersection.min.y},
    };
    const geom::Box2 top = {
        .min = {.x = intersection.min.x, .y = intersection.max.y},
        .max = {.x = intersection.max.x, .y = free_rectangle.max.y},
    };

    for (const geom::Box2 &candidate : {left, right, bottom, top}) {
      if (box_has_area(candidate)) {
        updated.push_back(candidate);
      }
    }
  }

  free_rectangles.clear();
  for (std::size_t index = 0; index < updated.size(); ++index) {
    bool contained = false;
    for (std::size_t other_index = 0; other_index < updated.size();
         ++other_index) {
      if (index == other_index) {
        continue;
      }
      if (contains_box(updated[other_index], updated[index])) {
        contained = true;
        break;
      }
    }
    if (!contained) {
      free_rectangles.push_back(updated[index]);
    }
  }
}

[[nodiscard]] auto
total_polygon_area(std::span<const geom::PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += polygon_area(polygon);
  }
  return total;
}

[[nodiscard]] auto overlaps_exclusion_zone(const geom::PolygonWithHoles &piece,
                                           const geom::Box2 &piece_bounds,
                                           const place::BedExclusionZone &zone)
    -> bool {
  const auto zone_polygon = as_polygon_with_holes(zone);
  if (piece.outer.empty() || zone_polygon.outer.empty() ||
      !boxes_overlap(piece_bounds, compute_bounds(zone_polygon))) {
    return false;
  }

  const auto remaining = poly::difference_polygons(piece, zone_polygon);
  return total_polygon_area(remaining) + kAreaEpsilon < polygon_area(piece);
}

[[nodiscard]] auto overlaps_any_exclusion_zone(
    const geom::PolygonWithHoles &piece, const geom::Box2 &piece_bounds,
    std::span<const place::BedExclusionZone> exclusion_zones,
    const std::uint32_t bin_id) -> bool {
  for (const auto &zone : exclusion_zones) {
    if (!place::exclusion_zone_applies_to_bin(zone, bin_id)) {
      continue;
    }
    if (overlaps_exclusion_zone(piece, piece_bounds, zone)) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto resulting_envelope_area(const BinPackingState &state,
                                           const geom::Box2 &candidate_bounds)
    -> double {
  double min_x = candidate_bounds.min.x;
  double max_x = candidate_bounds.max.x;
  double max_y = candidate_bounds.max.y;
  for (const auto &shelf : state.shelves) {
    min_x = std::min(min_x, shelf.used_min_x);
    max_x = std::max(max_x, shelf.used_max_x);
    max_y = std::max(max_y, shelf.y + shelf.height);
  }

  return std::max(0.0, max_x - min_x) *
         std::max(0.0, max_y - state.container_bounds.min.y);
}

[[nodiscard]] auto
start_corner_on_right(const place::PlacementStartCorner start_corner) -> bool {
  return start_corner == place::PlacementStartCorner::bottom_right ||
         start_corner == place::PlacementStartCorner::top_right;
}

[[nodiscard]] auto
start_corner_on_top(const place::PlacementStartCorner start_corner) -> bool {
  return start_corner == place::PlacementStartCorner::top_left ||
         start_corner == place::PlacementStartCorner::top_right;
}

[[nodiscard]] auto primary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  if (start_corner_on_top(start_corner)) {
    return container_bounds.max.y - piece_bounds.max.y;
  }
  return piece_bounds.min.y - container_bounds.min.y;
}

[[nodiscard]] auto secondary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  if (start_corner_on_right(start_corner)) {
    return container_bounds.max.x - piece_bounds.max.x;
  }
  return piece_bounds.min.x - container_bounds.min.x;
}

[[nodiscard]] auto better_candidate(const BinPackingState &state,
                                    const PlacementCandidate &lhs,
                                    const PlacementCandidate &rhs,
                                    place::PlacementPolicy policy) -> bool {
  const double lhs_primary =
      primary_edge_distance(state.container_bounds, lhs.translated_bounds,
                            state.bin_state.start_corner);
  const double rhs_primary =
      primary_edge_distance(state.container_bounds, rhs.translated_bounds,
                            state.bin_state.start_corner);
  const double lhs_secondary =
      secondary_edge_distance(state.container_bounds, lhs.translated_bounds,
                              state.bin_state.start_corner);
  const double rhs_secondary =
      secondary_edge_distance(state.container_bounds, rhs.translated_bounds,
                              state.bin_state.start_corner);
  switch (policy) {
  case place::PlacementPolicy::bottom_left:
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::minimum_length:
    if (!almost_equal(lhs.translated_bounds.max.x,
                      rhs.translated_bounds.max.x)) {
      return lhs.translated_bounds.max.x < rhs.translated_bounds.max.x;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::maximum_utilization:
    if (!almost_equal(lhs.resulting_utilization, rhs.resulting_utilization)) {
      return lhs.resulting_utilization > rhs.resulting_utilization;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  }

  if (lhs.placement.rotation_index.value !=
      rhs.placement.rotation_index.value) {
    return lhs.placement.rotation_index.value <
           rhs.placement.rotation_index.value;
  }

  return false;
}

[[nodiscard]] auto fits_on_existing_shelf(const ShelfState &shelf,
                                          const geom::Box2 &rotated_bounds)
    -> bool {
  const auto width = rotated_bounds.max.x - rotated_bounds.min.x;
  const auto height = rotated_bounds.max.y - rotated_bounds.min.y;
  return height <= shelf.height + kCoordinateSnap && width >= 0.0 &&
         height >= 0.0;
}

[[nodiscard]] auto make_empty_bin(const BinInput &bin) -> BinPackingState {
  BinPackingState state{};
  state.bin_state.bin_id = bin.bin_id;
  state.bin_state.container = geom::normalize_polygon(bin.polygon);
  state.bin_state.container_geometry_revision = bin.geometry_revision;
  state.bin_state.start_corner = bin.start_corner;
  state.bin_state.utilization = summarize_bin(state.bin_state);
  state.container_bounds = compute_bounds(state.bin_state.container);
  state.free_rectangles.push_back(state.container_bounds);
  return state;
}

[[nodiscard]] auto piece_allows_bin(const PieceInput &piece,
                                    const std::uint32_t bin_id) -> bool {
  return piece.allowed_bin_ids.empty() ||
         std::find(piece.allowed_bin_ids.begin(), piece.allowed_bin_ids.end(),
                   bin_id) != piece.allowed_bin_ids.end();
}

void mark_remaining_unplaced(std::span<const PieceInput> pieces,
                             std::size_t start_index,
                             std::vector<std::uint32_t> &unplaced_piece_ids) {
  for (std::size_t index = start_index; index < pieces.size(); ++index) {
    unplaced_piece_ids.push_back(pieces[index].piece_id);
  }
}

[[nodiscard]] auto piece_metrics_for(const PieceInput &piece,
                                     const std::size_t original_index)
    -> PieceOrderingMetrics {
  const geom::Box2 bounds = compute_bounds(piece.polygon);
  const double width = bounds.max.x - bounds.min.x;
  const double height = bounds.max.y - bounds.min.y;
  return {
      .original_index = original_index,
      .width = width,
      .height = height,
      .area = polygon_area(piece.polygon),
      .max_dimension = std::max(width, height),
  };
}

template <typename MetricSelector>
auto sort_order_descending(std::vector<std::size_t> &order,
                           std::span<const PieceOrderingMetrics> metrics,
                           const MetricSelector &selector) -> void {
  std::stable_sort(order.begin(), order.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     const double lhs_metric = selector(metrics[lhs]);
                     const double rhs_metric = selector(metrics[rhs]);
                     if (!almost_equal(lhs_metric, rhs_metric)) {
                       return lhs_metric > rhs_metric;
                     }
                     return metrics[lhs].original_index <
                            metrics[rhs].original_index;
                   });
}

[[nodiscard]] auto piece_order_key(const DecoderRequest &request,
                                   std::span<const std::size_t> order)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> key;
  key.reserve(order.size());
  for (const std::size_t index : order) {
    key.push_back(request.pieces[index].piece_id);
  }
  return key;
}

auto append_unique_order(const DecoderRequest &request,
                         std::vector<std::vector<std::size_t>> &orders,
                         std::vector<std::vector<std::uint32_t>> &keys,
                         std::vector<std::size_t> order) -> void {
  const std::vector<std::uint32_t> key = piece_order_key(request, order);
  if (std::find(keys.begin(), keys.end(), key) != keys.end()) {
    return;
  }
  keys.push_back(key);
  orders.push_back(std::move(order));
}

[[nodiscard]] auto reorder_request(const DecoderRequest &request,
                                   std::span<const std::size_t> order)
    -> DecoderRequest {
  DecoderRequest reordered = request;
  reordered.pieces.clear();
  reordered.pieces.reserve(order.size());
  for (const std::size_t index : order) {
    reordered.pieces.push_back(request.pieces[index]);
  }
  return reordered;
}

[[nodiscard]] auto build_attempt_requests(const DecoderRequest &request)
    -> std::vector<DecoderRequest> {
  const std::size_t piece_count = request.pieces.size();
  std::vector<std::size_t> original_order(piece_count);
  for (std::size_t index = 0; index < piece_count; ++index) {
    original_order[index] = index;
  }

  std::vector<PieceOrderingMetrics> metrics;
  metrics.reserve(piece_count);
  for (std::size_t index = 0; index < piece_count; ++index) {
    metrics.push_back(piece_metrics_for(request.pieces[index], index));
  }

  std::vector<std::vector<std::size_t>> orders;
  std::vector<std::vector<std::uint32_t>> order_keys;
  orders.reserve(request.config.deterministic_attempts.max_attempts);
  order_keys.reserve(request.config.deterministic_attempts.max_attempts);

  append_unique_order(request, orders, order_keys, original_order);

  std::vector<std::size_t> reversed_order = original_order;
  std::reverse(reversed_order.begin(), reversed_order.end());
  append_unique_order(request, orders, order_keys, std::move(reversed_order));

  std::vector<std::size_t> descending_area = original_order;
  sort_order_descending(
      descending_area, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.area; });
  append_unique_order(request, orders, order_keys, std::move(descending_area));

  std::vector<std::size_t> descending_max_dimension = original_order;
  sort_order_descending(
      descending_max_dimension, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.max_dimension; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_max_dimension));

  std::vector<std::size_t> descending_width = original_order;
  sort_order_descending(
      descending_width, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.width; });
  append_unique_order(request, orders, order_keys, std::move(descending_width));

  std::vector<std::size_t> descending_height = original_order;
  sort_order_descending(
      descending_height, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.height; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_height));

  if (orders.size() > request.config.deterministic_attempts.max_attempts) {
    orders.resize(request.config.deterministic_attempts.max_attempts);
  }

  std::vector<DecoderRequest> attempts;
  attempts.reserve(orders.size());
  for (const std::vector<std::size_t> &order : orders) {
    attempts.push_back(reorder_request(request, order));
  }
  return attempts;
}

[[nodiscard]] auto
find_best_shelf_candidate(const BinPackingState &state, const PieceInput &piece,
                          const DecoderRequest &request,
                          const geom::PolygonWithHoles &rotated_piece,
                          const geom::Box2 &rotated_bounds,
                          const geom::ResolvedRotation &resolved_rotation,
                          const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  const double clearance = request.config.placement.part_clearance;
  const double width = box_width(rotated_bounds);
  const double height = box_height(rotated_bounds);

  const auto consider_candidate = [&](double target_min_x, double target_min_y,
                                      std::size_t shelf_index,
                                      bool starts_new_shelf) {
    const geom::Box2 translated_bounds{
        .min = {.x = target_min_x, .y = target_min_y},
        .max = {.x = target_min_x + width, .y = target_min_y + height},
    };
    if (!contains_box(state.container_bounds, translated_bounds) ||
        overlaps_any_occupied_bounds(state.occupied_bounds,
                                     translated_bounds)) {
      return;
    }

    const geom::Point2 translation{
        .x = target_min_x - rotated_bounds.min.x,
        .y = target_min_y - rotated_bounds.min.y,
    };
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_exclusion_zone(translated_piece, translated_bounds,
                                    request.config.placement.exclusion_zones,
                                    state.bin_state.bin_id)) {
      return;
    }

    const auto envelope_area =
        resulting_envelope_area(state, translated_bounds);
    const auto piece_area = polygon_area(rotated_piece);
    const auto utilization =
        envelope_area > kCoordinateSnap
            ? (state.occupied_area + piece_area) / envelope_area
            : 0.0;
    PlacementCandidate candidate{
        .shelf_index = shelf_index,
        .starts_new_shelf = starts_new_shelf,
        .placement =
            {
                .piece_id = piece.piece_id,
                .bin_id = state.bin_state.bin_id,
                .rotation_index = rotation_index,
                .translation = translation,
            },
        .resolved_rotation = resolved_rotation,
        .rotated_piece = rotated_piece,
        .translated_bounds = translated_bounds,
        .resulting_utilization = utilization,
    };

    if (!best.has_value() ||
        better_candidate(state, candidate, *best, request.policy)) {
      best = std::move(candidate);
    }
  };

  for (std::size_t shelf_index = 0; shelf_index < state.shelves.size();
       ++shelf_index) {
    const ShelfState &shelf = state.shelves[shelf_index];
    if (!fits_on_existing_shelf(shelf, rotated_bounds)) {
      continue;
    }
    const double target_min_x =
        start_corner_on_right(state.bin_state.start_corner)
            ? shelf.next_x - width
            : shelf.next_x;
    consider_candidate(target_min_x, shelf.y, shelf_index, false);
  }

  const double next_shelf_y =
      state.shelves.empty()
          ? (start_corner_on_top(state.bin_state.start_corner)
                 ? state.container_bounds.max.y - height
                 : state.container_bounds.min.y)
          : (start_corner_on_top(state.bin_state.start_corner)
                 ? state.shelves.back().y - height - clearance
                 : state.shelves.back().y + state.shelves.back().height +
                       clearance);
  const double new_shelf_min_x =
      start_corner_on_right(state.bin_state.start_corner)
          ? state.container_bounds.max.x - width
          : state.container_bounds.min.x;
  consider_candidate(new_shelf_min_x, next_shelf_y, state.shelves.size(), true);
  return best;
}

[[nodiscard]] auto
skyline_candidate_min_xs(const geom::Box2 &container_bounds,
                         std::span<const geom::Box2> occupied_bounds,
                         double width) -> std::vector<double> {
  std::vector<double> candidates{
      container_bounds.min.x,
      container_bounds.max.x - width,
  };
  candidates.reserve(2U + occupied_bounds.size() * 4U);
  for (const geom::Box2 &occupied_bounds_entry : occupied_bounds) {
    candidates.push_back(occupied_bounds_entry.min.x);
    candidates.push_back(occupied_bounds_entry.max.x);
    candidates.push_back(occupied_bounds_entry.min.x - width);
    candidates.push_back(occupied_bounds_entry.max.x - width);
  }

  std::vector<double> unique = unique_sorted_values(std::move(candidates));
  unique.erase(std::remove_if(unique.begin(), unique.end(),
                              [&](double value) {
                                return value < container_bounds.min.x -
                                                   kCoordinateSnap ||
                                       value + width > container_bounds.max.x +
                                                           kCoordinateSnap;
                              }),
               unique.end());
  return unique;
}

[[nodiscard]] auto find_best_skyline_candidate(
    const BinPackingState &state, const PieceInput &piece,
    const DecoderRequest &request, const geom::PolygonWithHoles &rotated_piece,
    const geom::Box2 &rotated_bounds,
    const geom::ResolvedRotation &resolved_rotation,
    const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  const geom::Box2 canonical_container =
      box_for_start_corner(state.container_bounds, state.container_bounds,
                           state.bin_state.start_corner);
  std::vector<geom::Box2> canonical_occupied_bounds;
  canonical_occupied_bounds.reserve(state.occupied_bounds.size());
  for (const geom::Box2 &occupied_bounds_entry : state.occupied_bounds) {
    canonical_occupied_bounds.push_back(
        box_for_start_corner(occupied_bounds_entry, state.container_bounds,
                             state.bin_state.start_corner));
  }

  const geom::Box2 canonical_rotated_bounds = box_for_start_corner(
      rotated_bounds, rotated_bounds, place::PlacementStartCorner::bottom_left);
  const double width = box_width(canonical_rotated_bounds);
  const double height = box_height(canonical_rotated_bounds);

  for (const double candidate_min_x : skyline_candidate_min_xs(
           canonical_container, canonical_occupied_bounds, width)) {
    double candidate_min_y = canonical_container.min.y;
    for (const geom::Box2 &occupied_bounds_entry : canonical_occupied_bounds) {
      if (!intervals_overlap_interior(candidate_min_x, candidate_min_x + width,
                                      occupied_bounds_entry.min.x,
                                      occupied_bounds_entry.max.x)) {
        continue;
      }
      candidate_min_y = std::max(candidate_min_y, occupied_bounds_entry.max.y);
    }

    const geom::Box2 canonical_candidate_bounds{
        .min = {.x = candidate_min_x, .y = candidate_min_y},
        .max = {.x = candidate_min_x + width, .y = candidate_min_y + height},
    };
    const geom::Box2 actual_candidate_bounds =
        box_for_start_corner(canonical_candidate_bounds, state.container_bounds,
                             state.bin_state.start_corner);
    if (!contains_box(state.container_bounds, actual_candidate_bounds)) {
      continue;
    }

    const geom::Point2 translation{
        .x = actual_candidate_bounds.min.x - rotated_bounds.min.x,
        .y = actual_candidate_bounds.min.y - rotated_bounds.min.y,
    };
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_occupied_bounds(state.occupied_bounds,
                                     actual_candidate_bounds) ||
        overlaps_any_exclusion_zone(translated_piece, actual_candidate_bounds,
                                    request.config.placement.exclusion_zones,
                                    state.bin_state.bin_id)) {
      continue;
    }

    const auto envelope_area =
        resulting_envelope_area(state, actual_candidate_bounds);
    const auto piece_area = polygon_area(rotated_piece);
    const auto utilization =
        envelope_area > kCoordinateSnap
            ? (state.occupied_area + piece_area) / envelope_area
            : 0.0;
    PlacementCandidate candidate{
        .shelf_index = 0,
        .starts_new_shelf = false,
        .placement =
            {
                .piece_id = piece.piece_id,
                .bin_id = state.bin_state.bin_id,
                .rotation_index = rotation_index,
                .translation = translation,
            },
        .resolved_rotation = resolved_rotation,
        .rotated_piece = rotated_piece,
        .translated_bounds = actual_candidate_bounds,
        .resulting_utilization = utilization,
    };

    if (!best.has_value() ||
        better_candidate(state, candidate, *best, request.policy)) {
      best = std::move(candidate);
    }
  }

  return best;
}

[[nodiscard]] auto free_rectangle_better(const BinPackingState &state,
                                         const geom::Box2 &lhs_bounds,
                                         const geom::Box2 &rhs_bounds,
                                         const geom::Box2 &lhs_free_rectangle,
                                         const geom::Box2 &rhs_free_rectangle,
                                         place::PlacementPolicy policy)
    -> bool {
  const double lhs_short_side_fit = std::min(
      std::abs(box_width(lhs_free_rectangle) - box_width(lhs_bounds)),
      std::abs(box_height(lhs_free_rectangle) - box_height(lhs_bounds)));
  const double rhs_short_side_fit = std::min(
      std::abs(box_width(rhs_free_rectangle) - box_width(rhs_bounds)),
      std::abs(box_height(rhs_free_rectangle) - box_height(rhs_bounds)));
  if (!almost_equal(lhs_short_side_fit, rhs_short_side_fit)) {
    return lhs_short_side_fit < rhs_short_side_fit;
  }

  const double lhs_area_fit =
      box_width(lhs_free_rectangle) * box_height(lhs_free_rectangle) -
      box_width(lhs_bounds) * box_height(lhs_bounds);
  const double rhs_area_fit =
      box_width(rhs_free_rectangle) * box_height(rhs_free_rectangle) -
      box_width(rhs_bounds) * box_height(rhs_bounds);
  if (!almost_equal(lhs_area_fit, rhs_area_fit)) {
    return lhs_area_fit < rhs_area_fit;
  }

  return better_candidate(
      state, PlacementCandidate{.translated_bounds = lhs_bounds},
      PlacementCandidate{.translated_bounds = rhs_bounds}, policy);
}

[[nodiscard]] auto find_best_free_rectangle_candidate(
    const BinPackingState &state, const PieceInput &piece,
    const DecoderRequest &request, const geom::PolygonWithHoles &rotated_piece,
    const geom::Box2 &rotated_bounds,
    const geom::ResolvedRotation &resolved_rotation,
    const geom::RotationIndex &rotation_index)
    -> std::optional<PlacementCandidate> {
  std::optional<PlacementCandidate> best;
  std::optional<geom::Box2> best_free_rectangle;
  const double width = box_width(rotated_bounds);
  const double height = box_height(rotated_bounds);

  for (const geom::Box2 &free_rectangle : state.free_rectangles) {
    const geom::Box2 canonical_free_rectangle = box_for_start_corner(
        free_rectangle, state.container_bounds, state.bin_state.start_corner);
    if (box_width(canonical_free_rectangle) + kCoordinateSnap < width ||
        box_height(canonical_free_rectangle) + kCoordinateSnap < height) {
      continue;
    }

    const geom::Box2 canonical_candidate_bounds{
        .min = canonical_free_rectangle.min,
        .max = {.x = canonical_free_rectangle.min.x + width,
                .y = canonical_free_rectangle.min.y + height},
    };
    const geom::Box2 actual_candidate_bounds =
        box_for_start_corner(canonical_candidate_bounds, state.container_bounds,
                             state.bin_state.start_corner);
    if (!contains_box(state.container_bounds, actual_candidate_bounds) ||
        overlaps_any_occupied_bounds(state.occupied_bounds,
                                     actual_candidate_bounds)) {
      continue;
    }

    const geom::Point2 translation{
        .x = actual_candidate_bounds.min.x - rotated_bounds.min.x,
        .y = actual_candidate_bounds.min.y - rotated_bounds.min.y,
    };
    const auto translated_piece = translate_polygon(rotated_piece, translation);
    if (overlaps_any_exclusion_zone(translated_piece, actual_candidate_bounds,
                                    request.config.placement.exclusion_zones,
                                    state.bin_state.bin_id)) {
      continue;
    }

    const auto envelope_area =
        resulting_envelope_area(state, actual_candidate_bounds);
    const auto piece_area = polygon_area(rotated_piece);
    const auto utilization =
        envelope_area > kCoordinateSnap
            ? (state.occupied_area + piece_area) / envelope_area
            : 0.0;
    PlacementCandidate candidate{
        .shelf_index = 0,
        .starts_new_shelf = false,
        .placement =
            {
                .piece_id = piece.piece_id,
                .bin_id = state.bin_state.bin_id,
                .rotation_index = rotation_index,
                .translation = translation,
            },
        .resolved_rotation = resolved_rotation,
        .rotated_piece = rotated_piece,
        .translated_bounds = actual_candidate_bounds,
        .resulting_utilization = utilization,
    };

    if (!best.has_value() ||
        free_rectangle_better(state, candidate.translated_bounds,
                              best->translated_bounds, canonical_free_rectangle,
                              *best_free_rectangle, request.policy)) {
      best = std::move(candidate);
      best_free_rectangle = canonical_free_rectangle;
    }
  }

  return best;
}

[[nodiscard]] auto find_best_for_bin(const BinPackingState &state,
                                     const PieceInput &piece,
                                     const DecoderRequest &request)
    -> std::optional<PlacementCandidate> {
  SHINY_DEBUG("BoundingBoxPacker::find_best_for_bin: piece_id={} bin_id={} "
              "shelves={} rotations={}",
              piece.piece_id, state.bin_state.bin_id, state.shelves.size(),
              request.config.placement.allowed_rotations.angles_degrees.size());
  std::optional<PlacementCandidate> best;

  for (std::size_t rotation_value = 0;
       rotation_value <
       request.config.placement.allowed_rotations.angles_degrees.size();
       ++rotation_value) {
    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        place::resolve_rotation(rotation_index, request.config.placement);
    if (!resolved_rotation.has_value() ||
        !place::grain_compatibility_allows_rotation(
            *resolved_rotation, request.config.placement.bed_grain_direction,
            piece.grain_compatibility)) {
      continue;
    }

    const auto rotated_piece =
        rotate_polygon(piece.polygon, resolved_rotation->degrees);
    if (rotated_piece.outer.empty()) {
      continue;
    }

    const auto rotated_bounds = compute_bounds(rotated_piece);
    const auto width = rotated_bounds.max.x - rotated_bounds.min.x;
    const auto height = rotated_bounds.max.y - rotated_bounds.min.y;
    if (width < 0.0 || height < 0.0) {
      continue;
    }

    std::optional<PlacementCandidate> rotation_best;
    switch (request.config.bounding_box.heuristic) {
    case BoundingBoxHeuristic::shelf:
      rotation_best = find_best_shelf_candidate(
          state, piece, request, rotated_piece, rotated_bounds,
          *resolved_rotation, rotation_index);
      break;
    case BoundingBoxHeuristic::skyline:
      rotation_best = find_best_skyline_candidate(
          state, piece, request, rotated_piece, rotated_bounds,
          *resolved_rotation, rotation_index);
      break;
    case BoundingBoxHeuristic::free_rectangle_backfill:
      rotation_best = find_best_free_rectangle_candidate(
          state, piece, request, rotated_piece, rotated_bounds,
          *resolved_rotation, rotation_index);
      break;
    }

    if (rotation_best.has_value() &&
        (!best.has_value() ||
         better_candidate(state, *rotation_best, *best, request.policy))) {
      best = std::move(rotation_best);
    }
  }

  if (best.has_value()) {
    SHINY_DEBUG("BoundingBoxPacker::find_best_for_bin: selected piece_id={} "
                "bin_id={} shelf_index={} new_shelf={} rotation={} "
                "utilization={} translation=({}, {})",
                piece.piece_id, state.bin_state.bin_id, best->shelf_index,
                best->starts_new_shelf ? 1 : 0,
                best->placement.rotation_index.value,
                best->resulting_utilization, best->placement.translation.x,
                best->placement.translation.y);
  } else {
    SHINY_DEBUG(
        "BoundingBoxPacker::find_best_for_bin: no fit piece_id={} bin_id={}",
        piece.piece_id, state.bin_state.bin_id);
  }
  return best;
}

void apply_selection(BinPackingState &state, const PieceInput &piece,
                     const PlacementCandidate &selection,
                     std::vector<PlacementTraceEntry> &trace,
                     bool opened_new_bin, double clearance) {
  const auto translated_piece = translate_polygon(
      selection.rotated_piece, selection.placement.translation);

  state.bin_state.placements.push_back({
      .placement = selection.placement,
      .resolved_rotation = selection.resolved_rotation,
      .polygon = translated_piece,
      .source = place::PlacementCandidateSource::bin_boundary,
      .inside_hole = false,
      .hole_index = -1,
      .score = selection.resulting_utilization,
  });

  if (state.bin_state.occupied.regions.empty()) {
    state.bin_state.occupied = poly::make_merged_region(translated_piece);
  } else {
    state.bin_state.occupied = poly::merge_polygon_into_region(
        state.bin_state.occupied, translated_piece);
  }
  ++state.bin_state.occupied_region_revision;
  state.bin_state.holes.clear();
  ++state.bin_state.hole_set_revision;
  state.occupied_area += polygon_area(translated_piece);
  state.occupied_bounds.push_back(selection.translated_bounds);
  split_free_rectangles(state.free_rectangles, selection.translated_bounds);

  const auto width =
      selection.translated_bounds.max.x - selection.translated_bounds.min.x;
  const auto height =
      selection.translated_bounds.max.y - selection.translated_bounds.min.y;
  if (selection.starts_new_shelf) {
    state.shelves.push_back({
        .y = selection.translated_bounds.min.y,
        .height = height,
        .next_x = start_corner_on_right(state.bin_state.start_corner)
                      ? selection.translated_bounds.min.x - clearance
                      : selection.translated_bounds.min.x + width + clearance,
        .used_min_x = selection.translated_bounds.min.x,
        .used_max_x = selection.translated_bounds.max.x,
    });
  } else if (selection.shelf_index < state.shelves.size()) {
    ShelfState &shelf = state.shelves[selection.shelf_index];
    shelf.next_x = start_corner_on_right(state.bin_state.start_corner)
                       ? selection.translated_bounds.min.x - clearance
                       : selection.translated_bounds.max.x + clearance;
    shelf.used_min_x =
        std::min(shelf.used_min_x, selection.translated_bounds.min.x);
    shelf.used_max_x =
        std::max(shelf.used_max_x, selection.translated_bounds.max.x);
  }

  state.bin_state.utilization = summarize_bin(state.bin_state);
  trace.push_back({
      .piece_id = piece.piece_id,
      .bin_id = state.bin_state.bin_id,
      .rotation_index = selection.placement.rotation_index,
      .resolved_rotation = selection.resolved_rotation,
      .translation = selection.placement.translation,
      .source = place::PlacementCandidateSource::bin_boundary,
      .opened_new_bin = opened_new_bin,
      .inside_hole = false,
      .hole_index = -1,
      .score = selection.resulting_utilization,
  });
}

[[nodiscard]] auto
decode_single(const DecoderRequest &request,
              const InterruptionProbe &interruption_requested)
    -> DecoderResult {
  DecoderResult result{};
  SHINY_DEBUG(
      "BoundingBoxPacker::decode_single: start pieces={} bins={} policy={} "
      "rotations={} clearance={}",
      request.pieces.size(), request.bins.size(),
      static_cast<std::uint32_t>(request.policy),
      request.config.placement.allowed_rotations.angles_degrees.size(),
      request.config.placement.part_clearance);
  if (!request.config.is_valid() || request.bins.empty()) {
    SHINY_WARN(
        "BoundingBoxPacker::decode_single: invalid request config_valid={} "
        "bin_count={}",
        request.config.is_valid() ? 1 : 0, request.bins.size());
    for (const auto &piece : request.pieces) {
      result.layout.unplaced_piece_ids.push_back(piece.piece_id);
    }
    return result;
  }

  const auto interrupt_from = [&](std::size_t piece_index) {
    result.interrupted = true;
    mark_remaining_unplaced(request.pieces, piece_index,
                            result.layout.unplaced_piece_ids);
  };

  std::vector<BinPackingState> working_bins;
  std::vector<bool> opened_bins(request.bins.size(), false);
  for (std::size_t piece_index = 0; piece_index < request.pieces.size();
       ++piece_index) {
    if (interrupted(interruption_requested)) {
      interrupt_from(piece_index);
      break;
    }

    const PieceInput &piece = request.pieces[piece_index];
    SHINY_DEBUG(
        "BoundingBoxPacker::decode_single: placing piece_index={} piece_id={} "
        "working_bins={}",
        piece_index, piece.piece_id, working_bins.size());
    bool placed = false;

    for (auto &bin : working_bins) {
      if (interrupted(interruption_requested)) {
        interrupt_from(piece_index);
        break;
      }

      if (!piece_allows_bin(piece, bin.bin_state.bin_id)) {
        continue;
      }

      if (const auto selection = find_best_for_bin(bin, piece, request);
          selection.has_value()) {
        apply_selection(bin, piece, *selection, result.layout.placement_trace,
                        false, request.config.placement.part_clearance);
        SHINY_DEBUG(
            "BoundingBoxPacker::decode_single: placed into existing bin "
            "piece_id={} bin_id={} placements_in_bin={}",
            piece.piece_id, bin.bin_state.bin_id,
            bin.bin_state.placements.size());
        placed = true;
        break;
      }
    }

    if (result.interrupted) {
      break;
    }

    if (placed) {
      continue;
    }

    for (std::size_t request_bin_index = 0;
         request_bin_index < request.bins.size(); ++request_bin_index) {
      if (opened_bins[request_bin_index]) {
        continue;
      }
      const BinInput &bin_input = request.bins[request_bin_index];
      if (!piece_allows_bin(piece, bin_input.bin_id)) {
        continue;
      }

      auto new_bin = make_empty_bin(bin_input);
      SHINY_DEBUG(
          "BoundingBoxPacker::decode_single: opened candidate bin piece_id={} "
          "new_bin_id={} request_bin_index={}",
          piece.piece_id, new_bin.bin_state.bin_id, request_bin_index);
      if (const auto selection = find_best_for_bin(new_bin, piece, request);
          selection.has_value()) {
        apply_selection(new_bin, piece, *selection,
                        result.layout.placement_trace, true,
                        request.config.placement.part_clearance);
        SHINY_DEBUG("BoundingBoxPacker::decode_single: placed into new bin "
                    "piece_id={} bin_id={} total_bins={}",
                    piece.piece_id, new_bin.bin_state.bin_id,
                    working_bins.size() + 1U);
        opened_bins[request_bin_index] = true;
        working_bins.push_back(std::move(new_bin));
        placed = true;
        break;
      }
    }

    if (placed) {
      continue;
    }

    if (interrupted(interruption_requested)) {
      interrupt_from(piece_index);
      break;
    }

    result.layout.unplaced_piece_ids.push_back(piece.piece_id);
    SHINY_DEBUG("BoundingBoxPacker::decode_single: piece remained unplaced "
                "piece_id={} unplaced_count={}",
                piece.piece_id, result.layout.unplaced_piece_ids.size());
  }

  result.bins.reserve(working_bins.size());
  result.layout.bins.reserve(working_bins.size());
  for (auto &bin : working_bins) {
    result.bins.push_back(bin.bin_state);
    result.layout.bins.push_back({
        .bin_id = bin.bin_state.bin_id,
        .container = bin.bin_state.container,
        .occupied = bin.bin_state.occupied,
        .placements = bin.bin_state.placements,
        .utilization = bin.bin_state.utilization,
    });
  }

  SHINY_DEBUG("BoundingBoxPacker::decode_single: finish bins={} placements={} "
              "unplaced={} interrupted={}",
              result.layout.bins.size(), result.layout.placement_trace.size(),
              result.layout.unplaced_piece_ids.size(),
              result.interrupted ? 1 : 0);
  return result;
}

} // namespace

auto BoundingBoxPacker::decode_attempts(
    const DecoderRequest &request,
    const InterruptionProbe &interruption_requested,
    const AttemptObserver &on_attempt_complete)
    -> std::vector<DecoderResult> {
  std::vector<DecoderResult> results;
  const std::vector<DecoderRequest> attempt_requests =
      build_attempt_requests(request);
  results.reserve(attempt_requests.size());
  for (std::size_t attempt_index = 0; attempt_index < attempt_requests.size();
       ++attempt_index) {
    if (!results.empty() && interrupted(interruption_requested)) {
      break;
    }
    DecoderResult result =
        decode_single(attempt_requests[attempt_index], interruption_requested);
    const bool was_interrupted = result.interrupted;
    results.push_back(std::move(result));
    if (on_attempt_complete) {
      on_attempt_complete(attempt_index, results.back());
    }
    if (was_interrupted) {
      break;
    }
  }

  if (results.empty()) {
    results.push_back(decode_single(request, interruption_requested));
    if (on_attempt_complete) {
      on_attempt_complete(0U, results.back());
    }
  }
  return results;
}

auto BoundingBoxPacker::decode(const DecoderRequest &request,
                               const InterruptionProbe &interruption_requested)
    -> DecoderResult {
  std::vector<DecoderResult> results =
      decode_attempts(request, interruption_requested);
  if (results.empty()) {
    return {};
  }
  return std::move(results.front());
}

} // namespace shiny::nesting::pack
