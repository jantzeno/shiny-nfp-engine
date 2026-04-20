#include "packing/bounding_box/ordering.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "packing/common.hpp"

namespace shiny::nesting::pack {

auto piece_metrics_for(const PieceInput &piece,
                       const std::size_t original_index)
    -> PieceOrderingMetrics {
  const geom::Box2 bounds = compute_bounds(piece.polygon);
  const double width = bounds.max.x - bounds.min.x;
  const double height = bounds.max.y - bounds.min.y;
  const double polygon_piece_area = polygon_area(piece.polygon);
  const double piece_box_area = width * height;
  return {
      .original_index = original_index,
      .width = width,
      .height = height,
      .polygon_area = polygon_piece_area,
      .box_area = piece_box_area,
      .max_dimension = std::max(width, height),
      .min_dimension = std::min(width, height),
      .box_waste_area = std::max(0.0, piece_box_area - polygon_piece_area),
      .fill_ratio = piece_box_area > kCoordinateSnap
                        ? polygon_piece_area / piece_box_area
                        : 0.0,
  };
}

namespace {

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

template <typename Comparator>
auto sort_order(std::vector<std::size_t> &order, const Comparator &comparator)
    -> void {
  std::stable_sort(order.begin(), order.end(), comparator);
}

} // namespace

auto piece_order_key(const DecoderRequest &request,
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

auto reorder_request(const DecoderRequest &request,
                     std::span<const std::size_t> order) -> DecoderRequest {
  DecoderRequest reordered = request;
  reordered.pieces.clear();
  reordered.pieces.reserve(order.size());
  for (const std::size_t index : order) {
    reordered.pieces.push_back(request.pieces[index]);
  }
  return reordered;
}

auto find_piece_index_by_id(const DecoderRequest &request,
                            const std::uint32_t piece_id)
    -> std::optional<std::size_t> {
  for (std::size_t index = 0; index < request.pieces.size(); ++index) {
    if (request.pieces[index].piece_id == piece_id) {
      return index;
    }
  }
  return std::nullopt;
}

auto move_order_index(std::vector<std::size_t> &order, const std::size_t from,
                      const std::size_t to) -> void {
  if (from >= order.size() || to >= order.size() || from == to) {
    return;
  }

  const std::size_t value = order[from];
  order.erase(order.begin() + static_cast<std::ptrdiff_t>(from));
  order.insert(order.begin() + static_cast<std::ptrdiff_t>(to), value);
}

auto build_attempt_requests(const DecoderRequest &request)
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

  std::vector<std::size_t> descending_box_area_then_fill = original_order;
  sort_order(descending_box_area_then_fill, [&metrics](const std::size_t lhs,
                                                       const std::size_t rhs) {
    const PieceOrderingMetrics &lhs_metrics = metrics[lhs];
    const PieceOrderingMetrics &rhs_metrics = metrics[rhs];
    if (!almost_equal(lhs_metrics.box_area, rhs_metrics.box_area)) {
      return lhs_metrics.box_area > rhs_metrics.box_area;
    }
    if (!almost_equal(lhs_metrics.fill_ratio, rhs_metrics.fill_ratio)) {
      return lhs_metrics.fill_ratio < rhs_metrics.fill_ratio;
    }
    if (!almost_equal(lhs_metrics.max_dimension, rhs_metrics.max_dimension)) {
      return lhs_metrics.max_dimension > rhs_metrics.max_dimension;
    }
    return lhs_metrics.original_index < rhs_metrics.original_index;
  });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_box_area_then_fill));

  std::vector<std::size_t> descending_dimension_pair = original_order;
  sort_order(descending_dimension_pair, [&metrics](const std::size_t lhs,
                                                   const std::size_t rhs) {
    const PieceOrderingMetrics &lhs_metrics = metrics[lhs];
    const PieceOrderingMetrics &rhs_metrics = metrics[rhs];
    if (!almost_equal(lhs_metrics.max_dimension, rhs_metrics.max_dimension)) {
      return lhs_metrics.max_dimension > rhs_metrics.max_dimension;
    }
    if (!almost_equal(lhs_metrics.min_dimension, rhs_metrics.min_dimension)) {
      return lhs_metrics.min_dimension > rhs_metrics.min_dimension;
    }
    if (!almost_equal(lhs_metrics.box_area, rhs_metrics.box_area)) {
      return lhs_metrics.box_area > rhs_metrics.box_area;
    }
    return lhs_metrics.original_index < rhs_metrics.original_index;
  });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_dimension_pair));

  std::vector<std::size_t> descending_box_area = original_order;
  sort_order_descending(
      descending_box_area, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.box_area; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_box_area));

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

  std::vector<std::size_t> descending_min_dimension = original_order;
  sort_order_descending(
      descending_min_dimension, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.min_dimension; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_min_dimension));

  std::vector<std::size_t> descending_box_waste = original_order;
  sort_order_descending(
      descending_box_waste, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.box_waste_area; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_box_waste));

  std::vector<std::size_t> descending_polygon_area = original_order;
  sort_order_descending(
      descending_polygon_area, metrics,
      [](const PieceOrderingMetrics &entry) { return entry.polygon_area; });
  append_unique_order(request, orders, order_keys,
                      std::move(descending_polygon_area));

  append_unique_order(request, orders, order_keys, original_order);

  std::vector<std::size_t> reversed_order = original_order;
  std::reverse(reversed_order.begin(), reversed_order.end());
  append_unique_order(request, orders, order_keys, std::move(reversed_order));

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

} // namespace shiny::nesting::pack
