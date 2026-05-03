#include "packing/sparrow/quantify/collision_tracker.hpp"

#include <algorithm>

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/polygon.hpp"

namespace shiny::nesting::pack::sparrow::quantify {

namespace {

[[nodiscard]] auto lerp(const double min_value, const double max_value,
                        const double ratio) -> double {
  return min_value + (max_value - min_value) * ratio;
}

} // namespace

CollisionTracker::CollisionTracker(adapters::PortPolygon container,
                                   std::vector<CollisionTrackerItem> items)
    : container_(adapters::to_engine_polygon(container)) {
  items_.reserve(items.size());
  for (auto &item : items) {
    items_.push_back({.piece_id = item.piece_id,
                      .polygon = adapters::to_engine_polygon(item.polygon)});
  }

  const auto pair_count =
      items_.size() < 2U ? 0U : (items_.size() * (items_.size() - 1U)) / 2U;
  pair_entries_.resize(pair_count);
  container_losses_.resize(items_.size(), 0.0);
  container_weights_.resize(items_.size(), 1.0);

  for (std::size_t index = 0; index < items_.size(); ++index) {
    container_losses_[index] = compute_container_loss(index);
  }
  for (std::size_t lhs = 0; lhs < items_.size(); ++lhs) {
    for (std::size_t rhs = lhs + 1U; rhs < items_.size(); ++rhs) {
      pair_entries_[pair_index(lhs, rhs)].loss = compute_pair_loss(lhs, rhs);
    }
  }
}

auto CollisionTracker::item_count() const -> std::size_t {
  return items_.size();
}

auto CollisionTracker::pair_entry_count() const -> std::size_t {
  return pair_entries_.size();
}

auto CollisionTracker::container_polygon() const
    -> const geom::PolygonWithHoles & {
  return container_;
}

auto CollisionTracker::item_piece_id(const std::size_t index) const
    -> std::uint32_t {
  return items_[index].piece_id;
}

auto CollisionTracker::item_polygon(const std::size_t index) const
    -> const geom::PolygonWithHoles & {
  return items_[index].polygon;
}

auto CollisionTracker::pair_index(const std::size_t lhs,
                                  const std::size_t rhs) const -> std::size_t {
  const auto a = std::min(lhs, rhs);
  const auto b = std::max(lhs, rhs);
  return a * items_.size() - (a * (a + 1U)) / 2U + (b - a - 1U);
}

auto CollisionTracker::pair_loss(const std::size_t lhs,
                                 const std::size_t rhs) const -> double {
  return pair_entries_[pair_index(lhs, rhs)].loss;
}

auto CollisionTracker::pair_weight(const std::size_t lhs,
                                   const std::size_t rhs) const -> double {
  return pair_entries_[pair_index(lhs, rhs)].weight;
}

auto CollisionTracker::container_loss(const std::size_t index) const -> double {
  return container_losses_[index];
}

auto CollisionTracker::container_weight(const std::size_t index) const
    -> double {
  return container_weights_[index];
}

auto CollisionTracker::total_loss() const -> double {
  double total = 0.0;
  for (const auto &entry : pair_entries_) {
    total += entry.loss;
  }
  for (const auto loss : container_losses_) {
    total += loss;
  }
  return total;
}

auto CollisionTracker::weighted_total_loss() const -> double {
  double total = 0.0;
  for (const auto &entry : pair_entries_) {
    total += entry.weight * entry.loss;
  }
  for (std::size_t index = 0; index < container_losses_.size(); ++index) {
    total += container_weights_[index] * container_losses_[index];
  }
  return total;
}

auto CollisionTracker::register_item_polygon(
    const std::size_t index, const adapters::PortPolygon &polygon) -> void {
  items_[index].polygon = adapters::to_engine_polygon(polygon);
  container_losses_[index] = compute_container_loss(index);
  for (std::size_t other = 0; other < items_.size(); ++other) {
    if (other == index) {
      continue;
    }
    pair_entries_[pair_index(index, other)].loss =
        compute_pair_loss(index, other);
  }
}

auto CollisionTracker::update_gls_weights(const double weight_cap) -> void {
  const auto apply_decay = [](const double weight) {
    return std::max(1.0, weight * 0.95);
  };

  double max_loss = 0.0;
  for (const auto &entry : pair_entries_) {
    max_loss = std::max(max_loss, entry.loss);
  }
  for (const auto loss : container_losses_) {
    max_loss = std::max(max_loss, loss);
  }

  if (max_loss <= 0.0) {
    for (auto &entry : pair_entries_) {
      entry.weight = apply_decay(entry.weight);
    }
    for (auto &weight : container_weights_) {
      weight = apply_decay(weight);
    }
    return;
  }

  for (auto &entry : pair_entries_) {
    if (entry.loss > 0.0) {
      entry.weight = std::min(
          weight_cap, entry.weight * lerp(1.2, 2.0, entry.loss / max_loss));
    } else {
      entry.weight = apply_decay(entry.weight);
    }
  }
  for (std::size_t index = 0; index < container_losses_.size(); ++index) {
    if (container_losses_[index] > 0.0) {
      container_weights_[index] = std::min(
          weight_cap, container_weights_[index] *
                          lerp(1.2, 2.0, container_losses_[index] / max_loss));
    } else {
      container_weights_[index] = apply_decay(container_weights_[index]);
    }
  }
}

auto CollisionTracker::compute_pair_loss(const std::size_t lhs,
                                         const std::size_t rhs) const
    -> double {
  return geom::polygon_area_sum(
      geom::intersection_polygons(items_[lhs].polygon, items_[rhs].polygon));
}

auto CollisionTracker::compute_container_loss(const std::size_t index) const
    -> double {
  return geom::polygon_area_sum(
      geom::difference_polygons(items_[index].polygon, container_));
}

} // namespace shiny::nesting::pack::sparrow::quantify