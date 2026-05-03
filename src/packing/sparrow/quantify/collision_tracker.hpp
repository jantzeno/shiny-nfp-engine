#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "packing/sparrow/adapters/geometry_adapter.hpp"

namespace shiny::nesting::pack::sparrow::quantify {

struct CollisionTrackerItem {
  std::uint32_t piece_id{0};
  adapters::PortPolygon polygon{};
};

class CollisionTracker {
public:
  CollisionTracker(adapters::PortPolygon container,
                   std::vector<CollisionTrackerItem> items);

  [[nodiscard]] auto item_count() const -> std::size_t;
  [[nodiscard]] auto pair_entry_count() const -> std::size_t;
  [[nodiscard]] auto pair_index(std::size_t lhs, std::size_t rhs) const
      -> std::size_t;
  [[nodiscard]] auto container_polygon() const
      -> const geom::PolygonWithHoles &;
  [[nodiscard]] auto item_piece_id(std::size_t index) const -> std::uint32_t;
  [[nodiscard]] auto item_polygon(std::size_t index) const
      -> const geom::PolygonWithHoles &;

  [[nodiscard]] auto pair_loss(std::size_t lhs, std::size_t rhs) const
      -> double;
  [[nodiscard]] auto pair_weight(std::size_t lhs, std::size_t rhs) const
      -> double;
  [[nodiscard]] auto container_loss(std::size_t index) const -> double;
  [[nodiscard]] auto container_weight(std::size_t index) const -> double;
  [[nodiscard]] auto total_loss() const -> double;
  [[nodiscard]] auto weighted_total_loss() const -> double;

  auto register_item_polygon(std::size_t index,
                             const adapters::PortPolygon &polygon) -> void;
  auto update_gls_weights(double weight_cap = 1e6) -> void;

private:
  struct PairEntry {
    double loss{0.0};
    double weight{1.0};
  };

  struct TrackedItem {
    std::uint32_t piece_id{0};
    geom::PolygonWithHoles polygon{};
  };

  [[nodiscard]] auto compute_pair_loss(std::size_t lhs, std::size_t rhs) const
      -> double;
  [[nodiscard]] auto compute_container_loss(std::size_t index) const -> double;

  geom::PolygonWithHoles container_{};
  std::vector<TrackedItem> items_{};
  std::vector<PairEntry> pair_entries_{};
  std::vector<double> container_losses_{};
  std::vector<double> container_weights_{};
};

} // namespace shiny::nesting::pack::sparrow::quantify