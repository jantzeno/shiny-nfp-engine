#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "cache/collision_pair_loss_cache.hpp"
#include "cache/penetration_depth_cache.hpp"
#include "cache/pole_cache.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::pack {

// Free-function form of `CollisionTracker::compute_pair_loss` (the
// non-weighted overlap loss between two polygons). Intentionally
// shared with the tracker so `sample_evaluator` and the tracker
// always agree on the pair-loss formula. Returns the same `loss` /
// `exact` decomposition as the tracker.
[[nodiscard]] auto compute_polygon_pair_loss(
    const geom::PolygonWithHoles &lhs, std::uint64_t lhs_revision,
    const geom::PolygonWithHoles &rhs, std::uint64_t rhs_revision,
    cache::PoleCache *pole_cache = nullptr,
    cache::PenetrationDepthCache *pd_cache = nullptr)
    -> std::pair<double, double>;

[[nodiscard]] auto compute_polygon_pair_loss(const geom::PolygonWithHoles &lhs,
                                             const geom::PolygonWithHoles &rhs)
    -> std::pair<double, double>;

struct CollisionTrackerItem {
  std::uint32_t item_id{0};
  std::uint64_t geometry_revision{0};
  geom::PolygonWithHoles polygon{};
  bool rotation_locked{false};
};

class CollisionTracker {
public:
  CollisionTracker(geom::PolygonWithHoles container,
                   std::vector<CollisionTrackerItem> items);

  [[nodiscard]] auto item_count() const -> std::size_t;
  [[nodiscard]] auto item(std::size_t index) const
      -> const CollisionTrackerItem &;
  [[nodiscard]] auto item_polygon_revision(std::size_t index) const
      -> std::uint64_t;
  [[nodiscard]] auto container() const -> const geom::PolygonWithHoles &;
  [[nodiscard]] auto pole_cache() const -> cache::PoleCache *;
  [[nodiscard]] auto penetration_depth_cache() const
      -> cache::PenetrationDepthCache *;
  [[nodiscard]] auto pair_loss_cache_size() const -> std::size_t;
  [[nodiscard]] auto pair_loss_cache_stats() const
      -> cache::CollisionPairLossCacheStats;

  auto register_item_move(std::size_t index,
                          const geom::PolygonWithHoles &polygon) -> void;

  [[nodiscard]] auto pair_loss(std::size_t lhs, std::size_t rhs) const
      -> double;
  [[nodiscard]] auto pair_weight(std::size_t lhs, std::size_t rhs) const
      -> double;
  [[nodiscard]] auto container_loss(std::size_t index) const -> double;
  [[nodiscard]] auto container_weight(std::size_t index) const -> double;
  [[nodiscard]] auto total_loss() const -> double;
  [[nodiscard]] auto weighted_total_loss() const -> double;

  // Sum of *actual* overlap areas (intersection-of-polygons) plus
  // out-of-container areas. Excludes the soft "proxy" pressure that
  // `compute_pair_loss` adds when polygons are merely close. This is
  // the correct convergence signal for separators and strip
  // optimisers — `exact_total_loss() == 0` ⇒ feasible packing.
  [[nodiscard]] auto exact_total_loss() const -> double;
  [[nodiscard]] auto pair_exact_loss(std::size_t lhs, std::size_t rhs) const
      -> double;

  auto update_gls_weights(double weight_cap = 1e6) -> void;

private:
  struct PairEntry {
    double loss{0.0};
    double exact{0.0};
    double weight{1.0};
  };

  [[nodiscard]] auto pair_index(std::size_t lhs, std::size_t rhs) const
      -> std::size_t;
  [[nodiscard]] auto compute_pair_loss(std::size_t lhs, std::size_t rhs) const
      -> PairEntry;
  [[nodiscard]] auto compute_container_loss(std::size_t index) const -> double;

  geom::PolygonWithHoles container_{};
  std::vector<CollisionTrackerItem> items_{};
  std::vector<std::uint64_t> polygon_revisions_{};
  std::vector<PairEntry> pair_entries_{};
  std::vector<double> container_losses_{};
  std::vector<double> container_weights_{};
  mutable cache::CollisionPairLossCache pair_loss_cache_{
      cache::default_collision_pair_loss_cache_config()};
  mutable std::uint64_t pair_loss_cache_hits_{0};
  mutable std::uint64_t pair_loss_cache_misses_{0};
  cache::PoleCache pole_cache_{cache::default_pole_cache_config()};
  cache::PenetrationDepthCache penetration_depth_cache_{
      cache::default_pd_cache_config()};
};

} // namespace shiny::nesting::pack
