#pragma once

#include <cstddef>
#include <optional>

#include "geometry/types.hpp"
#include "packing/collision_tracker.hpp"
#include "packing/sample_evaluator.hpp"
#include "runtime/deterministic_rng.hpp"

namespace shiny::nesting::pack {

struct ItemMoverConfig {
  std::size_t global_samples{32};
  std::size_t focused_samples{16};
  std::size_t coordinate_descent_iterations{24};
  double coarse_step_ratio{0.25};
  double coarse_min_step_ratio{0.02};
  double fine_step_ratio{0.01};
  double min_step_ratio{0.001};
  double angle_step_degrees{2.0};
  double min_angle_step_degrees{0.1};
  bool enable_rotation_axis{true};
};

struct ItemMove {
  geom::PolygonWithHoles polygon{};
  double weighted_loss{0.0};
};

[[nodiscard]] auto move_item(const CollisionTracker &tracker, std::size_t item_index,
                             const ItemMoverConfig &config,
                             runtime::DeterministicRng &rng) -> std::optional<ItemMove>;

} // namespace shiny::nesting::pack
