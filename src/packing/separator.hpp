#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "packing/collision_tracker.hpp"
#include "packing/item_mover.hpp"

namespace shiny::nesting::pack {

struct SeparatorConfig {
  std::size_t strike_limit{4};
  std::size_t iter_no_improvement_limit{10};
  std::size_t max_iterations{64};
  std::size_t worker_count{1};
  // Mirrors `ProductionSearchConfig::gls_weight_cap`; passed straight
  // to `CollisionTracker::update_gls_weights` each inner iteration.
  double gls_weight_cap{1e6};
  ItemMoverConfig mover{};
};

struct SeparationResult {
  std::vector<geom::PolygonWithHoles> polygons{};
  double total_loss{0.0};
  bool converged{false};
  std::size_t iterations{0};
};

[[nodiscard]] auto run_separator(const geom::PolygonWithHoles &container,
                                 const std::vector<CollisionTrackerItem> &items,
                                 const SeparatorConfig &config,
                                 std::uint64_t seed = 0) -> SeparationResult;

} // namespace shiny::nesting::pack
