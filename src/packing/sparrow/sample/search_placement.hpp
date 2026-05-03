#pragma once

#include <cstddef>
#include <optional>
#include <span>

#include "packing/sparrow/quantify/collision_tracker.hpp"
#include "packing/sparrow/runtime/rng.hpp"
#include "packing/sparrow/solution.hpp"

namespace shiny::nesting::pack::sparrow::sample {

struct SearchPlacementPolicy {
  SolveProfile profile{SolveProfile::balanced};
  std::size_t global_samples{0};
  std::size_t focused_samples{0};

  [[nodiscard]] static auto for_profile(SolveProfile profile)
      -> SearchPlacementPolicy;
};

struct SearchPlacementCandidate {
  geom::PolygonWithHoles polygon{};
  geom::Point2 translation{};
  double rotation_degrees{0.0};
  double weighted_loss{0.0};
  bool from_constructive_seed{false};
};

struct SearchPlacementRequest {
  const quantify::CollisionTracker *tracker{nullptr};
  std::size_t moving_index{0};
  const SeedSolution *seed_solution{nullptr};
  std::span<const double> allowed_rotations_degrees{};
  SearchPlacementPolicy policy{};
};

struct SearchPlacementResult {
  SearchPlacementCandidate best_candidate{};
  std::size_t sampled_candidates{0};
  std::size_t focused_candidates{0};
};

[[nodiscard]] auto search_placement(const SearchPlacementRequest &request,
                                    runtime::SplitMix64Rng &rng)
    -> std::optional<SearchPlacementResult>;

} // namespace shiny::nesting::pack::sparrow::sample