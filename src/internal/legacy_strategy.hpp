#pragma once

// Internal header: not part of the public exported surface.
//
// Contains the strategy-era execution types (StrategyKind,
// ProductionOptimizerKind, ExecutionPolicy, and associated configs) that have
// been moved off the primary API path. Include this header only in
// implementation files and internal engine boundaries that need to dispatch
// through the legacy NestingRequest path.
//
// Downstream consumers should use ProfileRequest and ProfileRequestBuilder
// instead. The legacy NestingRequest solve overload is declared in
// src/internal/legacy_solve.hpp.

#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>

#include "geometry/polygon.hpp"
#include "packing/config.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"

namespace shiny::nesting {

// These enums are also used by the public ProfileRequest surface and are
// transitively visible to any file that includes request.hpp (which includes
// this header). They live here because both public and legacy types depend on
// them and this avoids a circular include.

enum class PieceOrdering : std::uint8_t {
  input = 0,
  largest_area_first = 1,
  hull_diameter_first = 2,
  priority = 3,
};

enum class ObjectiveMode : std::uint8_t {
  placement_count = 0,
  maximize_value = 1,
};

enum class StrategyKind : std::uint8_t {
  bounding_box = 0,
  metaheuristic_search = 2,
};

enum class ProductionOptimizerKind : std::uint8_t {
  brkga = 0,
};

enum class CoolingScheduleKind : std::uint8_t {
  geometric = 0,
  linear = 1,
  adaptive = 2,
  lundy_mees = 3,
};

enum class CandidateStrategy : std::uint8_t {
  // Uses constructive anchors, skyline points, and container/exclusion-domain
  // vertices only; obstacle overlap rejection stays in the placement evaluator,
  // so this path avoids expensive obstacle NFPs by default.
  anchor_vertex = 0,
  // Uses exact IFP/domain vertices plus exact blocked-region vertices when NFP
  // computation succeeds. Conservative bbox fallback entries stay explicitly
  // marked as fallback.
  nfp_perfect = 1,
  // Extends `nfp_perfect` with blocked/domain boundary intersections to capture
  // sliding placements, again with explicit fallback labelling when bbox NFP
  // approximation was required.
  nfp_arrangement = 2,
  // Unions constructive anchors/skyline with the NFP-driven candidates above so
  // robust fallback remains visible without hiding exact-vs-fallback semantics.
  nfp_hybrid = 3,
  count = 4,
};

struct ProductionSearchConfig {
  std::size_t population_size{24};
  std::size_t elite_count{6};
  std::size_t mutant_count{4};
  std::size_t max_iterations{24};
  double elite_bias{0.7};
  std::size_t diversification_swaps{2};
  std::size_t polishing_passes{1};
  double strip_exploration_ratio{0.8};
  double strip_exploration_shrink_max_ratio{0.25};
  double strip_exploration_shrink_min_ratio{0.02};
  double strip_compression_shrink_max_ratio{0.01};
  double strip_compression_shrink_min_ratio{0.001};
  std::size_t infeasible_pool_capacity{4};
  std::size_t infeasible_rollback_after{2};
  std::size_t separator_worker_count{1};
  std::size_t separator_max_iterations{48};
  std::size_t separator_iter_no_improvement_limit{8};
  std::size_t separator_strike_limit{3};
  std::size_t separator_global_samples{32};
  std::size_t separator_focused_samples{16};
  std::size_t separator_coordinate_descent_iterations{24};
  // Upper bound applied by `CollisionTracker::update_gls_weights` to the
  // multiplicative GLS weight per pair/container constraint. Chosen
  // (1e6) as the practical ceiling from Sparrow's reference; raising
  // it risks saturating doubles after many consecutive amplifications.
  double gls_weight_cap{1e6};
  // Plateau detection window for the Sparrow strip optimizer's internal
  // acceptance history. Equivalent to SAConfig::plateau_window but lives here
  // so that SAConfig can be fully decoupled from ExecutionPolicy.
  std::size_t plateau_window{8};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct SAConfig {
  CoolingScheduleKind cooling_schedule{CoolingScheduleKind::geometric};
  std::size_t max_refinements{48};
  std::size_t restart_count{2};
  double initial_temperature{0.25};
  double final_temperature{0.01};
  double lundy_beta{0.025};
  std::size_t plateau_window{8};
  double reheating_factor{1.5};
  std::size_t perturbation_swaps{2};
};

struct ALNSConfig {
  std::size_t max_refinements{48};
  std::size_t destroy_min_count{1};
  std::size_t destroy_max_count{3};
  double initial_acceptance_ratio{0.04};
  double final_acceptance_ratio{0.005};
  double reaction_factor{0.2};
  double reward_improve{4.0};
  double reward_accept{1.5};
  double reward_reject{0.25};
  std::size_t segment_length{8};
};

struct LAHCConfig {
  std::size_t max_refinements{48};
  std::size_t history_length{12};
  std::size_t plateau_limit{16};
  std::size_t perturbation_swaps{2};
};

struct IrregularOptions {
  CandidateStrategy candidate_strategy{CandidateStrategy::nfp_hybrid};
  std::size_t max_candidate_points{1200};
  double candidate_gaussian_sigma{0.5};
  PieceOrdering piece_ordering{PieceOrdering::largest_area_first};
  bool merge_free_regions{true};
  bool enable_direct_overlap_check{true};
  bool enable_compaction{true};
  bool enable_backtracking{true};
  bool enable_gap_fill{true};
  std::uint32_t max_backtrack_pieces{3};
  std::uint32_t compaction_passes{2};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct StrategyConfig {
  StrategyKind kind{StrategyKind::bounding_box};
  using Payload = std::variant<std::monostate>;
  Payload payload{};
};

struct ProductionStrategyConfig {
  ProductionOptimizerKind kind{ProductionOptimizerKind::brkga};
  using Payload = std::variant<std::monostate, ProductionSearchConfig>;
  Payload payload{};
};

struct ExecutionPolicy {
  StrategyKind strategy{StrategyKind::bounding_box};
  ProductionOptimizerKind production_optimizer{ProductionOptimizerKind::brkga};
  ObjectiveMode objective_mode{ObjectiveMode::placement_count};
  StrategyConfig strategy_config{};
  ProductionStrategyConfig production_strategy_config{};
  place::PlacementPolicy placement_policy{place::PlacementPolicy::bottom_left};
  geom::DiscreteRotationSet default_rotations{{0.0, 90.0, 180.0, 270.0}};
  double part_spacing{0.0};
  bool allow_part_overflow{true};
  bool enable_part_in_part_placement{false};
  bool explore_concave_candidates{false};
  std::vector<std::uint32_t> selected_bin_ids{};
  pack::BoundingBoxPackingConfig bounding_box{};
  pack::DeterministicAttemptConfig deterministic_attempts{};
  IrregularOptions irregular{};
  ProductionSearchConfig production{};
};

template <typename Config>
[[nodiscard]] auto
resolve_primary_strategy_config(const ExecutionPolicy &execution,
                                StrategyKind direct_kind, const Config &legacy)
    -> const Config & {
  if (execution.strategy_config.kind == direct_kind) {
    if (const auto *config =
            std::get_if<Config>(&execution.strategy_config.payload);
        config != nullptr) {
      return *config;
    }
  }
  return legacy;
}

template <typename Config>
[[nodiscard]] auto has_primary_strategy_config(const ExecutionPolicy &execution,
                                               StrategyKind direct_kind)
    -> bool {
  return execution.strategy_config.kind == direct_kind &&
         std::get_if<Config>(&execution.strategy_config.payload) != nullptr;
}

template <typename Config>
auto set_primary_strategy_config(ExecutionPolicy &execution,
                                 StrategyKind direct_kind, Config config)
    -> void {
  execution.strategy_config.kind = direct_kind;
  execution.strategy_config.payload = std::move(config);
}

template <typename Config>
[[nodiscard]] auto
has_production_strategy_config(const ExecutionPolicy &execution,
                               ProductionOptimizerKind production_kind)
    -> bool {
  return execution.production_strategy_config.kind == production_kind &&
         std::get_if<Config>(&execution.production_strategy_config.payload) !=
             nullptr;
}

template <typename Config>
auto set_production_strategy_config(ExecutionPolicy &execution,
                                    ProductionOptimizerKind production_kind,
                                    Config config) -> void {
  execution.production_strategy_config.kind = production_kind;
  execution.production_strategy_config.payload = std::move(config);
}

template <typename Config>
[[nodiscard]] auto primary_strategy_config_ptr(const ExecutionPolicy &execution,
                                               StrategyKind direct_kind)
    -> const Config * {
  if (execution.strategy_config.kind == direct_kind) {
    return std::get_if<Config>(&execution.strategy_config.payload);
  }
  return nullptr;
}

template <typename Config>
[[nodiscard]] auto
production_strategy_config_ptr(const ExecutionPolicy &execution,
                               ProductionOptimizerKind production_kind)
    -> const Config * {
  if (execution.production_strategy_config.kind == production_kind) {
    return std::get_if<Config>(&execution.production_strategy_config.payload);
  }
  return nullptr;
}

template <typename Config>
[[nodiscard]] auto
resolve_production_strategy_config(const ExecutionPolicy &execution,
                                   ProductionOptimizerKind production_kind,
                                   const Config &legacy) -> const Config & {
  if (const auto *config =
          production_strategy_config_ptr<Config>(execution, production_kind);
      config != nullptr) {
    return *config;
  }
  return legacy;
}

template <typename Config>
[[nodiscard]] auto
resolve_strategy_config(const ExecutionPolicy &execution,
                        StrategyKind direct_kind,
                        ProductionOptimizerKind production_kind,
                        const Config &legacy) -> const Config & {
  if (execution.strategy == StrategyKind::metaheuristic_search &&
      execution.production_optimizer == production_kind) {
    return resolve_production_strategy_config(execution, production_kind,
                                              legacy);
  }
  return resolve_primary_strategy_config(execution, direct_kind, legacy);
}

} // namespace shiny::nesting
