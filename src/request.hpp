#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "geometry/polygon.hpp"
#include "packing/bin_identity.hpp"
#include "packing/config.hpp"
#include "packing/decoder.hpp"
#include "placement/config.hpp"
#include "placement/types.hpp"
#include "polygon_ops/simplify.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

enum class StrategyKind : std::uint8_t {
  bounding_box = 0,
  sequential_backtrack = 1,
  metaheuristic_search = 2,
  simulated_annealing = 3,
  alns = 4,
  gdrr = 5,
  lahc = 6,
};

enum class ProductionOptimizerKind : std::uint8_t {
  brkga = 0,
  simulated_annealing = 1,
  alns = 2,
  gdrr = 3,
  lahc = 4,
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

enum class PieceOrdering : std::uint8_t {
  input = 0,
  largest_area_first = 1,
  hull_diameter_first = 2,
  priority = 3,
};

struct PreprocessPolicy {
  double simplify_epsilon{0.0};
  bool normalize_piece_origins{true};
  bool discard_empty_bins{true};
};

struct PieceRequest {
  std::uint32_t piece_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint32_t quantity{1};
  std::uint64_t geometry_revision{0};
  std::int32_t priority{0};
  bool allow_mirror{false};
  // When provided, this narrows `ExecutionPolicy::default_rotations`.
  std::optional<geom::DiscreteRotationSet> allowed_rotations{};
  place::PartGrainCompatibility grain_compatibility{
      place::PartGrainCompatibility::unrestricted};
  // Effective bin eligibility is the intersection of these ids with
  // `ExecutionPolicy::selected_bin_ids`.
  std::vector<std::uint32_t> allowed_bin_ids{};
};

struct BinRequest {
  std::uint32_t bin_id{0};
  geom::PolygonWithHoles polygon{};
  std::uint32_t stock{1};
  std::uint64_t geometry_revision{0};
  place::PlacementStartCorner start_corner{
      place::PlacementStartCorner::bottom_left};
  std::vector<place::BedExclusionZone> exclusion_zones{};
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

  [[nodiscard]] auto is_valid() const -> bool;
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

  [[nodiscard]] auto is_valid() const -> bool;
};

struct GDRRConfig {
  std::size_t max_refinements{48};
  double initial_goal_ratio{0.98};
  double goal_decay{0.995};
  std::size_t ruin_swap_count{2};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct LAHCConfig {
  std::size_t max_refinements{48};
  std::size_t history_length{12};
  std::size_t plateau_limit{16};
  std::size_t perturbation_swaps{2};

  [[nodiscard]] auto is_valid() const -> bool;
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
  std::uint32_t max_backtrack_pieces{3};
  std::uint32_t compaction_passes{2};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct StrategyConfig {
  StrategyKind kind{StrategyKind::bounding_box};
  using Payload = std::variant<std::monostate, SAConfig, ALNSConfig, GDRRConfig,
                               LAHCConfig>;
  Payload payload{};
};

struct ProductionStrategyConfig {
  ProductionOptimizerKind kind{ProductionOptimizerKind::brkga};
  using Payload = std::variant<std::monostate, ProductionSearchConfig, SAConfig,
                               ALNSConfig, GDRRConfig, LAHCConfig>;
  Payload payload{};
};

struct ExecutionPolicy {
  StrategyKind strategy{StrategyKind::bounding_box};
  ProductionOptimizerKind production_optimizer{ProductionOptimizerKind::brkga};
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
  SAConfig simulated_annealing{};
  ALNSConfig alns{};
  GDRRConfig gdrr{};
  LAHCConfig lahc{};
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

struct NestingRequest {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  ExecutionPolicy execution{};

  [[nodiscard]] auto is_valid() const -> bool;
};

struct ExpandedPieceInstance {
  std::uint32_t source_piece_id{0};
  std::uint32_t expanded_piece_id{0};
  std::uint32_t instance_index{0};
};

struct ExpandedBinInstance {
  std::uint32_t source_bin_id{0};
  std::uint32_t expanded_bin_id{0};
  std::uint32_t stock_index{0};
  pack::BinIdentity identity{};
};

struct NormalizedRequest {
  NestingRequest request{};
  std::vector<ExpandedPieceInstance> expanded_pieces{};
  std::vector<ExpandedBinInstance> expanded_bins{};
  std::vector<std::optional<geom::RotationIndex>> forced_rotations{};
};

[[nodiscard]] auto normalize_request(const NestingRequest &request)
    -> util::StatusOr<NormalizedRequest>;

[[nodiscard]] auto
to_bounding_box_decoder_request(const NormalizedRequest &request)
    -> util::StatusOr<pack::DecoderRequest>;

} // namespace shiny::nesting
