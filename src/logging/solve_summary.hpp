#pragma once

#include <algorithm>
#include <cstdint>
#include <format>
#include <numeric>
#include <span>
#include <string>
#include <string_view>

#include "request.hpp"
#include "solve.hpp"

namespace shiny::nesting::log {

[[nodiscard]] constexpr auto bool_name(const bool value) -> std::string_view {
  return value ? "true" : "false";
}

[[nodiscard]] constexpr auto strategy_name(const StrategyKind kind)
    -> std::string_view {
  switch (kind) {
  case StrategyKind::bounding_box:
    return "bounding_box";
  case StrategyKind::sequential_backtrack:
    return "sequential_backtrack";
  case StrategyKind::metaheuristic_search:
    return "metaheuristic_search";
  case StrategyKind::simulated_annealing:
    return "simulated_annealing";
  case StrategyKind::alns:
    return "alns";
  case StrategyKind::gdrr:
    return "gdrr";
  case StrategyKind::lahc:
    return "lahc";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
production_optimizer_name(const ProductionOptimizerKind kind)
    -> std::string_view {
  switch (kind) {
  case ProductionOptimizerKind::brkga:
    return "brkga";
  case ProductionOptimizerKind::simulated_annealing:
    return "simulated_annealing";
  case ProductionOptimizerKind::alns:
    return "alns";
  case ProductionOptimizerKind::gdrr:
    return "gdrr";
  case ProductionOptimizerKind::lahc:
    return "lahc";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
candidate_strategy_name(const CandidateStrategy strategy) -> std::string_view {
  switch (strategy) {
  case CandidateStrategy::anchor_vertex:
    return "anchor_vertex";
  case CandidateStrategy::nfp_perfect:
    return "nfp_perfect";
  case CandidateStrategy::nfp_arrangement:
    return "nfp_arrangement";
  case CandidateStrategy::nfp_hybrid:
    return "nfp_hybrid";
  case CandidateStrategy::count:
    return "count";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto piece_ordering_name(const PieceOrdering ordering)
    -> std::string_view {
  switch (ordering) {
  case PieceOrdering::input:
    return "input";
  case PieceOrdering::largest_area_first:
    return "largest_area_first";
  case PieceOrdering::hull_diameter_first:
    return "hull_diameter_first";
  case PieceOrdering::priority:
    return "priority";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
placement_policy_name(const place::PlacementPolicy policy) -> std::string_view {
  switch (policy) {
  case place::PlacementPolicy::bottom_left:
    return "bottom_left";
  case place::PlacementPolicy::minimum_length:
    return "minimum_length";
  case place::PlacementPolicy::maximum_utilization:
    return "maximum_utilization";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
bounding_box_heuristic_name(const pack::BoundingBoxHeuristic heuristic)
    -> std::string_view {
  switch (heuristic) {
  case pack::BoundingBoxHeuristic::shelf:
    return "shelf";
  case pack::BoundingBoxHeuristic::skyline:
    return "skyline";
  case pack::BoundingBoxHeuristic::free_rectangle_backfill:
    return "free_rectangle_backfill";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
cooling_schedule_name(const CoolingScheduleKind schedule) -> std::string_view {
  switch (schedule) {
  case CoolingScheduleKind::geometric:
    return "geometric";
  case CoolingScheduleKind::linear:
    return "linear";
  case CoolingScheduleKind::adaptive:
    return "adaptive";
  case CoolingScheduleKind::lundy_mees:
    return "lundy_mees";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto seed_mode_name(const SeedProgressionMode mode)
    -> std::string_view {
  switch (mode) {
  case SeedProgressionMode::increment:
    return "increment";
  case SeedProgressionMode::decrement:
    return "decrement";
  case SeedProgressionMode::random:
    return "random";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto stop_reason_name(const StopReason reason)
    -> std::string_view {
  switch (reason) {
  case StopReason::none:
    return "none";
  case StopReason::completed:
    return "completed";
  case StopReason::cancelled:
    return "cancelled";
  case StopReason::operation_limit_reached:
    return "operation_limit_reached";
  case StopReason::time_limit_reached:
    return "time_limit_reached";
  case StopReason::invalid_request:
    return "invalid_request";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto runner_class_name(const StrategyKind strategy)
    -> std::string_view {
  switch (strategy) {
  case StrategyKind::bounding_box:
    return "shiny::nesting::pack::BoundingBoxPacker";
  case StrategyKind::sequential_backtrack:
    return "shiny::nesting::pack::SequentialBacktrackPacker";
  case StrategyKind::metaheuristic_search:
    return "shiny::nesting::search::BrkgaProductionSearch";
  case StrategyKind::simulated_annealing:
    return "shiny::nesting::search::SimulatedAnnealingSearch";
  case StrategyKind::alns:
    return "shiny::nesting::search::AlnsSearch";
  case StrategyKind::gdrr:
    return "shiny::nesting::search::GdrrSearch";
  case StrategyKind::lahc:
    return "shiny::nesting::search::LahcSearch";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
production_runner_class_name(const ProductionOptimizerKind optimizer)
    -> std::string_view {
  switch (optimizer) {
  case ProductionOptimizerKind::brkga:
    return "shiny::nesting::search::BrkgaProductionSearch";
  case ProductionOptimizerKind::simulated_annealing:
    return "shiny::nesting::search::SimulatedAnnealingSearch";
  case ProductionOptimizerKind::alns:
    return "shiny::nesting::search::AlnsSearch";
  case ProductionOptimizerKind::gdrr:
    return "shiny::nesting::search::GdrrSearch";
  case ProductionOptimizerKind::lahc:
    return "shiny::nesting::search::LahcSearch";
  }
  return "unknown";
}

[[nodiscard]] inline auto
effective_runner_class_name(const ExecutionPolicy &execution)
    -> std::string_view {
  return execution.strategy == StrategyKind::metaheuristic_search
             ? production_runner_class_name(execution.production_optimizer)
             : runner_class_name(execution.strategy);
}

[[nodiscard]] inline auto format_id_list(std::span<const std::uint32_t> ids)
    -> std::string {
  if (ids.empty()) {
    return "all";
  }
  std::string formatted = "[";
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (index > 0U) {
      formatted += ",";
    }
    formatted += std::format("{}", ids[index]);
  }
  formatted += "]";
  return formatted;
}

[[nodiscard]] inline auto
total_requested_quantity(const NestingRequest &request) -> std::size_t {
  return std::accumulate(
      request.pieces.begin(), request.pieces.end(), std::size_t{0},
      [](const std::size_t total, const PieceRequest &piece) {
        return total + static_cast<std::size_t>(piece.quantity);
      });
}

[[nodiscard]] inline auto
count_pieces_with_allowed_bins(const NestingRequest &request) -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(request.pieces.begin(), request.pieces.end(),
                    [](const PieceRequest &piece) {
                      return !piece.allowed_bin_ids.empty();
                    }));
}

[[nodiscard]] inline auto
count_pieces_with_custom_rotations(const NestingRequest &request)
    -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(request.pieces.begin(), request.pieces.end(),
                    [](const PieceRequest &piece) {
                      return piece.allowed_rotations.has_value();
                    }));
}

[[nodiscard]] inline auto count_mirrored_pieces(const NestingRequest &request)
    -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      request.pieces.begin(), request.pieces.end(),
      [](const PieceRequest &piece) { return piece.allow_mirror; }));
}

[[nodiscard]] inline auto count_priority_pieces(const NestingRequest &request)
    -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      request.pieces.begin(), request.pieces.end(),
      [](const PieceRequest &piece) { return piece.priority != 0; }));
}

[[nodiscard]] inline auto
count_bins_with_non_default_start_corner(const NestingRequest &request)
    -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      request.bins.begin(), request.bins.end(), [](const BinRequest &bin) {
        return bin.start_corner != place::PlacementStartCorner::bottom_left;
      }));
}

[[nodiscard]] inline auto
count_total_exclusion_zones(const NestingRequest &request) -> std::size_t {
  return std::accumulate(request.bins.begin(), request.bins.end(),
                         std::size_t{0},
                         [](const std::size_t total, const BinRequest &bin) {
                           return total + bin.exclusion_zones.size();
                         });
}

[[nodiscard]] inline auto request_surface_summary(const NestingRequest &request)
    -> std::string {
  return std::format(
      "pieces={} qty_total={} bins={} selected_bins={} spacing={:.3g} "
      "default_rotations={} restricted_pieces={} custom_rot_pieces={} "
      "mirror_pieces={} priority_pieces={} start_corner_overrides={} "
      "exclusion_zones={} part_in_part={} explore_concave={}",
      request.pieces.size(), total_requested_quantity(request),
      request.bins.size(), format_id_list(request.execution.selected_bin_ids),
      request.execution.part_spacing,
      geom::rotation_count(request.execution.default_rotations),
      count_pieces_with_allowed_bins(request),
      count_pieces_with_custom_rotations(request),
      count_mirrored_pieces(request), count_priority_pieces(request),
      count_bins_with_non_default_start_corner(request),
      count_total_exclusion_zones(request),
      bool_name(request.execution.enable_part_in_part_placement),
      bool_name(request.execution.explore_concave_candidates));
}

[[nodiscard]] inline auto control_surface_summary(const SolveControl &control)
    -> std::string {
  return std::format("seed={} seed_mode={} operation_limit={} time_limit_ms={} "
                     "has_progress={} has_workspace={}",
                     control.random_seed, seed_mode_name(control.seed_mode),
                     control.operation_limit, control.time_limit_milliseconds,
                     bool_name(static_cast<bool>(control.on_progress)),
                     bool_name(control.workspace != nullptr));
}

[[nodiscard]] inline auto
bounding_box_settings_summary(const ExecutionPolicy &execution) -> std::string {
  return std::format(
      "heuristic={} placement_policy={} attempts={}",
      bounding_box_heuristic_name(execution.bounding_box.heuristic),
      placement_policy_name(execution.placement_policy),
      execution.deterministic_attempts.max_attempts);
}

[[nodiscard]] inline auto
irregular_settings_summary(const ExecutionPolicy &execution) -> std::string {
  return std::format(
      "placement_policy={} candidate_strategy={} piece_ordering={} "
      "max_candidate_points={} gaussian_sigma={:.3g} merge_free_regions={} "
      "direct_overlap={} backtracking={} max_backtrack_pieces={} "
      "compaction={} compaction_passes={}",
      placement_policy_name(execution.placement_policy),
      candidate_strategy_name(execution.irregular.candidate_strategy),
      piece_ordering_name(execution.irregular.piece_ordering),
      execution.irregular.max_candidate_points,
      execution.irregular.candidate_gaussian_sigma,
      bool_name(execution.irregular.merge_free_regions),
      bool_name(execution.irregular.enable_direct_overlap_check),
      bool_name(execution.irregular.enable_backtracking),
      execution.irregular.max_backtrack_pieces,
      bool_name(execution.irregular.enable_compaction),
      execution.irregular.compaction_passes);
}

[[nodiscard]] inline auto
production_settings_summary(const ExecutionPolicy &execution,
                            const ProductionOptimizerKind optimizer)
    -> std::string {
  const std::string shared = std::format(
      "optimizer={} placement_policy={} population_size={} elite_count={} "
      "mutant_count={} max_operations={} polishing_passes={} "
      "diversification_swaps={} elite_bias={:.3g}",
      production_optimizer_name(optimizer),
      placement_policy_name(execution.placement_policy),
      execution.production.population_size, execution.production.elite_count,
      execution.production.mutant_count, execution.production.max_iterations,
      execution.production.polishing_passes,
      execution.production.diversification_swaps,
      execution.production.elite_bias);
  switch (optimizer) {
  case ProductionOptimizerKind::brkga:
    return shared;
  case ProductionOptimizerKind::simulated_annealing:
    return std::format(
        "{} sa[max_refinements={} restart_count={} cooling_schedule={} "
        "initial_temperature={:.3g} final_temperature={:.3g}]",
        shared, execution.simulated_annealing.max_refinements,
        execution.simulated_annealing.restart_count,
        cooling_schedule_name(execution.simulated_annealing.cooling_schedule),
        execution.simulated_annealing.initial_temperature,
        execution.simulated_annealing.final_temperature);
  case ProductionOptimizerKind::alns:
    return std::format(
        "{} alns[max_refinements={} destroy_min={} destroy_max={} "
        "segment_length={}]",
        shared, execution.alns.max_refinements,
        execution.alns.destroy_min_count, execution.alns.destroy_max_count,
        execution.alns.segment_length);
  case ProductionOptimizerKind::gdrr:
    return std::format("{} gdrr[max_refinements={} initial_goal_ratio={:.3g} "
                       "goal_decay={:.3g} ruin_swap_count={}]",
                       shared, execution.gdrr.max_refinements,
                       execution.gdrr.initial_goal_ratio,
                       execution.gdrr.goal_decay,
                       execution.gdrr.ruin_swap_count);
  case ProductionOptimizerKind::lahc:
    return std::format(
        "{} lahc[max_refinements={} history_length={} plateau_limit={} "
        "perturbation_swaps={}]",
        shared, execution.lahc.max_refinements, execution.lahc.history_length,
        execution.lahc.plateau_limit, execution.lahc.perturbation_swaps);
  }
  return shared;
}

[[nodiscard]] inline auto
strategy_settings_summary(const ExecutionPolicy &execution) -> std::string {
  switch (execution.strategy) {
  case StrategyKind::bounding_box:
    return bounding_box_settings_summary(execution);
  case StrategyKind::sequential_backtrack:
    return irregular_settings_summary(execution);
  case StrategyKind::metaheuristic_search:
    return production_settings_summary(execution,
                                       execution.production_optimizer);
  case StrategyKind::simulated_annealing:
    return production_settings_summary(
        execution, ProductionOptimizerKind::simulated_annealing);
  case StrategyKind::alns:
    return production_settings_summary(execution,
                                       ProductionOptimizerKind::alns);
  case StrategyKind::gdrr:
    return production_settings_summary(execution,
                                       ProductionOptimizerKind::gdrr);
  case StrategyKind::lahc:
    return production_settings_summary(execution,
                                       ProductionOptimizerKind::lahc);
  }
  return "unknown";
}

} // namespace shiny::nesting::log
