#include "solve.hpp"

#include "internal/legacy_solve.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "api/profiles.hpp"
#include "internal/request_normalization.hpp"
#include "logging/shiny_log.hpp"
#include "logging/solve_summary.hpp"
#include "packing/bounding_box_packer.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "packing/irregular/workspace.hpp"
#include "packing/sparrow/adapters/progress_adapter.hpp"
#include "packing/sparrow/optimize/optimize.hpp"
#include "packing/sparrow/search/brkga_search.hpp"
#include "packing/sparrow/search/detail/driver_scaffolding.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/progress.hpp"
#include "runtime/timing.hpp"
#include "validation/layout_validation.hpp"

namespace shiny::nesting {
namespace {

[[nodiscard]] auto compute_efficiency_percent(const pack::Layout &layout)
    -> double {
  double total_occupied_area = 0.0;
  double total_container_area = 0.0;
  for (const auto &bin : layout.bins) {
    total_occupied_area += bin.utilization.occupied_area;
    total_container_area += bin.utilization.container_area;
  }
  if (total_container_area <= 0.0) {
    return 0.0;
  }
  return (total_occupied_area / total_container_area) * 100.0;
}

[[nodiscard]] auto bounding_box_result_better(const pack::DecoderResult &lhs,
                                              const pack::DecoderResult &rhs)
    -> bool {
  const std::size_t lhs_placed = lhs.layout.placement_trace.size();
  const std::size_t rhs_placed = rhs.layout.placement_trace.size();
  if (lhs_placed != rhs_placed) {
    return lhs_placed > rhs_placed;
  }

  const double lhs_efficiency = compute_efficiency_percent(lhs.layout);
  const double rhs_efficiency = compute_efficiency_percent(rhs.layout);
  if (std::abs(lhs_efficiency - rhs_efficiency) > 1e-9) {
    return lhs_efficiency > rhs_efficiency;
  }

  const bool lhs_completed = !lhs.interrupted;
  const bool rhs_completed = !rhs.interrupted;
  if (lhs_completed != rhs_completed) {
    return lhs_completed;
  }

  return lhs.layout.bins.size() < rhs.layout.bins.size();
}

[[nodiscard]] auto
best_bounding_box_attempt_index(const std::vector<pack::DecoderResult> &results)
    -> std::optional<std::size_t> {
  if (results.empty()) {
    return std::nullopt;
  }

  std::size_t best_index = 0;
  for (std::size_t index = 1; index < results.size(); ++index) {
    if (bounding_box_result_better(results[index], results[best_index])) {
      best_index = index;
    }
  }
  return best_index;
}

auto emit_snapshot(const SolveControl &control,
                   const ProgressSnapshot &snapshot) -> void {
  if (!control.on_progress) {
    return;
  }

  control.on_progress(snapshot);
}

[[nodiscard]] auto stop_reason_for_bounding_box(
    const SolveControl &control, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch, const bool interrupted,
    const bool hit_operation_limit) -> StopReason {
  if (control.cancellation.stop_requested()) {
    return StopReason::cancelled;
  }
  if (time_budget.expired(stopwatch)) {
    return StopReason::time_limit_reached;
  }
  if (hit_operation_limit) {
    return StopReason::operation_limit_reached;
  }
  if (!interrupted) {
    return StopReason::completed;
  }
  return StopReason::cancelled;
}

[[nodiscard]] auto profile_layout_better(const NormalizedRequest &request,
                                         const pack::Layout &lhs,
                                         const pack::Layout &rhs) -> bool {
  return search::better_metrics(search::metrics_for_layout(request, lhs),
                                search::metrics_for_layout(request, rhs));
}

[[nodiscard]] auto active_bin_id_for_layout(const pack::Layout &layout)
    -> std::optional<std::uint32_t> {
  if (!layout.placement_trace.empty()) {
    return layout.placement_trace.back().bin_id;
  }
  if (!layout.bins.empty()) {
    return layout.bins.back().bin_id;
  }
  return std::nullopt;
}

[[nodiscard]] auto find_piece_request(const NormalizedRequest &request,
                                      const std::uint32_t source_piece_id)
    -> const PieceRequest * {
  const auto it =
      std::find_if(request.request.pieces.begin(), request.request.pieces.end(),
                   [source_piece_id](const PieceRequest &piece) {
                     return piece.piece_id == source_piece_id;
                   });
  return it == request.request.pieces.end() ? nullptr : &*it;
}

[[nodiscard]] auto
rotation_index_for_entry(const NormalizedRequest &request,
                         const std::size_t piece_index,
                         const geom::ResolvedRotation &resolved_rotation,
                         const std::optional<geom::RotationIndex> fallback)
    -> std::optional<geom::RotationIndex> {
  if (piece_index >= request.expanded_pieces.size()) {
    return fallback;
  }

  const auto *piece = find_piece_request(
      request, request.expanded_pieces[piece_index].source_piece_id);
  if (piece == nullptr) {
    return fallback;
  }

  const auto &rotations = piece->allowed_rotations.has_value()
                              ? *piece->allowed_rotations
                              : request.request.execution.default_rotations;
  const auto rotation_degrees = geom::materialize_rotations(rotations);
  if (rotation_degrees.size() <= 1U) {
    return fallback;
  }

  for (std::size_t index = 0; index < rotation_degrees.size(); ++index) {
    if (std::abs(rotation_degrees[index] - resolved_rotation.degrees) <= 1e-9) {
      return geom::RotationIndex{static_cast<std::uint16_t>(index)};
    }
  }
  return fallback;
}

[[nodiscard]] auto build_sparrow_seed_entry(const NormalizedRequest &request,
                                            const NestingResult &seed_result)
    -> search::SolutionPoolEntry {
  std::unordered_map<std::uint32_t, std::size_t> piece_index_by_id;
  piece_index_by_id.reserve(request.expanded_pieces.size());
  for (std::size_t index = 0; index < request.expanded_pieces.size(); ++index) {
    piece_index_by_id.emplace(request.expanded_pieces[index].expanded_piece_id,
                              index);
  }

  auto forced_rotations = search::detail::original_forced_rotations(request);
  if (forced_rotations.size() != request.expanded_pieces.size()) {
    forced_rotations.assign(request.expanded_pieces.size(), std::nullopt);
  }

  std::vector<std::size_t> order;
  order.reserve(request.expanded_pieces.size());
  std::vector<bool> seen(request.expanded_pieces.size(), false);
  const auto append_piece = [&](const std::uint32_t piece_id) {
    const auto it = piece_index_by_id.find(piece_id);
    if (it == piece_index_by_id.end() || seen[it->second]) {
      return;
    }
    order.push_back(it->second);
    seen[it->second] = true;
  };

  for (const auto &entry : seed_result.layout.placement_trace) {
    append_piece(entry.piece_id);
    const auto it = piece_index_by_id.find(entry.piece_id);
    if (it != piece_index_by_id.end()) {
      forced_rotations[it->second] =
          rotation_index_for_entry(request, it->second, entry.resolved_rotation,
                                   forced_rotations[it->second]);
    }
  }
  for (const auto piece_id : seed_result.layout.unplaced_piece_ids) {
    append_piece(piece_id);
  }
  for (std::size_t index = 0; index < request.expanded_pieces.size(); ++index) {
    if (!seen[index]) {
      order.push_back(index);
    }
  }

  return {
      .order = std::move(order),
      .piece_indexed_forced_rotations = std::move(forced_rotations),
      .metrics = search::metrics_for_layout(request, seed_result.layout),
      .result = seed_result,
  };
}

[[nodiscard]] auto
make_sparrow_optimize_config(const ProductionSearchConfig &production,
                             const SolveControl &control)
    -> pack::sparrow::optimize::OptimizeConfig {
  std::size_t total_iterations =
      std::max<std::size_t>(1U, production.max_iterations);
  if (control.operation_limit > 0U) {
    total_iterations = std::min(total_iterations, control.operation_limit);
  }

  std::size_t exploration_budget =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::llround(
                                    static_cast<double>(total_iterations) *
                                    production.strip_exploration_ratio)));
  exploration_budget = std::min(exploration_budget, total_iterations);
  std::size_t compression_budget = total_iterations - exploration_budget;
  if (compression_budget == 0U && total_iterations > 1U) {
    --exploration_budget;
    compression_budget = 1U;
  }

  return {
      .exploration =
          {
              .iteration_budget = exploration_budget,
              .plateau_limit = std::max<std::size_t>(
                  1U, production.infeasible_rollback_after),
              .acceptance_window_max_ratio =
                  production.strip_exploration_shrink_max_ratio,
              .acceptance_window_min_ratio =
                  production.strip_exploration_shrink_min_ratio,
              .enable_rotation_moves = true,
              .enable_disruption_moves =
                  production.infeasible_pool_capacity > 0U,
              .infeasible_pool_capacity = production.infeasible_pool_capacity,
              .rollback_after = production.infeasible_rollback_after,
          },
      .compression =
          {
              .iteration_budget = compression_budget,
              .plateau_limit = std::max<std::size_t>(
                  1U, production.infeasible_rollback_after),
              .shrink_max_ratio = production.strip_compression_shrink_max_ratio,
              .shrink_min_ratio = production.strip_compression_shrink_min_ratio,
              .separator_worker_count = production.separator_worker_count,
              .separator_max_iterations = production.separator_max_iterations,
              .separator_iter_no_improvement_limit =
                  production.separator_iter_no_improvement_limit,
              .separator_strike_limit = production.separator_strike_limit,
              .separator_global_samples = production.separator_global_samples,
              .separator_focused_samples = production.separator_focused_samples,
              .separator_coordinate_descent_iterations =
                  production.separator_coordinate_descent_iterations,
              .gls_weight_cap = production.gls_weight_cap,
          },
  };
}

[[nodiscard]] auto placed_parts(const NestingResult &result) -> std::size_t {
  return result.layout.placement_trace.size();
}

auto log_solve_finish(const NestingRequest &request,
                      const SolveControl &control,
                      std::string_view dispatch_name,
                      std::string_view runner_class,
                      const NestingResult &result) -> void {
  const auto placed = placed_parts(result);
  SHINY_DEBUG("solve: finish strategy={} dispatch={} runner={} stop_reason={} "
              "placed={}/{} bins={} unplaced={} requested_seed={} "
              "effective_seed={}",
              log::strategy_name(request.execution.strategy), dispatch_name,
              runner_class, log::stop_reason_name(result.stop_reason), placed,
              result.total_parts, result.layout.bins.size(),
              result.layout.unplaced_piece_ids.size(), control.random_seed,
              result.effective_seed);

  if (result.stop_reason == StopReason::completed &&
      placed < result.total_parts) {
    SHINY_WARN("solve: completed with unplaced parts strategy={} dispatch={} "
               "runner={} "
               "placed={}/{} bins={} request=[{}] control/settings=[{} | {}]",
               log::strategy_name(request.execution.strategy), dispatch_name,
               runner_class, placed, result.total_parts,
               result.layout.bins.size(), log::request_surface_summary(request),
               log::control_surface_summary(control),
               log::strategy_settings_summary(request.execution));
  }
}

} // namespace

auto solve(const NestingRequest &request, const SolveControl &control)
    -> std::expected<NestingResult, util::Status> {
  auto normalized_request = normalize_request(request);
  if (!normalized_request.has_value()) {
    return std::unexpected(normalized_request.error());
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);

  if (request.execution.strategy == StrategyKind::bounding_box) {
    auto decoder_request =
        to_bounding_box_decoder_request(normalized_request.value());
    if (!decoder_request.has_value()) {
      return std::unexpected(decoder_request.error());
    }

    const std::uint32_t configured_attempts =
        decoder_request.value().config.deterministic_attempts.max_attempts;
    if (control.operation_limit > 0U) {
      decoder_request.value().config.deterministic_attempts.max_attempts =
          static_cast<std::uint32_t>(std::min<std::size_t>(
              control.operation_limit, configured_attempts));
    }

    const auto runner_class =
        log::effective_runner_class_name(request.execution);
    SHINY_DEBUG(
        "solve: start strategy={} dispatch={} runner={} request=[{}] "
        "control=[{}] settings=[{} effective_attempts={} expanded_pieces={} "
        "expanded_bins={}]",
        log::strategy_name(request.execution.strategy),
        log::strategy_name(request.execution.strategy), runner_class,
        log::request_surface_summary(request),
        log::control_surface_summary(control),
        log::strategy_settings_summary(request.execution),
        decoder_request.value().config.deterministic_attempts.max_attempts,
        normalized_request.value().expanded_pieces.size(),
        normalized_request.value().expanded_bins.size());

    pack::BoundingBoxPacker packer;
    const auto interruption_requested = [&]() {
      return control.cancellation.stop_requested() ||
             time_budget.expired(stopwatch);
    };
    std::vector<pack::DecoderResult> results = packer.decode_attempts(
        decoder_request.value(), interruption_requested,
        [&](const std::size_t attempt_index,
            const pack::DecoderResult &result) {
          emit_snapshot(
              control,
              ProgressSnapshot{
                  .sequence = attempt_index + 1U,
                  .placements_successful = result.layout.placement_trace.size(),
                  .total_requested_parts =
                      normalized_request.value().expanded_pieces.size(),
                  .layout = result.layout,
                  .stop_reason = StopReason::none,
              });
        });
    const auto best_attempt_index = best_bounding_box_attempt_index(results);
    if (!best_attempt_index.has_value()) {
      return std::unexpected(util::Status::invalid_input);
    }

    const bool hit_operation_limit =
        control.operation_limit > 0U &&
        decoder_request.value().config.deterministic_attempts.max_attempts <
            configured_attempts &&
        results.size() >= decoder_request.value()
                              .config.deterministic_attempts.max_attempts &&
        !results.back().interrupted && !control.cancellation.stop_requested() &&
        !time_budget.expired(stopwatch);
    const auto &best_result = results[*best_attempt_index];

    NestingResult result{
        .strategy = StrategyKind::bounding_box,
        .layout = best_result.layout,
        .total_parts = normalized_request.value().expanded_pieces.size(),
        .effective_seed = control.random_seed,
        .stop_reason = stop_reason_for_bounding_box(
            control, time_budget, stopwatch, best_result.interrupted,
            hit_operation_limit),
    };
    validation::finalize_result(normalized_request.value(), result);
    log_solve_finish(request, control,
                     log::strategy_name(request.execution.strategy),
                     runner_class, result);
    return result;
  }

  if (request.execution.strategy != StrategyKind::metaheuristic_search ||
      request.execution.production_optimizer !=
          ProductionOptimizerKind::brkga) {
    SHINY_WARN(
        "solve: unsupported legacy strategy={} optimizer={} request=[{}]",
        log::strategy_name(request.execution.strategy),
        log::production_optimizer_name(request.execution.production_optimizer),
        log::request_surface_summary(request));
    return std::unexpected(util::Status::invalid_input);
  }

  const auto runner_class = log::effective_runner_class_name(request.execution);
  SHINY_DEBUG("solve: start strategy={} dispatch={} runner={} request=[{}] "
              "control=[{}] settings=[{} expanded_pieces={} expanded_bins={}]",
              log::strategy_name(request.execution.strategy), "brkga",
              runner_class, log::request_surface_summary(request),
              log::control_surface_summary(control),
              log::strategy_settings_summary(request.execution),
              normalized_request.value().expanded_pieces.size(),
              normalized_request.value().expanded_bins.size());

  search::BrkgaProductionSearch search;
  auto result = search.solve(normalized_request.value(), control);
  if (result.has_value()) {
    result.value().strategy = StrategyKind::metaheuristic_search;
    result.value().effective_seed = control.random_seed;
    validation::finalize_result(normalized_request.value(), result.value());
    log_solve_finish(request, control, "brkga", runner_class, result.value());
  }
  return result;
}

auto solve(const ProfileRequest &request, const ProfileSolveControl &control)
    -> std::expected<NestingResult, util::Status> {
  // Validate basic profile preconditions (profile recognized, time limit
  // present for non-Quick profiles) before the full translation step.
  if (const auto v = request.validate(); !v) {
    return std::unexpected(v.error());
  }
  auto translated_request = to_nesting_request(request);
  if (!translated_request.has_value()) {
    return std::unexpected(translated_request.error());
  }
  const auto normalized_profile_request =
      normalize_request(translated_request.value());
  if (!normalized_profile_request.has_value()) {
    return std::unexpected(normalized_profile_request.error());
  }

  const auto started_at = std::chrono::steady_clock::now();
  runtime::CancellationSource enforced_cancellation;
  std::atomic<bool> time_limit_requested = false;
  pack::Layout best_layout{};
  bool have_best_layout = false;

  const auto remaining_time =
      [&](const std::uint64_t elapsed_ms) -> std::optional<std::uint64_t> {
    if (request.profile == SolveProfile::quick ||
        !request.time_limit_milliseconds.has_value()) {
      return std::nullopt;
    }
    const auto budget_ms = *request.time_limit_milliseconds;
    return elapsed_ms >= budget_ms ? 0U : budget_ms - elapsed_ms;
  };

  const auto sparrow_phase = [&]() {
    return request.profile == SolveProfile::maximum_search
               ? pack::sparrow::SparrowPhase::profile_maximum_search
               : pack::sparrow::SparrowPhase::profile_balanced;
  };

  const auto emit_profile_snapshot = [&](const pack::Layout &current_layout,
                                         const StopReason stop_reason,
                                         const bool improved,
                                         const bool use_sparrow_surface = false,
                                         const std::size_t accepted_moves =
                                             0U) {
    if (!control.on_progress) {
      return;
    }

    if (!have_best_layout ||
        profile_layout_better(normalized_profile_request.value(),
                              current_layout, best_layout)) {
      best_layout = current_layout;
      have_best_layout = true;
    }

    const auto elapsed_ms = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started_at)
            .count());
    const auto phase = stop_reason == StopReason::none
                           ? (use_sparrow_surface ? ProgressPhase::refinement
                                                  : ProgressPhase::placement)
                           : ProgressPhase::completed;
    if (use_sparrow_surface) {
      control.on_progress(pack::sparrow::adapters::to_profile_progress_snapshot(
          pack::sparrow::runtime::PortProgressSnapshot{
              .current_layout = current_layout,
              .best_layout = have_best_layout ? best_layout : current_layout,
              .active_bin_id = active_bin_id_for_layout(current_layout),
              .accepted_moves = accepted_moves,
              .elapsed_time_milliseconds = elapsed_ms,
              .remaining_time_milliseconds = remaining_time(elapsed_ms),
              .stop_reason = stop_reason,
              .phase = sparrow_phase(),
              .improved = improved,
          }));
      return;
    }

    control.on_progress(runtime::make_profile_progress_snapshot(
        request.profile, phase, std::string(api::profile_name(request.profile)),
        current_layout, have_best_layout ? best_layout : current_layout,
        active_bin_id_for_layout(current_layout), elapsed_ms,
        remaining_time(elapsed_ms), stop_reason, improved));
  };

  std::optional<std::jthread> watchdog;
  if (request.profile != SolveProfile::quick ||
      control.cancellation_requested) {
    watchdog.emplace([&](std::stop_token stop_token) {
      while (!stop_token.stop_requested()) {
        if (control.cancellation_requested ||
            control.cancellation.stop_requested()) {
          enforced_cancellation.request_stop();
          return;
        }
        if (request.profile != SolveProfile::quick &&
            request.time_limit_milliseconds.has_value()) {
          const auto elapsed_ms = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - started_at)
                  .count());
          if (elapsed_ms >= *request.time_limit_milliseconds) {
            time_limit_requested.store(true);
            enforced_cancellation.request_stop();
            return;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  SolveControl translated_control{
      .on_progress =
          control.on_progress
              ? ProgressObserver([&](const ProgressSnapshot &snapshot) {
                  emit_profile_snapshot(snapshot.layout, snapshot.stop_reason,
                                        snapshot.improved);
                })
              : ProgressObserver{},
      .cancellation = watchdog.has_value() ? enforced_cancellation.token()
                                           : control.cancellation,
      .operation_limit = control.operation_limit,
      .time_limit_milliseconds =
          request.profile == SolveProfile::quick
              ? 0U
              : request.time_limit_milliseconds.value_or(0U),
      .random_seed = control.random_seed,
      .cancellation_requested = control.cancellation_requested,
      .seed_mode = control.seed_mode,
      .workspace = control.workspace,
  };

  auto result = [&]() -> std::expected<NestingResult, util::Status> {
    if (request.profile == SolveProfile::quick) {
      return solve(translated_request.value(), translated_control);
    }

    const auto &normalized_request = normalized_profile_request.value();

    const auto seed_flow =
        pack::sparrow::build_seed_flow_plan(control, request.profile);
    SolveControl constructive_control = translated_control;
    constructive_control.on_progress = {};
    constructive_control.random_seed = seed_flow.constructive_seed;
    constructive_control.time_limit_milliseconds = 0U;

    pack::constructive::FillFirstEngine constructive_engine;
    auto constructive_result =
        constructive_engine.solve(normalized_request, constructive_control);
    if (!constructive_result.has_value()) {
      return std::unexpected(constructive_result.error());
    }

    emit_profile_snapshot(constructive_result.value().layout, StopReason::none,
                          true, true);

    if (translated_control.cancellation.stop_requested()) {
      auto cancelled = constructive_result.value();
      cancelled.strategy = StrategyKind::metaheuristic_search;
      cancelled.effective_seed = control.random_seed;
      cancelled.stop_reason = StopReason::cancelled;
      cancelled.search.optimizer = OptimizerKind::none;
      cancelled.search.sparrow_polished = false;
      validation::finalize_result(normalized_request, cancelled);
      return cancelled;
    }

    SolveControl optimize_control = translated_control;
    optimize_control.on_progress = {};
    optimize_control.random_seed = seed_flow.worker_seed_base;
    optimize_control.time_limit_milliseconds = 0U;

    runtime::Stopwatch sparrow_stopwatch;
    const runtime::TimeBudget disabled_budget(0U);
    const auto optimize_config = make_sparrow_optimize_config(
        normalized_request.request.execution.production, optimize_control);
    const auto seed_entry = build_sparrow_seed_entry(
        normalized_request, constructive_result.value());
    const auto optimized = pack::sparrow::optimize::run_optimize(
        normalized_request, optimize_control, disabled_budget,
        sparrow_stopwatch, seed_entry, optimize_config);

    auto sparrow_result = optimized.best_solution.result;
    sparrow_result.strategy = StrategyKind::metaheuristic_search;
    sparrow_result.effective_seed = control.random_seed;
    sparrow_result.search.optimizer = OptimizerKind::none;
    sparrow_result.search.sparrow_polished = true;
    sparrow_result.search.phase_metrics = optimized.phase_metrics;
    sparrow_result.search.separator_metrics = optimized.separator_metrics;
    validation::finalize_result(normalized_request, sparrow_result);

    const bool solved_all_parts =
        sparrow_result.placed_parts() == sparrow_result.total_parts &&
        sparrow_result.layout.unplaced_piece_ids.empty();
    const bool hit_operation_limit =
        optimize_control.operation_limit > 0U &&
        optimized.phase_metrics.exploration_iterations +
                optimized.phase_metrics.compression_iterations >=
            optimize_control.operation_limit;
    sparrow_result.stop_reason =
        solved_all_parts ? StopReason::completed
                         : search::detail::driver_stop_reason(
                               optimize_control, disabled_budget,
                               sparrow_stopwatch, hit_operation_limit);
    emit_profile_snapshot(
        sparrow_result.layout, sparrow_result.stop_reason,
        profile_layout_better(normalized_request, sparrow_result.layout,
                              constructive_result.value().layout),
        true, optimized.accepted_moves);
    return sparrow_result;
  }();

  if (result.has_value()) {
    if (time_limit_requested.load() &&
        result.value().stop_reason == StopReason::cancelled) {
      result.value().stop_reason = StopReason::time_limit_reached;
    }
    emit_profile_snapshot(result.value().layout, result.value().stop_reason,
                          true);
  }
  return result;
}

} // namespace shiny::nesting
