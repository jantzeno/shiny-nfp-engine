#include "search/gdrr_search.hpp"

#include "observer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/neighborhood_search.hpp"
#include "search/strategy_registry.hpp"
#include <algorithm>
#include <cstddef>
#include <format>

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto run_gdrr_strategy(const NormalizedRequest &request,
                                     const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  GdrrSearch search;
  return search.solve(request, control);
}

} // namespace

void register_gdrr_strategy() {
  StrategyRegistry::instance().register_strategy({
      .name = "gdrr",
      .kind = StrategyKind::gdrr,
      .run = &run_gdrr_strategy,
  });
  StrategyRegistry::instance().register_production_strategy({
      .name = "gdrr",
      .kind = ProductionOptimizerKind::gdrr,
      .run = &run_gdrr_strategy,
  });
}

namespace {

auto emit_improvement(const SolveControl &control, SearchReplay &replay,
                      const std::size_t sequence, const std::size_t iteration,
                      const SolutionPoolEntry &best, const BudgetState &budget,
                      const double goal_strip_length,
                      const detail::NeighborhoodSearch op) -> void {
  replay.progress.push_back({
      .iteration = iteration,
      .improved = true,
      .layout = best.result.layout,
      .budget = budget,
  });
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placements_successful = best.metrics.placed_parts,
      .total_requested_parts = best.result.total_parts,
      .refinement_steps_completed = iteration,
      .layout = best.result.layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail =
          std::format("GDRR iter {} improved via {} (goal {:.3f})", iteration,
                      detail::operator_label(op), goal_strip_length),
      .utilization_percent = best.metrics.utilization * 100.0,
      .improved = true,
  });
}

auto emit_initial_progress(const SolveControl &control,
                           const SolutionPoolEntry &best,
                           const BudgetState &budget,
                           const double goal_strip_length) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 1U,
      .placements_successful = best.metrics.placed_parts,
      .total_requested_parts = best.result.total_parts,
      .refinement_steps_completed = 0U,
      .layout = best.result.layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail =
          std::format("GDRR initial seed (goal {:.3f})", goal_strip_length),
      .utilization_percent = best.metrics.utilization * 100.0,
      .improved = true,
  });
}

auto emit_iteration_progress(const SolveControl &control,
                             const std::size_t iteration,
                             const SolutionPoolEntry &current,
                             const BudgetState &budget,
                             const double goal_strip_length,
                             const detail::NeighborhoodSearch op,
                             const bool accepted) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placements_successful = current.metrics.placed_parts,
      .total_requested_parts = current.result.total_parts,
      .refinement_steps_completed = iteration,
      .layout = current.result.layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail =
          std::format("GDRR iter {} {} via {} (goal {:.3f})", iteration,
                      accepted ? "accepted" : "rejected",
                      detail::operator_label(op), goal_strip_length),
      .utilization_percent = current.metrics.utilization * 100.0,
      .improved = false,
  });
}

} // namespace

// GDRR — Goal-Driven Ruin & Recreate.
//
// Tracks a `goal_strip_length` that decays multiplicatively each
// iteration (`goal_decay`). The candidate is accepted if it is better
// than current OR within record-window of best, where the tolerance
// shrinks as the goal decays. While current strip > goal,
// "pressure > 0" forces area_destroy_repair (the most aggressive
// operator); otherwise a random operator is chosen.
//
// Plateau handling: after 4 rejects, snap current = best (escape).
auto GdrrSearch::solve(const NormalizedRequest &request,
                       const SolveControl &control) const
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_strategy_config(
      request.request.execution, StrategyKind::gdrr,
      ProductionOptimizerKind::gdrr, request.request.execution.gdrr);
  const std::size_t operation_limit = control.operation_limit > 0U
                                          ? control.operation_limit
                                          : config.max_refinements;

  SearchReplay replay{.optimizer = OptimizerKind::gdrr};
  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::gdrr, std::move(replay),
                                       control, time_budget, stopwatch);
  }

  detail::OrderEvaluator evaluator(request, control, time_budget, stopwatch);
  runtime::DeterministicRng rng(control.random_seed);

  const auto forced_rotations = detail::original_forced_rotations(request);
  auto current =
      evaluator.evaluate(detail::original_order(request), forced_rotations, 1U);
  auto area_seed =
      evaluator.evaluate(detail::descending_area_order(evaluator.piece_areas()),
                         forced_rotations, 2U);
  auto reverse_seed =
      evaluator.evaluate(detail::reverse_order(request), forced_rotations, 3U);
  auto best =
      better_metrics(area_seed.metrics, current.metrics) ? area_seed : current;
  if (better_metrics(reverse_seed.metrics, best.metrics)) {
    best = reverse_seed;
  }
  current = best;

  double goal_strip_length =
      std::max(best.metrics.strip_length * config.initial_goal_ratio, 0.0);
  std::size_t operations_completed = 0;
  std::size_t sequence = 1U;
  bool hit_operation_limit = false;
  std::size_t plateau = 0;
  replay.progress.push_back({
      .iteration = 0,
      .improved = true,
      .layout = best.result.layout,
      .budget = evaluator.make_budget(0),
  });
  emit_initial_progress(control, best, evaluator.make_budget(0),
                        goal_strip_length);

  for (std::size_t iteration = 0; iteration < operation_limit; ++iteration) {
    if (evaluator.interrupted()) {
      break;
    }

    const auto pressure =
        current.metrics.strip_length > goal_strip_length ? 1U : 0U;
    const auto op = pressure > 0U
                        ? detail::NeighborhoodSearch::area_destroy_repair
                        : detail::random_operator(rng, false);
    const auto move = detail::propose_move(
        current.order, current.forced_rotations, request,
        evaluator.piece_areas(), evaluator.piece_rotation_counts(), rng, op,
        std::max<std::size_t>(1U, config.ruin_swap_count + pressure));
    if (!move.changed) {
      // No-op move: do NOT decay `goal_strip_length` here. The goal
      // is meant to relax only when a genuine evaluation has been
      // performed and rejected. Decaying on a no-op (which doesn't
      // even invoke the decoder) artificially shrinks the goal across
      // long stretches of "no neighbour proposed", causing the
      // record-window threshold to collapse without exploring anything.
      continue;
    }

    auto candidate =
        evaluator.evaluate(move.order, move.forced_rotations, iteration + 21U);
    ++operations_completed;

    const bool accepted =
        better_metrics(candidate.metrics, current.metrics) ||
        detail::within_record_window(
            candidate.metrics, best.metrics,
            goal_strip_length > 0.0 && best.metrics.strip_length > 0.0
                ? std::max(0.0,
                           (goal_strip_length / best.metrics.strip_length) -
                               1.0)
                : 0.0);
    if (accepted) {
      current = candidate;
      plateau = 0;
    } else {
      ++plateau;
    }
    if (better_metrics(candidate.metrics, best.metrics)) {
      best = candidate;
      ++sequence;
      emit_improvement(control, replay, sequence, operations_completed, best,
                       evaluator.make_budget(operations_completed),
                       goal_strip_length, op);
    }

    emit_iteration_progress(control, operations_completed, current,
                            evaluator.make_budget(operations_completed),
                            goal_strip_length, op, accepted);

    goal_strip_length = std::max(goal_strip_length * config.goal_decay, 0.0);
    if (plateau >= 4U) {
      current = best;
      plateau = 0;
    }
    if (operations_completed >= operation_limit) {
      hit_operation_limit = control.operation_limit > 0U ||
                            iteration + 1U >= config.max_refinements;
      break;
    }
  }

  NestingResult result = best.result;
  result.strategy = StrategyKind::gdrr;
  result.budget = evaluator.make_budget(operations_completed);
  result.stop_reason = detail::driver_stop_reason(
      control, time_budget, stopwatch, hit_operation_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
