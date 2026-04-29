#include "search/lahc_search.hpp"

#include <cstddef>
#include <format>

#include "observer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/lahc.hpp"
#include "search/detail/neighborhood_search.hpp"
#include "search/strategy_registry.hpp"

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto run_lahc_strategy(const NormalizedRequest &request,
                                     const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  LahcSearch search;
  return search.solve(request, control);
}

} // namespace

void register_lahc_strategy() {
  StrategyRegistry::instance().register_strategy({
      .name = "lahc",
      .kind = StrategyKind::lahc,
      .run = &run_lahc_strategy,
  });
  StrategyRegistry::instance().register_production_strategy({
      .name = "lahc",
      .kind = ProductionOptimizerKind::lahc,
      .run = &run_lahc_strategy,
  });
}

namespace {

auto emit_initial_progress(const SolveControl &control,
                           const SolutionPoolEntry &best) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 1U,
      .placements_successful = best.metrics.placed_parts,
      .total_requested_parts = best.result.total_parts,
      .refinement_steps_completed = 0U,
      .layout = best.result.layout,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail = "LAHC initial seed",
      .utilization_percent = best.metrics.utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
      .improved = true,
  });
}

auto emit_improvement(const SolveControl &control, SearchReplay &replay,
                      const std::size_t sequence, const std::size_t iteration,
                      const SolutionPoolEntry &best,
                      const detail::NeighborhoodSearch op) -> void {
  replay.progress.push_back({
      .iteration = iteration,
      .improved = true,
      .layout = best.result.layout,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
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
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail = std::format("LAHC iter {} improved via {}", iteration,
                                  detail::operator_label(op)),
      .utilization_percent = best.metrics.utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
      .improved = true,
  });
}

auto emit_iteration_progress(const SolveControl &control,
                             const std::size_t iteration,
                             const SolutionPoolEntry &current,
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
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::refinement,
      .phase_detail = std::format("LAHC iter {} {} via {}", iteration,
                                  accepted ? "accepted" : "rejected",
                                  detail::operator_label(op)),
      .utilization_percent = current.metrics.utilization * 100.0,
      .layout_valid = current.result.layout_valid(),
      .validation_issue_count = current.result.validation.issues.size(),
      .improved = false,
  });
}

} // namespace

// LAHC — Late Acceptance Hill Climbing (Burke & Bykov 2017).
//
// Maintains a length-`history_length` ring buffer of past-iteration
// objective scores. Accepts candidate iff it beats current OR beats
// the score `history_length` iterations ago. The history is updated
// every iteration with either the candidate or current score.
//
// Plateau: after `plateau_limit` non-improving moves, snap current
// back to best (escape).
auto LahcSearch::solve(const NormalizedRequest &request,
                       const SolveControl &control) const
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_strategy_config(
      request.request.execution, StrategyKind::lahc,
      ProductionOptimizerKind::lahc, request.request.execution.lahc);
  const auto operation_budget =
      detail::make_operation_budget(control, config.max_refinements);
  const std::size_t operation_limit = operation_budget.iteration_limit();

  SearchReplay replay{.optimizer = OptimizerKind::lahc};
  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::lahc, std::move(replay));
  }

  detail::OrderEvaluator evaluator(request, control, time_budget, stopwatch);
  runtime::DeterministicRng rng(control.random_seed);

  const auto forced_rotations = detail::original_forced_rotations(request);
  auto current =
      evaluator.evaluate(detail::original_order(request), forced_rotations, 1U);
  auto area_seed =
      evaluator.evaluate(detail::descending_area_order(evaluator.piece_areas()),
                         forced_rotations, 2U);
  if (better_metrics(area_seed.metrics, current.metrics)) {
    current = area_seed;
  }
  auto best = current;
  detail::LateAcceptanceHistory lahc(config.history_length,
                                     detail::objective_score(current.metrics));

  std::size_t operations_completed = 0;
  std::size_t sequence = 1U;
  bool hit_operation_limit = false;
  std::size_t plateau = 0;
  replay.progress.push_back({
      .iteration = 0,
      .improved = true,
      .layout = best.result.layout,
  });
  emit_initial_progress(control, best);

  for (std::size_t iteration = 0; iteration < operation_limit; ++iteration) {
    if (evaluator.interrupted()) {
      break;
    }

    const auto op = detail::random_operator(rng, true);
    const auto move = detail::propose_move(
        current.order, current.piece_indexed_forced_rotations, request,
        evaluator.piece_areas(), evaluator.piece_rotation_counts(), rng, op,
        1U + rng.uniform_index(
                 std::max<std::size_t>(config.perturbation_swaps, 1U)));
    if (!move.changed) {
      continue;
    }

    auto candidate =
        evaluator.evaluate(move.order, move.forced_rotations, iteration + 31U);
    ++operations_completed;

    const double current_score = detail::objective_score(current.metrics);
    const double candidate_score = detail::objective_score(candidate.metrics);
    const bool accepted = lahc.accepts(candidate_score, current_score);
    if (accepted) {
      current = candidate;
      plateau = 0;
    } else {
      ++plateau;
    }
    // LAHC variant: history is updated with the *current* working
    // solution after the accept/reject decision. When the candidate
    // is accepted, current == candidate and we feed `candidate_score`
    // into the history; when rejected, current is unchanged and we
    // feed the OLD `current_score` back. This corresponds to Burke
    // & Bykov's "Late Acceptance Hill Climbing" (2017,
    // doi:10.1016/j.ejor.2016.07.012, §3) and is intentional — it is
    // *not* the LAHC bug where the candidate score is always written.
    lahc.advance(accepted ? candidate_score : current_score);

    if (better_metrics(candidate.metrics, best.metrics)) {
      best = candidate;
      ++sequence;
      emit_improvement(control, replay, sequence, operations_completed, best,
                       op);
    }
    emit_iteration_progress(control, operations_completed, current, op,
                            accepted);
    if (plateau >= config.plateau_limit) {
      current = best;
      plateau = 0;
    }
    if (operations_completed >= operation_limit) {
      hit_operation_limit =
          operation_budget.external_limit_reached(operations_completed);
      break;
    }
  }

  NestingResult result = best.result;
  result.strategy = StrategyKind::lahc;
  result.stop_reason = detail::driver_stop_reason(
      control, time_budget, stopwatch, hit_operation_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
