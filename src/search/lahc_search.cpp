#include "search/lahc_search.hpp"

#include <cstddef>
#include <format>

#include "observer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/lahc.hpp"
#include "search/detail/neighborhood_ops.hpp"
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

auto emit_improvement(const SolveControl &control, SearchReplay &replay,
                      const std::size_t sequence, const std::size_t iteration,
                      const SolutionPoolEntry &best, const BudgetState &budget,
                      const detail::NeighborhoodOperator op) -> void {
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
      .placed_parts = best.metrics.placed_parts,
      .total_parts = best.result.total_parts,
      .layout = best.result.layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::part_refinement,
      .phase_detail = std::format("LAHC iter {} improved via {}", iteration,
                                  detail::operator_label(op)),
      .utilization_percent = best.metrics.utilization * 100.0,
      .improved = true,
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
  const auto &config = resolve_strategy_config(request.request.execution,
                                               StrategyKind::lahc,
                                               ProductionOptimizerKind::lahc,
                                               request.request.execution.lahc);
  const std::size_t iteration_limit =
      control.iteration_limit > 0U ? control.iteration_limit : config.max_refinements;

  SearchReplay replay{.optimizer = OptimizerKind::lahc};
  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::lahc, std::move(replay),
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
  if (better_metrics(area_seed.metrics, current.metrics)) {
    current = area_seed;
  }
  auto best = current;
  detail::LateAcceptanceHistory lahc(config.history_length,
                                     detail::objective_score(current.metrics));

  std::size_t iterations_completed = 0;
  std::size_t sequence = 1U;
  bool hit_iteration_limit = false;
  std::size_t plateau = 0;
  replay.progress.push_back({
      .iteration = 0,
      .improved = true,
      .layout = best.result.layout,
      .budget = evaluator.make_budget(0),
  });

  for (std::size_t iteration = 0; iteration < iteration_limit; ++iteration) {
    if (evaluator.interrupted()) {
      break;
    }

    const auto op = detail::random_operator(rng, true);
    const auto move = detail::propose_move(
        current.order, current.forced_rotations, request, evaluator.piece_areas(),
        evaluator.piece_rotation_counts(), rng, op,
        1U + rng.uniform_index(std::max<std::size_t>(config.perturbation_swaps, 1U)));
    if (!move.changed) {
      continue;
    }

    auto candidate =
        evaluator.evaluate(move.order, move.forced_rotations, iteration + 31U);
    ++iterations_completed;

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
      emit_improvement(control, replay, sequence, iterations_completed, best,
                       evaluator.make_budget(iterations_completed), op);
    }
    if (plateau >= config.plateau_limit) {
      current = best;
      plateau = 0;
    }
    if (iterations_completed >= iteration_limit) {
      hit_iteration_limit = control.iteration_limit > 0U ||
                            iteration + 1U >= config.max_refinements;
      break;
    }
  }

  NestingResult result = best.result;
  result.strategy = StrategyKind::lahc;
  result.budget = evaluator.make_budget(iterations_completed);
  result.stop_reason =
      detail::driver_stop_reason(control, time_budget, stopwatch, hit_iteration_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
