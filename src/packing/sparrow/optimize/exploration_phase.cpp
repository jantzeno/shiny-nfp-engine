#include "packing/sparrow/optimize/exploration_phase.hpp"

#include <algorithm>

#include "packing/sparrow/optimize/disruption.hpp"
#include "runtime/hash.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

namespace {

using search::detail::NeighborhoodSearch;

[[nodiscard]] auto
phase_interrupted(const SolveControl &control,
                  const ::shiny::nesting::runtime::TimeBudget &time_budget,
                  const ::shiny::nesting::runtime::Stopwatch &stopwatch)
    -> bool {
  return control.cancellation.stop_requested() ||
         time_budget.expired(stopwatch);
}

[[nodiscard]] auto allowed_operators(const bool enable_rotation_moves)
    -> std::vector<NeighborhoodSearch> {
  std::vector<NeighborhoodSearch> operators{
      NeighborhoodSearch::adjacent_swap,
      NeighborhoodSearch::random_swap,
      NeighborhoodSearch::relocate,
      NeighborhoodSearch::inversion,
  };
  if (enable_rotation_moves) {
    operators.push_back(NeighborhoodSearch::rotation_change);
  }
  return operators;
}

[[nodiscard]] auto schedule_ratio(const std::size_t iteration,
                                  const std::size_t total_iterations,
                                  const double max_ratio,
                                  const double min_ratio) -> double {
  const double upper = std::max(max_ratio, min_ratio);
  const double lower = std::min(max_ratio, min_ratio);
  if (total_iterations <= 1U) {
    return upper;
  }

  const auto clamped_iteration = std::min(iteration, total_iterations - 1U);
  const double progress = static_cast<double>(clamped_iteration) /
                          static_cast<double>(total_iterations - 1U);
  return upper - ((upper - lower) * progress);
}

} // namespace

auto run_exploration_phase(
    const NormalizedRequest &request, const SolveControl &control,
    const ::shiny::nesting::runtime::TimeBudget &time_budget,
    const ::shiny::nesting::runtime::Stopwatch &stopwatch,
    const search::SolutionPoolEntry &seed, const ExplorationPhaseConfig &config,
    runtime::TraceCapture *trace) -> ExplorationPhaseResult {
  ExplorationPhaseResult result{
      .best_solution = seed,
      .incumbent_solution = seed,
  };
  result.phase_metrics.exploration_iteration_budget = config.iteration_budget;

  if (config.iteration_budget == 0U || request.expanded_pieces.size() < 2U ||
      seed.order.size() < 2U) {
    return result;
  }

  search::detail::OrderEvaluator evaluator(request, control, time_budget,
                                           stopwatch);
  search::SolutionPool pool(8U, &request);
  pool.insert(seed);

  ::shiny::nesting::runtime::DeterministicRng rng(
      control.random_seed ^ ::shiny::nesting::runtime::hash::kGoldenRatio64);
  const auto operators = allowed_operators(config.enable_rotation_moves);
  search::SolutionPool infeasible_pool(config.infeasible_pool_capacity,
                                       &request);
  std::size_t no_improvement = 0U;
  std::size_t rejected_since_rollback = 0U;

  for (std::size_t iteration = 0; iteration < config.iteration_budget;
       ++iteration) {
    if (phase_interrupted(control, time_budget, stopwatch) ||
        evaluator.interrupted()) {
      break;
    }

    const auto tolerance_ratio = schedule_ratio(
        iteration, config.iteration_budget, config.acceptance_window_max_ratio,
        config.acceptance_window_min_ratio);
    const auto *base = pool.select(rng);
    if (base == nullptr) {
      base = &result.incumbent_solution;
    }

    const auto op = operators.empty() ? NeighborhoodSearch::random_swap
                                      : operators[iteration % operators.size()];
    search::detail::NeighborhoodMove move;
    if (config.enable_disruption_moves &&
        iteration % (operators.size() + 1U) == operators.size()) {
      const auto disrupted = disrupt_large_items(base->order, request,
                                                 evaluator.piece_areas(), rng);
      move.op = search::detail::NeighborhoodSearch::large_item_swap;
      move.order = disrupted.order;
      move.forced_rotations = base->piece_indexed_forced_rotations;
      move.primary_index = disrupted.first_position;
      move.secondary_index = disrupted.second_position;
      move.changed = disrupted.applied;
    } else {
      move = search::detail::propose_move(
          base->order, base->piece_indexed_forced_rotations, request,
          evaluator.piece_areas(), evaluator.piece_rotation_counts(), rng, op,
          1U);
    }
    const auto candidate =
        evaluator.evaluate(move.order, move.forced_rotations, iteration + 1U);

    const bool improved_best =
        better_metrics(candidate.metrics, result.best_solution.metrics);
    const bool escape_move =
        !improved_best &&
        search::detail::within_record_window(
            candidate.metrics, result.best_solution.metrics, tolerance_ratio) &&
        (candidate.order != result.incumbent_solution.order ||
         candidate.metrics.strip_length !=
             result.incumbent_solution.metrics.strip_length ||
         candidate.metrics.utilization !=
             result.incumbent_solution.metrics.utilization);
    const bool accepted = improved_best || escape_move;

    result.steps.push_back({
        .iteration = iteration + 1U,
        .operator_name = search::detail::operator_label(move.op),
        .accepted = accepted,
        .improved_best = improved_best,
        .escape_move = escape_move,
    });
    ++result.iterations;

    if (accepted) {
      result.incumbent_solution = candidate;
      pool.insert(candidate);
      ++result.accepted_moves;
      ++result.phase_metrics.accepted_moves;
      if (escape_move) {
        ++result.escape_moves;
      }
      if (improved_best) {
        result.best_solution = candidate;
      }
      no_improvement = 0U;
      rejected_since_rollback = 0U;
    } else {
      ++result.phase_metrics.infeasible_candidates;
      ++rejected_since_rollback;
      if (config.infeasible_pool_capacity > 0U) {
        infeasible_pool.insert(candidate);
      }
      if (config.infeasible_pool_capacity > 0U && config.rollback_after > 0U &&
          rejected_since_rollback >= config.rollback_after) {
        const auto decision = resolve_rollback_candidate(
            result.incumbent_solution,
            select_rollback_candidate(infeasible_pool.entries(),
                                      result.incumbent_solution,
                                      control.random_seed, trace));
        if (decision.rolled_back) {
          result.incumbent_solution = decision.next_incumbent;
          ++result.phase_metrics.infeasible_rollbacks;
        } else if (decision.restored_previous_incumbent) {
          ++result.restored_incumbents;
        }
        rejected_since_rollback = 0U;
      }
      ++no_improvement;
      if (config.plateau_limit > 0U && no_improvement >= config.plateau_limit) {
        break;
      }
    }
  }

  result.phase_metrics.exploration_iterations = result.iterations;
  return result;
}

} // namespace shiny::nesting::pack::sparrow::optimize