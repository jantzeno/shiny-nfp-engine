#include "search/alns_search.hpp"

#include <algorithm>
#include <cstddef>
#include <format>
#include <numeric>
#include <vector>

#include "observer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/detail/cooling_schedule.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/neighborhood_search.hpp"
#include "search/solution_pool.hpp"
#include "search/strategy_registry.hpp"

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto run_alns_strategy(const NormalizedRequest &request,
                                     const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  AlnsSearch search;
  return search.solve(request, control);
}

} // namespace

void register_alns_strategy() {
  StrategyRegistry::instance().register_strategy({
      .name = "alns",
      .kind = StrategyKind::alns,
      .run = &run_alns_strategy,
  });
  StrategyRegistry::instance().register_production_strategy({
      .name = "alns",
      .kind = ProductionOptimizerKind::alns,
      .run = &run_alns_strategy,
  });
}

namespace {

[[nodiscard]] auto choose_operator(const std::vector<double> &weights,
                                   runtime::DeterministicRng &rng)
    -> std::size_t {
  const double total = std::accumulate(weights.begin(), weights.end(), 0.0);
  if (total <= 0.0) {
    return 0U;
  }
  double probe = rng.uniform_real() * total;
  for (std::size_t index = 0; index < weights.size(); ++index) {
    probe -= weights[index];
    if (probe <= 0.0) {
      return index;
    }
  }
  return weights.empty() ? 0U : weights.size() - 1U;
}

auto emit_improvement(const SolveControl &control, SearchReplay &replay,
                      const std::size_t sequence, const std::size_t iteration,
                      const SolutionPoolEntry &best,
                      const detail::NeighborhoodSearch op,
                      const double acceptance_ratio) -> void {
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
      .phase_detail =
          std::format("ALNS iter {} improved via {} (tol {:.3f})", iteration,
                      detail::operator_label(op), acceptance_ratio),
      .utilization_percent = best.metrics.utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
      .improved = true,
  });
}

auto emit_iteration_progress(const SolveControl &control,
                             ProgressThrottle &throttle,
                             const std::size_t iteration,
                             const SolutionPoolEntry &current,
                             const detail::NeighborhoodSearch op,
                             const bool accepted) -> void {
  if (!control.on_progress || !throttle.should_emit()) {
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
      .phase_detail = std::format("ALNS iter {} {} via {}", iteration,
                                  accepted ? "accepted" : "rejected",
                                  detail::operator_label(op)),
      .utilization_percent = current.metrics.utilization * 100.0,
      .layout_valid = current.result.layout_valid(),
      .validation_issue_count = current.result.validation.issues.size(),
      .improved = false,
  });
}

} // namespace

// ALNS — Adaptive Large Neighbourhood Search (Ropke & Pisinger 2006).
//
// Maintains a weight per neighbourhood operator. Each iteration:
//   1. Roulette-select an operator proportional to its weight.
//   2. Pull a base solution from the SolutionPool (gaussian-biased).
//   3. Apply destroy+repair / swap / rotation move.
//   4. Accept iff better OR within record window (acceptance_ratio).
//   5. Reward operator by reaction_factor × (improve|accept|reject).
//   6. Every `segment_length` iterations: decay all weights by 0.95
//      (with a 0.25 floor) to prevent total dominance by one operator.
//
// Acceptance decay: `acceptance_ratio` cools via the same
// CoolingSchedule used by SA (geometric), interpolating from
// initial_acceptance_ratio → final_acceptance_ratio.
auto AlnsSearch::solve(const NormalizedRequest &request,
                       const SolveControl &control) const
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_strategy_config(
      request.request.execution, StrategyKind::alns,
      ProductionOptimizerKind::alns, request.request.execution.alns);
  const auto operation_budget =
      detail::make_operation_budget(control, config.max_refinements);
  const std::size_t operation_limit = operation_budget.iteration_limit();

  SearchReplay replay{.optimizer = OptimizerKind::alns};
  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::alns, std::move(replay));
  }

  detail::OrderEvaluator evaluator(request, control, time_budget, stopwatch);
  runtime::DeterministicRng rng(control.random_seed);
  ProgressThrottle throttle;
  SolutionPool pool(8U, &request);

  SAConfig cooling_config;
  cooling_config.max_refinements =
      std::max<std::size_t>(config.max_refinements, 1U);
  cooling_config.initial_temperature = config.initial_acceptance_ratio;
  cooling_config.final_temperature = config.final_acceptance_ratio;
  cooling_config.cooling_schedule = CoolingScheduleKind::geometric;
  detail::CoolingSchedule cooling(cooling_config);

  const auto forced_rotations = detail::original_forced_rotations(request);
  auto current =
      evaluator.evaluate(detail::original_order(request), forced_rotations, 1U);
  pool.insert(current);
  auto area_seed =
      evaluator.evaluate(detail::descending_area_order(evaluator.piece_areas()),
                         forced_rotations, 2U);
  pool.insert(area_seed);
  auto reverse_seed =
      evaluator.evaluate(detail::reverse_order(request), forced_rotations, 3U);
  pool.insert(reverse_seed);
  auto best =
      better_metrics(area_seed.metrics, current.metrics) ? area_seed : current;
  if (better_metrics(reverse_seed.metrics, best.metrics)) {
    best = reverse_seed;
  }
  current = best;

  std::vector<detail::NeighborhoodSearch> operators =
      detail::all_alns_operators();
  std::vector<double> weights(operators.size(), 1.0);

  std::size_t sequence = 1U;
  replay.progress.push_back({
      .iteration = 0,
      .improved = true,
      .layout = best.result.layout,
  });
  std::size_t operations_completed = 0;
  bool hit_operation_limit = false;
  double acceptance_ratio = cooling.initial_temperature();

  for (std::size_t iteration = 0; iteration < operation_limit; ++iteration) {
    if (evaluator.interrupted()) {
      break;
    }

    const auto selected = choose_operator(weights, rng);
    const auto destroy_span =
        config.destroy_max_count - config.destroy_min_count + 1U;
    const auto intensity =
        config.destroy_min_count +
        rng.uniform_index(std::max<std::size_t>(destroy_span, 1U));
    const auto *base = pool.select(rng);
    const auto move = detail::propose_move(
        base == nullptr ? current.order : base->order,
        base == nullptr ? current.piece_indexed_forced_rotations
                        : base->piece_indexed_forced_rotations,
        request, evaluator.piece_areas(), evaluator.piece_rotation_counts(),
        rng, operators[selected], intensity);
    if (!move.changed) {
      continue;
    }

    auto candidate =
        evaluator.evaluate(move.order, move.forced_rotations, iteration + 11U);
    ++operations_completed;

    const bool accepted =
        better_metrics(candidate.metrics, current.metrics) ||
        detail::within_record_window(candidate.metrics, current.metrics,
                                     acceptance_ratio);
    double reward = config.reward_reject;
    if (accepted) {
      current = candidate;
      pool.insert(candidate);
      reward = config.reward_accept;
    }
    if (better_metrics(candidate.metrics, best.metrics)) {
      best = candidate;
      pool.insert(candidate);
      reward = config.reward_improve;
      ++sequence;
      emit_improvement(control, replay, sequence, operations_completed, best,
                       operators[selected], acceptance_ratio);
    } else {
      emit_iteration_progress(control, throttle, operations_completed, current,
                              operators[selected], accepted);
    }

    weights[selected] =
        std::max(0.1, ((1.0 - config.reaction_factor) * weights[selected]) +
                          (config.reaction_factor * reward));
    if ((iteration + 1U) % config.segment_length == 0U) {
      for (auto &weight : weights) {
        weight = std::max(0.25, weight * 0.95);
      }
    }

    acceptance_ratio =
        cooling.next_temperature(acceptance_ratio, iteration, accepted);
    if (operations_completed >= operation_limit) {
      hit_operation_limit =
          operation_budget.external_limit_reached(operations_completed);
      break;
    }
  }

  NestingResult result = best.result;
  result.strategy = StrategyKind::alns;
  result.stop_reason = detail::driver_stop_reason(
      control, time_budget, stopwatch, hit_operation_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
