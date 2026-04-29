#include "search/simulated_annealing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <optional>
#include <vector>

#include "observer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/detail/cooling_schedule.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/neighborhood_search.hpp"
#include "search/strategy_registry.hpp"

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto
run_simulated_annealing_strategy(const NormalizedRequest &request,
                                 const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  SimulatedAnnealingSearch search;
  return search.solve(request, control);
}

} // namespace

void register_simulated_annealing_strategy() {
  StrategyRegistry::instance().register_strategy({
      .name = "simulated_annealing",
      .kind = StrategyKind::simulated_annealing,
      .run = &run_simulated_annealing_strategy,
  });
  StrategyRegistry::instance().register_production_strategy({
      .name = "simulated_annealing",
      .kind = ProductionOptimizerKind::simulated_annealing,
      .run = &run_simulated_annealing_strategy,
  });
}

namespace {

auto emit_improvement(const SolveControl &control, SearchReplay &replay,
                      const std::size_t sequence, const std::size_t iteration,
                      const SolutionPoolEntry &best,
                      const std::size_t restart_index,
                      const detail::NeighborhoodSearch op,
                      const double temperature) -> void {
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
      .phase_detail = std::format("SA restart {} iter {} via {} @ T={:.3f}",
                                  restart_index + 1U, iteration,
                                  detail::operator_label(op), temperature),
      .utilization_percent = best.metrics.utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
      .improved = true,
  });
}

auto emit_iteration_progress(
    const SolveControl &control, ProgressThrottle &throttle,
    const std::size_t iteration, const SolutionPoolEntry &current,
    const std::size_t restart_index, const detail::NeighborhoodSearch op,
    const double temperature, const bool accepted) -> void {
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
      .phase_detail = std::format("SA restart {} iter {} {} via {} @ T={:.3f}",
                                  restart_index + 1U, iteration,
                                  accepted ? "accepted" : "rejected",
                                  detail::operator_label(op), temperature),
      .utilization_percent = current.metrics.utilization * 100.0,
      .layout_valid = current.result.layout_valid(),
      .validation_issue_count = current.result.validation.issues.size(),
      .improved = false,
  });
}

[[nodiscard]] auto best_seed(const detail::OrderEvaluator &evaluator,
                             runtime::DeterministicRng &rng,
                             const NormalizedRequest &request,
                             const std::uint64_t seed_bias)
    -> SolutionPoolEntry {
  std::vector<SolutionPoolEntry> seeds;
  const auto forced_rotations = detail::original_forced_rotations(request);
  seeds.push_back(evaluator.evaluate(detail::original_order(request),
                                     forced_rotations, seed_bias));
  seeds.push_back(
      evaluator.evaluate(detail::descending_area_order(evaluator.piece_areas()),
                         forced_rotations, seed_bias + 1U));
  seeds.push_back(evaluator.evaluate(detail::reverse_order(request),
                                     forced_rotations, seed_bias + 2U));
  seeds.push_back(evaluator.evaluate(
      detail::random_order(request.expanded_pieces.size(), rng),
      forced_rotations, seed_bias + 3U));

  auto best = seeds.front();
  for (std::size_t index = 1; index < seeds.size(); ++index) {
    if (better_metrics(seeds[index].metrics, best.metrics)) {
      best = std::move(seeds[index]);
    }
  }
  return best;
}

} // namespace

// Sparrow §8.2 metaheuristic — simulated annealing over piece order +
// forced-rotation vector.
//
// Architecture: shared OrderEvaluator decodes orders into layouts via
// the irregular constructive packer (Phase 10). Per-iteration this
// driver:
//   1. Picks a neighbourhood operator (random_operator).
//   2. Proposes a move (propose_move) of intensity 1..perturbation_swaps.
//   3. Decodes the candidate order; computes Δ = candidate - current.
//   4. Accepts unconditionally if better, with prob exp(Δ/T) otherwise.
//   5. Cools T per the configured schedule (geometric / linear /
//      adaptive / lundy-mees — see CoolingSchedule).
//   6. After `plateau_window` consecutive non-improvements, reheat by
//      `reheating_factor` (capped at initial_temperature).
//
// Restarts: outer loop seeds each restart from the best of
// {original, area-desc, reverse, random} orders.
auto SimulatedAnnealingSearch::solve(const NormalizedRequest &request,
                                     const SolveControl &control) const
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_strategy_config(
      request.request.execution, StrategyKind::simulated_annealing,
      ProductionOptimizerKind::simulated_annealing,
      request.request.execution.simulated_annealing);
  const std::size_t configured_iterations =
      config.max_refinements * std::max<std::size_t>(config.restart_count, 1U);
  const auto operation_budget =
      detail::make_operation_budget(control, configured_iterations);
  const std::size_t operation_limit = operation_budget.iteration_limit();

  SearchReplay replay{.optimizer = OptimizerKind::simulated_annealing};
  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::simulated_annealing,
                                       std::move(replay));
  }

  detail::OrderEvaluator evaluator(request, control, time_budget, stopwatch);
  runtime::DeterministicRng rng(control.random_seed);
  detail::CoolingSchedule cooling(config);
  ProgressThrottle throttle;

  std::optional<SolutionPoolEntry> best;
  std::size_t operations_completed = 0;
  std::size_t sequence = 0;
  bool hit_operation_limit = false;

  for (std::size_t restart = 0; restart < config.restart_count; ++restart) {
    if (evaluator.interrupted()) {
      break;
    }
    if (operations_completed >= operation_limit) {
      hit_operation_limit =
          operation_budget.external_limit_reached(operations_completed);
      break;
    }

    auto current = best_seed(evaluator, rng, request, restart * 17U);
    if (!best.has_value() || better_metrics(current.metrics, best->metrics)) {
      best = current;
      ++sequence;
      emit_improvement(control, replay, sequence, operations_completed, *best,
                       restart, detail::NeighborhoodSearch::random_swap,
                       cooling.initial_temperature());
    }

    double temperature = cooling.initial_temperature();
    std::size_t plateau = 0;
    const std::size_t restart_budget = std::min(
        config.max_refinements, operation_limit - operations_completed);

    for (std::size_t restart_iteration = 0; restart_iteration < restart_budget;
         ++restart_iteration) {
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
          evaluator.evaluate(move.order, move.forced_rotations,
                             operations_completed + restart_iteration + 1U);
      ++operations_completed;

      const double current_score = detail::objective_score(current.metrics);
      const double candidate_score = detail::objective_score(candidate.metrics);
      const double delta = candidate_score - current_score;
      // Maximisation Metropolis criterion: `objective_score` is built
      // so that "better" means "larger" (placed_parts dominates, then
      // -bin_count, then -strip_length, then +utilization). Positive
      // Δ always accepts (exp(Δ/T) ≥ 1); negative Δ accepts with the
      // usual Boltzmann probability. Δ == 0 falls into `rng < 1.0`
      // and is therefore accepted with probability 1.0, matching
      // `delta >= 0` in the textbook formulation. We keep the
      // `delta > 0` short-circuit only to avoid the exp() call on the
      // happy path.
      const bool accepted =
          delta > 0.0 ||
          rng.uniform_real() < std::exp(delta / std::max(temperature, 1e-9));
      if (accepted) {
        current = candidate;
      }

      if (!best.has_value() ||
          better_metrics(candidate.metrics, best->metrics)) {
        best = candidate;
        plateau = 0;
        ++sequence;
        emit_improvement(control, replay, sequence, operations_completed, *best,
                         restart, op, temperature);
      } else {
        ++plateau;
        emit_iteration_progress(control, throttle, operations_completed,
                                current, restart, op, temperature, accepted);
      }

      temperature =
          cooling.next_temperature(temperature, restart_iteration, accepted);
      if (plateau >= config.plateau_window && config.reheating_factor > 1.0) {
        temperature = std::min(config.initial_temperature,
                               temperature * config.reheating_factor);
        plateau = 0;
      }
      if (operations_completed >= operation_limit) {
        hit_operation_limit =
            operation_budget.external_limit_reached(operations_completed);
        break;
      }
    }
  }

  if (!best.has_value()) {
    return NestingResult{
        .strategy = StrategyKind::simulated_annealing,
        .total_parts = request.expanded_pieces.size(),
        .stop_reason = detail::driver_stop_reason(
            control, time_budget, stopwatch, hit_operation_limit),
        .search = std::move(replay),
    };
  }

  NestingResult result = best->result;
  result.strategy = StrategyKind::simulated_annealing;
  result.stop_reason = detail::driver_stop_reason(
      control, time_budget, stopwatch, hit_operation_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
