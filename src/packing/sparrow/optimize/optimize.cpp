#include "packing/sparrow/optimize/optimize.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

auto run_optimize(const NormalizedRequest &request, const SolveControl &control,
                  const ::shiny::nesting::runtime::TimeBudget &time_budget,
                  const ::shiny::nesting::runtime::Stopwatch &stopwatch,
                  const search::SolutionPoolEntry &seed,
                  const OptimizeConfig &config, runtime::TraceCapture *trace)
    -> OptimizeResult {
  OptimizeResult result;
  result.exploration =
      run_exploration_phase(request, control, time_budget, stopwatch, seed,
                            config.exploration, trace);

  const auto &compression_seed = result.exploration.iterations > 0U
                                     ? result.exploration.best_solution
                                     : seed;
  result.compression =
      run_compression_phase(request, control, time_budget, stopwatch,
                            compression_seed, config.compression, trace);

  result.best_solution = result.exploration.iterations > 0U
                             ? result.exploration.best_solution
                             : seed;
  if (result.compression.iterations > 0U &&
      compare_objective(result.compression.best_solution,
                        result.best_solution) == ObjectiveOrdering::better) {
    result.best_solution = result.compression.best_solution;
  }

  result.best_objective = evaluate_objective(result.best_solution);
  result.accepted_moves =
      result.exploration.accepted_moves + result.compression.accepted_moves;
  result.phase_metrics.exploration_iteration_budget =
      result.exploration.phase_metrics.exploration_iteration_budget;
  result.phase_metrics.exploration_iterations =
      result.exploration.phase_metrics.exploration_iterations;
  result.phase_metrics.compression_iteration_budget =
      result.compression.phase_metrics.compression_iteration_budget;
  result.phase_metrics.compression_iterations =
      result.compression.phase_metrics.compression_iterations;
  result.phase_metrics.accepted_moves = result.accepted_moves;
  result.phase_metrics.infeasible_candidates =
      result.exploration.phase_metrics.infeasible_candidates +
      result.compression.phase_metrics.infeasible_candidates;
  result.phase_metrics.infeasible_rollbacks =
      result.exploration.phase_metrics.infeasible_rollbacks +
      result.compression.phase_metrics.infeasible_rollbacks;
  result.separator_metrics = result.compression.separator_metrics;
  return result;
}

} // namespace shiny::nesting::pack::sparrow::optimize