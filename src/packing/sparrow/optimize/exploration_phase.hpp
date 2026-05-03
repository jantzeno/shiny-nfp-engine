#pragma once

#include <cstddef>
#include <string_view>
#include <vector>

#include "internal/request_normalization.hpp"
#include "packing/sparrow/runtime/trace.hpp"
#include "runtime/timing.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "solve.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct ExplorationPhaseConfig {
  std::size_t iteration_budget{0};
  std::size_t plateau_limit{0};
  double acceptance_window_max_ratio{0.25};
  double acceptance_window_min_ratio{0.02};
  bool enable_rotation_moves{true};
  bool enable_disruption_moves{false};
  std::size_t infeasible_pool_capacity{0};
  std::size_t rollback_after{0};
};

struct ExplorationPhaseStep {
  std::size_t iteration{0};
  std::string_view operator_name{};
  bool accepted{false};
  bool improved_best{false};
  bool escape_move{false};
};

struct ExplorationPhaseResult {
  search::SolutionPoolEntry best_solution{};
  search::SolutionPoolEntry incumbent_solution{};
  std::size_t iterations{0};
  std::size_t accepted_moves{0};
  std::size_t escape_moves{0};
  std::size_t restored_incumbents{0};
  SearchPhaseMetrics phase_metrics{};
  std::vector<ExplorationPhaseStep> steps{};
};

[[nodiscard]] auto run_exploration_phase(
    const NormalizedRequest &request, const SolveControl &control,
    const ::shiny::nesting::runtime::TimeBudget &time_budget,
    const ::shiny::nesting::runtime::Stopwatch &stopwatch,
    const search::SolutionPoolEntry &seed, const ExplorationPhaseConfig &config,
    runtime::TraceCapture *trace = nullptr) -> ExplorationPhaseResult;

} // namespace shiny::nesting::pack::sparrow::optimize