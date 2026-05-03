#pragma once

#include "internal/request_normalization.hpp"
#include "packing/sparrow/optimize/compression_phase.hpp"
#include "packing/sparrow/optimize/exploration_phase.hpp"
#include "packing/sparrow/optimize/objective.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct OptimizeConfig {
  ExplorationPhaseConfig exploration{};
  CompressionPhaseConfig compression{};
};

struct OptimizeResult {
  search::SolutionPoolEntry best_solution{};
  ObjectiveValue best_objective{};
  ExplorationPhaseResult exploration{};
  CompressionPhaseResult compression{};
  std::size_t accepted_moves{0};
  SearchPhaseMetrics phase_metrics{};
  SeparatorReplayMetrics separator_metrics{};
};

[[nodiscard]] auto
run_optimize(const NormalizedRequest &request, const SolveControl &control,
             const ::shiny::nesting::runtime::TimeBudget &time_budget,
             const ::shiny::nesting::runtime::Stopwatch &stopwatch,
             const search::SolutionPoolEntry &seed,
             const OptimizeConfig &config,
             runtime::TraceCapture *trace = nullptr) -> OptimizeResult;

} // namespace shiny::nesting::pack::sparrow::optimize