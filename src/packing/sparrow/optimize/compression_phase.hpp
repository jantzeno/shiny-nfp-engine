#pragma once

#include <cstddef>
#include <vector>

#include "internal/request_normalization.hpp"
#include "packing/sparrow/runtime/trace.hpp"
#include "runtime/timing.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "solve.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct CompressionPhaseConfig {
  std::size_t iteration_budget{0};
  std::size_t plateau_limit{0};
  double shrink_max_ratio{0.01};
  double shrink_min_ratio{0.001};
  std::size_t separator_worker_count{1};
  std::size_t separator_max_iterations{16};
  std::size_t separator_iter_no_improvement_limit{10};
  std::size_t separator_strike_limit{4};
  std::size_t separator_global_samples{12};
  std::size_t separator_focused_samples{8};
  std::size_t separator_coordinate_descent_iterations{8};
  double gls_weight_cap{1e6};
};

struct CompressionPhaseStep {
  std::size_t iteration{0};
  double shrink_ratio{0.0};
  bool accepted{false};
  bool improved_best{false};
};

struct CompressionPhaseResult {
  search::SolutionPoolEntry best_solution{};
  search::SolutionPoolEntry incumbent_solution{};
  std::size_t iterations{0};
  std::size_t accepted_moves{0};
  SearchPhaseMetrics phase_metrics{};
  SeparatorReplayMetrics separator_metrics{};
  std::vector<CompressionPhaseStep> steps{};
};

[[nodiscard]] auto run_compression_phase(
    const NormalizedRequest &request, const SolveControl &control,
    const ::shiny::nesting::runtime::TimeBudget &time_budget,
    const ::shiny::nesting::runtime::Stopwatch &stopwatch,
    const search::SolutionPoolEntry &seed, const CompressionPhaseConfig &config,
    runtime::TraceCapture *trace = nullptr) -> CompressionPhaseResult;

} // namespace shiny::nesting::pack::sparrow::optimize