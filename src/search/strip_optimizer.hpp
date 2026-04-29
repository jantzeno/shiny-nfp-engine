#pragma once

#include <cstddef>

#include "request.hpp"
#include "runtime/timing.hpp"
#include "search/solution_pool.hpp"
#include "solve.hpp"

namespace shiny::nesting::search {

namespace detail {

[[nodiscard]] auto derive_iteration_budget(const ProductionSearchConfig &config,
                                           std::size_t piece_count)
    -> std::size_t;

[[nodiscard]] auto schedule_ratio(std::size_t iteration,
                                  std::size_t total_iterations,
                                  double max_ratio,
                                  double min_ratio) -> double;

} // namespace detail

struct StripOptimizerResult {
  SolutionPoolEntry best_solution{};
  std::size_t exploration_iterations{0};
  std::size_t compression_iterations{0};
  std::size_t accepted_moves{0};
  SearchPhaseMetrics phase_metrics{};
  SeparatorReplayMetrics separator_metrics{};
};

class StripOptimizer {
public:
  [[nodiscard]] auto optimize(const NormalizedRequest &request,
                              const SolveControl &control,
                              const runtime::TimeBudget &time_budget,
                              const runtime::Stopwatch &stopwatch,
                              const SolutionPoolEntry &seed,
                              const ProductionSearchConfig &config) const
      -> StripOptimizerResult;
};

} // namespace shiny::nesting::search
