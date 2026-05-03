#include "packing/constructive/fill_first_engine.hpp"

#include "packing/irregular/sequential/detail.hpp"
#include "packing/sparrow/adapters/layout_adapter.hpp"

namespace shiny::nesting::pack::constructive {

auto make_fill_first_config(const ExecutionPolicy &execution)
    -> FillFirstConfig {
  return {
      .candidate_strategy = execution.irregular.candidate_strategy,
      .piece_ordering = execution.irregular.piece_ordering,
      .max_backtrack_pieces = execution.irregular.max_backtrack_pieces,
      .compaction_passes = execution.irregular.compaction_passes,
      .enable_backtracking = execution.irregular.enable_backtracking,
      .enable_compaction = execution.irregular.enable_compaction,
      .enable_gap_fill = execution.irregular.enable_gap_fill,
      .allow_part_overflow = execution.allow_part_overflow,
  };
}

auto export_sparrow_seed(const NestingResult &result) -> sparrow::SeedSolution {
  return sparrow::adapters::to_seed_solution(result.layout,
                                             result.constructive);
}

auto FillFirstEngine::solve(const NormalizedRequest &request,
                            const SolveControl &control)
    -> std::expected<NestingResult, util::Status> {
  return detail::solve_irregular_constructive(request, control);
}

} // namespace shiny::nesting::pack::constructive