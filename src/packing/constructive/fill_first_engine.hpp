#pragma once

#include "internal/request_normalization.hpp"
#include "packing/sparrow/solution.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::pack::constructive {

struct FillFirstConfig {
  CandidateStrategy candidate_strategy{CandidateStrategy::nfp_hybrid};
  PieceOrdering piece_ordering{PieceOrdering::largest_area_first};
  std::uint32_t max_backtrack_pieces{3};
  std::uint32_t compaction_passes{2};
  bool enable_backtracking{true};
  bool enable_compaction{true};
  bool enable_gap_fill{true};
  bool allow_part_overflow{true};
};

[[nodiscard]] auto make_fill_first_config(const ExecutionPolicy &execution)
    -> FillFirstConfig;

[[nodiscard]] auto export_sparrow_seed(const NestingResult &result)
    -> sparrow::SeedSolution;

class FillFirstEngine {
public:
  [[nodiscard]] auto solve(const NormalizedRequest &request,
                           const SolveControl &control)
      -> std::expected<NestingResult, util::Status>;
};

} // namespace shiny::nesting::pack::constructive