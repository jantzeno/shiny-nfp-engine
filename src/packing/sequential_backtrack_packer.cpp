#include "packing/sequential_backtrack_packer.hpp"

#include "packing/constructive_detail.hpp"

namespace shiny::nesting::pack {

auto SequentialBacktrackPacker::solve(const NormalizedRequest &request,
                                      const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  return detail::solve_sequential_backtrack(request, control);
}

} // namespace shiny::nesting::pack
