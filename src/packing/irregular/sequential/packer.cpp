#include "packing/irregular/sequential/packer.hpp"

#include "packing/irregular/sequential/detail.hpp"

namespace shiny::nesting::pack {

auto SequentialBacktrackPacker::solve(const NormalizedRequest &request,
                                      const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  return detail::solve_irregular_constructive(request, control);
}

} // namespace shiny::nesting::pack
