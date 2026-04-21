#include "packing/irregular_constructive_packer.hpp"

#include "packing/constructive_detail.hpp"

namespace shiny::nesting::pack {

auto IrregularConstructivePacker::solve(const NormalizedRequest &request,
                                        const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  return detail::solve_irregular_constructive(request, control);
}

} // namespace shiny::nesting::pack
