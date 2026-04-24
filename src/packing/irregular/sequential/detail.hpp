#pragma once

#include "packing/irregular/core.hpp"

namespace shiny::nesting::pack::detail {

[[nodiscard]] auto solve_irregular_constructive(const NormalizedRequest &request,
                                                const SolveControl &control)
    -> util::StatusOr<NestingResult>;

} // namespace shiny::nesting::pack::detail
