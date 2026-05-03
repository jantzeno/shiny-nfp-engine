#pragma once

#include "internal/request_normalization.hpp"
#include "packing/irregular/core.hpp"

namespace shiny::nesting::pack::detail {

[[nodiscard]] auto
solve_irregular_constructive(const NormalizedRequest &request,
                             const SolveControl &control)
    -> std::expected<NestingResult, util::Status>;

} // namespace shiny::nesting::pack::detail
