#pragma once

#include "request.hpp"
#include "result.hpp"

namespace shiny::nesting::search::detail {

[[nodiscard]] auto
constructive_layout_has_geometry_violation(const NormalizedRequest &request,
                                           const NestingResult &result) -> bool;

} // namespace shiny::nesting::search::detail
