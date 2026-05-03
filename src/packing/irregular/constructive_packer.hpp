#pragma once

#include "internal/request_normalization.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::pack {

class IrregularConstructivePacker {
public:
  [[nodiscard]] auto solve(const NormalizedRequest &request,
                           const SolveControl &control)
      -> std::expected<NestingResult, util::Status>;
};

} // namespace shiny::nesting::pack
