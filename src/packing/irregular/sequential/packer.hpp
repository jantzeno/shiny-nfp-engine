#pragma once

#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::pack {

class SequentialBacktrackPacker {
public:
  [[nodiscard]] auto solve(const NormalizedRequest &request,
                           const SolveControl &control)
      -> util::StatusOr<NestingResult>;
};

} // namespace shiny::nesting::pack
