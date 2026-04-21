#pragma once

#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::search {

class AlnsSearch {
public:
  [[nodiscard]] auto solve(const NormalizedRequest &request,
                           const SolveControl &control) const
      -> util::StatusOr<NestingResult>;
};

} // namespace shiny::nesting::search
