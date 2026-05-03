#pragma once

#include "packing/sparrow/solution.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

[[nodiscard]] auto to_seed_solution(const pack::Layout &layout,
                                    const ConstructiveReplay &constructive = {})
    -> SeedSolution;

[[nodiscard]] auto to_layout(const SeedSolution &seed) -> pack::Layout;

[[nodiscard]] auto to_layout(const PortSolution &solution) -> pack::Layout;

[[nodiscard]] auto to_nesting_result(const PortSolution &solution,
                                     StrategyKind strategy,
                                     std::size_t total_parts) -> NestingResult;

} // namespace shiny::nesting::pack::sparrow::adapters