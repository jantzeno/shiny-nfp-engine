#pragma once

#include <cstdint>

#include "packing/sparrow/search/solution_pool.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct MultiBinObjectiveBreakdown {
  std::size_t active_bin_count{0};
  double active_bin_compaction{0.0};
  double active_bin_utilization{0.0};
};

struct ObjectiveValue {
  search::LayoutMetrics metrics{};
  MultiBinObjectiveBreakdown multi_bin{};
  double placed_value{0.0};
  ObjectiveMode objective_mode{ObjectiveMode::placement_count};
  std::uint64_t order_signature{0};
  std::uint64_t rotation_signature{0};
};

enum class ObjectiveOrdering : std::uint8_t {
  better = 0,
  worse = 1,
  equivalent = 2,
};

[[nodiscard]] auto evaluate_objective(const search::SolutionPoolEntry &entry)
    -> ObjectiveValue;

[[nodiscard]] auto compare_objective(const ObjectiveValue &lhs,
                                     const ObjectiveValue &rhs)
    -> ObjectiveOrdering;

[[nodiscard]] auto compare_objective(const search::SolutionPoolEntry &lhs,
                                     const search::SolutionPoolEntry &rhs)
    -> ObjectiveOrdering;

[[nodiscard]] auto objective_better(const ObjectiveValue &lhs,
                                    const ObjectiveValue &rhs) -> bool;

} // namespace shiny::nesting::pack::sparrow::optimize