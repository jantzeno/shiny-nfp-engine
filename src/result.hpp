#pragma once

#include <cstddef>

#include "observer.hpp"
#include "request.hpp"

namespace shiny::nesting {

enum class OptimizerKind : std::uint8_t {
  none = 0,
  brkga = 1,
  simulated_annealing = 2,
  alns = 3,
  gdrr = 4,
  lahc = 5,
};

struct SearchProgressEntry {
  std::size_t iteration{0};
  bool improved{false};
  pack::Layout layout{};
  BudgetState budget{};
};

struct SearchReplay {
  OptimizerKind optimizer{OptimizerKind::none};
  bool sparrow_polished{false};
  std::vector<SearchProgressEntry> progress{};
};

struct NestingResult {
  StrategyKind strategy{StrategyKind::bounding_box};
  pack::Layout layout{};
  std::size_t total_parts{0};
  BudgetState budget{};
  StopReason stop_reason{StopReason::none};
  SearchReplay search{};
};

} // namespace shiny::nesting
