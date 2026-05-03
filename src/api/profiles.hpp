#pragma once

#include <string_view>

#include "request.hpp"

namespace shiny::nesting::api {

struct ProfilePreset {
  StrategyKind strategy{StrategyKind::bounding_box};
  ProductionOptimizerKind production_optimizer{ProductionOptimizerKind::brkga};
  ProductionSearchConfig production{};
};

[[nodiscard]] auto profile_is_valid(SolveProfile profile) -> bool;

[[nodiscard]] auto profile_requires_time_limit(SolveProfile profile) -> bool;

[[nodiscard]] auto profile_name(SolveProfile profile) -> std::string_view;

[[nodiscard]] auto profile_preset(SolveProfile profile) -> ProfilePreset;

} // namespace shiny::nesting::api