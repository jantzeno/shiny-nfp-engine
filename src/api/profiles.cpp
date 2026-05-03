#include "api/profiles.hpp"

namespace shiny::nesting::api {

auto profile_is_valid(const SolveProfile profile) -> bool {
  switch (profile) {
  case SolveProfile::quick:
  case SolveProfile::balanced:
  case SolveProfile::maximum_search:
    return true;
  }
  return false;
}

auto profile_requires_time_limit(const SolveProfile profile) -> bool {
  return profile == SolveProfile::balanced ||
         profile == SolveProfile::maximum_search;
}

auto profile_name(const SolveProfile profile) -> std::string_view {
  switch (profile) {
  case SolveProfile::quick:
    return "quick";
  case SolveProfile::balanced:
    return "balanced";
  case SolveProfile::maximum_search:
    return "maximum_search";
  }
  return "unknown";
}

auto profile_preset(const SolveProfile profile) -> ProfilePreset {
  switch (profile) {
  case SolveProfile::quick:
    return {
        .strategy = StrategyKind::bounding_box,
    };
  case SolveProfile::balanced: {
    auto production = ProductionSearchConfig{};
    production.population_size = 12;
    production.elite_count = 3;
    production.mutant_count = 2;
    production.max_iterations = 8;
    production.polishing_passes = 0;
    production.infeasible_pool_capacity = 2;
    production.infeasible_rollback_after = 2;
    production.separator_worker_count = 1;
    production.separator_max_iterations = 24;
    production.separator_iter_no_improvement_limit = 6;
    production.separator_strike_limit = 2;
    production.separator_global_samples = 16;
    production.separator_focused_samples = 8;
    production.separator_coordinate_descent_iterations = 12;
    return {
        .strategy = StrategyKind::metaheuristic_search,
        .production_optimizer = ProductionOptimizerKind::brkga,
        .production = production,
    };
  }
  case SolveProfile::maximum_search: {
    auto production = ProductionSearchConfig{};
    production.population_size = 48;
    production.elite_count = 8;
    production.mutant_count = 8;
    production.max_iterations = 64;
    production.polishing_passes = 2;
    production.infeasible_pool_capacity = 6;
    production.infeasible_rollback_after = 4;
    production.separator_worker_count = 2;
    production.separator_max_iterations = 64;
    production.separator_iter_no_improvement_limit = 12;
    production.separator_strike_limit = 4;
    production.separator_global_samples = 48;
    production.separator_focused_samples = 24;
    production.separator_coordinate_descent_iterations = 32;
    return {
        .strategy = StrategyKind::metaheuristic_search,
        .production_optimizer = ProductionOptimizerKind::brkga,
        .production = production,
    };
  }
  }

  return {};
}

} // namespace shiny::nesting::api