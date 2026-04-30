#include "search/strategy_registry.hpp"

#include <algorithm>

#include "search/strategy_catalog.hpp"

// The free `register_*_strategy` functions declared in
// `search/strategy_catalog.hpp` and defined alongside each driver
// (`bounding_box`, `sequential_backtrack`, `metaheuristic_search`,
// `simulated_annealing`, `alns`, `gdrr`, `lahc`) are wired into the
// singleton exclusively by `ensure_builtin_strategy_registrations()` (invoked
// from `StrategyRegistry::resolve`). For static-link builds — where there is
// no shared-library load-time side effect to populate the table — this
// indirection through `ensure_builtin_strategy_registrations()` is the only
// entry point that guarantees every built-in driver is present before
// dispatch.
namespace shiny::nesting::search {

auto StrategyRegistry::instance() -> StrategyRegistry & {
  static StrategyRegistry registry;
  return registry;
}

void StrategyRegistry::register_strategy(StrategyDescriptor descriptor) {
  const auto existing = std::find_if(strategies_.begin(), strategies_.end(),
                                     [&](const StrategyDescriptor &entry) {
                                       return entry.kind == descriptor.kind;
                                     });
  if (existing != strategies_.end()) {
    *existing = descriptor;
    return;
  }
  strategies_.push_back(descriptor);
}

void StrategyRegistry::register_production_strategy(
    ProductionStrategyDescriptor descriptor) {
  const auto existing =
      std::find_if(production_strategies_.begin(), production_strategies_.end(),
                   [&](const ProductionStrategyDescriptor &entry) {
                     return entry.kind == descriptor.kind;
                   });
  if (existing != production_strategies_.end()) {
    *existing = descriptor;
    return;
  }
  production_strategies_.push_back(descriptor);
}

auto StrategyRegistry::find(const StrategyKind kind) const
    -> const StrategyDescriptor * {
  const auto existing = std::find_if(
      strategies_.begin(), strategies_.end(),
      [&](const StrategyDescriptor &entry) { return entry.kind == kind; });
  return existing == strategies_.end() ? nullptr : &*existing;
}

auto StrategyRegistry::find(const std::string_view name) const
    -> const StrategyDescriptor * {
  const auto existing = std::find_if(
      strategies_.begin(), strategies_.end(),
      [&](const StrategyDescriptor &entry) { return entry.name == name; });
  return existing == strategies_.end() ? nullptr : &*existing;
}

auto StrategyRegistry::find(const ProductionOptimizerKind kind) const
    -> const ProductionStrategyDescriptor * {
  const auto existing =
      std::find_if(production_strategies_.begin(), production_strategies_.end(),
                   [&](const ProductionStrategyDescriptor &entry) {
                     return entry.kind == kind;
                   });
  return existing == production_strategies_.end() ? nullptr : &*existing;
}

auto StrategyRegistry::resolve(const ExecutionPolicy &execution) const
    -> ResolvedStrategy {
  ensure_builtin_strategy_registrations();

  if (execution.strategy == StrategyKind::metaheuristic_search) {
    const auto *production = find(execution.production_optimizer);
    if (production == nullptr) {
      return {};
    }
    return {
        .name = production->name,
        .run = production->run,
        .result_strategy_override = StrategyKind::metaheuristic_search,
    };
  }

  const auto *strategy = find(execution.strategy);
  if (strategy == nullptr) {
    return {};
  }
  return {
      .name = strategy->name,
      .run = strategy->run,
      .result_strategy_override = std::nullopt,
  };
}

} // namespace shiny::nesting::search
