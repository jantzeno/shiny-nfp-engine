#pragma once

// Strategy dispatch table for `solve()`.
//
// `StrategyRegistry` is a Meyers singleton owning two parallel tables:
//   * `StrategyDescriptor`           — top-level user-selectable strategies
//                                      keyed by `StrategyKind` (bounding-box,
//                                      irregular-constructive, the four
//                                      metaheuristics, and `irregular_production`
//                                      which delegates to a production optimizer).
//   * `ProductionStrategyDescriptor` — production-side optimizers keyed by
//                                      `ProductionOptimizerKind`, only consulted
//                                      when the resolved strategy is
//                                      `irregular_production`.
//
// Thread-safety: the singleton instance and its internal vectors are NOT
// guarded by a mutex. All registrations are expected to occur during
// process-startup serialization through
// `ensure_builtin_strategy_registrations()` (called from `resolve()`), before
// any concurrent solver invocation. After that point the registry is treated
// as read-only and is safe to query from multiple threads. Callers that
// register custom strategies at runtime are responsible for external
// synchronization.
//
// Registration ordering: built-in registrations are wired exactly once via
// the C++11 static-local guard inside
// `ensure_builtin_strategy_registrations()`. Re-registering a descriptor for
// an existing kind replaces the previous entry in place (last writer wins),
// which is the supported way for tests or downstream code to override a
// built-in runner.

#include <optional>
#include <string_view>
#include <vector>

#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::search {

using StrategyRunner =
    util::StatusOr<NestingResult> (*)(const NormalizedRequest &request,
                                      const SolveControl &control);

struct StrategyDescriptor {
  std::string_view name{};
  StrategyKind kind{StrategyKind::bounding_box};
  StrategyRunner run{nullptr};
};

struct ProductionStrategyDescriptor {
  std::string_view name{};
  ProductionOptimizerKind kind{ProductionOptimizerKind::brkga};
  StrategyRunner run{nullptr};
};

struct ResolvedStrategy {
  std::string_view name{};
  StrategyRunner run{nullptr};
  std::optional<StrategyKind> result_strategy_override{};
};

class StrategyRegistry {
public:
  static auto instance() -> StrategyRegistry &;

  void register_strategy(StrategyDescriptor descriptor);
  void register_production_strategy(ProductionStrategyDescriptor descriptor);

  [[nodiscard]] auto find(StrategyKind kind) const -> const StrategyDescriptor *;
  [[nodiscard]] auto find(std::string_view name) const -> const StrategyDescriptor *;
  [[nodiscard]] auto find(ProductionOptimizerKind kind) const
      -> const ProductionStrategyDescriptor *;

  [[nodiscard]] auto resolve(const ExecutionPolicy &execution) const
      -> ResolvedStrategy;

private:
  std::vector<StrategyDescriptor> strategies_{};
  std::vector<ProductionStrategyDescriptor> production_strategies_{};
};

} // namespace shiny::nesting::search
