#include <catch2/catch_test_macros.hpp>

#include "request.hpp"
#include "result.hpp"
#include "search/strategy_catalog.hpp"
#include "search/strategy_registry.hpp"
#include "solve.hpp"
#include "util/status.hpp"

namespace {

using shiny::nesting::ALNSConfig;
using shiny::nesting::BinRequest;
using shiny::nesting::ExecutionPolicy;
using shiny::nesting::NestingRequest;
using shiny::nesting::NestingResult;
using shiny::nesting::PieceRequest;
using shiny::nesting::SolveControl;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::primary_strategy_config_ptr;
using shiny::nesting::search::StrategyDescriptor;
using shiny::nesting::search::StrategyRegistry;
using shiny::nesting::set_primary_strategy_config;
using shiny::nesting::util::Status;
using shiny::nesting::util::StatusOr;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return {
      .outer = {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
  };
}

auto trivial_request() -> NestingRequest {
  NestingRequest request;
  // ALNS routes through `StrategyRegistry`, unlike bounding-box and
  // sequential-backtrack which have inline branches in `solve.cpp`.
  request.execution.strategy = StrategyKind::alns;
  request.execution.default_rotations = {{0.0}};
  request.execution.alns.max_refinements = 1;
  request.bins = {{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  }};
  request.pieces = {{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 1.0, 1.0),
  }};
  return request;
}

auto stub_runner_a(const shiny::nesting::NormalizedRequest & /*request*/,
                   const SolveControl & /*control*/) -> StatusOr<NestingResult> {
  return Status::invalid_input;
}

auto stub_runner_b(const shiny::nesting::NormalizedRequest & /*request*/,
                   const SolveControl & /*control*/) -> StatusOr<NestingResult> {
  return Status::invalid_input;
}

// RAII helper that snapshots a built-in descriptor for `kind`, lets the test
// mutate the registry, and restores the original entry on destruction.
class ScopedStrategyOverride {
public:
  explicit ScopedStrategyOverride(StrategyKind kind) : kind_(kind) {
    // Force built-in registrations so the snapshot below is meaningful.
    (void)StrategyRegistry::instance().resolve(ExecutionPolicy{
        .strategy = kind_,
    });
    const auto *existing = StrategyRegistry::instance().find(kind_);
    REQUIRE(existing != nullptr);
    original_ = *existing;
  }

  ScopedStrategyOverride(const ScopedStrategyOverride &) = delete;
  auto operator=(const ScopedStrategyOverride &) -> ScopedStrategyOverride & = delete;
  ScopedStrategyOverride(ScopedStrategyOverride &&) = delete;
  auto operator=(ScopedStrategyOverride &&) -> ScopedStrategyOverride & = delete;

  ~ScopedStrategyOverride() {
    StrategyRegistry::instance().register_strategy(original_);
  }

private:
  StrategyKind kind_;
  StrategyDescriptor original_{};
};

} // namespace

TEST_CASE("strategy registry replaces descriptors keyed by the same kind",
          "[strategy-registry]") {
  ScopedStrategyOverride guard{StrategyKind::alns};

  auto &registry = StrategyRegistry::instance();

  registry.register_strategy(StrategyDescriptor{
      .name = "test/stub-a",
      .kind = StrategyKind::alns,
      .run = &stub_runner_a,
  });
  const auto *first = registry.find(StrategyKind::alns);
  REQUIRE(first != nullptr);
  REQUIRE(first->name == "test/stub-a");
  REQUIRE(first->run == &stub_runner_a);

  registry.register_strategy(StrategyDescriptor{
      .name = "test/stub-b",
      .kind = StrategyKind::alns,
      .run = &stub_runner_b,
  });
  const auto *second = registry.find(StrategyKind::alns);
  REQUIRE(second != nullptr);
  REQUIRE(second->name == "test/stub-b");
  REQUIRE(second->run == &stub_runner_b);
}

TEST_CASE("solve returns invalid_input when the registry has no runner",
          "[strategy-registry][solve]") {
  ScopedStrategyOverride guard{StrategyKind::alns};

  StrategyRegistry::instance().register_strategy(StrategyDescriptor{
      .name = "test/null-run",
      .kind = StrategyKind::alns,
      .run = nullptr,
  });

  const auto result = shiny::nesting::solve(trivial_request());
  REQUIRE_FALSE(result.ok());
  REQUIRE(result.status() == Status::invalid_input);
}

TEST_CASE("typed strategy config variant resolves without compatibility helpers",
          "[strategy-config][request]") {
  ExecutionPolicy execution;
  REQUIRE(primary_strategy_config_ptr<ALNSConfig>(execution, StrategyKind::alns) ==
          nullptr);

  ALNSConfig alns_config;
  alns_config.max_refinements = 31;
  set_primary_strategy_config(execution, StrategyKind::alns, alns_config);

  REQUIRE(primary_strategy_config_ptr<ALNSConfig>(execution, StrategyKind::lahc) ==
          nullptr);
  const auto *alns =
      primary_strategy_config_ptr<ALNSConfig>(execution, StrategyKind::alns);
  REQUIRE(alns != nullptr);
  REQUIRE(alns->max_refinements == 31U);
}
