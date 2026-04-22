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
using shiny::nesting::StrategyConfig;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::search::StrategyDescriptor;
using shiny::nesting::search::StrategyRegistry;
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
  request.execution.alns.max_iterations = 1;
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

TEST_CASE("StrategyConfig::get_if is a no-throw dispatch path",
          "[strategy-config][request]") {
  // Default-constructed: kind=bounding_box, payload empty. A request for the
  // matching kind must still return nullptr (empty `std::any`) and must not
  // throw `std::bad_any_cast`.
  StrategyConfig empty{};
  REQUIRE(empty.kind == StrategyKind::bounding_box);
  REQUIRE_FALSE(empty.has_value());
  REQUIRE_NOTHROW(empty.get_if<ALNSConfig>(StrategyKind::bounding_box));
  REQUIRE(empty.get_if<ALNSConfig>(StrategyKind::bounding_box) == nullptr);

  // Kind mismatch: payload carries an ALNSConfig but caller asks under a
  // different kind — must return nullptr without throwing.
  auto populated = StrategyConfig::make(StrategyKind::alns, ALNSConfig{});
  REQUIRE(populated.has_value());
  REQUIRE_NOTHROW(populated.get_if<ALNSConfig>(StrategyKind::lahc));
  REQUIRE(populated.get_if<ALNSConfig>(StrategyKind::lahc) == nullptr);

  // Type mismatch under the right kind: `std::any` holds ALNSConfig but the
  // caller asks for `int` — `any_cast` returns nullptr (no throw).
  REQUIRE_NOTHROW(populated.get_if<int>(StrategyKind::alns));
  REQUIRE(populated.get_if<int>(StrategyKind::alns) == nullptr);

  // Sanity: matching kind + matching type yields the payload.
  const auto *alns = populated.get_if<ALNSConfig>(StrategyKind::alns);
  REQUIRE(alns != nullptr);
}
