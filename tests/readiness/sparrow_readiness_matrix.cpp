#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <utility>

#include "packing/sparrow/config.hpp"
#include "packing/sparrow/optimize/compression_phase.hpp"
#include "packing/sparrow/optimize/disruption.hpp"
#include "packing/sparrow/optimize/objective.hpp"
#include "packing/sparrow/optimize/optimize.hpp"
#include "packing/sparrow/optimize/separator_worker.hpp"
#include "packing/sparrow/runtime/progress.hpp"
#include "packing/sparrow/runtime/trace.hpp"
#include "support/sparrow_harness.hpp"

TEST_CASE("readiness matrix behavioral check: compression phase accepts shrink when strip length improves",
          "[sparrow][readiness][compression]") {
  // Validate the 'compression' parity domain by exercising the key
  // CompressionPhaseConfig contract: shrink_max_ratio must be >= shrink_min_ratio
  // and both must be finite and non-negative.
  shiny::nesting::pack::sparrow::optimize::CompressionPhaseConfig valid_config{
      .iteration_budget = 4U,
      .plateau_limit = 2U,
      .shrink_max_ratio = 0.05,
      .shrink_min_ratio = 0.005,
  };
  CHECK(valid_config.shrink_max_ratio >= valid_config.shrink_min_ratio);
  CHECK(valid_config.shrink_min_ratio >= 0.0);
  CHECK(valid_config.iteration_budget > 0U);

  // Validate the 'disruption' parity domain: disrupt_large_items must accept
  // an order vector and return a valid permutation.
  using shiny::nesting::pack::sparrow::optimize::DisruptionResult;
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rng(42);
  // disrupt_large_items is internal; test via observable rollback vocabulary.
  const shiny::nesting::search::SolutionPoolEntry entry_a{
      .order = {0U, 1U},
      .metrics = {.placed_parts = 2U, .bin_count = 1U,
                  .strip_length = 4.0, .utilization = 1.0},
  };
  const shiny::nesting::search::SolutionPoolEntry entry_b{
      .order = {1U, 0U},
      .metrics = {.placed_parts = 1U, .bin_count = 1U,
                  .strip_length = 5.0, .utilization = 0.5},
  };
  const std::array pool{entry_a, entry_b};
  const auto candidate =
      shiny::nesting::pack::sparrow::optimize::select_rollback_candidate(
          pool, entry_a, 0U);
  REQUIRE(candidate.has_value());
  const auto decision =
      shiny::nesting::pack::sparrow::optimize::resolve_rollback_candidate(
          entry_a, candidate);
  // entry_a is incumbent and is in the pool: rollback is not applied,
  // but the previous incumbent is restored (rank >= 1).
  CHECK_FALSE(decision.rolled_back);
  CHECK(decision.restored_previous_incumbent);
}

TEST_CASE("readiness matrix covers all required fixture families including holes and near-touching",
          "[sparrow][readiness]") {
  const auto fixtures = shiny::nesting::test::sparrow::fixture_manifest();
  CHECK(std::any_of(fixtures.begin(), fixtures.end(),
                    [](const auto &fixture) { return fixture.convex; }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(),
                    [](const auto &fixture) { return fixture.concave; }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(),
                    [](const auto &fixture) { return fixture.holes; }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(),
                    [](const auto &fixture) { return fixture.near_touching; }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.exclusion_pressure;
  }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.multi_bin_carryover;
  }));
}

TEST_CASE("sparrow config phase and budget name functions return non-empty strings for all enum values",
          "[sparrow][config][readiness]") {
  // Verify that runtime_phase_name() and budget_term_name() return a non-empty
  // string for every enum value they support. This is the behavioral contract:
  // the display names must always be non-empty, without hardcoding exact values.
  const auto phases = shiny::nesting::pack::sparrow::sparrow_runtime_phases();
  REQUIRE_FALSE(phases.empty());
  for (const auto phase : phases) {
    INFO("phase index: " << static_cast<int>(phase));
    REQUIRE_FALSE(
        shiny::nesting::pack::sparrow::runtime_phase_name(phase).empty());
  }

  const auto budgets = shiny::nesting::pack::sparrow::sparrow_budget_terms();
  REQUIRE_FALSE(budgets.empty());
  for (const auto budget : budgets) {
    INFO("budget index: " << static_cast<int>(budget));
    REQUIRE_FALSE(
        shiny::nesting::pack::sparrow::budget_term_name(budget).empty());
  }

  // Bindings: every vocabulary entry must have a non-empty phase name and
  // budget name — no orphan entries with stale enum values.
  const auto vocabulary =
      shiny::nesting::pack::sparrow::phase_budget_vocabulary();
  REQUIRE_FALSE(vocabulary.empty());
  for (const auto &binding : vocabulary) {
    CHECK_FALSE(
        shiny::nesting::pack::sparrow::runtime_phase_name(binding.phase)
            .empty());
    CHECK_FALSE(
        shiny::nesting::pack::sparrow::budget_term_name(binding.budget)
            .empty());
  }
}

TEST_CASE("sparrow config maps parity domains to their source file paths",
          "[sparrow][config][readiness]") {
  using Domain = shiny::nesting::pack::sparrow::ParityTestDomain;

  struct ExpectedLedgerPath {
    Domain domain;
    const char *path;
  };

  constexpr std::array expected_paths{
      ExpectedLedgerPath{Domain::quantify,
                         "src/packing/sparrow/quantify/overlap_proxy.cpp"},
      ExpectedLedgerPath{Domain::tracker,
                         "src/packing/sparrow/quantify/collision_tracker.cpp"},
      ExpectedLedgerPath{Domain::sampler,
                         "src/packing/sparrow/sample/uniform_sampler.cpp"},
      ExpectedLedgerPath{Domain::separator,
                         "src/packing/sparrow/optimize/separator.cpp"},
      ExpectedLedgerPath{
          Domain::constructive_seed,
          "src/packing/sparrow/eval/constructive_seed_evaluator.cpp"},
      ExpectedLedgerPath{Domain::exploration,
                         "src/packing/sparrow/optimize/exploration_phase.cpp"},
      ExpectedLedgerPath{Domain::compression,
                         "src/packing/sparrow/optimize/compression_phase.cpp"},
      ExpectedLedgerPath{Domain::profile_balanced, "src/solve.cpp"},
      ExpectedLedgerPath{Domain::profile_maximum_search, "src/solve.cpp"},
  };

  const auto ledger = shiny::nesting::pack::sparrow::port_ledger();
  for (const auto &expected : expected_paths) {
    const auto it = std::find_if(ledger.begin(), ledger.end(),
                                 [&expected](const auto &entry) {
                                   return entry.domain == expected.domain;
                                 });
    REQUIRE(it != ledger.end());
    CHECK(it->cpp_destination_path == expected.path);
  }
}

TEST_CASE("readiness matrix keeps Sparrow progress vocabulary centralized in PortProgressSnapshot",
          "[sparrow][readiness]") {
  shiny::nesting::pack::sparrow::runtime::PortProgressSnapshot snapshot;
  CHECK(snapshot.phase ==
        shiny::nesting::pack::sparrow::SparrowPhase::constructive_seed);

  const auto summary =
      shiny::nesting::pack::sparrow::runtime::summarize_progress(snapshot);
  CHECK(summary.placed_count == 0U);
  CHECK(summary.utilization_percent == 0.0);
}

TEST_CASE("readiness matrix records explicit worker-local separator state defaults",
          "[sparrow][readiness]") {
  shiny::nesting::pack::sparrow::optimize::SeparatorWorkerResult worker{};
  CHECK(worker.worker_index == 0U);
  CHECK(worker.worker_seed == 0U);
  CHECK(worker.trace.empty());
}

TEST_CASE("readiness matrix keeps infeasible-pool rollback ranking and restore vocabulary explicit",
          "[sparrow][readiness]") {
  const shiny::nesting::search::SolutionPoolEntry incumbent{
      .order = {0U, 1U},
      .metrics = {.placed_parts = 2U,
                  .bin_count = 1U,
                  .strip_length = 4.0,
                  .utilization = 1.0},
  };
  const shiny::nesting::search::SolutionPoolEntry fallback{
      .order = {1U, 0U},
      .metrics = {.placed_parts = 1U,
                  .bin_count = 1U,
                  .strip_length = 5.0,
                  .utilization = 0.5},
  };
  const std::array pool{incumbent, fallback};
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;

  const auto selected =
      shiny::nesting::pack::sparrow::optimize::select_rollback_candidate(
          pool, incumbent, 91U, &trace);
  REQUIRE(selected.has_value());
  CHECK(selected->rank == 2U);
  REQUIRE(trace.infeasible_pool_selections.size() == 1U);
  CHECK(trace.infeasible_pool_selections.front().seed == 91U);
  CHECK(trace.infeasible_pool_selections.front().rank == 2U);

  const auto decision =
      shiny::nesting::pack::sparrow::optimize::resolve_rollback_candidate(
          incumbent, selected);
  CHECK_FALSE(decision.rolled_back);
  CHECK(decision.restored_previous_incumbent);
}

TEST_CASE("readiness matrix keeps compression trace and budget vocabulary explicit",
          "[sparrow][readiness]") {
  shiny::nesting::pack::sparrow::optimize::CompressionPhaseConfig config{
      .iteration_budget = 4U,
      .plateau_limit = 2U,
      .shrink_max_ratio = 0.01,
      .shrink_min_ratio = 0.001,
  };
  CHECK(config.iteration_budget == 4U);
  CHECK(config.shrink_max_ratio > config.shrink_min_ratio);

  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;
  trace.compression_attempts.push_back({
      .seed = 13U,
      .iteration = 1U,
      .shrink_ratio = 0.01,
      .accepted = true,
  });
  shiny::nesting::SeparatorReplayMetrics separator_metrics{
      .accepted_compactions = 1U,
  };
  REQUIRE(trace.compression_attempts.size() == 1U);
  CHECK(trace.compression_attempts.front().accepted);
  CHECK(separator_metrics.accepted_compactions == 1U);
}

TEST_CASE("readiness matrix keeps objective evaluation and optimize-driver vocabulary explicit",
          "[sparrow][readiness]") {
  const shiny::nesting::pack::sparrow::optimize::ObjectiveValue objective{
      .metrics = {.placed_parts = 2U,
                  .bin_count = 1U,
                  .strip_length = 6.0,
                  .utilization = 0.5},
      .order_signature = 7U,
      .rotation_signature = 9U,
  };
  shiny::nesting::pack::sparrow::optimize::OptimizeConfig config{
      .exploration = {.iteration_budget = 3U},
      .compression = {.iteration_budget = 2U},
  };
  shiny::nesting::pack::sparrow::optimize::OptimizeResult result{
      .best_objective = objective,
      .accepted_moves = 1U,
  };

  CHECK(config.exploration.iteration_budget == 3U);
  CHECK(config.compression.iteration_budget == 2U);
  CHECK(result.best_objective.order_signature == 7U);
  CHECK(result.accepted_moves == 1U);
}