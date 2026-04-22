// MTG metaheuristic-search integration coverage.
//
// Exercises every ProductionOptimizerKind on the MTG fixture using
// conservative budgets so the per-cell run-time stays bounded.

#include <algorithm>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "runtime/cancellation.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

MtgRequestOptions make_production_options(ProductionOptimizerKind kind) {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = kind;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;

  // Conservative budgets shared across optimizers.
  options.production.population_size = 8;
  options.production.max_generations = 4;

  options.simulated_annealing.max_iterations = 8;
  options.simulated_annealing.restart_count = 1;

  options.alns.max_iterations = 8;
  options.alns.destroy_min_count = 1;
  options.alns.destroy_max_count = 2;

  options.gdrr.max_iterations = 8;

  options.lahc.max_iterations = 8;
  options.lahc.history_length = 4;
  return options;
}

std::size_t bed1_piece_count(const MtgFixture &fixture) {
  return static_cast<std::size_t>(std::count_if(
      fixture.pieces.begin(), fixture.pieces.end(),
      [](const MtgPiece &p) { return p.source_bed_id == kBed1Id; }));
}

} // namespace

TEST_CASE("mtg metaheuristic-search places every part for every optimizer",
          "[mtg][nesting-matrix][metaheuristic-search][.][slow]") {
  const auto kind = GENERATE(ProductionOptimizerKind::brkga,
                             ProductionOptimizerKind::simulated_annealing,
                             ProductionOptimizerKind::alns,
                             ProductionOptimizerKind::gdrr,
                             ProductionOptimizerKind::lahc);

  const auto fixture = load_mtg_fixture();
  auto options = make_production_options(kind);
  options.selected_bin_ids = {kBed1Id};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 31;

  auto solved_a = solve(request, control);
  REQUIRE(solved_a.has_value());
  const auto &result_a = solved_a.value();

  ExpectedOutcome expected{};
  expected.expected_placed_count = bed1_piece_count(fixture);
  validate_layout(fixture, request, options, result_a, expected);

  // Determinism: same seed must produce the same bed1 placement hash.
  auto solved_b = solve(request, control);
  REQUIRE(solved_b.has_value());
  REQUIRE(hash_bin_placements(result_a, kBed1Id) ==
          hash_bin_placements(solved_b.value(), kBed1Id));
}

TEST_CASE("mtg metaheuristic-search all-beds end-to-end",
          "[mtg][nesting-matrix][metaheuristic-search][.][slow]") {
  const auto kind = GENERATE(ProductionOptimizerKind::brkga,
                             ProductionOptimizerKind::simulated_annealing,
                             ProductionOptimizerKind::alns,
                             ProductionOptimizerKind::gdrr,
                             ProductionOptimizerKind::lahc);

  const auto fixture = load_mtg_fixture();
  auto options = make_production_options(kind);
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 31;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg metaheuristic-search cancellation stops the search",
          "[mtg][nesting-matrix][metaheuristic-search][cancellation]") {
  const auto fixture = load_mtg_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  options.production.population_size = 8;
  options.production.elite_count = 2;
  options.production.mutant_count = 2;
  options.production.max_generations = 64;
  options.allow_part_overflow = false;
  options.maintain_bed_assignment = true;
  options.selected_bin_ids = {kBed1Id};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  runtime::CancellationSource cancel_source{};
  std::size_t observed_calls = 0;
  SolveControl control{};
  control.random_seed = 5;
  control.cancellation = cancel_source.token();
  control.on_progress = [&](const ProgressSnapshot & /*snap*/) {
    ++observed_calls;
    cancel_source.request_stop();
  };

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  INFO("stop_reason=" << static_cast<int>(result.stop_reason)
                      << " iterations="
                      << result.budget.iterations_completed
                      << " observed_calls=" << observed_calls);

  REQUIRE(observed_calls >= 1);
  REQUIRE(result.stop_reason == StopReason::cancelled);
  REQUIRE(result.budget.cancellation_requested);
}
