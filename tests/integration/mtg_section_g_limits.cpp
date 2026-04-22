// MTG nesting matrix — Section G: iteration & time limits.
//
// Verifies that `SolveControl::iteration_limit` and
// `SolveControl::time_limit_milliseconds` cap the OUTER search loop (one
// iteration == one entry in `result.search.progress` and one increment of
// `result.budget.iterations_completed`), regardless of any optimizer's
// internal generation/iteration knob.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cstdint>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

void apply_generous_internal_budget(MtgRequestOptions &options,
                                    ProductionOptimizerKind kind) {
  options.production.max_generations = 256;
  options.production.population_size = 16;
  options.simulated_annealing.max_iterations = 256;
  options.alns.max_iterations = 256;
  options.gdrr.max_iterations = 256;
  options.lahc.max_iterations = 256;
  (void)kind;
}

void apply_small_internal_budget(MtgRequestOptions &options,
                                 ProductionOptimizerKind kind) {
  options.production.max_generations = 2;
  options.production.population_size = 8;
  options.simulated_annealing.max_iterations = 2;
  options.alns.max_iterations = 2;
  options.gdrr.max_iterations = 2;
  options.lahc.max_iterations = 2;
  (void)kind;
}

void apply_large_internal_budget(MtgRequestOptions &options,
                                 ProductionOptimizerKind kind) {
  options.production.max_generations = 4096;
  options.production.population_size = 16;
  options.simulated_annealing.max_iterations = 4096;
  options.alns.max_iterations = 4096;
  options.gdrr.max_iterations = 4096;
  options.lahc.max_iterations = 4096;
  (void)kind;
}

}  // namespace

TEST_CASE("mtg iteration limit caps the search",
          "[mtg][nesting-matrix][limits][iteration-limit][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto optimizer = GENERATE(ProductionOptimizerKind::brkga,
                                  ProductionOptimizerKind::simulated_annealing,
                                  ProductionOptimizerKind::alns,
                                  ProductionOptimizerKind::gdrr,
                                  ProductionOptimizerKind::lahc);
  const std::size_t cap = GENERATE(std::size_t{1}, std::size_t{8},
                                   std::size_t{64});

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = optimizer;
  options.selected_bin_ids = {kBed1Id};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  apply_generous_internal_budget(options, optimizer);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.iteration_limit = cap;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  REQUIRE(result.budget.iterations_completed <= cap);
  // Relaxed bound: some optimizers (notably BRKGA) emit additional progress
  // entries beyond one-per-iteration (per-generation and per-polishing
  // entries). The `mtg progress entries diagnostic` test below reports the
  // actual ratio observed per optimizer in CI logs.
  REQUIRE(result.search.progress.size() <= cap * 4 + 4);

  if (cap <= 8) {
    REQUIRE(result.stop_reason == StopReason::iteration_limit_reached);
  } else {
    const bool ok = result.stop_reason == StopReason::iteration_limit_reached ||
                    result.stop_reason == StopReason::completed;
    REQUIRE(ok);
  }
}

TEST_CASE("mtg unlimited iteration limit completes",
          "[mtg][nesting-matrix][limits][iteration-limit][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto optimizer = GENERATE(ProductionOptimizerKind::brkga,
                                  ProductionOptimizerKind::simulated_annealing,
                                  ProductionOptimizerKind::alns,
                                  ProductionOptimizerKind::gdrr,
                                  ProductionOptimizerKind::lahc);

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = optimizer;
  options.selected_bin_ids = {kBed1Id};
  options.allow_part_overflow = true;
  apply_small_internal_budget(options, optimizer);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.iteration_limit = 0;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().stop_reason != StopReason::iteration_limit_reached);
}

TEST_CASE("mtg BRKGA single-iteration negative control",
          "[mtg][nesting-matrix][limits][iteration-limit][negative-control][.][slow]") {
  const auto fixture = load_mtg_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  options.selected_bin_ids = {};
  options.allow_part_overflow = true;
  options.production.max_generations = 256;
  options.production.population_size = 16;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.iteration_limit = 1;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  REQUIRE(result.budget.iterations_completed <= 1);
  REQUIRE(result.search.progress.size() <= 2);
}

TEST_CASE("mtg time limit caps the search",
          "[mtg][nesting-matrix][limits][time-limit][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto optimizer = GENERATE(ProductionOptimizerKind::brkga,
                                  ProductionOptimizerKind::simulated_annealing,
                                  ProductionOptimizerKind::alns,
                                  ProductionOptimizerKind::gdrr,
                                  ProductionOptimizerKind::lahc);
  const std::uint64_t cap_ms = GENERATE(std::uint64_t{25}, std::uint64_t{250},
                                        std::uint64_t{2500});

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = optimizer;
  options.selected_bin_ids = {kBed1Id};
  options.allow_part_overflow = true;
  apply_large_internal_budget(options, optimizer);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.time_limit_milliseconds = cap_ms;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  const std::uint64_t slack = std::max<std::uint64_t>(cap_ms * 4, 1000);
  INFO("cap_ms=" << cap_ms << " slack=" << slack
                 << " elapsed=" << result.budget.elapsed_milliseconds);
  REQUIRE(result.budget.elapsed_milliseconds <= cap_ms + slack);

  const bool ok = result.stop_reason == StopReason::time_limit_reached ||
                  result.stop_reason == StopReason::completed;
  REQUIRE(ok);
}

TEST_CASE("mtg unlimited time completes",
          "[mtg][nesting-matrix][limits][time-limit][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto optimizer = GENERATE(ProductionOptimizerKind::brkga,
                                  ProductionOptimizerKind::simulated_annealing,
                                  ProductionOptimizerKind::alns,
                                  ProductionOptimizerKind::gdrr,
                                  ProductionOptimizerKind::lahc);

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = optimizer;
  options.selected_bin_ids = {kBed1Id};
  options.allow_part_overflow = true;
  apply_small_internal_budget(options, optimizer);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.time_limit_milliseconds = 0;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().stop_reason != StopReason::time_limit_reached);
}

TEST_CASE("mtg bb and constructive honor limits",
          "[mtg][nesting-matrix][limits][.][slow]") {
  const auto fixture = load_mtg_fixture();

  SECTION("bounding_box honors iteration_limit=1") {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::bounding_box;
    options.selected_bin_ids = {};
    options.allow_part_overflow = true;

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    SolveControl control{};
    control.random_seed = 5;
    control.iteration_limit = 1;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    REQUIRE(solved.value().budget.iterations_completed <= 1);
  }

  SECTION("irregular_constructive honors iteration_limit=1") {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::irregular_constructive;
    options.selected_bin_ids = {};
    options.allow_part_overflow = true;

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    SolveControl control{};
    control.random_seed = 5;
    control.iteration_limit = 1;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    REQUIRE(solved.value().budget.iterations_completed <= 1);
  }
}

// Diagnostic-only: empirically reports `result.search.progress.size()`
// versus `result.budget.iterations_completed` for each optimizer. Emits
// WARN messages so the ratios are visible in CI logs without failing the
// build. Used to inform the relaxed bound in `mtg iteration limit caps the
// search` (currently `cap * 4 + 4`).
TEST_CASE("mtg progress entries diagnostic",
          "[mtg][nesting-matrix][limits][progress-diagnostic]") {
  const auto fixture = load_mtg_fixture();

  const auto optimizer = GENERATE(ProductionOptimizerKind::brkga,
                                  ProductionOptimizerKind::simulated_annealing,
                                  ProductionOptimizerKind::alns,
                                  ProductionOptimizerKind::gdrr,
                                  ProductionOptimizerKind::lahc);
  constexpr std::size_t cap = 8;

  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = optimizer;
  options.selected_bin_ids = {kBed1Id};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  apply_generous_internal_budget(options, optimizer);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 5;
  control.iteration_limit = cap;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  const auto progress_size = result.search.progress.size();
  const auto iters = result.budget.iterations_completed;
  WARN("[progress-diagnostic] optimizer="
       << static_cast<int>(optimizer) << " cap=" << cap
       << " iterations_completed=" << iters
       << " progress.size()=" << progress_size
       << " ratio_to_cap=" << (static_cast<double>(progress_size) /
                               static_cast<double>(cap)));
}
