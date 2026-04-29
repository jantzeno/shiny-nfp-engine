#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "packing/irregular/sequential/packer.hpp"
#include "request.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/disruption.hpp"
#include "search/solution_pool.hpp"
#include "search/strip_optimizer.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::SolveControl;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::SequentialBacktrackPacker;
using shiny::nesting::runtime::DeterministicRng;
using shiny::nesting::search::SolutionPool;
using shiny::nesting::search::SolutionPoolEntry;

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

auto strip_case_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.enable_direct_overlap_check = true;
  request.execution.production.population_size = 8;
  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 4;
  request.execution.production.polishing_passes = 1;
  request.execution.production.diversification_swaps = 1;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(0.0, 0.0, 4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 11,
      .polygon = rectangle(0.0, 0.0, 4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 12,
      .polygon = rectangle(0.0, 0.0, 6.0, 4.0),
  });
  return request;
}

auto make_entry(std::vector<std::size_t> order, std::size_t placed_parts,
                double strip_length, double utilization) -> SolutionPoolEntry {
  return {
      .order = std::move(order),
      .metrics =
          {
              .placed_parts = placed_parts,
              .bin_count = 1,
              .strip_length = strip_length,
              .utilization = utilization,
          },
  };
}

} // namespace

TEST_CASE("solution pool keeps the strongest recent candidates", "[search][pool]") {
  SolutionPool pool(3);
  pool.insert(make_entry({0, 1, 2}, 2, 9.0, 0.55));
  pool.insert(make_entry({2, 0, 1}, 3, 8.0, 0.80));
  pool.insert(make_entry({1, 0, 2}, 3, 8.5, 0.70));
  pool.insert(make_entry({1, 2, 0}, 1, 10.0, 0.20));

  REQUIRE(pool.size() == 3U);
  REQUIRE(pool.best() != nullptr);
  REQUIRE(pool.best()->order == std::vector<std::size_t>{2U, 0U, 1U});

  DeterministicRng rng(11);
  std::size_t best_hits = 0;
  std::size_t tail_hits = 0;
  for (std::size_t sample = 0; sample < 64U; ++sample) {
    const auto *selected = pool.select(rng);
    REQUIRE(selected != nullptr);
    if (selected->order == std::vector<std::size_t>{2U, 0U, 1U}) {
      ++best_hits;
    }
    if (selected->order == std::vector<std::size_t>{1U, 0U, 2U}) {
      ++tail_hits;
    }
  }

  REQUIRE(best_hits > tail_hits);
}

TEST_CASE("large-item disruption swaps a large dissimilar piece forward",
          "[search][disruption]") {
  auto normalized = shiny::nesting::normalize_request(strip_case_request());
  REQUIRE(normalized.ok());

  const std::array<double, 3> areas{8.0, 8.0, 24.0};
  const std::array<std::size_t, 3> order{0U, 1U, 2U};
  DeterministicRng rng(7);
  const auto disrupted = shiny::nesting::search::disrupt_large_items(
      order, normalized.value(), areas, rng);

  REQUIRE(disrupted.applied);
  REQUIRE(disrupted.order == std::vector<std::size_t>{2U, 1U, 0U});
}

TEST_CASE("large-item disruption keeps two candidates on two-piece fixtures",
          "[search][disruption]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 8.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 20,
      .polygon = rectangle(0.0, 0.0, 4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 21,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  const std::array<double, 2> areas{8.0, 4.0};
  const std::array<std::size_t, 2> order{0U, 1U};
  DeterministicRng rng(5);
  const auto disrupted = shiny::nesting::search::disrupt_large_items(
      order, normalized.value(), areas, rng);

  REQUIRE(disrupted.applied);
  REQUIRE(disrupted.order == std::vector<std::size_t>{1U, 0U});
}

TEST_CASE("strip optimizer helper schedule hits phase boundaries",
          "[search][strip-optimizer]") {
  using shiny::nesting::search::detail::schedule_ratio;

  REQUIRE(schedule_ratio(0U, 5U, 0.25, 0.02) == Catch::Approx(0.25));
  REQUIRE(schedule_ratio(4U, 5U, 0.25, 0.02) == Catch::Approx(0.02));
  REQUIRE(schedule_ratio(0U, 4U, 0.01, 0.001) == Catch::Approx(0.01));
  REQUIRE(schedule_ratio(3U, 4U, 0.01, 0.001) == Catch::Approx(0.001));
}

TEST_CASE("strip optimizer iteration budget scales with piece count",
          "[search][strip-optimizer]") {
  shiny::nesting::ProductionSearchConfig config;
  config.polishing_passes = 2U;
  config.diversification_swaps = 3U;

  REQUIRE(
      shiny::nesting::search::detail::derive_iteration_budget(config, 200U) ==
      219U);
}

TEST_CASE("strip optimizer improves a constructive seed on a strip-style case",
          "[search][strip-optimizer]") {
  auto normalized = shiny::nesting::normalize_request(strip_case_request());
  REQUIRE(normalized.ok());

  SequentialBacktrackPacker packer;
  const auto seed = packer.solve(normalized.value(), SolveControl{.random_seed = 3});
  REQUIRE(seed.ok());
  REQUIRE(seed.value().layout.placement_trace.size() == 2U);

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget budget(0);
  shiny::nesting::search::StripOptimizer optimizer;
  const auto optimized = optimizer.optimize(
      normalized.value(), SolveControl{.random_seed = 7}, budget, stopwatch,
      SolutionPoolEntry{
          .order = {0U, 1U, 2U},
          .metrics = shiny::nesting::search::metrics_for_layout(seed.value().layout),
          .result = seed.value(),
      },
      normalized.value().request.execution.production);

  REQUIRE(optimized.accepted_moves > 0U);
  REQUIRE(optimized.best_solution.metrics.placed_parts == 3U);
}

TEST_CASE("strip optimizer reports configurable phase and separator replay",
          "[search][strip-optimizer]") {
  auto request = strip_case_request();
  request.execution.production.max_iterations = 6;
  request.execution.production.polishing_passes = 2;
  request.execution.production.strip_exploration_ratio = 0.5;
  request.execution.production.separator_worker_count = 2;
  request.execution.production.separator_max_iterations = 24;
  request.execution.production.separator_iter_no_improvement_limit = 5;
  request.execution.production.separator_strike_limit = 2;
  request.execution.production.separator_global_samples = 20;
  request.execution.production.separator_focused_samples = 10;
  request.execution.production.separator_coordinate_descent_iterations = 12;
  request.execution.production.infeasible_pool_capacity = 2;
  request.execution.production.infeasible_rollback_after = 1;

  auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  SequentialBacktrackPacker packer;
  const auto seed = packer.solve(normalized.value(), SolveControl{.random_seed = 3});
  REQUIRE(seed.ok());

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget budget(0);
  shiny::nesting::search::StripOptimizer optimizer;
  const auto optimized = optimizer.optimize(
      normalized.value(), SolveControl{.random_seed = 7}, budget, stopwatch,
      SolutionPoolEntry{
          .order = {0U, 1U, 2U},
          .metrics = shiny::nesting::search::metrics_for_layout(seed.value().layout),
          .result = seed.value(),
      },
      request.execution.production);

  REQUIRE(optimized.phase_metrics.exploration_iteration_budget > 0U);
  REQUIRE(optimized.phase_metrics.compression_iteration_budget > 0U);
  REQUIRE(optimized.phase_metrics.exploration_iterations ==
          optimized.exploration_iterations);
  REQUIRE(optimized.phase_metrics.compression_iterations ==
          optimized.compression_iterations);
  REQUIRE(optimized.phase_metrics.accepted_moves == optimized.accepted_moves);
  REQUIRE(optimized.separator_metrics.worker_count == 2U);
  REQUIRE(optimized.separator_metrics.max_iterations == 24U);
  REQUIRE(optimized.separator_metrics.iter_no_improvement_limit == 5U);
  REQUIRE(optimized.separator_metrics.strike_limit == 2U);
  REQUIRE(optimized.separator_metrics.global_samples == 20U);
  REQUIRE(optimized.separator_metrics.focused_samples == 10U);
  REQUIRE(optimized.separator_metrics.coordinate_descent_iterations == 12U);
}

TEST_CASE("strip optimizer is deterministic for one worker and bounded for multiple",
          "[search][strip-optimizer][deterministic][workers]") {
  auto request = strip_case_request();
  request.execution.production.max_iterations = 5;
  request.execution.production.separator_worker_count = 1;
  auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  SequentialBacktrackPacker packer;
  const auto seed = packer.solve(normalized.value(), SolveControl{.random_seed = 3});
  REQUIRE(seed.ok());
  const auto seed_entry = SolutionPoolEntry{
      .order = {0U, 1U, 2U},
      .metrics = shiny::nesting::search::metrics_for_layout(seed.value().layout),
      .result = seed.value(),
  };

  shiny::nesting::search::StripOptimizer optimizer;
  shiny::nesting::runtime::Stopwatch first_stopwatch;
  const shiny::nesting::runtime::TimeBudget budget(0);
  const auto first = optimizer.optimize(
      normalized.value(), SolveControl{.random_seed = 13}, budget,
      first_stopwatch, seed_entry, request.execution.production);

  shiny::nesting::runtime::Stopwatch second_stopwatch;
  const auto second = optimizer.optimize(
      normalized.value(), SolveControl{.random_seed = 13}, budget,
      second_stopwatch, seed_entry, request.execution.production);

  REQUIRE(first.best_solution.metrics.placed_parts ==
          second.best_solution.metrics.placed_parts);
  REQUIRE(first.best_solution.metrics.bin_count ==
          second.best_solution.metrics.bin_count);
  REQUIRE(first.best_solution.metrics.strip_length ==
          Catch::Approx(second.best_solution.metrics.strip_length));

  request.execution.production.separator_worker_count = 2;
  normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  shiny::nesting::runtime::Stopwatch multi_stopwatch;
  const auto multi = optimizer.optimize(
      normalized.value(), SolveControl{.random_seed = 13}, budget,
      multi_stopwatch, seed_entry, request.execution.production);
  REQUIRE(multi.separator_metrics.worker_count == 2U);
  REQUIRE(multi.best_solution.metrics.placed_parts >=
          first.best_solution.metrics.placed_parts);
}

TEST_CASE("irregular production preserves or improves the constructive strip seed",
          "[solve][production][strip]") {
  auto constructive_request = strip_case_request();
  constructive_request.execution.strategy = StrategyKind::sequential_backtrack;
  const auto constructive = shiny::nesting::solve(constructive_request);

  auto production_request = strip_case_request();
  production_request.execution.strategy = StrategyKind::metaheuristic_search;
  const auto production =
      shiny::nesting::solve(production_request, SolveControl{.random_seed = 7});

  REQUIRE(constructive.ok());
  REQUIRE(production.ok());
  REQUIRE(production.value().layout.placement_trace.size() >=
          constructive.value().layout.placement_trace.size());
  REQUIRE(production.value().layout.placement_trace.size() == 3U);
}
