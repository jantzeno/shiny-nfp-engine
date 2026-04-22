#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <vector>

#include "runtime/cancellation.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProgressSnapshot;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;

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

auto frame(double min_x, double min_y, double max_x, double max_y, double hole_min_x,
           double hole_min_y, double hole_max_x, double hole_max_y)
    -> PolygonWithHoles {
  return {
      .outer = {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
      .holes = {{
          {hole_min_x, hole_min_y},
          {hole_min_x, hole_max_y},
          {hole_max_x, hole_max_y},
          {hole_max_x, hole_min_y},
      }},
  };
}

auto improvement_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.enable_part_in_part_placement = true;
  request.execution.production.population_size = 12;
  request.execution.production.elite_count = 3;
  request.execution.production.mutant_count = 2;
  request.execution.production.max_generations = 6;
  request.execution.production.polishing_passes = 1;
  request.execution.production.diversification_swaps = 1;

  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  return request;
}

} // namespace

TEST_CASE("production search improves the constructive seed and records replayable progress",
          "[solve][production][brkga]") {
  auto constructive_request = improvement_request();
  constructive_request.execution.strategy = StrategyKind::sequential_backtrack;
  const auto constructive = shiny::nesting::solve(constructive_request);

  auto production_request = improvement_request();
  production_request.execution.strategy = StrategyKind::metaheuristic_search;

  std::vector<ProgressSnapshot> snapshots;
  const auto production = shiny::nesting::solve(
      production_request, SolveControl{
                              .on_progress = [&](const ProgressSnapshot &snapshot) {
                                snapshots.push_back(snapshot);
                              },
                              .iteration_limit = 4,
                              .random_seed = 17,
                          });

  REQUIRE(constructive.ok());
  REQUIRE(production.ok());
  REQUIRE(constructive.value().layout.placement_trace.size() == 1U);
  REQUIRE(production.value().layout.placement_trace.size() == 2U);
  REQUIRE(production.value().layout.unplaced_piece_ids.empty());
  REQUIRE(production.value().search.optimizer == OptimizerKind::brkga);
  REQUIRE(production.value().search.sparrow_polished);
  REQUIRE_FALSE(production.value().search.progress.empty());
  REQUIRE(production.value().search.progress.front().iteration == 1U);
  REQUIRE(production.value().search.progress.back().layout.placement_trace.size() ==
          production.value().layout.placement_trace.size());
  REQUIRE_FALSE(snapshots.empty());
}

TEST_CASE("production search respects iteration limits and cancellation",
          "[solve][production][budget]") {
  auto request = improvement_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;

  const auto limited =
      shiny::nesting::solve(request, SolveControl{.iteration_limit = 1, .random_seed = 3});
  REQUIRE(limited.ok());
  REQUIRE(limited.value().stop_reason == StopReason::iteration_limit_reached);
  REQUIRE(limited.value().budget.iterations_completed == 1U);

  shiny::nesting::runtime::CancellationSource source;
  source.request_stop();
  const auto cancelled =
      shiny::nesting::solve(request, SolveControl{.cancellation = source.token()});
  REQUIRE(cancelled.ok());
  REQUIRE(cancelled.value().stop_reason == StopReason::cancelled);
}

TEST_CASE("production search obeys time budgets under search",
          "[solve][production][time]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.default_rotations = {{0.0}};
  request.execution.production.population_size = 48;
  request.execution.production.elite_count = 8;
  request.execution.production.mutant_count = 8;
  request.execution.production.max_generations = 64;

  request.bins.push_back(BinRequest{
      .bin_id = 70,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  for (std::size_t index = 0; index < 18U; ++index) {
    request.pieces.push_back(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(200U + index),
        .polygon = rectangle(0.0, 0.0, 4.0 + (index % 3U), 2.0 + (index % 2U)),
    });
  }

  const auto result =
      shiny::nesting::solve(request, SolveControl{.time_limit_milliseconds = 1});
  REQUIRE(result.ok());
  REQUIRE(result.value().stop_reason == StopReason::time_limit_reached);
}

TEST_CASE("production search keeps strict small-population BRKGA validation",
          "[solve][production][config]") {
  auto request = improvement_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production.population_size = 4;
  request.execution.production.elite_count = 1;
  request.execution.production.mutant_count = 1;
  REQUIRE(request.execution.production.is_valid());
  REQUIRE(request.is_valid());

  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 2;
  REQUIRE_FALSE(request.execution.production.is_valid());
  REQUIRE_FALSE(request.is_valid());
  REQUIRE_FALSE(shiny::nesting::solve(request).ok());
}
