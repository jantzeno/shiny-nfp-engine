#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <vector>

#include "request.hpp"
#include "runtime/deterministic_rng.hpp"
#include "search/detail/cooling_schedule.hpp"
#include "search/detail/lahc.hpp"
#include "search/detail/neighborhood_ops.hpp"
#include "search/strategy_registry.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::ALNSConfig;
using shiny::nesting::BinRequest;
using shiny::nesting::CoolingScheduleKind;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::SAConfig;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::runtime::DeterministicRng;
using shiny::nesting::search::detail::CoolingSchedule;
using shiny::nesting::search::detail::LateAcceptanceHistory;
using shiny::nesting::search::detail::NeighborhoodOperator;
using shiny::nesting::search::detail::OrderEvaluator;

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
  request.execution.production.max_iterations = 6;
  request.execution.production.polishing_passes = 1;
  request.execution.production.diversification_swaps = 1;
  request.execution.simulated_annealing.max_refinements = 10;
  request.execution.simulated_annealing.restart_count = 2;
  request.execution.alns.max_refinements = 12;
  request.execution.alns.destroy_min_count = 1;
  request.execution.alns.destroy_max_count = 2;
  request.execution.gdrr.max_refinements = 12;
  request.execution.lahc.max_refinements = 12;
  request.execution.lahc.history_length = 4;

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

auto neighborhood_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::simulated_annealing;
  request.execution.default_rotations = {{0.0, 90.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 30.0, 10.0),
  });
  for (std::size_t index = 0; index < 4U; ++index) {
    request.pieces.push_back(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(10U + index),
        .polygon = rectangle(0.0, 0.0, 2.0 + index, 2.0),
    });
  }
  return request;
}

auto strip_benchmark_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.enable_direct_overlap_check = true;
  request.execution.simulated_annealing.max_refinements = 12;
  request.execution.simulated_annealing.restart_count = 2;
  request.execution.alns.max_refinements = 12;
  request.execution.alns.destroy_min_count = 1;
  request.execution.alns.destroy_max_count = 2;
  request.execution.gdrr.max_refinements = 12;
  request.execution.lahc.max_refinements = 12;
  request.execution.lahc.history_length = 4;

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

} // namespace

TEST_CASE("cooling schedules cool and clamp to the configured floor",
          "[search][metaheuristic][cooling]") {
  for (const auto kind : {CoolingScheduleKind::geometric, CoolingScheduleKind::linear,
                          CoolingScheduleKind::adaptive,
                          CoolingScheduleKind::lundy_mees}) {
    SAConfig config;
    config.cooling_schedule = kind;
    config.max_refinements = 10;
    config.initial_temperature = 1.0;
    config.final_temperature = 0.1;

    CoolingSchedule schedule(config);
    double temperature = schedule.initial_temperature();
    for (std::size_t iteration = 0; iteration < 10U; ++iteration) {
      const double next = schedule.next_temperature(temperature, iteration, true);
      REQUIRE(next <= temperature + 1e-9);
      REQUIRE(next >= config.final_temperature - 1e-9);
      temperature = next;
    }
  }
}

TEST_CASE("late acceptance history compares against historical scores",
          "[search][metaheuristic][lahc]") {
  LateAcceptanceHistory history(3, 10.0);
  REQUIRE(history.accepts(10.0, 10.0));
  REQUIRE_FALSE(history.accepts(9.0, 10.0));
  history.advance(8.0);
  history.advance(7.0);
  history.advance(6.0);
  REQUIRE(history.accepts(7.0, 6.0));
}

TEST_CASE("shared neighborhood operators produce order changes",
          "[search][metaheuristic][neighborhood]") {
  const auto normalized = shiny::nesting::normalize_request(neighborhood_request());
  REQUIRE(normalized.ok());

  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget time_budget;
  OrderEvaluator evaluator(normalized.value(), SolveControl{.random_seed = 19},
                           time_budget, stopwatch);
  const auto order = shiny::nesting::search::detail::original_order(normalized.value());
  const auto forced_rotations =
      shiny::nesting::search::detail::original_forced_rotations(normalized.value());

  for (const auto op : std::array{NeighborhoodOperator::adjacent_swap,
                                  NeighborhoodOperator::random_swap,
                                  NeighborhoodOperator::relocate,
                                  NeighborhoodOperator::inversion,
                                  NeighborhoodOperator::large_item_swap,
                                  NeighborhoodOperator::rotation_change,
                                  NeighborhoodOperator::random_destroy_repair,
                                  NeighborhoodOperator::area_destroy_repair,
                                  NeighborhoodOperator::related_destroy_repair,
                                  NeighborhoodOperator::cluster_destroy_repair,
                                  NeighborhoodOperator::regret_destroy_repair}) {
    bool changed = false;
    for (std::size_t attempt = 0; attempt < 4U && !changed; ++attempt) {
      DeterministicRng rng(19 + attempt);
      const auto move = shiny::nesting::search::detail::propose_move(
          order, forced_rotations, normalized.value(), evaluator.piece_areas(),
          evaluator.piece_rotation_counts(), rng, op, 2U);
      changed = move.changed &&
                (move.order != order || move.forced_rotations != forced_rotations);
    }
    INFO(static_cast<int>(op));
    REQUIRE(changed);
  }
}

TEST_CASE("order evaluator honors forced rotation assignments",
          "[search][metaheuristic][rotation]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::simulated_annealing;
  request.execution.default_rotations = {{0.0, 90.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 2.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(0.0, 0.0, 3.0, 1.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget time_budget;
  OrderEvaluator evaluator(normalized.value(), SolveControl{.random_seed = 7},
                           time_budget, stopwatch);
  const auto order = shiny::nesting::search::detail::original_order(normalized.value());

  auto forced_zero =
      shiny::nesting::search::detail::original_forced_rotations(normalized.value());
  forced_zero[0] = shiny::nesting::geom::RotationIndex{0};
  const auto unplaced = evaluator.evaluate(order, forced_zero, 1U);
  REQUIRE(unplaced.metrics.placed_parts == 0U);

  auto forced_ninety = forced_zero;
  forced_ninety[0] = shiny::nesting::geom::RotationIndex{1};
  const auto placed = evaluator.evaluate(order, forced_ninety, 2U);
  REQUIRE(placed.metrics.placed_parts == 1U);
  REQUIRE(placed.result.layout.placement_trace.size() == 1U);
  REQUIRE(placed.result.layout.placement_trace.front().rotation_index.value == 1U);
}

TEST_CASE("metaheuristic strategies improve the constructive baseline",
          "[solve][metaheuristic]") {
  auto constructive_request = improvement_request();
  constructive_request.execution.strategy = StrategyKind::sequential_backtrack;
  const auto constructive = shiny::nesting::solve(constructive_request);
  REQUIRE(constructive.ok());
  REQUIRE(constructive.value().layout.placement_trace.size() == 1U);

  const std::array strategies{
      std::pair{StrategyKind::simulated_annealing,
                OptimizerKind::simulated_annealing},
      std::pair{StrategyKind::alns, OptimizerKind::alns},
      std::pair{StrategyKind::gdrr, OptimizerKind::gdrr},
      std::pair{StrategyKind::lahc, OptimizerKind::lahc},
  };

  for (const auto &[strategy, optimizer] : strategies) {
    auto request = improvement_request();
    request.execution.strategy = strategy;
    const auto result =
        shiny::nesting::solve(request, SolveControl{.iteration_limit = 12,
                                                    .random_seed = 17});
    INFO(static_cast<int>(strategy));
    REQUIRE(result.ok());
    REQUIRE(result.value().layout.placement_trace.size() == 2U);
    REQUIRE(result.value().layout.unplaced_piece_ids.empty());
    REQUIRE(result.value().search.optimizer == optimizer);
    REQUIRE_FALSE(result.value().search.progress.empty());
  }
}

TEST_CASE("metaheuristic strategies preserve or improve benchmark-style cases",
          "[solve][metaheuristic][benchmark]") {
  const std::array benchmark_cases{
      std::pair{"hole-host", improvement_request()},
      std::pair{"strip-collapse", strip_benchmark_request()},
  };
  const std::array strategies{
      std::pair{StrategyKind::simulated_annealing,
                OptimizerKind::simulated_annealing},
      std::pair{StrategyKind::alns, OptimizerKind::alns},
      std::pair{StrategyKind::gdrr, OptimizerKind::gdrr},
      std::pair{StrategyKind::lahc, OptimizerKind::lahc},
  };

  bool saw_strict_improvement = false;
  for (const auto &[case_id, constructive_seed] : benchmark_cases) {
    auto constructive_request = constructive_seed;
    constructive_request.execution.strategy = StrategyKind::sequential_backtrack;
    const auto constructive = shiny::nesting::solve(constructive_request);
    INFO(case_id);
    REQUIRE(constructive.ok());

    const auto constructive_placed =
        constructive.value().layout.placement_trace.size();
    const auto constructive_bins = constructive.value().layout.bins.size();

    for (const auto &[strategy, optimizer] : strategies) {
      auto request = constructive_seed;
      request.execution.strategy = strategy;
      const auto result =
          shiny::nesting::solve(request, SolveControl{.iteration_limit = 16,
                                                      .random_seed = 17});
      INFO(static_cast<int>(strategy));
      REQUIRE(result.ok());
      REQUIRE(result.value().search.optimizer == optimizer);

      const auto placed = result.value().layout.placement_trace.size();
      const auto bins = result.value().layout.bins.size();
      REQUIRE(placed >= constructive_placed);
      if (placed == constructive_placed) {
        REQUIRE(bins <= constructive_bins);
      }
      saw_strict_improvement |=
          placed > constructive_placed ||
          (placed == constructive_placed && bins < constructive_bins);
    }
  }

  REQUIRE(saw_strict_improvement);
}

TEST_CASE("irregular production dispatch can route to the new optimizers",
          "[solve][production][dispatch]") {
  const std::array optimizers{
      std::pair{ProductionOptimizerKind::simulated_annealing,
                OptimizerKind::simulated_annealing},
      std::pair{ProductionOptimizerKind::alns, OptimizerKind::alns},
      std::pair{ProductionOptimizerKind::gdrr, OptimizerKind::gdrr},
      std::pair{ProductionOptimizerKind::lahc, OptimizerKind::lahc},
  };

  for (const auto &[optimizer, expected] : optimizers) {
    auto request = improvement_request();
    request.execution.strategy = StrategyKind::metaheuristic_search;
    request.execution.production_optimizer = optimizer;

    const auto result =
        shiny::nesting::solve(request, SolveControl{.iteration_limit = 12,
                                                    .random_seed = 17});
    INFO(static_cast<int>(optimizer));
    REQUIRE(result.ok());
    REQUIRE(result.value().strategy == StrategyKind::metaheuristic_search);
    REQUIRE(result.value().search.optimizer == expected);
    REQUIRE(result.value().layout.placement_trace.size() == 2U);
    const auto stop_reason = result.value().stop_reason;
    REQUIRE((stop_reason == StopReason::iteration_limit_reached ||
             stop_reason == StopReason::completed));
  }
}

TEST_CASE("strategy registry resolves direct and production strategies",
          "[solve][strategy-registry]") {
  auto direct_request = improvement_request();
  direct_request.execution.strategy = StrategyKind::alns;
  direct_request.execution.alns.max_refinements = 19;
  const auto direct_normalized = shiny::nesting::normalize_request(direct_request);
  REQUIRE(direct_normalized.ok());

  const auto direct_resolved = shiny::nesting::search::StrategyRegistry::instance().resolve(
      direct_normalized.value().request.execution);
  REQUIRE(direct_resolved.run != nullptr);
  REQUIRE_FALSE(direct_resolved.result_strategy_override.has_value());
  const auto *direct_config =
      direct_normalized.value().request.execution.strategy_config.get_if<ALNSConfig>(
          StrategyKind::alns);
  REQUIRE(direct_config != nullptr);
  REQUIRE(direct_config->max_refinements == 19U);

  auto production_request = improvement_request();
  production_request.execution.strategy = StrategyKind::metaheuristic_search;
  production_request.execution.production_optimizer = ProductionOptimizerKind::lahc;
  production_request.execution.lahc.max_refinements = 13;
  const auto production_normalized =
      shiny::nesting::normalize_request(production_request);
  REQUIRE(production_normalized.ok());

  const auto production_resolved =
      shiny::nesting::search::StrategyRegistry::instance().resolve(
          production_normalized.value().request.execution);
  REQUIRE(production_resolved.run != nullptr);
  REQUIRE(production_resolved.result_strategy_override ==
          StrategyKind::metaheuristic_search);
  const auto *production_config =
      production_normalized.value().request.execution.production_strategy_config
          .get_if<shiny::nesting::LAHCConfig>(ProductionOptimizerKind::lahc);
  REQUIRE(production_config != nullptr);
  REQUIRE(production_config->max_refinements == 13U);
}
