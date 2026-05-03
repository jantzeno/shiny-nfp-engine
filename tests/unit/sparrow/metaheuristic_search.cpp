#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <vector>

#include "internal/request_normalization.hpp"
#include "packing/sparrow/search/detail/cooling_schedule.hpp"
#include "packing/sparrow/search/detail/lahc.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"
#include "request.hpp"
#include "runtime/deterministic_rng.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::CoolingScheduleKind;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::SAConfig;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::runtime::DeterministicRng;
using shiny::nesting::search::detail::CoolingSchedule;
using shiny::nesting::search::detail::LateAcceptanceHistory;
using shiny::nesting::search::detail::NeighborhoodSearch;
using shiny::nesting::search::detail::OrderEvaluator;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

auto neighborhood_request() -> NestingRequest {
  NestingRequest request;
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

} // namespace

TEST_CASE("cooling schedules cool and clamp to the configured floor",
          "[search][metaheuristic][cooling]") {
  for (const auto kind :
       {CoolingScheduleKind::geometric, CoolingScheduleKind::linear,
        CoolingScheduleKind::adaptive, CoolingScheduleKind::lundy_mees}) {
    SAConfig config;
    config.cooling_schedule = kind;
    config.max_refinements = 10;
    config.initial_temperature = 1.0;
    config.final_temperature = 0.1;

    CoolingSchedule schedule(config);
    double temperature = schedule.initial_temperature();
    for (std::size_t iteration = 0; iteration < 10U; ++iteration) {
      const double next =
          schedule.next_temperature(temperature, iteration, true);
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
  const auto normalized =
      shiny::nesting::normalize_request(neighborhood_request());
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget time_budget;
  OrderEvaluator evaluator(normalized.value(), SolveControl{.random_seed = 19},
                           time_budget, stopwatch);
  const auto order =
      shiny::nesting::search::detail::original_order(normalized.value());
  const auto forced_rotations =
      shiny::nesting::search::detail::original_forced_rotations(
          normalized.value());

  for (const auto op : std::array{
           NeighborhoodSearch::adjacent_swap, NeighborhoodSearch::random_swap,
           NeighborhoodSearch::relocate, NeighborhoodSearch::inversion,
           NeighborhoodSearch::large_item_swap,
           NeighborhoodSearch::rotation_change,
           NeighborhoodSearch::random_destroy_repair,
           NeighborhoodSearch::area_destroy_repair,
           NeighborhoodSearch::related_destroy_repair,
           NeighborhoodSearch::cluster_destroy_repair,
           NeighborhoodSearch::regret_destroy_repair}) {
    bool changed = false;
    for (std::size_t attempt = 0; attempt < 4U && !changed; ++attempt) {
      DeterministicRng rng(19 + attempt);
      const auto move = shiny::nesting::search::detail::propose_move(
          order, forced_rotations, normalized.value(), evaluator.piece_areas(),
          evaluator.piece_rotation_counts(), rng, op, 2U);
      changed = move.changed && (move.order != order ||
                                 move.forced_rotations != forced_rotations);
    }
    INFO(static_cast<int>(op));
    REQUIRE(changed);
  }
}

TEST_CASE("order evaluator honors forced rotation assignments",
          "[search][metaheuristic][rotation]") {
  NestingRequest request;
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
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget time_budget;
  OrderEvaluator evaluator(normalized.value(), SolveControl{.random_seed = 7},
                           time_budget, stopwatch);
  const auto order =
      shiny::nesting::search::detail::original_order(normalized.value());

  auto forced_zero = shiny::nesting::search::detail::original_forced_rotations(
      normalized.value());
  forced_zero[0] = shiny::nesting::geom::RotationIndex{0};
  const auto unplaced = evaluator.evaluate(order, forced_zero, 1U);
  REQUIRE(unplaced.metrics.placed_parts == 0U);

  auto forced_ninety = forced_zero;
  forced_ninety[0] = shiny::nesting::geom::RotationIndex{1};
  const auto placed = evaluator.evaluate(order, forced_ninety, 2U);
  REQUIRE(placed.metrics.placed_parts == 1U);
  REQUIRE(placed.result.layout.placement_trace.size() == 1U);
  REQUIRE(placed.result.layout.placement_trace.front().rotation_index.value ==
          1U);
}