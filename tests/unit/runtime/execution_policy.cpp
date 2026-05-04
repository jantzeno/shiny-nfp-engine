#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::SolveControl;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}}));
}

// Base request with a single 10×10 bin and two small pieces that trivially fit.
auto base_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces = {
      {.piece_id = 1, .polygon = rectangle(0.0, 0.0, 2.0, 2.0)},
      {.piece_id = 2, .polygon = rectangle(0.0, 0.0, 2.0, 2.0)},
  };
  return request;
}

// Minimal production config for metaheuristic_search that finishes quickly.
auto minimal_production_request() -> NestingRequest {
  auto request = base_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production_optimizer = ProductionOptimizerKind::brkga;
  request.execution.production.population_size = 6;
  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 2;
  request.execution.production.polishing_passes = 0;
  request.execution.production.diversification_swaps = 1;
  request.execution.production.infeasible_pool_capacity = 2;
  request.execution.production.infeasible_rollback_after = 1;
  return request;
}

} // namespace

TEST_CASE("strategy kinds route to the declared solver path",
          "[strategy][routing]") {
  SECTION("bounding_box strategy routes to bounding-box packer") {
    auto request = base_request();
    request.execution.strategy = StrategyKind::bounding_box;
    REQUIRE(request.is_valid());

    const auto result =
        shiny::nesting::solve(request, SolveControl{.random_seed = 17});
    REQUIRE(result.has_value());
    REQUIRE(result.value().strategy == StrategyKind::bounding_box);
    REQUIRE(result.value().layout_valid());
  }

  SECTION("metaheuristic_search + brkga routes to BRKGA optimizer") {
    auto request = minimal_production_request();
    REQUIRE(request.is_valid());

    const auto result =
        shiny::nesting::solve(request, SolveControl{.random_seed = 17});
    REQUIRE(result.has_value());
    REQUIRE(result.value().strategy == StrategyKind::metaheuristic_search);
    REQUIRE(result.value().search.optimizer == OptimizerKind::brkga);
    REQUIRE(result.value().all_parts_placed());
    REQUIRE(result.value().layout_valid());
  }
}
