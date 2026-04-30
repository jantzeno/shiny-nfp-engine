#include <catch2/catch_test_macros.hpp>

#include "api/dto.hpp"
#include "api/request_builder.hpp"
#include "runtime/cancellation.hpp"
#include "solve.hpp"

namespace {

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> shiny::nesting::geom::PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

} // namespace

TEST_CASE("README public API examples compile and execute",
          "[readme][api][examples]") {
  using namespace shiny::nesting;

  const auto request =
      api::NestingRequestBuilder{}
          .with_strategy(StrategyKind::sequential_backtrack)
          .with_default_rotations(geom::DiscreteRotationSet{{0.0, 90.0}})
          .add_bin(BinRequest{
              .bin_id = 1,
              .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 7,
              .polygon = rectangle(0.0, 0.0, 3.0, 2.0),
          })
          .build_checked();
  REQUIRE(request.ok());

  const auto solved = solve(request.value());
  REQUIRE(solved.ok());
  REQUIRE(solved.value().is_full_success());

  const auto production_request =
      api::NestingRequestBuilder{}
          .with_strategy(StrategyKind::metaheuristic_search)
          .with_production_optimizer(ProductionOptimizerKind::brkga)
          .with_production_config(ProductionOptimizerKind::brkga,
                                  ProductionSearchConfig{
                                      .population_size = 8,
                                      .elite_count = 2,
                                      .mutant_count = 1,
                                      .max_iterations = 2,
                                  })
          .add_bin(BinRequest{
              .bin_id = 1,
              .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 7,
              .polygon = rectangle(0.0, 0.0, 3.0, 2.0),
          })
          .build_checked();
  REQUIRE(production_request.ok());

  runtime::CancellationSource cancel_source;
  const auto timed =
      solve(production_request.value(),
            api::SolveControlBuilder{}.with_time_limit_ms(1).build());
  REQUIRE(timed.ok());

  const auto request_dto = api::to_dto(request.value());
  const auto roundtrip = solve(api::to_request(request_dto));
  REQUIRE(roundtrip.ok());
  const auto result_dto = api::to_dto(roundtrip.value());
  REQUIRE(result_dto.summary.full_success);
}
