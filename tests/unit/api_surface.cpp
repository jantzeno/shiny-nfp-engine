#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "api/dto.hpp"
#include "api/request_builder.hpp"
#include "runtime/cancellation.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::ALNSConfig;
using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::ProductionSearchConfig;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::api::NestingRequestBuilder;
using shiny::nesting::api::SolveControlBuilder;
using shiny::nesting::geom::DiscreteRotationSet;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::RotationRange;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return {
      .outer =
          {
              {min_x, min_y},
              {max_x, min_y},
              {max_x, max_y},
              {min_x, max_y},
          },
  };
}

auto trapezoid() -> PolygonWithHoles {
  return {
      .outer =
          {
              {0.0, 0.0},
              {4.0, 0.0},
              {3.0, 2.0},
              {0.0, 3.0},
          },
  };
}

auto simple_request() -> NestingRequest {
  return NestingRequestBuilder{}
      .with_strategy(StrategyKind::sequential_backtrack)
      .with_default_rotations(DiscreteRotationSet{{0.0}})
      .add_bin(BinRequest{
          .bin_id = 10,
          .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
      })
      .add_piece(PieceRequest{
          .piece_id = 100,
          .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      })
      .build();
}

auto heavy_production_request() -> NestingRequest {
  auto builder = NestingRequestBuilder{};
  builder.with_strategy(StrategyKind::metaheuristic_search)
      .with_production_optimizer(ProductionOptimizerKind::brkga)
      .with_production_config(ProductionOptimizerKind::brkga,
                              ProductionSearchConfig{
                                  .population_size = 48,
                                  .elite_count = 8,
                                  .mutant_count = 8,
                                  .max_iterations = 64,
                              })
      .with_default_rotations(DiscreteRotationSet{{0.0}})
      .add_bin(BinRequest{
          .bin_id = 70,
          .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
      });
  for (std::size_t index = 0; index < 18U; ++index) {
    builder.add_piece(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(200U + index),
        .polygon = rectangle(0.0, 0.0, 4.0 + (index % 3U), 2.0 + (index % 2U)),
    });
  }
  return builder.build();
}

} // namespace

TEST_CASE("request builder supports simple solve and full-success helpers",
          "[api][builder]") {
  const auto request = simple_request();
  REQUIRE(request.is_valid());

  const auto result = shiny::nesting::solve(request);
  REQUIRE(result.ok());
  REQUIRE(result.value().is_full_success());
  REQUIRE(result.value().layout_valid());
  REQUIRE(result.value().summary().full_success);
}

TEST_CASE("request builder provides typed BRKGA config and rotation-range requests",
          "[api][builder][config]") {
  auto request =
      NestingRequestBuilder{}
          .with_strategy(StrategyKind::metaheuristic_search)
          .with_production_optimizer(ProductionOptimizerKind::brkga)
          .with_production_config(ProductionOptimizerKind::brkga,
                                  ProductionSearchConfig{
                                      .population_size = 12,
                                      .elite_count = 3,
                                      .mutant_count = 2,
                                      .max_iterations = 5,
                                  })
          .with_default_rotations(DiscreteRotationSet{
              .range_degrees = RotationRange{
                  .min_degrees = 0.0,
                  .max_degrees = 180.0,
                  .step_degrees = 90.0,
              },
          })
          .add_bin(BinRequest{
              .bin_id = 15,
              .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 42,
              .polygon = trapezoid(),
          })
          .build();

  REQUIRE(request.is_valid());
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  const auto *config =
      production_strategy_config_ptr<ProductionSearchConfig>(
          normalized.value().request.execution, ProductionOptimizerKind::brkga);
  REQUIRE(config != nullptr);
  REQUIRE(config->max_iterations == 5U);
  REQUIRE(shiny::nesting::geom::materialize_rotations(
              normalized.value().request.execution.default_rotations) ==
          std::vector<double>{0.0, 90.0, 180.0});
}

TEST_CASE("production config validates strip and separator tuning knobs",
          "[api][builder][config]") {
  ProductionSearchConfig config;
  config.separator_worker_count = 2;
  config.separator_max_iterations = 16;
  config.separator_iter_no_improvement_limit = 4;
  config.separator_strike_limit = 2;
  config.separator_global_samples = 12;
  config.separator_focused_samples = 6;
  config.separator_coordinate_descent_iterations = 8;
  config.strip_exploration_ratio = 0.6;
  config.infeasible_pool_capacity = 3;
  config.infeasible_rollback_after = 2;

  REQUIRE(config.is_valid());

  config.strip_exploration_ratio = 1.0;
  REQUIRE_FALSE(config.is_valid());
  config.strip_exploration_ratio = 0.6;
  config.separator_worker_count = 0;
  REQUIRE_FALSE(config.is_valid());
}

TEST_CASE("debug-sized BRKGA configs keep auxiliary budgets in range",
          "[api][builder][config]") {
  ProductionSearchConfig config;
  config.max_iterations = 1;
  config.population_size = 3;
  config.elite_count = 1;
  config.mutant_count = 1;
  config.infeasible_pool_capacity = 1;
  config.infeasible_rollback_after = 1;
  config.polishing_passes = 0;

  REQUIRE(config.is_valid());

  config.infeasible_pool_capacity = 4;
  REQUIRE_FALSE(config.is_valid());
  config.infeasible_pool_capacity = 1;
  config.infeasible_rollback_after = 2;
  REQUIRE_FALSE(config.is_valid());
}

TEST_CASE("solve control builder covers time limits and cancellation",
          "[api][builder][control]") {
  const auto request = heavy_production_request();
  REQUIRE(request.is_valid());

  const auto timed = shiny::nesting::solve(
      request, SolveControlBuilder{}.with_time_limit_ms(1).build());
  REQUIRE(timed.ok());
  REQUIRE(timed.value().stop_reason == StopReason::time_limit_reached);

  shiny::nesting::runtime::CancellationSource source;
  source.request_stop();
  const auto cancelled = shiny::nesting::solve(
      request, SolveControlBuilder{}.with_cancellation(source.token()).build());
  REQUIRE(cancelled.ok());
  REQUIRE(cancelled.value().stop_reason == StopReason::cancelled);
}

TEST_CASE("stable DTO helpers round-trip requests and expose result summaries",
          "[api][dto]") {
  const auto request = simple_request();
  const auto control = SolveControlBuilder{}
                           .with_time_limit_ms(25)
                           .with_seed_mode(shiny::nesting::SeedProgressionMode::decrement)
                           .build();

  const auto request_dto = shiny::nesting::api::to_dto(request, control);
  const auto roundtrip_request = shiny::nesting::api::to_request(request_dto);
  const auto roundtrip_control =
      shiny::nesting::api::to_solve_control(request_dto.control);

  REQUIRE(roundtrip_request.is_valid());
  REQUIRE(roundtrip_control.time_limit_milliseconds == 25U);
  REQUIRE(roundtrip_control.seed_mode ==
          shiny::nesting::SeedProgressionMode::decrement);

  const auto result = shiny::nesting::solve(roundtrip_request, roundtrip_control);
  REQUIRE(result.ok());

  const auto result_dto = shiny::nesting::api::to_dto(result.value());
  REQUIRE(result_dto.summary.full_success);
  REQUIRE(result_dto.summary.layout_valid);
  REQUIRE(result_dto.validation.valid);
  REQUIRE(result_dto.total_parts == 1U);
  REQUIRE(result_dto.layout.placement_count == 1U);
  REQUIRE(result_dto.layout.unplaced_piece_ids.empty());
  REQUIRE(result_dto.stop_reason == StopReason::completed);
}

TEST_CASE("request builder preserves invalid-input semantics",
          "[api][builder][invalid]") {
  ALNSConfig invalid_alns{};
  invalid_alns.destroy_min_count = 0;
  const auto request =
      NestingRequestBuilder{}
          .with_strategy(StrategyKind::alns)
          .with_strategy_config(StrategyKind::alns, invalid_alns)
          .add_bin(BinRequest{
              .bin_id = 1,
              .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 1,
              .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
          })
          .build();

  REQUIRE_FALSE(request.is_valid());
  REQUIRE_FALSE(shiny::nesting::solve(request).ok());

  const auto checked =
      NestingRequestBuilder{}
          .with_strategy(StrategyKind::alns)
          .with_strategy_config(StrategyKind::alns, invalid_alns)
          .add_bin(BinRequest{
              .bin_id = 1,
              .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 1,
              .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
          })
          .build_checked();
  REQUIRE_FALSE(checked.ok());
}
