#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "api/dto.hpp"
#include "api/request_builder.hpp"
#include "solve.hpp"
#include "runtime/cancellation.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::ObjectiveMode;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::ProductionSearchConfig;
using shiny::nesting::ProfileSolveControl;
using shiny::nesting::set_production_strategy_config;
using shiny::nesting::SolveProfile;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::api::NestingRequestBuilder;
using shiny::nesting::api::ProfileRequestBuilder;
using shiny::nesting::api::SolveControlBuilder;
using shiny::nesting::geom::DiscreteRotationSet;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::geom::RotationRange;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto trapezoid() -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(
      Ring{{0.0, 0.0}, {4.0, 0.0}, {3.0, 2.0}, {0.0, 3.0}});
}

auto simple_request() -> NestingRequest {
  return NestingRequestBuilder{}
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
  builder.with_default_rotations(DiscreteRotationSet{{0.0}})
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
  NestingRequest result = builder.build();
  result.execution.strategy = StrategyKind::metaheuristic_search;
  set_production_strategy_config(result.execution,
                                 ProductionOptimizerKind::brkga,
                                 ProductionSearchConfig{
                                     .population_size = 48,
                                     .elite_count = 8,
                                     .mutant_count = 8,
                                     .max_iterations = 64,
                                 });
  return result;
}

auto heavy_profile_request(const SolveProfile profile)
    -> shiny::nesting::ProfileRequest {
  auto builder = ProfileRequestBuilder{};
  builder.with_profile(profile).with_time_limit_ms(1'000U).add_bin(BinRequest{
      .bin_id = 80,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  for (std::size_t index = 0; index < 12U; ++index) {
    builder.add_piece(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(500U + index),
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
  REQUIRE(result.has_value());
  REQUIRE(result.value().is_full_success());
  REQUIRE(result.value().layout_valid());
  REQUIRE(result.value().summary().full_success);
}

TEST_CASE(
    "request builder provides typed BRKGA config and rotation-range requests",
    "[api][builder][config]") {
  auto request = NestingRequestBuilder{}
                     .with_default_rotations(DiscreteRotationSet{
                         .range_degrees =
                             RotationRange{
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
  request.execution.strategy = StrategyKind::metaheuristic_search;
  set_production_strategy_config(request.execution,
                                 ProductionOptimizerKind::brkga,
                                 ProductionSearchConfig{
                                     .population_size = 12,
                                     .elite_count = 3,
                                     .mutant_count = 2,
                                     .max_iterations = 5,
                                 });

  REQUIRE(request.is_valid());
  const auto normalized = shiny::nesting::normalize_nesting_request(request);
  REQUIRE(normalized.has_value());

  const auto *config = production_strategy_config_ptr<ProductionSearchConfig>(
      normalized.value().execution,
      shiny::nesting::ProductionOptimizerKind::brkga);
  REQUIRE(config != nullptr);
  REQUIRE(config->max_iterations == 5U);
  REQUIRE(shiny::nesting::geom::materialize_rotations(
              normalized.value().execution.default_rotations) ==
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

  shiny::nesting::runtime::CancellationSource source;
  source.request_stop();
  const auto cancelled = shiny::nesting::solve(
      request, SolveControlBuilder{}.with_cancellation(source.token()).build());
  REQUIRE(cancelled.has_value());
  REQUIRE(cancelled.value().stop_reason == StopReason::cancelled);
}

TEST_CASE("profile builder keeps search presets internal to the public solve "
          "surface",
          "[api][builder][profile]") {
  const auto request = heavy_profile_request(SolveProfile::maximum_search);

  const auto result =
      shiny::nesting::solve(request, ProfileSolveControl{.random_seed = 29U});
  REQUIRE(result.has_value());
  CHECK(result.value().strategy == StrategyKind::metaheuristic_search);
  CHECK(result.value().search.optimizer == OptimizerKind::none);

  const auto dto = shiny::nesting::api::to_dto(result.value());
  CHECK(dto.optimizer == OptimizerKind::none);
}

TEST_CASE("profile builder round-trips knapsack objective mode and piece "
          "values",
          "[api][builder][profile]") {
  const auto request = ProfileRequestBuilder{}
                           .with_profile(SolveProfile::maximum_search)
                           .with_objective_mode(ObjectiveMode::maximize_value)
                           .with_time_limit_ms(1'000U)
                           .add_bin(BinRequest{
                               .bin_id = 81,
                               .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
                           })
                           .add_piece(PieceRequest{
                               .piece_id = 900,
                               .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
                               .value = 9.5,
                           })
                           .build_checked();

  REQUIRE(request.has_value());
  const auto dto = shiny::nesting::api::to_dto(request.value());
  CHECK(dto.objective_mode == ObjectiveMode::maximize_value);
  REQUIRE(dto.pieces.size() == 1U);
  CHECK(dto.pieces.front().value == 9.5);

  const auto roundtrip = shiny::nesting::api::to_request(dto);
  CHECK(roundtrip.objective_mode == ObjectiveMode::maximize_value);
  REQUIRE(roundtrip.pieces.size() == 1U);
  CHECK(roundtrip.pieces.front().value == 9.5);
}

TEST_CASE("stable DTO helpers round-trip requests and expose result summaries",
          "[api][dto]") {
  const auto request = simple_request();
  const auto control =
      SolveControlBuilder{}
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

  const auto result =
      shiny::nesting::solve(roundtrip_request, roundtrip_control);
  REQUIRE(result.has_value());

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
  // population_size - elite_count - mutant_count == 0 makes BRKGA config
  // invalid; this verifies the solve path rejects it rather than silently
  // running with a degenerate population.
  const auto make_invalid_request = [] {
    NestingRequest r = NestingRequestBuilder{}
                           .add_bin(BinRequest{
                               .bin_id = 1,
                               .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
                           })
                           .add_piece(PieceRequest{
                               .piece_id = 1,
                               .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
                           })
                           .build();
    r.execution.strategy = StrategyKind::metaheuristic_search;
    set_production_strategy_config(r.execution,
                                   ProductionOptimizerKind::brkga,
                                   ProductionSearchConfig{
                                       .population_size = 2,
                                       .elite_count = 1,
                                       .mutant_count = 1,
                                       .max_iterations = 4,
                                   });
    return r;
  };

  const auto request = make_invalid_request();
  REQUIRE_FALSE(request.is_valid());
  REQUIRE_FALSE(shiny::nesting::solve(request).has_value());

  const auto request2 = make_invalid_request();
  const auto checked =
      request2.is_valid()
          ? std::expected<NestingRequest, shiny::nesting::util::Status>{request2}
          : std::expected<NestingRequest, shiny::nesting::util::Status>{
                std::unexpected(shiny::nesting::util::Status::invalid_input)};
  REQUIRE_FALSE(checked.has_value());
}
