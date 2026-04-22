#include <catch2/catch_test_macros.hpp>

#include "geometry/normalize.hpp"
#include "packing/common.hpp"
#include "request.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::ExecutionPolicy;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::PreprocessPolicy;
using shiny::nesting::StrategyKind;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y)
    -> shiny::nesting::geom::PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles{.outer = {
          {.x = min_x, .y = min_y},
          {.x = max_x, .y = min_y},
          {.x = max_x, .y = max_y},
          {.x = min_x, .y = max_y},
      }});
}

} // namespace

TEST_CASE("normalized requests preserve engine-owned constraints",
          "[request][normalize]") {
  NestingRequest request;
  request.preprocess = PreprocessPolicy{
      .simplify_epsilon = 0.0,
      .normalize_piece_origins = true,
      .discard_empty_bins = true,
  };
  request.execution = ExecutionPolicy{
      .strategy = StrategyKind::sequential_backtrack,
      .default_rotations = {{0.0, 90.0, 180.0, 270.0}},
      .part_spacing = 1.5,
      .enable_part_in_part_placement = true,
      .selected_bin_ids = {20},
  };
  request.bins = {
      {
          .bin_id = 10,
          .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
          .stock = 1,
      },
      {
          .bin_id = 20,
          .polygon = rectangle(0.0, 0.0, 30.0, 15.0),
          .stock = 2,
          .exclusion_zones = {{
              .zone_id = 7,
              .region = {.outer = {
                             {.x = 2.0, .y = 2.0},
                             {.x = 4.0, .y = 2.0},
                             {.x = 4.0, .y = 4.0},
                             {.x = 2.0, .y = 4.0},
                         }},
          }},
      },
  };
  request.pieces = {{
      .piece_id = 100,
      .polygon = rectangle(10.0, 20.0, 13.0, 24.0),
      .quantity = 2,
      .allowed_rotations =
          shiny::nesting::geom::DiscreteRotationSet{{90.0, 270.0}},
      .allowed_bin_ids = {20},
  }};

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  REQUIRE(normalized.value().request.bins.size() == 1);
  REQUIRE(normalized.value().request.bins.front().bin_id == 20);
  REQUIRE(normalized.value().expanded_bins.size() == 2);
  REQUIRE(normalized.value().expanded_pieces.size() == 2);
  REQUIRE(normalized.value().request.pieces.front().allowed_rotations.has_value());

  const auto piece_bounds = shiny::nesting::geom::compute_bounds(
      normalized.value().request.pieces.front().polygon);
  REQUIRE(piece_bounds.min.x == 0.0);
  REQUIRE(piece_bounds.min.y == 0.0);
  REQUIRE(piece_bounds.max.x == 3.0);
  REQUIRE(piece_bounds.max.y == 4.0);

  const auto decoder_request =
      shiny::nesting::to_bounding_box_decoder_request(normalized.value());
  REQUIRE(decoder_request.ok());
  REQUIRE(decoder_request.value().bins.size() == 2);
  REQUIRE(decoder_request.value().pieces.size() == 2);
  REQUIRE(decoder_request.value().config.placement.part_clearance == 1.5);
  REQUIRE(decoder_request.value().config.placement.exclusion_zones.size() == 2);
  REQUIRE(decoder_request.value().pieces.front().allowed_rotations.has_value());
  REQUIRE(shiny::nesting::geom::materialize_rotations(
              *decoder_request.value().pieces.front().allowed_rotations) ==
          std::vector<double>{90.0, 270.0});
  REQUIRE(decoder_request.value().pieces.front().restricted_to_allowed_bins);
  REQUIRE(decoder_request.value().pieces.front().allowed_bin_ids.size() == 2);
}

TEST_CASE("bounding-box decoder preserves explicit empty bin restrictions",
          "[request][normalize][bins]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.selected_bin_ids = {20};
  request.bins = {
      {
          .bin_id = 10,
          .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
      },
      {
          .bin_id = 20,
          .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
      },
  };
  request.pieces = {{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 5.0, 5.0),
      .allowed_bin_ids = {10},
  }};

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  const auto decoder_request =
      shiny::nesting::to_bounding_box_decoder_request(normalized.value());
  REQUIRE(decoder_request.ok());
  REQUIRE(decoder_request.value().bins.size() == 1);
  REQUIRE(decoder_request.value().pieces.size() == 1);
  REQUIRE(decoder_request.value().pieces.front().restricted_to_allowed_bins);
  REQUIRE(decoder_request.value().pieces.front().allowed_bin_ids.empty());
  REQUIRE_FALSE(shiny::nesting::pack::piece_allows_bin(
      decoder_request.value().pieces.front(),
      decoder_request.value().bins.front().bin_id));
}

TEST_CASE("piece-local rotations must narrow the execution default set",
          "[request][normalize][rotations]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0, 90.0}};
  request.bins = {{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  }};
  request.pieces = {{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
      .allowed_rotations = shiny::nesting::geom::DiscreteRotationSet{{180.0}},
  }};

  REQUIRE_FALSE(request.is_valid());
  REQUIRE_FALSE(shiny::nesting::normalize_request(request).ok());
}

TEST_CASE("normalized requests materialize type-erased strategy configs",
          "[request][normalize][strategy-config]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::alns;
  request.execution.alns.max_refinements = 23;
  request.execution.production_optimizer =
      shiny::nesting::ProductionOptimizerKind::lahc;
  request.execution.lahc.max_refinements = 17;
  request.bins = {{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  }};
  request.pieces = {{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  }};

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());

  const auto *alns =
      normalized.value().request.execution.strategy_config.get_if<shiny::nesting::ALNSConfig>(
          StrategyKind::alns);
  REQUIRE(alns != nullptr);
  REQUIRE(alns->max_refinements == 23U);

  const auto *lahc =
      normalized.value().request.execution.production_strategy_config.get_if<
          shiny::nesting::LAHCConfig>(
          shiny::nesting::ProductionOptimizerKind::lahc);
  REQUIRE(lahc != nullptr);
  REQUIRE(lahc->max_refinements == 17U);
}
