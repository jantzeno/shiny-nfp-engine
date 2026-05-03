#include <catch2/catch_test_macros.hpp>

#include "geometry/queries/normalize.hpp"
#include "internal/request_normalization.hpp"
#include "packing/common.hpp"
#include "placement/types.hpp"
#include "request.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::ExecutionPolicy;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::PreprocessPolicy;
using shiny::nesting::StrategyKind;
using shiny::nesting::place::PartGrainCompatibility;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> shiny::nesting::geom::PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(min_x, min_y),
          shiny::nesting::geom::Point2(max_x, min_y),
          shiny::nesting::geom::Point2(max_x, max_y),
          shiny::nesting::geom::Point2(min_x, max_y),
      }));
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
      .strategy = StrategyKind::bounding_box,
      .default_rotations = {{0.0, 90.0, 180.0, 270.0}},
      .part_spacing = 1.5,
      .allow_part_overflow = false,
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
          .exclusion_zones = {shiny::nesting::place::BedExclusionZone{
              .zone_id = 7,
              .region =
                  shiny::nesting::geom::Polygon(shiny::nesting::geom::Ring{
                      shiny::nesting::geom::Point2(2.0, 2.0),
                      shiny::nesting::geom::Point2(4.0, 2.0),
                      shiny::nesting::geom::Point2(4.0, 4.0),
                      shiny::nesting::geom::Point2(2.0, 4.0),
                  }),
          }},
      },
  };
  request.pieces = {{
      .piece_id = 100,
      .polygon = rectangle(10.0, 20.0, 13.0, 24.0),
      .quantity = 2,
      .allowed_rotations =
          shiny::nesting::geom::DiscreteRotationSet{{90.0, 270.0}},
      .grain_compatibility = PartGrainCompatibility::parallel_to_bed,
      .allowed_bin_ids = {20},
  }};

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());
  REQUIRE(normalized.value().request.bins.size() == 1);
  REQUIRE(normalized.value().request.bins.front().bin_id == 20);
  REQUIRE(normalized.value().expanded_bins.size() == 2);
  REQUIRE(normalized.value().expanded_pieces.size() == 2);
  REQUIRE_FALSE(normalized.value().request.execution.allow_part_overflow);
  REQUIRE(
      normalized.value().request.pieces.front().allowed_rotations.has_value());
  REQUIRE(normalized.value().expanded_bins.front().identity.lifecycle ==
          shiny::nesting::pack::BinLifecycle::user_created);
  REQUIRE(
      normalized.value().expanded_bins.front().identity.source_request_bin_id ==
      20U);

  const auto piece_bounds = shiny::nesting::geom::compute_bounds(
      normalized.value().request.pieces.front().polygon);
  REQUIRE(piece_bounds.min.x() == 0.0);
  REQUIRE(piece_bounds.min.y() == 0.0);
  REQUIRE(piece_bounds.max.x() == 3.0);
  REQUIRE(piece_bounds.max.y() == 4.0);

  const auto decoder_request =
      shiny::nesting::to_bounding_box_decoder_request(normalized.value());
  REQUIRE(decoder_request.has_value());
  REQUIRE(decoder_request.value().bins.size() == 2);
  REQUIRE(decoder_request.value().pieces.size() == 2);
  REQUIRE(decoder_request.value().config.placement.part_clearance == 1.5);
  REQUIRE(decoder_request.value().config.placement.exclusion_zones.size() == 2);
  REQUIRE(decoder_request.value().pieces.front().allowed_rotations.has_value());
  REQUIRE(shiny::nesting::geom::materialize_rotations(
              *decoder_request.value().pieces.front().allowed_rotations) ==
          std::vector<double>{90.0, 270.0});
  REQUIRE(decoder_request.value().pieces.front().grain_compatibility ==
          PartGrainCompatibility::parallel_to_bed);
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
  REQUIRE(normalized.has_value());

  const auto decoder_request =
      shiny::nesting::to_bounding_box_decoder_request(normalized.value());
  REQUIRE(decoder_request.has_value());
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
  REQUIRE_FALSE(shiny::nesting::normalize_request(request).has_value());
}

TEST_CASE("normalized requests materialize type-erased strategy configs",
          "[request][normalize][strategy-config]") {
  // Only brkga is accepted; alns/lahc/sa/gdrr are permanently rejected by
  // production_optimizer_is_valid() and must not appear here.
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production_optimizer =
      shiny::nesting::ProductionOptimizerKind::brkga;
  request.execution.production.population_size = 6;
  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 5;
  request.bins = {{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  }};
  request.pieces = {{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  }};

  REQUIRE(request.is_valid());
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  const auto *prod =
      shiny::nesting::production_strategy_config_ptr<
          shiny::nesting::ProductionSearchConfig>(
          normalized.value().request.execution,
          shiny::nesting::ProductionOptimizerKind::brkga);
  REQUIRE(prod != nullptr);
  REQUIRE(prod->population_size == 6U);
  REQUIRE(prod->elite_count == 2U);
  REQUIRE(prod->mutant_count == 1U);
  REQUIRE(prod->max_iterations == 5U);
}
