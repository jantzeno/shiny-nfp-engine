#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>

#include "internal/request_normalization.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "packing/constructive/ordering.hpp"
#include "request.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::normalize_request;
using shiny::nesting::PieceRequest;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::ConstructivePlacementPhase;
using shiny::nesting::pack::constructive::build_fill_first_gap_fill_order;
using shiny::nesting::pack::constructive::build_fill_first_primary_order;
using shiny::nesting::pack::constructive::FillFirstEngine;
using shiny::nesting::pack::constructive::make_fill_first_config;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto ordering_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 32.0, 32.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 5.0, 5.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(0.0, 0.0, 5.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 103,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 104,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  return request;
}

auto overflow_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.allow_part_overflow = true;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 201,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 202,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  return request;
}

} // namespace

TEST_CASE("fill-first engine exposes deterministic primary order and gap-fill order builders",
          "[constructive][fill-first][ordering]") {
  const auto request = ordering_request();
  const auto normalized = normalize_request(request);
  REQUIRE(normalized.ok());

  const auto config = make_fill_first_config(request.execution);
  REQUIRE(config.allow_part_overflow == request.execution.allow_part_overflow);
  REQUIRE(config.candidate_strategy ==
          request.execution.irregular.candidate_strategy);
  REQUIRE(config.piece_ordering == request.execution.irregular.piece_ordering);

  const auto primary_order = build_fill_first_primary_order(normalized.value());
  REQUIRE(primary_order.size() == 4U);
  CHECK(primary_order[0].source_piece_id == 101U);
  CHECK(primary_order[1].source_piece_id == 102U);
  CHECK(primary_order[2].source_piece_id == 103U);
  CHECK(primary_order[3].source_piece_id == 104U);
  CHECK(std::all_of(
      primary_order.begin(), primary_order.end(), [](const auto &piece) {
        return piece.phase == ConstructivePlacementPhase::primary_order;
      }));

  const auto gap_fill_order = build_fill_first_gap_fill_order(
      normalized.value(), std::array<std::uint32_t, 2>{103U, 104U});
  REQUIRE(gap_fill_order.size() == 2U);
  CHECK(gap_fill_order[0].source_piece_id == 104U);
  CHECK(gap_fill_order[1].source_piece_id == 103U);
  CHECK(std::all_of(
      gap_fill_order.begin(), gap_fill_order.end(), [](const auto &piece) {
        return piece.phase == ConstructivePlacementPhase::gap_fill;
      }));
}

TEST_CASE("fill-first engine records frontier advance and overflow events in the constructive replay",
          "[constructive][fill-first][replay]") {
  const auto normalized = normalize_request(overflow_request());
  REQUIRE(normalized.ok());

  FillFirstEngine engine;
  const auto result_or = engine.solve(normalized.value(), SolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.bins.size() == 2U);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.placement_trace.size() == 2U);
  CHECK(std::all_of(result.layout.placement_trace.begin(),
                    result.layout.placement_trace.end(), [](const auto &entry) {
                      return entry.phase ==
                             ConstructivePlacementPhase::primary_order;
                    }));

  REQUIRE(result.constructive.frontier_changes.size() == 2U);
  CHECK(result.constructive.frontier_changes[0].reason ==
        shiny::nesting::ConstructiveFrontierAdvanceReason::initial_bin_opened);
  CHECK(result.constructive.frontier_changes[1].reason ==
        shiny::nesting::ConstructiveFrontierAdvanceReason::overflow_bin_opened);
  CHECK(result.constructive.frontier_changes[1].previous_bin_id ==
        std::optional<std::uint32_t>(1U));

  REQUIRE(result.constructive.overflow_events.size() == 1U);
  CHECK(result.constructive.overflow_events[0].template_bin_id == 1U);
  CHECK(result.constructive.overflow_events[0].source_request_bin_id == 1U);
  CHECK(result.constructive.overflow_events[0].overflow_bin_id != 1U);

  REQUIRE_FALSE(result.constructive.exhaustion_events.empty());
  CHECK(result.constructive.exhaustion_events.back().decision ==
        shiny::nesting::ConstructiveFrontierExhaustionDecision::exhausted);
}