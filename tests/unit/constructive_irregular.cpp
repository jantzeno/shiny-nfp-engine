#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <span>
#include <vector>

#include "polygon_ops/boolean_ops.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProgressSnapshot;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::place::PlacementCandidateSource;

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

auto total_intersection_area(std::span<const PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (std::size_t lhs_index = 0; lhs_index < polygons.size(); ++lhs_index) {
    for (std::size_t rhs_index = lhs_index + 1U; rhs_index < polygons.size();
         ++rhs_index) {
      const auto overlap =
          shiny::nesting::poly::intersection_polygons(polygons[lhs_index],
                                                      polygons[rhs_index]);
      for (const auto &polygon : overlap) {
        total += shiny::nesting::geom::polygon_area(polygon);
      }
    }
  }
  return total;
}

} // namespace

TEST_CASE("irregular constructive solver places hole-contained parts and emits progress",
          "[solve][irregular][hole][progress]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::irregular_constructive;
  request.execution.enable_part_in_part_placement = true;
  request.execution.default_rotations = {{0.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 2,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });

  std::vector<ProgressSnapshot> snapshots;
  const auto result_or = shiny::nesting::solve(
      request, SolveControl{
                   .on_progress = [&](const ProgressSnapshot &snapshot) {
                     snapshots.push_back(snapshot);
                   },
               });

  REQUIRE(result_or.ok());
  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.placement_trace.size() == 2U);
  REQUIRE(result.layout.bins.size() == 1U);
  REQUIRE(result.layout.bins.front().placements.size() == 2U);
  REQUIRE(result.layout.bins.front().placements[1].inside_hole);
  REQUIRE(result.layout.bins.front().placements[1].source ==
          PlacementCandidateSource::hole_boundary);
  REQUIRE(result.layout.bins.front().placements[1].placement.translation ==
          Point2{2.0, 2.0});

  const std::vector<PolygonWithHoles> placed_polygons{
      result.layout.bins.front().placements[0].polygon,
      result.layout.bins.front().placements[1].polygon,
  };
  REQUIRE(total_intersection_area(placed_polygons) ==
          Catch::Approx(0.0).margin(1e-8));

  REQUIRE(snapshots.size() == 2U);
  REQUIRE(snapshots[0].placed_parts == 1U);
  REQUIRE(snapshots[1].placed_parts == 2U);
  REQUIRE(snapshots[1].stop_reason == StopReason::completed);
}

TEST_CASE("irregular constructive solver honors exclusion zones and allowed bins",
          "[solve][irregular][exclusion][bins]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::irregular_constructive;
  request.execution.default_rotations = {{0.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(0.0, 0.0, 8.0, 8.0),
      .exclusion_zones =
          {{
              .zone_id = 1,
              .region = {.outer = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}}},
          }},
  });
  request.bins.push_back(BinRequest{
      .bin_id = 21,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 3,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      .allowed_bin_ids = {20},
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 4,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
      .allowed_bin_ids = {21},
  });

  const auto result_or = shiny::nesting::solve(request);

  REQUIRE(result_or.ok());
  const auto &result = result_or.value();
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.bins.size() == 2U);

  const auto &first_bin = result.layout.bins[0];
  REQUIRE(first_bin.bin_id == 20U);
  REQUIRE(first_bin.placements.size() == 1U);
  REQUIRE(first_bin.placements.front().placement.translation == Point2{5.0, 0.0});

  const auto exclusion = PolygonWithHoles{
      .outer = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}},
  };
  REQUIRE(shiny::nesting::poly::polygon_distance(first_bin.placements.front().polygon,
                                                 exclusion) ==
          Catch::Approx(0.0).margin(1e-8));

  const auto &second_bin = result.layout.bins[1];
  REQUIRE(second_bin.bin_id == 21U);
  REQUIRE(second_bin.placements.size() == 1U);
  REQUIRE(second_bin.placements.front().placement.piece_id !=
          first_bin.placements.front().placement.piece_id);
}

TEST_CASE("irregular constructive solver is deterministic for fixed input",
          "[solve][irregular][deterministic]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::irregular_constructive;
  request.execution.default_rotations = {{0.0, 90.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 30,
      .polygon = rectangle(0.0, 0.0, 10.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 5,
      .polygon = rectangle(0.0, 0.0, 6.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 6,
      .polygon = rectangle(0.0, 0.0, 4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 7,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto first = shiny::nesting::solve(request, SolveControl{.iteration_limit = 1, .random_seed = 17});
  const auto second = shiny::nesting::solve(request, SolveControl{.iteration_limit = 1, .random_seed = 17});

  REQUIRE(first.ok());
  REQUIRE(second.ok());
  REQUIRE(first.value().layout.placement_trace.size() ==
          second.value().layout.placement_trace.size());
  for (std::size_t index = 0; index < first.value().layout.placement_trace.size();
       ++index) {
    const auto &lhs = first.value().layout.placement_trace[index];
    const auto &rhs = second.value().layout.placement_trace[index];
    REQUIRE(lhs.piece_id == rhs.piece_id);
    REQUIRE(lhs.bin_id == rhs.bin_id);
    REQUIRE(lhs.rotation_index == rhs.rotation_index);
    REQUIRE(lhs.translation == rhs.translation);
    REQUIRE(lhs.inside_hole == rhs.inside_hole);
  }
}
