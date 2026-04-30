#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

#include "packing/irregular/workspace.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::CandidateStrategy;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceOrdering;
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
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

auto frame(double min_x, double min_y, double max_x, double max_y,
           double hole_min_x, double hole_min_y, double hole_max_x,
           double hole_max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(
      {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
      {{
          {hole_min_x, hole_min_y},
          {hole_min_x, hole_max_y},
          {hole_max_x, hole_max_y},
          {hole_max_x, hole_min_y},
      }});
}

auto total_intersection_area(std::span<const PolygonWithHoles> polygons)
    -> double {
  double total = 0.0;
  for (std::size_t lhs_index = 0; lhs_index < polygons.size(); ++lhs_index) {
    for (std::size_t rhs_index = lhs_index + 1U; rhs_index < polygons.size();
         ++rhs_index) {
      const auto overlap = shiny::nesting::poly::intersection_polygons(
          polygons[lhs_index], polygons[rhs_index]);
      for (const auto &polygon : overlap) {
        total += shiny::nesting::geom::polygon_area(polygon);
      }
    }
  }
  return total;
}

auto require_same_trace(const shiny::nesting::pack::Layout &lhs,
                        const shiny::nesting::pack::Layout &rhs) -> void {
  REQUIRE(lhs.placement_trace.size() == rhs.placement_trace.size());
  for (std::size_t index = 0; index < lhs.placement_trace.size(); ++index) {
    const auto &lhs_entry = lhs.placement_trace[index];
    const auto &rhs_entry = rhs.placement_trace[index];
    REQUIRE(lhs_entry.piece_id == rhs_entry.piece_id);
    REQUIRE(lhs_entry.bin_id == rhs_entry.bin_id);
    REQUIRE(lhs_entry.rotation_index == rhs_entry.rotation_index);
    REQUIRE(lhs_entry.translation == rhs_entry.translation);
    REQUIRE(lhs_entry.mirrored == rhs_entry.mirrored);
    REQUIRE(lhs_entry.inside_hole == rhs_entry.inside_hole);
  }
}

} // namespace

TEST_CASE("irregular constructive solver places hole-contained parts and emits "
          "progress",
          "[solve][irregular][hole][progress]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
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
                   .on_progress =
                       [&](const ProgressSnapshot &snapshot) {
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
  REQUIRE(snapshots[0].placements_successful == 1U);
  REQUIRE(snapshots[1].placements_successful == 2U);
  REQUIRE(snapshots[1].stop_reason == StopReason::completed);
}

TEST_CASE(
    "irregular constructive solver honors exclusion zones and allowed bins",
    "[solve][irregular][exclusion][bins]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(0.0, 0.0, 8.0, 8.0),
      .exclusion_zones = {shiny::nesting::place::BedExclusionZone{
          .zone_id = 1,
          .region = shiny::nesting::geom::Polygon(shiny::nesting::geom::Ring{
              {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}}),
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
  REQUIRE(first_bin.placements.front().placement.translation ==
          Point2{5.0, 0.0});

  const auto exclusion =
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          {0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}});
  REQUIRE(shiny::nesting::poly::polygon_distance(
              first_bin.placements.front().polygon, exclusion) ==
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
  request.execution.strategy = StrategyKind::sequential_backtrack;
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

  const auto first = shiny::nesting::solve(
      request, SolveControl{.operation_limit = 1, .random_seed = 17});
  const auto second = shiny::nesting::solve(
      request, SolveControl{.operation_limit = 1, .random_seed = 17});

  REQUIRE(first.ok());
  REQUIRE(second.ok());
  REQUIRE(first.value().layout.placement_trace.size() ==
          second.value().layout.placement_trace.size());
  for (std::size_t index = 0;
       index < first.value().layout.placement_trace.size(); ++index) {
    const auto &lhs = first.value().layout.placement_trace[index];
    const auto &rhs = second.value().layout.placement_trace[index];
    REQUIRE(lhs.piece_id == rhs.piece_id);
    REQUIRE(lhs.bin_id == rhs.bin_id);
    REQUIRE(lhs.rotation_index == rhs.rotation_index);
    REQUIRE(lhs.translation == rhs.translation);
    REQUIRE(lhs.inside_hole == rhs.inside_hole);
  }
}

TEST_CASE(
    "irregular constructive solver uses NFP perfect candidates when requested",
    "[solve][irregular][nfp][candidates]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.candidate_strategy =
      CandidateStrategy::nfp_perfect;
  request.execution.irregular.piece_ordering = PieceOrdering::input;

  request.bins.push_back(BinRequest{
      .bin_id = 40,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 8,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 9,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto result_or = shiny::nesting::solve(request);

  REQUIRE(result_or.ok());
  const auto &placements = result_or.value().layout.bins.front().placements;
  REQUIRE(placements.size() == 2U);
  REQUIRE(placements[1].source == PlacementCandidateSource::perfect_fit);
  REQUIRE(placements[1].placement.translation == Point2{6.0, 0.0});
}

TEST_CASE("irregular constructive solver honors configured piece ordering",
          "[solve][irregular][ordering]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;

  request.bins.push_back(BinRequest{
      .bin_id = 41,
      .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 11,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  const auto result_or = shiny::nesting::solve(request);

  REQUIRE(result_or.ok());
  REQUIRE(result_or.value().layout.placement_trace.size() == 2U);
  REQUIRE(result_or.value().layout.placement_trace.front().piece_id == 11U);
}

TEST_CASE("irregular constructive solver defaults to largest-area-first "
          "ordering",
          "[solve][irregular][ordering][defaults]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.enable_backtracking = false;
  request.execution.irregular.enable_compaction = false;

  request.bins.push_back(BinRequest{
      .bin_id = 45,
      .polygon = rectangle(0.0, 0.0, 12.0, 12.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 21,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 22,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  REQUIRE(request.execution.irregular.piece_ordering ==
          PieceOrdering::largest_area_first);

  const auto result_or = shiny::nesting::solve(request);

  REQUIRE(result_or.ok());
  REQUIRE(result_or.value().layout.placement_trace.size() == 2U);
  REQUIRE(result_or.value().layout.placement_trace.front().piece_id == 22U);
}

TEST_CASE("irregular constructive shared workspace preserves repeated results",
          "[solve][irregular][workspace]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0, 90.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 44,
      .polygon = rectangle(0.0, 0.0, 10.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 18,
      .polygon = rectangle(0.0, 0.0, 6.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 19,
      .polygon = rectangle(0.0, 0.0, 4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 20,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto baseline = shiny::nesting::solve(
      request, SolveControl{.operation_limit = 1, .random_seed = 23});

  shiny::nesting::pack::PackerWorkspace workspace;
  const auto first =
      shiny::nesting::solve(request, SolveControl{.operation_limit = 1,
                                                  .random_seed = 23,
                                                  .workspace = &workspace});
  const auto second =
      shiny::nesting::solve(request, SolveControl{.operation_limit = 1,
                                                  .random_seed = 23,
                                                  .workspace = &workspace});

  REQUIRE(baseline.ok());
  REQUIRE(first.ok());
  REQUIRE(second.ok());
  require_same_trace(baseline.value().layout, first.value().layout);
  require_same_trace(first.value().layout, second.value().layout);
}

TEST_CASE(
    "irregular constructive backtracking can free a hole for a blocked part",
    "[solve][irregular][backtracking]") {
  NestingRequest baseline_request;
  baseline_request.execution.strategy = StrategyKind::sequential_backtrack;
  baseline_request.execution.enable_part_in_part_placement = true;
  baseline_request.execution.default_rotations = {{0.0}};
  baseline_request.execution.irregular.piece_ordering = PieceOrdering::input;

  baseline_request.bins.push_back(BinRequest{
      .bin_id = 42,
      .polygon = rectangle(0.0, 0.0, 8.0, 10.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 12,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 13,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 14,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto baseline = shiny::nesting::solve(baseline_request);
  REQUIRE(baseline.ok());
  REQUIRE(baseline.value().layout.placement_trace.size() == 2U);
  REQUIRE(baseline.value().layout.unplaced_piece_ids ==
          std::vector<std::uint32_t>{14U});

  auto backtracking_request = baseline_request;
  backtracking_request.execution.irregular.enable_backtracking = true;
  backtracking_request.execution.irregular.max_backtrack_pieces = 1;

  const auto recovered = shiny::nesting::solve(backtracking_request);
  REQUIRE(recovered.ok());
  REQUIRE(recovered.value().layout.unplaced_piece_ids.empty());
  REQUIRE(recovered.value().layout.placement_trace.size() == 3U);

  const auto &placements = recovered.value().layout.bins.front().placements;
  const auto large_it = std::find_if(
      placements.begin(), placements.end(), [](const auto &placement) {
        return placement.placement.piece_id == 14U;
      });
  const auto small_it = std::find_if(
      placements.begin(), placements.end(), [](const auto &placement) {
        return placement.placement.piece_id == 13U;
      });
  REQUIRE(large_it != placements.end());
  REQUIRE(small_it != placements.end());
  REQUIRE(large_it->inside_hole);
  REQUIRE_FALSE(small_it->inside_hole);
}

TEST_CASE("failed irregular constructive backtracking rolls back trial state",
          "[solve][irregular][backtracking][rollback]") {
  NestingRequest baseline_request;
  baseline_request.execution.strategy = StrategyKind::sequential_backtrack;
  baseline_request.execution.default_rotations = {{0.0}};

  baseline_request.bins.push_back(BinRequest{
      .bin_id = 45,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 21,
      .polygon = rectangle(0.0, 0.0, 3.0, 4.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 22,
      .polygon = rectangle(0.0, 0.0, 3.0, 4.0),
  });

  const auto baseline = shiny::nesting::solve(baseline_request);
  REQUIRE(baseline.ok());
  REQUIRE(baseline.value().layout.placement_trace.size() == 1U);
  REQUIRE(baseline.value().layout.unplaced_piece_ids ==
          std::vector<std::uint32_t>{22U});

  auto backtracking_request = baseline_request;
  backtracking_request.execution.irregular.enable_backtracking = true;
  backtracking_request.execution.irregular.max_backtrack_pieces = 1;

  const auto recovered = shiny::nesting::solve(backtracking_request);
  REQUIRE(recovered.ok());
  require_same_trace(baseline.value().layout, recovered.value().layout);
  REQUIRE(recovered.value().layout.unplaced_piece_ids ==
          baseline.value().layout.unplaced_piece_ids);
}

TEST_CASE("irregular constructive compaction and backtracking stay compatible",
          "[solve][irregular][backtracking][compaction]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.enable_part_in_part_placement = true;
  request.execution.default_rotations = {{0.0}};
  request.execution.irregular.enable_backtracking = true;
  request.execution.irregular.max_backtrack_pieces = 1;
  request.execution.irregular.enable_compaction = true;
  request.execution.irregular.compaction_passes = 2;

  request.bins.push_back(BinRequest{
      .bin_id = 43,
      .polygon = rectangle(0.0, 0.0, 8.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 15,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 16,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 17,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto result = shiny::nesting::solve(request);
  REQUIRE(result.ok());
  REQUIRE(result.value().layout.unplaced_piece_ids.empty());
  REQUIRE(result.value().layout.placement_trace.size() == 3U);

  const auto &placements = result.value().layout.bins.front().placements;
  const auto large_it = std::find_if(
      placements.begin(), placements.end(), [](const auto &placement) {
        return placement.placement.piece_id == 17U;
      });
  const auto small_it = std::find_if(
      placements.begin(), placements.end(), [](const auto &placement) {
        return placement.placement.piece_id == 16U;
      });
  REQUIRE(large_it != placements.end());
  REQUIRE(small_it != placements.end());
  REQUIRE(large_it->inside_hole);
  REQUIRE_FALSE(small_it->inside_hole);
}

TEST_CASE(
    "backtracking trial that shrinks the trace then fails fully restores it",
    "[solve][irregular][backtracking][rollback][trace-shrink]") {
  // Construct a layout that places several pieces successfully, then
  // hits a piece that cannot fit. Backtracking must remove (shrink the
  // trace by) the most-recent placements and try to re-pack them along
  // with the failing piece. Because the failing piece never fits, every
  // trial size is rejected and BinTrialGuard must restore the trace,
  // bin states, and opened-bin flags to the pre-trial snapshot.
  //
  // Regression for review Phase 10 §5: exercises the rollback path
  // where the trace shrinks (removed_trace_entries_) before any
  // appended entries — the path that historically depended on a fragile
  // pre/post size diff instead of an explicit appended counter.
  NestingRequest baseline_request;
  baseline_request.execution.strategy = StrategyKind::sequential_backtrack;
  baseline_request.execution.default_rotations = {{0.0}};

  baseline_request.bins.push_back(BinRequest{
      .bin_id = 80,
      .polygon = rectangle(0.0, 0.0, 6.0, 2.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 81,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 82,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 83,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  // Final piece is wider than the bin: cannot ever be placed, so every
  // backtrack trial that removes the trailing 1..N pieces and retries
  // will still fail and trigger rollback.
  baseline_request.pieces.push_back(PieceRequest{
      .piece_id = 84,
      .polygon = rectangle(0.0, 0.0, 7.0, 2.0),
  });

  const auto baseline = shiny::nesting::solve(baseline_request);
  REQUIRE(baseline.ok());
  REQUIRE(baseline.value().layout.placement_trace.size() == 3U);
  REQUIRE(baseline.value().layout.unplaced_piece_ids ==
          std::vector<std::uint32_t>{84U});

  auto backtracking_request = baseline_request;
  backtracking_request.execution.irregular.enable_backtracking = true;
  backtracking_request.execution.irregular.max_backtrack_pieces = 3;

  const auto recovered = shiny::nesting::solve(backtracking_request);
  REQUIRE(recovered.ok());
  // After every failed trial of every size rolls back, the layout must
  // match the no-backtracking baseline byte-for-byte: the trace, the
  // unplaced set, and the placement order.
  require_same_trace(baseline.value().layout, recovered.value().layout);
  REQUIRE(recovered.value().layout.unplaced_piece_ids ==
          baseline.value().layout.unplaced_piece_ids);
}
