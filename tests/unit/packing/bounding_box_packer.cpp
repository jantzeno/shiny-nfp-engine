#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

#include "algorithm_kind.hpp"
#include "internal/legacy_solve.hpp"
#include "packing/bounding_box_packer.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::AlgorithmKind;
using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProgressSnapshot;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::BinInput;
using shiny::nesting::pack::BoundingBoxHeuristic;
using shiny::nesting::pack::BoundingBoxPacker;
using shiny::nesting::pack::DecoderRequest;
using shiny::nesting::pack::PieceInput;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto make_l_shape() -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(Ring{
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {1.0, 2.0}, {1.0, 1.0}, {0.0, 1.0}});
}

auto make_piece(std::uint32_t piece_id, PolygonWithHoles polygon,
                std::uint64_t geometry_revision) -> PieceInput {
  return {
      .piece_id = piece_id,
      .polygon = std::move(polygon),
      .geometry_revision = geometry_revision,
  };
}

auto make_bins(BinInput base_bin, const std::size_t count)
    -> std::vector<BinInput> {
  std::vector<BinInput> bins;
  bins.reserve(std::max<std::size_t>(count, 1));
  for (std::size_t index = 0; index < std::max<std::size_t>(count, 1);
       ++index) {
    BinInput bin = base_bin;
    bin.bin_id = base_bin.bin_id + static_cast<std::uint32_t>(index);
    bins.push_back(std::move(bin));
  }
  return bins;
}

auto make_request(BinInput bin, std::vector<PieceInput> pieces,
                  const std::size_t bin_count = 1) -> DecoderRequest {
  DecoderRequest request{
      .bins = make_bins(std::move(bin), bin_count),
      .pieces = std::move(pieces),
  };
  request.config.placement.allowed_rotations.angles_degrees = {0.0};
  return request;
}

auto make_solve_request(PolygonWithHoles bin, std::vector<PieceRequest> pieces)
    -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = std::move(bin),
  });
  request.pieces = std::move(pieces);
  return request;
}

} // namespace

TEST_CASE("bounding box algorithm kind uses canonical vocabulary",
          "[packing][bounding-box][algorithm]") {
  REQUIRE(shiny::nesting::to_string(AlgorithmKind::bounding_box) ==
          "bounding_box");
  const auto parsed = shiny::nesting::parse_algorithm_kind("bounding_box");
  REQUIRE(parsed.has_value());
  REQUIRE(*parsed == AlgorithmKind::bounding_box);
}

TEST_CASE("bounding box packer packs rectangles on a single bin",
          "[packing][bounding-box]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 10,
          .polygon = make_rectangle(0.0, 0.0, 6.0, 4.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 3.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 3.0, 2.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 2.0, 2.0), 3),
      },
      1);

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 3);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[0].placement.translation.y() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.y() == 0.0);
  REQUIRE(result.bins.front().placements[2].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[2].placement.translation.y() == 2.0);
}

TEST_CASE("bounding box packer uses AABB for non-rectangular pieces",
          "[packing][bounding-box]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 20,
          .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_l_shape(), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 1.0, 1.0), 2),
      },
      1);

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.bins.front().placements.size() == 1);
  REQUIRE(result.bins.front().placements.front().placement.piece_id == 1);
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{2U});
}

TEST_CASE("bounding box packer opens additional bins on overflow",
          "[packing][bounding-box]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 30,
          .polygon = make_rectangle(0.0, 0.0, 3.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 2.0, 2.0), 2),
      },
      2);

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 2);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins[0].placements.size() == 1);
  REQUIRE(result.bins[1].placements.size() == 1);
  REQUIRE(result.bins[0].bin_id == 30);
  REQUIRE(result.bins[1].bin_id == 31);
  REQUIRE(result.layout.placement_trace.size() == 2);
  REQUIRE(result.layout.placement_trace[1].opened_new_bin);
}

TEST_CASE("bounding box packer respects piece spacing",
          "[packing][bounding-box]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 40,
          .polygon = make_rectangle(0.0, 0.0, 5.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 2.0, 2.0), 2),
      },
      1);
  request.config.placement.part_clearance = 1.0;

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 2);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 3.0);
}

TEST_CASE("bounding box free rectangle heuristic respects piece spacing",
          "[packing][bounding-box][free-rectangle][spacing]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 41,
          .polygon = make_rectangle(0.0, 0.0, 5.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 2.0, 2.0), 2),
      },
      1);
  request.config.bounding_box.heuristic =
      BoundingBoxHeuristic::free_rectangle_backfill;
  request.config.placement.part_clearance = 1.0;

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 2);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[0].placement.translation.y() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.y() == 0.0);
}

TEST_CASE("bounding box skyline heuristic respects piece spacing",
          "[packing][bounding-box][skyline][spacing]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 42,
          .polygon = make_rectangle(0.0, 0.0, 5.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 2.0, 2.0), 2),
      },
      1);
  request.config.bounding_box.heuristic = BoundingBoxHeuristic::skyline;
  request.config.placement.part_clearance = 1.0;

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 2);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[0].placement.translation.y() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.y() == 0.0);
}

TEST_CASE("bounding box solve selects the best deterministic attempt and emits "
          "progress",
          "[solve][bounding-box][progress]") {
  auto request = make_solve_request(
      make_rectangle(0.0, 0.0, 4.0, 4.0),
      {
          PieceRequest{.piece_id = 1,
                       .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0)},
          PieceRequest{.piece_id = 2,
                       .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0)},
          PieceRequest{.piece_id = 3,
                       .polygon = make_rectangle(0.0, 0.0, 4.0, 2.0)},
      });
  request.execution.bounding_box.heuristic =
      BoundingBoxHeuristic::free_rectangle_backfill;
  request.execution.deterministic_attempts.max_attempts = 3;

  std::vector<ProgressSnapshot> snapshots;
  const auto result = shiny::nesting::solve(
      request,
      SolveControl{.on_progress = [&](const ProgressSnapshot &snapshot) {
        snapshots.push_back(snapshot);
      }});

  REQUIRE(result.ok());
  REQUIRE(result.value().layout.placement_trace.size() == 3U);
  REQUIRE(result.value().stop_reason == StopReason::completed);
  REQUIRE(snapshots.size() == 3U);
  REQUIRE(snapshots.front().sequence == 1U);
  REQUIRE(snapshots.back().sequence == 3U);
}

TEST_CASE(
    "bounding box solve obeys operation limits across deterministic attempts",
    "[solve][bounding-box][budget]") {
  auto request = make_solve_request(
      make_rectangle(0.0, 0.0, 8.0, 8.0),
      {
          PieceRequest{.piece_id = 1,
                       .polygon = make_rectangle(0.0, 0.0, 1.0, 4.0)},
          PieceRequest{.piece_id = 2,
                       .polygon = make_rectangle(0.0, 0.0, 3.0, 2.0)},
          PieceRequest{.piece_id = 3,
                       .polygon = make_rectangle(0.0, 0.0, 2.0, 1.0)},
      });
  request.execution.deterministic_attempts.max_attempts = 3;

  std::vector<ProgressSnapshot> snapshots;
  const auto result = shiny::nesting::solve(
      request, SolveControl{.on_progress =
                                [&](const ProgressSnapshot &snapshot) {
                                  snapshots.push_back(snapshot);
                                },
                            .operation_limit = 1});

  REQUIRE(result.ok());
  REQUIRE(result.value().stop_reason == StopReason::operation_limit_reached);
  REQUIRE(snapshots.size() == 1U);
}

TEST_CASE("bounding box packer respects exclusion zones",
          "[packing][bounding-box][exclusion-zones]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 45,
          .polygon = make_rectangle(0.0, 0.0, 5.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 2.0, 2.0), 2),
      },
      1);
  request.config.placement.exclusion_zones.push_back(
      shiny::nesting::place::BedExclusionZone{
          .zone_id = 7,
          .region = shiny::nesting::geom::Polygon(
              Ring{{2.0, 0.0}, {5.0, 0.0}, {5.0, 2.0}, {2.0, 2.0}}),
      });

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.bins.front().placements.size() == 1);
  REQUIRE(result.bins.front().placements.front().placement.piece_id == 1);
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{2U});
}

TEST_CASE("bounding box packer scopes exclusion zones to their bin",
          "[packing][bounding-box][exclusion-zones][bin-scope]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 70,
          .polygon = make_rectangle(0.0, 0.0, 2.0, 2.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 2.0, 2.0), 1),
      },
      2);
  request.config.placement.exclusion_zones.push_back(
      shiny::nesting::place::BedExclusionZone{
          .zone_id = 9,
          .bin_id = 70,
          .region = shiny::nesting::geom::Polygon(
              Ring{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}),
      });

  const auto result = packer.decode(request);

  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.bins.front().bin_id == 71);
  REQUIRE(result.bins.front().placements.size() == 1);
}

TEST_CASE("bounding box packer always starts from top-right origin corner",
          "[packing][bounding-box][start-corner]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 50,
          .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
          .geometry_revision = 1,
          .start_corner =
              shiny::nesting::place::PlacementStartCorner::top_right,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 6.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 4.0, 4.0), 2),
      },
      1);

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 2);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 4.0);
  REQUIRE(result.bins.front().placements[0].placement.translation.y() == 6.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.y() == 6.0);
}

TEST_CASE("bounding box packer evaluates bounded deterministic attempts",
          "[packing][bounding-box][attempts]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 60,
          .polygon = make_rectangle(0.0, 0.0, 8.0, 8.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 1.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 3.0, 2.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 2.0, 1.0), 3),
      },
      1);
  request.config.deterministic_attempts.max_attempts = 3;

  const auto attempts = packer.decode_attempts(request);

  REQUIRE(attempts.size() == 3);
  REQUIRE(attempts.front().layout.placement_trace.size() == 3);
  REQUIRE(attempts[1].layout.placement_trace.size() == 3);
  REQUIRE(attempts[2].layout.placement_trace.size() == 3);
  REQUIRE(attempts[1].layout.placement_trace.front().piece_id !=
          attempts.front().layout.placement_trace.front().piece_id);

  const auto single_result = packer.decode(request);
  REQUIRE(single_result.layout.placement_trace.size() ==
          attempts.front().layout.placement_trace.size());
  REQUIRE(single_result.layout.placement_trace.front().piece_id ==
          attempts.front().layout.placement_trace.front().piece_id);
}

TEST_CASE("bounding box packer reports each deterministic attempt",
          "[packing][bounding-box][attempts]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 70,
          .polygon = make_rectangle(0.0, 0.0, 8.0, 8.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 1.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 3.0, 2.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 2.0, 1.0), 3),
      },
      1);
  request.config.deterministic_attempts.max_attempts = 3;

  std::vector<std::size_t> observed_attempt_indices;
  std::vector<std::size_t> observed_placed_counts;
  const auto attempts = packer.decode_attempts(
      request, {},
      [&observed_attempt_indices, &observed_placed_counts](
          const std::size_t attempt_index,
          const shiny::nesting::pack::DecoderResult &result) {
        observed_attempt_indices.push_back(attempt_index);
        observed_placed_counts.push_back(result.layout.placement_trace.size());
      });

  REQUIRE(observed_attempt_indices == std::vector<std::size_t>{0U, 1U, 2U});
  REQUIRE(observed_placed_counts.size() == attempts.size());
  REQUIRE(observed_placed_counts.front() ==
          attempts.front().layout.placement_trace.size());
  REQUIRE(observed_placed_counts.back() ==
          attempts.back().layout.placement_trace.size());
}

TEST_CASE("bounding box skyline heuristic fills a vertical gap that shelf "
          "leaves unused",
          "[packing][bounding-box][skyline]") {
  BoundingBoxPacker packer;
  auto shelf_request = make_request(
      {
          .bin_id = 80,
          .polygon = make_rectangle(0.0, 0.0, 4.0, 4.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 3.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 1.0, 1.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 1.0, 1.0), 3),
          make_piece(4, make_rectangle(0.0, 0.0, 1.0, 1.0), 4),
          make_piece(5, make_rectangle(0.0, 0.0, 1.0, 1.0), 5),
      },
      1);
  auto skyline_request = shelf_request;
  skyline_request.config.bounding_box.heuristic = BoundingBoxHeuristic::skyline;

  const auto shelf_result = packer.decode(shelf_request);
  const auto skyline_result = packer.decode(skyline_request);

  REQUIRE(shelf_result.bins.size() == 1);
  REQUIRE(shelf_result.bins.front().placements.size() == 2);
  REQUIRE(shelf_result.layout.unplaced_piece_ids.size() == 3);

  REQUIRE(skyline_result.bins.size() == 1);
  REQUIRE(skyline_result.bins.front().placements.size() == 5);
  REQUIRE(skyline_result.layout.unplaced_piece_ids.empty());
  REQUIRE(skyline_result.bins.front().placements[1].placement.translation.x() ==
          3.0);
  REQUIRE(skyline_result.bins.front().placements[2].placement.translation.x() ==
          3.0);
  REQUIRE(skyline_result.bins.front().placements[3].placement.translation.x() ==
          3.0);
  REQUIRE(skyline_result.bins.front().placements[4].placement.translation.x() ==
          3.0);
}

TEST_CASE("bounding box skyline heuristic honors top-right start corners",
          "[packing][bounding-box][skyline][start-corner]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 81,
          .polygon = make_rectangle(0.0, 0.0, 4.0, 4.0),
          .geometry_revision = 1,
          .start_corner =
              shiny::nesting::place::PlacementStartCorner::top_right,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 3.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 1.0, 1.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 1.0, 1.0), 3),
          make_piece(4, make_rectangle(0.0, 0.0, 1.0, 1.0), 4),
          make_piece(5, make_rectangle(0.0, 0.0, 1.0, 1.0), 5),
      },
      1);
  request.config.bounding_box.heuristic = BoundingBoxHeuristic::skyline;

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 5);
  REQUIRE(result.bins.front().placements[0].placement.translation.x() == 1.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[1].placement.translation.y() == 3.0);
  REQUIRE(result.bins.front().placements[4].placement.translation.x() == 0.0);
  REQUIRE(result.bins.front().placements[4].placement.translation.y() == 0.0);
}

TEST_CASE(
    "bounding box free rectangle heuristic backfills the remaining column",
    "[packing][bounding-box][free-rectangle]") {
  BoundingBoxPacker packer;
  auto request = make_request(
      {
          .bin_id = 82,
          .polygon = make_rectangle(0.0, 0.0, 4.0, 4.0),
          .geometry_revision = 1,
      },
      {
          make_piece(1, make_rectangle(0.0, 0.0, 3.0, 4.0), 1),
          make_piece(2, make_rectangle(0.0, 0.0, 1.0, 1.0), 2),
          make_piece(3, make_rectangle(0.0, 0.0, 1.0, 1.0), 3),
          make_piece(4, make_rectangle(0.0, 0.0, 1.0, 1.0), 4),
          make_piece(5, make_rectangle(0.0, 0.0, 1.0, 1.0), 5),
      },
      1);
  request.config.bounding_box.heuristic =
      BoundingBoxHeuristic::free_rectangle_backfill;

  const auto result = packer.decode(request);

  REQUIRE(result.bins.size() == 1);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.bins.front().placements.size() == 5);
  REQUIRE(result.bins.front().placements[1].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[2].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[3].placement.translation.x() == 3.0);
  REQUIRE(result.bins.front().placements[4].placement.translation.x() == 3.0);
}
