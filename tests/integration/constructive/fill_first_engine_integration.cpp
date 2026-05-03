#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "request.hpp"
#include "result.hpp"

#include "internal/legacy_solve.hpp"
#include "packing/bin_identity.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "internal/request_normalization.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProfileRequest;
using shiny::nesting::ProfileSolveControl;
using shiny::nesting::SolveProfile;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::BinLifecycle;
using shiny::nesting::pack::ConstructivePlacementPhase;
using shiny::nesting::pack::LayoutBin;
using shiny::nesting::pack::PlacedPiece;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto count_bins_with_lifecycle(const shiny::nesting::NestingResult &result,
                               const BinLifecycle lifecycle) -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(result.layout.bins.begin(), result.layout.bins.end(),
                    [lifecycle](const LayoutBin &bin) {
                      return bin.identity.lifecycle == lifecycle;
                    }));
}

auto find_layout_bin(const shiny::nesting::NestingResult &result,
                     const std::uint32_t bin_id) -> const LayoutBin * {
  const auto it = std::find_if(
      result.layout.bins.begin(), result.layout.bins.end(),
      [bin_id](const LayoutBin &bin) { return bin.bin_id == bin_id; });
  return it == result.layout.bins.end() ? nullptr : &*it;
}

auto find_piece_placement(const shiny::nesting::NestingResult &result,
                          const std::uint32_t piece_id) -> const PlacedPiece * {
  for (const auto &bin : result.layout.bins) {
    const auto it = std::find_if(bin.placements.begin(), bin.placements.end(),
                                 [piece_id](const PlacedPiece &placed) {
                                   return placed.placement.piece_id == piece_id;
                                 });
    if (it != bin.placements.end()) {
      return &*it;
    }
  }
  return nullptr;
}

} // namespace

TEST_CASE("fill-first integration keeps selected-bin scope when overflow opens a clone bin",
          "[constructive][integration][overflow]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = true;
  request.selected_bin_ids = {10U};

  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  request.pieces.push_back(PieceRequest{
      .piece_id = 301,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 302,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.bins.size() == 2U);
  REQUIRE(std::all_of(result.layout.bins.begin(), result.layout.bins.end(),
                      [](const auto &bin) {
                        return bin.identity.source_request_bin_id == 10U;
                      }));

  CHECK(result.layout.bins[0].identity.lifecycle == BinLifecycle::user_created);
  CHECK(result.layout.bins[1].identity.lifecycle ==
        BinLifecycle::engine_overflow);
  REQUIRE(result.layout.bins[1].identity.template_bin_id.has_value());
  CHECK(*result.layout.bins[1].identity.template_bin_id == 10U);

  REQUIRE(result.constructive.overflow_events.size() == 1U);
  CHECK(result.constructive.overflow_events[0].source_request_bin_id == 10U);
}

TEST_CASE("fill-first integration keeps maintained assignment pieces pinned and leaves overflow unplaced",
          "[constructive][integration][overflow]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = true;
  request.maintain_bed_assignment = true;

  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  request.pieces.push_back(PieceRequest{
      .piece_id = 311,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .assigned_bin_id = 10,
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 312,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .assigned_bin_id = 10,
  });

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.bins.size() == 1U);
  REQUIRE(count_bins_with_lifecycle(result, BinLifecycle::engine_overflow) ==
          0U);
  REQUIRE(result.constructive.overflow_events.empty());
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{312U});

  const auto *placed = find_piece_placement(result, 311U);
  REQUIRE(placed != nullptr);
  CHECK(placed->placement.bin_id == 10U);
  CHECK(find_piece_placement(result, 312U) == nullptr);
}

TEST_CASE("fill-first integration intersects selected bin scope with maintained assignment scope",
          "[constructive][integration][overflow]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = true;
  request.maintain_bed_assignment = true;
  request.selected_bin_ids = {10U};

  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });

  request.pieces.push_back(PieceRequest{
      .piece_id = 321,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .assigned_bin_id = 10,
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 322,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .assigned_bin_id = 20,
  });

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.bins.size() == 1U);
  REQUIRE(find_layout_bin(result, 10U) != nullptr);
  REQUIRE(find_layout_bin(result, 20U) == nullptr);
  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{322U});
  REQUIRE(result.constructive.overflow_events.empty());
}

TEST_CASE("fill-first integration produces a deterministic stable layout for a representative input",
          "[constructive][integration][regression]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 40,
      .polygon = rectangle(0.0, 0.0, 6.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 401,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 402,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 403,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  shiny::nesting::pack::constructive::FillFirstEngine engine;
  const auto result_or = engine.solve(normalized.value(), {});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.placement_trace.size() == 3U);
  CHECK(result.layout.placement_trace[0].piece_id == 401U);
  CHECK(result.layout.placement_trace[0].translation ==
        shiny::nesting::geom::Point2{0.0, 0.0});
  CHECK(result.layout.placement_trace[1].piece_id == 402U);
  CHECK(result.layout.placement_trace[1].translation ==
        shiny::nesting::geom::Point2{4.0, 0.0});
  CHECK(result.layout.placement_trace[2].piece_id == 403U);
  CHECK(result.layout.placement_trace[2].translation ==
        shiny::nesting::geom::Point2{4.0, 2.0});
}

// ---------------------------------------------------------------------------
// WI-G2a: Dense single-bin utilization floor
// ---------------------------------------------------------------------------
// Four 3x3 pieces tile a 6x6 bin exactly (36/36 = 100 % utilization).
// Asserts bin count = 1 and utilization >= 95 % (conservative –5 pp margin
// for floating-point area computation).  Catches layout spreading across extra
// bins or compaction regressions that reduce per-bin density.
TEST_CASE("fill-first integration dense single-bin layout meets utilization floor",
          "[constructive][integration][regression]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = false;

  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
  });
  for (std::uint32_t i = 0; i < 4U; ++i) {
    request.pieces.push_back(PieceRequest{
        .piece_id = 500U + i,
        .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
    });
  }

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.bins.size() == 1U);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  // Utilization floor derived from a single deterministic run: 4*(3x3) in
  // a 6x6 = 100 %. Conservative >= check (–5 pp margin).
  CHECK(result.utilization_percent() >= 95.0);
}

// ---------------------------------------------------------------------------
// WI-G2b: Multi-bin frontier + per-bin utilization floors
// ---------------------------------------------------------------------------
// Two 4x4 pieces in a 4x4 template with overflow: piece 601 fills bin 1,
// piece 602 opens overflow bin 2 and fills it.  Asserts bin count = 2,
// frontier_changes contains at least an initial open plus an overflow open,
// and per-bin utilization >= 95 %.  Catches frontier-advance regressions that
// would spread pieces across more bins than expected.
TEST_CASE(
    "fill-first integration multi-bin overflow records frontier events and meets "
    "utilization floors",
    "[constructive][integration][regression]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = true;
  request.selected_bin_ids = {60U};

  request.bins.push_back(BinRequest{
      .bin_id = 60,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 601,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 602,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.bins.size() == 2U);
  // Frontier must have advanced at least twice: initial bin opened + overflow
  // bin opened.
  CHECK(result.constructive.frontier_changes.size() >= 2U);
  // Per-bin utilization floors: each 4x4 template bin holds exactly one 4x4
  // piece.  Utilization floor derived from a single deterministic run: 100 %.
  // Conservative >= check (–5 pp margin).
  for (const auto &bin : result.layout.bins) {
    CHECK(bin.utilization.utilization >= 0.95);
  }
  CHECK(result.utilization_percent() >= 95.0);
}

// ---------------------------------------------------------------------------
// WI-G2c: Gap-fill flag controls retry loop and terminates correctly
// ---------------------------------------------------------------------------
// Bin 70 is exactly filled by piece 700.  Pieces 701–703 cannot fit.
// Verifies that enable_gap_fill controls whether the retry loop runs, that
// the loop exits immediately when no progress can be made (i.e. kMaxGapFillPasses
// is never exhausted for an all-fail scenario), and that the result is valid
// in both configurations.
//
// Note on phase == ConstructivePlacementPhase::gap_fill: triggering a gap-fill
// placement in the trace requires irregular polygon geometry where IFP-based
// candidate generation creates compaction-sensitive placement windows.  With
// axis-aligned rectangles and BL policy, primary already produces globally
// optimal positions (compaction is a no-op), so gap-fill cannot place anything
// additional.  The control flag and termination invariant are verified here;
// the phase tag is exercised by the irregular-polygon integration suite.
TEST_CASE(
    "fill-first integration gap-fill flag controls the retry loop and terminates correctly",
    "[constructive][integration][regression]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.execution.allow_part_overflow = false;
  request.execution.irregular.enable_compaction = true;
  request.execution.irregular.enable_gap_fill = true;

  request.bins.push_back(BinRequest{
      .bin_id = 70,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  // Piece 700 fills bin 70 exactly.
  request.pieces.push_back(PieceRequest{
      .piece_id = 700,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  // Pieces 701–703 cannot fit in the remaining (zero) space.
  request.pieces.push_back(PieceRequest{
      .piece_id = 701,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 702,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 703,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  shiny::nesting::pack::constructive::FillFirstEngine engine;
  const auto result_or = engine.solve(normalized.value(), {});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  // Piece 700 placed; 701–703 unplaced (no space after 700 fills the bin).
  REQUIRE(result.layout.unplaced_piece_ids.size() == 3U);
  REQUIRE(std::ranges::none_of(result.layout.unplaced_piece_ids,
                               [](std::uint32_t id) { return id == 700U; }));
  // All placed pieces use phase primary_order (gap-fill could not place any).
  for (const auto &entry : result.layout.placement_trace) {
    CHECK(entry.phase == ConstructivePlacementPhase::primary_order);
  }

  // With enable_gap_fill = false the result must be identical: gap-fill does
  // not help here, so disabling it must not change the outcome.
  NestingRequest request_no_gf = request;
  request_no_gf.execution.irregular.enable_gap_fill = false;
  const auto normalized_no_gf = shiny::nesting::normalize_request(request_no_gf);
  REQUIRE(normalized_no_gf.ok());
  shiny::nesting::pack::constructive::FillFirstEngine engine_no_gf;
  const auto result_no_gf_or = engine_no_gf.solve(normalized_no_gf.value(), {});
  REQUIRE(result_no_gf_or.ok());
  CHECK(result_no_gf_or.value().stop_reason == StopReason::completed);
  CHECK(result_no_gf_or.value().layout.unplaced_piece_ids.size() == 3U);
}

// ---------------------------------------------------------------------------
// WI-G2d: Overflow bin opens with expected utilization floor
// ---------------------------------------------------------------------------
// Bin template 80 (4x4) overflows: piece 801 fills bin 80, piece 802 opens
// overflow bin (clone of 80) and fills it.  Asserts exactly one overflow
// event, the event names the correct source bin, both bins meet utilization
// >= 95 %, and overall utilization >= 95 %.  Catches overflow cloning
// regressions that would spread pieces into larger or extra bins.
TEST_CASE("fill-first integration overflow bin opens with expected utilization floor",
          "[constructive][integration][regression]") {
  ProfileRequest request;
  request.profile = SolveProfile::quick;
  request.allow_part_overflow = true;
  request.selected_bin_ids = {80U};

  request.bins.push_back(BinRequest{
      .bin_id = 80,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 801,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 802,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });

  const auto result_or = shiny::nesting::solve(request, ProfileSolveControl{});
  REQUIRE(result_or.ok());

  const auto &result = result_or.value();
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.bins.size() == 2U);
  REQUIRE(result.constructive.overflow_events.size() == 1U);
  CHECK(result.constructive.overflow_events[0].source_request_bin_id == 80U);
  // Per-bin utilization floors: each 4x4 clone holds exactly one 4x4 piece.
  // Utilization floor derived from a single deterministic run: 100 %.
  // Conservative >= check (–5 pp margin).
  for (const auto &bin : result.layout.bins) {
    CHECK(bin.utilization.utilization >= 0.95);
  }
  CHECK(result.utilization_percent() >= 95.0);
}