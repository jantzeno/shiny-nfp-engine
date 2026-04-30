// MTG nesting matrix tests.
//
// This binary exercises the engine's `solve(...)` surface against the
// `mtg_test.svg` fixture (2 part beds + 18 parts). Each section maps to a
// part of `plan.md` (Sections A through J).
//
// The fixture is loaded once per TEST_CASE via `load_mtg_fixture()`. Common
// invariants live in `tests/support/mtg_fixture.{hpp,cpp}` and are invoked
// through `validate_layout(...)`.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

TEST_CASE("mtg fixture loads with 2 beds and 18 parts",
          "[mtg][nesting-matrix][fixture]") {
  const auto fixture = load_mtg_fixture();
  REQUIRE(fixture.bed1.bed_id == kBed1Id);
  REQUIRE(fixture.bed2.bed_id == kBed2Id);
  REQUIRE(fixture.bed1.width_mm > 1000.0);
  REQUIRE(fixture.bed1.height_mm > 1000.0);
  REQUIRE(fixture.pieces.size() == kBaselinePieceCount);

  std::size_t bed1_pieces = 0;
  std::size_t bed2_pieces = 0;
  for (const auto &piece : fixture.pieces) {
    REQUIRE((piece.source_bed_id == kBed1Id || piece.source_bed_id == kBed2Id));
    REQUIRE(piece.width_mm > 0.0);
    REQUIRE(piece.height_mm > 0.0);
    if (piece.source_bed_id == kBed1Id) {
      ++bed1_pieces;
    } else {
      ++bed2_pieces;
    }
  }
  // The SVG layout puts the majority of parts on bed1; both beds host at
  // least one part.
  REQUIRE(bed1_pieces > 0);
  REQUIRE(bed2_pieces > 0);
  REQUIRE(bed1_pieces + bed2_pieces == kBaselinePieceCount);
}

TEST_CASE(
    "mtg actual-polygon fixture preserves rectangle-fixture metadata and AABBs",
    "[mtg][nesting-matrix][fixture][actual-polygons]") {
  const auto rectangle_fixture = load_mtg_fixture();
  const auto actual_fixture = load_mtg_fixture_with_actual_polygons();

  REQUIRE(actual_fixture.bed1.bed_id == rectangle_fixture.bed1.bed_id);
  REQUIRE(actual_fixture.bed2.bed_id == rectangle_fixture.bed2.bed_id);
  REQUIRE(actual_fixture.bed1.width_mm ==
          Catch::Approx(rectangle_fixture.bed1.width_mm).margin(1e-3));
  REQUIRE(actual_fixture.bed1.height_mm ==
          Catch::Approx(rectangle_fixture.bed1.height_mm).margin(1e-3));
  REQUIRE(actual_fixture.bed2.width_mm ==
          Catch::Approx(rectangle_fixture.bed2.width_mm).margin(1e-3));
  REQUIRE(actual_fixture.bed2.height_mm ==
          Catch::Approx(rectangle_fixture.bed2.height_mm).margin(1e-3));
  REQUIRE(actual_fixture.pieces.size() == rectangle_fixture.pieces.size());

  bool saw_non_rectangular_outline = false;
  for (std::size_t index = 0; index < rectangle_fixture.pieces.size();
       ++index) {
    const auto &rectangle_piece = rectangle_fixture.pieces[index];
    const auto &actual_piece = actual_fixture.pieces[index];

    REQUIRE(actual_piece.piece_id == rectangle_piece.piece_id);
    REQUIRE(actual_piece.label == rectangle_piece.label);
    REQUIRE(actual_piece.source_bed_id == rectangle_piece.source_bed_id);
    REQUIRE(actual_piece.width_mm ==
            Catch::Approx(rectangle_piece.width_mm).margin(1e-3));
    REQUIRE(actual_piece.height_mm ==
            Catch::Approx(rectangle_piece.height_mm).margin(1e-3));
    REQUIRE_FALSE(actual_piece.polygon.outer().empty());
    REQUIRE(actual_piece.polygon.holes().empty());
    if (actual_piece.polygon.outer().size() != 4U) {
      saw_non_rectangular_outline = true;
    }
  }

  REQUIRE(saw_non_rectangular_outline);
}

TEST_CASE("mtg baseline bounding-box solve places every part",
          "[mtg][nesting-matrix][bounding-box]") {
  const auto fixture = load_mtg_fixture();
  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.bounding_box_deterministic_attempts = 1;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.part_spacing_mm = 0.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 42;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.expected_stop_reason = StopReason::completed;
  validate_layout(fixture, request, options, result, expected);
}

TEST_CASE("mtg baseline sequential-backtrack solve places every part",
          "[mtg][nesting-matrix][sequential-backtrack][baseline]") {
  const auto fixture = load_mtg_fixture();
  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.part_spacing_mm = 0.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  // Keep this in the fast lane as a single constructive pass. A non-zero seed
  // on solve() activates the multi-start irregular wrapper instead.
  control.random_seed = 0;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.expected_stop_reason = StopReason::completed;
  validate_layout(fixture, request, options, result, expected);
}
