#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "geometry/queries/normalize.hpp"
#include "internal/request_normalization.hpp"
#include "packing/layout.hpp"
#include "placement/types.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"
#include "validation/layout_validation.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::LayoutValidationIssueKind;
using shiny::nesting::NestingRequest;
using shiny::nesting::NestingResult;
using shiny::nesting::NormalizedRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProfileRequest;
using shiny::nesting::SolveProfile;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::geom::RotationIndex;
using shiny::nesting::geom::normalize_polygon;
using shiny::nesting::pack::LayoutBin;
using shiny::nesting::pack::PlacedPiece;
using shiny::nesting::validation::validate_layout;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return normalize_polygon(PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}}));
}

// Returns a NestingRequest with one 10x10 bin and one 3x3 piece — enough
// for a minimal valid placement.
auto single_bin_single_piece_request() -> NestingRequest {
  NestingRequest req;
  req.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  req.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  return req;
}

// Returns a NestingRequest with one 10x10 bin and two 3x3 pieces.
auto single_bin_two_piece_request() -> NestingRequest {
  NestingRequest req;
  req.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  req.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  req.pieces.push_back(PieceRequest{
      .piece_id = 2,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  return req;
}

// Normalizes a NestingRequest and asserts success.
auto do_normalize(const NestingRequest &req) -> NormalizedRequest {
  auto result = shiny::nesting::normalize_request(req);
  REQUIRE(result.ok());
  return std::move(result).value();
}

// Builds a PlacedPiece at rotation 0 (index 0 in the default 4-rotation set =
// 0.0 degrees) with the given polygon as the placed footprint.
auto make_placed(const std::uint32_t piece_id, const std::uint32_t bin_id,
                 PolygonWithHoles polygon) -> PlacedPiece {
  PlacedPiece p;
  p.placement.piece_id = piece_id;
  p.placement.bin_id = bin_id;
  p.placement.rotation_index = RotationIndex{0};
  p.resolved_rotation.degrees = 0.0;
  p.polygon = std::move(polygon);
  return p;
}

// Builds a LayoutBin with the given bin_id and container polygon.
auto make_bin(const std::uint32_t bin_id, PolygonWithHoles container)
    -> LayoutBin {
  LayoutBin b;
  b.bin_id = bin_id;
  b.container = std::move(container);
  return b;
}

} // namespace

// ---------------------------------------------------------------------------
// B1 — isolation tests: validate_layout() called directly with hand-built
//       NormalizedRequest + NestingResult structures.
// ---------------------------------------------------------------------------

TEST_CASE("valid layout with one piece fully inside bin passes validation",
          "[layout-validation][unit]") {
  const auto norm = do_normalize(single_bin_single_piece_request());

  NestingResult result;
  auto &bin = result.layout.bins.emplace_back(
      make_bin(1, rectangle(0.0, 0.0, 10.0, 10.0)));
  bin.placements.push_back(
      make_placed(1, 1, rectangle(0.0, 0.0, 3.0, 3.0)));

  const auto report = validate_layout(norm, result);

  REQUIRE(report.valid);
  REQUIRE(report.issues.empty());
}

TEST_CASE("piece placed outside bin boundary reports outside_container issue",
          "[layout-validation][unit]") {
  const auto norm = do_normalize(single_bin_single_piece_request());

  NestingResult result;
  auto &bin = result.layout.bins.emplace_back(
      make_bin(1, rectangle(0.0, 0.0, 10.0, 10.0)));
  // Piece placed entirely outside the 10x10 bin — no intersection area.
  bin.placements.push_back(
      make_placed(1, 1, rectangle(20.0, 20.0, 23.0, 23.0)));

  const auto report = validate_layout(norm, result);

  REQUIRE_FALSE(report.valid);
  const bool has_outside_issue = std::any_of(
      report.issues.begin(), report.issues.end(), [](const auto &issue) {
        return issue.issue_kind ==
                   LayoutValidationIssueKind::outside_container &&
               issue.expanded_piece_id == 1U;
      });
  REQUIRE(has_outside_issue);
}

TEST_CASE("two pieces occupying the same footprint produce piece_overlap issue",
          "[layout-validation][unit]") {
  const auto norm = do_normalize(single_bin_two_piece_request());

  NestingResult result;
  auto &bin = result.layout.bins.emplace_back(
      make_bin(1, rectangle(0.0, 0.0, 10.0, 10.0)));
  // Both pieces at the same (0,0)-(3,3) position: fully overlapping.
  bin.placements.push_back(
      make_placed(1, 1, rectangle(0.0, 0.0, 3.0, 3.0)));
  bin.placements.push_back(
      make_placed(2, 1, rectangle(0.0, 0.0, 3.0, 3.0)));

  const auto report = validate_layout(norm, result);

  REQUIRE_FALSE(report.valid);
  const bool has_overlap_issue = std::any_of(
      report.issues.begin(), report.issues.end(), [](const auto &issue) {
        return issue.issue_kind == LayoutValidationIssueKind::piece_overlap;
      });
  REQUIRE(has_overlap_issue);
}

TEST_CASE("layout with all pieces listed as unplaced passes validation",
          "[layout-validation][unit]") {
  const auto norm = do_normalize(single_bin_single_piece_request());

  NestingResult result;
  // No bins, no placements — piece accounted for via unplaced_piece_ids.
  result.layout.unplaced_piece_ids.push_back(1);

  const auto report = validate_layout(norm, result);

  REQUIRE(report.valid);
  REQUIRE(report.issues.empty());
}

// ---------------------------------------------------------------------------
// B2 — integration test: solve() with allow_part_overflow=true produces a
//       result that passes layout validation, including overflow-bin pieces.
// ---------------------------------------------------------------------------

TEST_CASE("solve with allow_part_overflow=true produces a valid layout report",
          "[layout-validation][integration][overflow]") {
  ProfileRequest req;
  req.profile = SolveProfile::quick;
  req.allow_part_overflow = true;
  // Single 4x4 bin — only one 4x4 piece fits; the rest trigger overflow bins.
  req.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  req.pieces.push_back(
      PieceRequest{.piece_id = 1, .polygon = rectangle(0.0, 0.0, 4.0, 4.0)});
  req.pieces.push_back(
      PieceRequest{.piece_id = 2, .polygon = rectangle(0.0, 0.0, 4.0, 4.0)});
  req.pieces.push_back(
      PieceRequest{.piece_id = 3, .polygon = rectangle(0.0, 0.0, 4.0, 4.0)});

  const auto result = shiny::nesting::solve(req);

  REQUIRE(result.ok());
  // finalize_result() populates result->validation; overflow-bin placements
  // must not introduce outside_container or piece_overlap issues.
  REQUIRE(result.value().validation.valid);
}
