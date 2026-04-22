// MTG sequential-backtrack integration coverage.
//
// Two TEST_CASEs:
//   * Slow / full matrix (4 x 4 x 2 x 2 x 2 = 128 sub-cases) hidden behind
//     `[.][slow]` so it is excluded from the default lane.
//   * Readiness sweep (1 x 4 x 2 x 2 x 2 = 32 sub-cases) bound to
//     anchor_vertex with both beds enabled.

#include <algorithm>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto count_bed1_pieces(const MtgFixture &fixture) -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      fixture.pieces.begin(), fixture.pieces.end(),
      [](const MtgPiece &p) { return p.source_bed_id == kBed1Id; }));
}

}  // namespace

TEST_CASE("mtg sequential-backtrack option matrix places every part",
          "[mtg][nesting-matrix][sequential-backtrack][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto candidate_strategy = GENERATE(CandidateStrategy::anchor_vertex,
                                           CandidateStrategy::nfp_perfect,
                                           CandidateStrategy::nfp_arrangement,
                                           CandidateStrategy::nfp_hybrid);
  const auto piece_ordering = GENERATE(PieceOrdering::input,
                                       PieceOrdering::largest_area_first,
                                       PieceOrdering::hull_diameter_first,
                                       PieceOrdering::priority);
  const bool enable_direct_overlap_check = GENERATE(false, true);
  const bool enable_backtracking = GENERATE(false, true);
  const bool merge_free_regions = GENERATE(false, true);

  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.part_spacing_mm = 0.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.selected_bin_ids = {kBed1Id};
  options.irregular.candidate_strategy = candidate_strategy;
  options.irregular.piece_ordering = piece_ordering;
  options.irregular.enable_direct_overlap_check = enable_direct_overlap_check;
  options.irregular.enable_backtracking = enable_backtracking;
  options.irregular.merge_free_regions = merge_free_regions;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 99;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  const std::size_t bed1_count = count_bed1_pieces(fixture);

  ExpectedOutcome expected{};
  expected.expected_placed_count = bed1_count;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg sequential-backtrack readiness sweep",
          "[mtg][nesting-matrix][sequential-backtrack][readiness][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto candidate_strategy = CandidateStrategy::anchor_vertex;
  const auto piece_ordering = GENERATE(PieceOrdering::input,
                                       PieceOrdering::largest_area_first,
                                       PieceOrdering::hull_diameter_first,
                                       PieceOrdering::priority);
  const bool enable_direct_overlap_check = GENERATE(false, true);
  const bool enable_backtracking = GENERATE(false, true);
  const bool merge_free_regions = GENERATE(false, true);

  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.part_spacing_mm = 0.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.selected_bin_ids = {};
  options.irregular.candidate_strategy = candidate_strategy;
  options.irregular.piece_ordering = piece_ordering;
  options.irregular.enable_direct_overlap_check = enable_direct_overlap_check;
  options.irregular.enable_backtracking = enable_backtracking;
  options.irregular.merge_free_regions = merge_free_regions;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 99;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, solved.value(), expected);
}
