// MTG nesting matrix — bin-assignment matrix.
//
// For each algorithm in {bounding_box, sequential_backtrack,
// metaheuristic_search-BRKGA} this file exercises the
// maintain_bed_assignment × allow_part_overflow truth table at two
// part-spacing values (0 mm, 1 mm) with all beds selected, then a
// single-bed variant per source bed. Every cell asserts full placement.
//
// Truth table (mirrors the engine-level documentation in
// `tests/support/mtg_fixture.hpp`):
//   maintain=false, overflow=false -> pinned to source bed
//   maintain=false, overflow=true  -> free movement
//   maintain=true,  overflow=false -> pinned (overflow ignored)
//   maintain=true,  overflow=true  -> pinned (overflow ignored)

#include <algorithm>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto count_pieces_on(const MtgFixture &fixture,
                                   std::uint32_t bed_id) -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(fixture.pieces.begin(), fixture.pieces.end(),
                    [bed_id](const MtgPiece &piece) {
                      return piece.source_bed_id == bed_id;
                    }));
}

void apply_strategy_bounding_box(MtgRequestOptions &options) {
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.bounding_box_deterministic_attempts = 1;
}

void apply_strategy_sequential_backtrack(MtgRequestOptions &options) {
  options.strategy = StrategyKind::sequential_backtrack;
  options.irregular = {};
}

void apply_strategy_metaheuristic_search_brkga(MtgRequestOptions &options) {
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  options.production.max_iterations = 4;
  options.production.population_size = 8;
}

void run_truth_table(const MtgFixture &fixture,
                     void (*apply_strategy)(MtgRequestOptions &)) {
  const bool maintain = GENERATE(false, true);
  const bool overflow = GENERATE(false, true);
  const double spacing_mm = GENERATE(0.0, 1.0);

  CAPTURE(maintain, overflow, spacing_mm);

  MtgRequestOptions options{};
  apply_strategy(options);
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.part_spacing_mm = spacing_mm;
  options.maintain_bed_assignment = maintain;
  options.allow_part_overflow = overflow;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 1234;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.expected_stop_reason = StopReason::completed;
  validate_layout(fixture, request, options, solved.value(), expected);
}

void run_single_bed(const MtgFixture &fixture,
                    void (*apply_strategy)(MtgRequestOptions &),
                    std::uint32_t selected_bed_id) {
  CAPTURE(selected_bed_id);

  MtgRequestOptions options{};
  apply_strategy(options);
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.part_spacing_mm = 0.0;
  // selected_bin_ids restricts which BINS the engine may place into, but
  // does NOT filter the input piece list. To make "single-bed" mean "only
  // this bed's pieces", pin every piece to its source bed (maintain=true)
  // and disable overflow. Pieces sourced from the unselected bed are then
  // pinned to a bed that is unavailable for placement, so they remain
  // unplaced — and we assert exactly the source-bed count is placed.
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;
  options.selected_bin_ids = {selected_bed_id};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 1234;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = count_pieces_on(fixture, selected_bed_id);
  expected.expected_stop_reason = StopReason::completed;
  validate_layout(fixture, request, options, solved.value(), expected);
}

}  // namespace

TEST_CASE("mtg bin-assignment matrix (bounding_box)",
          "[mtg][nesting-matrix][bin-assignment][bounding-box]") {
  const auto fixture = load_mtg_fixture();

  SECTION("truth table x spacing (all beds)") {
    run_truth_table(fixture, &apply_strategy_bounding_box);
  }
  SECTION("single bed: bed1 only") {
    run_single_bed(fixture, &apply_strategy_bounding_box, kBed1Id);
  }
  SECTION("single bed: bed2 only") {
    run_single_bed(fixture, &apply_strategy_bounding_box, kBed2Id);
  }
}

TEST_CASE("mtg bin-assignment matrix (sequential_backtrack)",
          "[mtg][nesting-matrix][bin-assignment][sequential-backtrack][slow]") {
  const auto fixture = load_mtg_fixture();

  SECTION("truth table x spacing (all beds)") {
    run_truth_table(fixture, &apply_strategy_sequential_backtrack);
  }
  SECTION("single bed: bed1 only") {
    run_single_bed(fixture, &apply_strategy_sequential_backtrack, kBed1Id);
  }
  SECTION("single bed: bed2 only") {
    run_single_bed(fixture, &apply_strategy_sequential_backtrack, kBed2Id);
  }
}

TEST_CASE("mtg bin-assignment matrix (metaheuristic_search BRKGA)",
          "[mtg][nesting-matrix][bin-assignment][metaheuristic-search][slow]") {
  const auto fixture = load_mtg_fixture();

  SECTION("truth table x spacing (all beds)") {
    run_truth_table(fixture, &apply_strategy_metaheuristic_search_brkga);
  }
  SECTION("single bed: bed1 only") {
    run_single_bed(fixture, &apply_strategy_metaheuristic_search_brkga,
                   kBed1Id);
  }
  SECTION("single bed: bed2 only") {
    run_single_bed(fixture, &apply_strategy_metaheuristic_search_brkga,
                   kBed2Id);
  }
}
