// MTG nesting matrix — Section B: per-bed start-corner override matrix.
//
// For each algorithm in {bounding_box, sequential_backtrack} this file
// exercises the 4x4 cartesian product of `place::PlacementStartCorner`
// values for bed1 x bed2 (16 combinations). Each cell asserts full
// placement. After all 16 combinations have run for an algorithm we also
// assert that the bed-1 placement hashes show at least 2 distinct values
// across the matrix — this catches dead-code in the per-bed corner
// override path.
//
// We deliberately avoid Catch2 GENERATE here because the diversity check
// must run after every cell has been observed; an explicit nested loop
// keeps the accumulator scoped to a single TEST_CASE invocation.

#include <algorithm>
#include <array>
#include <cstdint>
#include <set>
#include <unordered_map>

#include <catch2/catch_test_macros.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

constexpr std::array<place::PlacementStartCorner, 4> kAllCorners{
    place::PlacementStartCorner::bottom_left,
    place::PlacementStartCorner::bottom_right,
    place::PlacementStartCorner::top_left,
    place::PlacementStartCorner::top_right,
};

void apply_strategy_bounding_box(MtgRequestOptions &options) {
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.bounding_box_deterministic_attempts = 1;
}

void apply_strategy_sequential_backtrack(MtgRequestOptions &options) {
  options.strategy = StrategyKind::sequential_backtrack;
  options.irregular = {};
}

void run_corner_matrix(const MtgFixture &fixture,
                       void (*apply_strategy)(MtgRequestOptions &)) {
  std::set<std::uint64_t> bed1_hashes_all{};
  std::unordered_map<int /*bed1_corner*/, std::set<std::uint64_t>>
      bed1_hashes_by_b1corner{};

  for (const auto bed1_corner : kAllCorners) {
    for (const auto bed2_corner : kAllCorners) {
      CAPTURE(static_cast<int>(bed1_corner));
      CAPTURE(static_cast<int>(bed2_corner));

      MtgRequestOptions options{};
      apply_strategy(options);
      options.placement_policy = place::PlacementPolicy::bottom_left;
      options.part_spacing_mm = 0.0;
      options.maintain_bed_assignment = false;
      options.allow_part_overflow = true;
      options.selected_bin_ids = {};
      options.bed1_start_corner = bed1_corner;
      options.bed2_start_corner = bed2_corner;

      const auto request = make_request(fixture, options);
      REQUIRE(request.is_valid());

      SolveControl control{};
      control.random_seed = 7;

      auto solved = solve(request, control);
      REQUIRE(solved.has_value());

      ExpectedOutcome expected{};
      expected.expected_placed_count = kBaselinePieceCount;
      validate_layout(fixture, request, options, solved.value(), expected);

      const auto h = hash_bin_placements(solved.value(), kBed1Id);
      bed1_hashes_all.insert(h);
      bed1_hashes_by_b1corner[static_cast<int>(bed1_corner)].insert(h);
    }
  }

  // Strong diversity check: each of the 4 bed-1 corners should produce a
  // distinct bed-1 layout. If this trips on the sequential_backtrack
  // (slow) lane it is a potential engine signal that the per-bed corner
  // override is being ignored — keep the assertion tight so regressions
  // surface.
  REQUIRE(bed1_hashes_all.size() >= 4);

  // Cross-bed isolation check: for at least one bed-1 corner, sweeping
  // bed-2 corners must NOT change the bed-1 hash (i.e. the bed-2 corner
  // override should not bleed into bed-1 placement).
  INFO("expected at least one bed1_corner where bed2-corner sweep produces "
       "exactly 1 unique bed1 hash (no cross-bed bleed)");
  REQUIRE(std::any_of(bed1_hashes_by_b1corner.begin(),
                      bed1_hashes_by_b1corner.end(),
                      [](const auto &kv) { return kv.second.size() == 1; }));
}

}  // namespace

TEST_CASE("mtg section B start-corner override matrix (bounding_box)",
          "[mtg][nesting-matrix][start-corners][bounding-box]") {
  const auto fixture = load_mtg_fixture();
  run_corner_matrix(fixture, &apply_strategy_bounding_box);
}

TEST_CASE("mtg section B start-corner override matrix (sequential_backtrack)",
          "[mtg][nesting-matrix][start-corners][sequential-backtrack][.][slow]") {
  const auto fixture = load_mtg_fixture();
  run_corner_matrix(fixture, &apply_strategy_sequential_backtrack);
}
