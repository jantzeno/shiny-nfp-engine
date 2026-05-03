// MTG nesting matrix — bounding-box heuristic matrix.
//
// Sweeps the cartesian product of (heuristic, spacing, placement policy,
// deterministic attempt count) for the bounding-box constructive packer and
// asserts every part places. Also asserts per-cell determinism by re-solving
// with identical inputs and comparing layout hashes.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "fixtures/export_surface/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

TEST_CASE("mtg bounding-box heuristics place every part",
          "[mtg][nesting-matrix][bounding-box][bb-heuristics]") {
  const auto fixture = load_mtg_fixture();

  const auto heuristic = GENERATE(
      pack::BoundingBoxHeuristic::shelf, pack::BoundingBoxHeuristic::skyline,
      pack::BoundingBoxHeuristic::free_rectangle_backfill);
  const auto spacing = GENERATE(0.0, 1.0);
  const auto policy = GENERATE(place::PlacementPolicy::bottom_left,
                               place::PlacementPolicy::minimum_length,
                               place::PlacementPolicy::maximum_utilization);
  const std::uint32_t attempts =
      GENERATE(std::uint32_t{1}, std::uint32_t{4}, std::uint32_t{8});

  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = heuristic;
  options.bounding_box_deterministic_attempts = attempts;
  options.placement_policy = policy;
  options.part_spacing_mm = spacing;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 12345;

  auto first = solve(request, control);
  REQUIRE(first.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, first.value(), expected);

  const auto first_b1 = hash_bin_placements(first.value(), kBed1Id);
  const auto first_b2 = hash_bin_placements(first.value(), kBed2Id);

  auto second = solve(request, control);
  REQUIRE(second.has_value());
  validate_layout(fixture, request, options, second.value(), expected);

  // Per-bed determinism: the previous XOR-combined hash would mask a
  // regression that swapped placements between beds.
  REQUIRE(hash_bin_placements(second.value(), kBed1Id) == first_b1);
  REQUIRE(hash_bin_placements(second.value(), kBed2Id) == first_b2);
}
