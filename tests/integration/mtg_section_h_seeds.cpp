// MTG nesting matrix — Section H: seed options.
//
// Validates determinism under fixed seeds, the behavior of the seed
// progression mode for sequential trials, and that varying the seed actually
// influences the resulting layout (i.e. the seed isn't dead-coded).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

std::size_t bed1_piece_count(const MtgFixture &fixture) {
  return static_cast<std::size_t>(std::count_if(
      fixture.pieces.begin(), fixture.pieces.end(),
      [](const MtgPiece &p) { return p.source_bed_id == kBed1Id; }));
}

enum class SeedAlgo { bounding_box, irregular_constructive, brkga };

MtgRequestOptions make_seed_options(SeedAlgo algo) {
  MtgRequestOptions options{};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  switch (algo) {
  case SeedAlgo::bounding_box:
    options.strategy = StrategyKind::bounding_box;
    options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
    options.bounding_box_deterministic_attempts = 1;
    break;
  case SeedAlgo::irregular_constructive:
    options.strategy = StrategyKind::irregular_constructive;
    break;
  case SeedAlgo::brkga:
    options.strategy = StrategyKind::irregular_production;
    options.production_optimizer = ProductionOptimizerKind::brkga;
    options.production.population_size = 8;
    // ProductionSearchConfig::is_valid() requires elite + mutant < population
    // strictly; defaults are elite=6/mutant=4, so we must shrink them when we
    // shrink the population to keep the request valid.
    options.production.elite_count = 2;
    options.production.mutant_count = 2;
    options.production.max_generations = 4;
    break;
  }
  return options;
}

MtgRequestOptions make_brkga_bed1_options() {
  auto options = make_seed_options(SeedAlgo::brkga);
  options.selected_bin_ids = {kBed1Id};
  return options;
}

} // namespace

TEST_CASE("mtg fixed seed produces deterministic layouts",
          "[mtg][nesting-matrix][seeds][.][slow]") {
  const auto algo = GENERATE(SeedAlgo::bounding_box,
                             SeedAlgo::irregular_constructive,
                             SeedAlgo::brkga);

  const auto fixture = load_mtg_fixture();
  auto options = make_seed_options(algo);
  // For irregular variants restrict to bed1 to keep run-time tight; the
  // bounding-box variant exercises both beds since it is cheap.
  if (algo != SeedAlgo::bounding_box) {
    options.selected_bin_ids = {kBed1Id};
  }

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 42;
  control.seed_mode = SeedProgressionMode::increment;

  auto solved_a = solve(request, control);
  REQUIRE(solved_a.has_value());
  auto solved_b = solve(request, control);
  REQUIRE(solved_b.has_value());

  REQUIRE(hash_bin_placements(solved_a.value(), kBed1Id) ==
          hash_bin_placements(solved_b.value(), kBed1Id));
  REQUIRE(hash_bin_placements(solved_a.value(), kBed2Id) ==
          hash_bin_placements(solved_b.value(), kBed2Id));

  // Guard against rare drift where the second call matches but a third
  // call diverges (e.g. lazily-initialized state that mutates after the
  // first repeat).
  auto solved_c = solve(request, control);
  REQUIRE(solved_c.has_value());
  REQUIRE(hash_bin_placements(solved_c.value(), kBed1Id) ==
          hash_bin_placements(solved_a.value(), kBed1Id));
  REQUIRE(hash_bin_placements(solved_c.value(), kBed2Id) ==
          hash_bin_placements(solved_a.value(), kBed2Id));
}

TEST_CASE("mtg seed_mode varies subsequent runs",
          "[mtg][nesting-matrix][seeds][seed-mode][.][slow]") {
  const auto fixture = load_mtg_fixture();
  const auto options = make_brkga_bed1_options();
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const auto mode = GENERATE(SeedProgressionMode::increment,
                             SeedProgressionMode::decrement,
                             SeedProgressionMode::random);

  const std::array<std::uint64_t, 3> trial_seeds{7, 11, 13};
  std::array<std::uint64_t, 3> trial_hashes{};
  const std::size_t expected_count = bed1_piece_count(fixture);

  for (std::size_t i = 0; i < trial_seeds.size(); ++i) {
    SolveControl control{};
    control.random_seed = trial_seeds[i];
    control.seed_mode = mode;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = expected_count;
    validate_layout(fixture, request, options, solved.value(), expected);

    trial_hashes[i] = hash_bin_placements(solved.value(), kBed1Id);

    // increment / decrement are deterministic for a given (seed, mode).
    if (mode == SeedProgressionMode::increment ||
        mode == SeedProgressionMode::decrement) {
      auto solved_again = solve(request, control);
      REQUIRE(solved_again.has_value());
      REQUIRE(hash_bin_placements(solved_again.value(), kBed1Id) ==
              trial_hashes[i]);
    }
  }

  if (mode == SeedProgressionMode::random) {
    std::set<std::uint64_t> distinct(trial_hashes.begin(), trial_hashes.end());
    REQUIRE(distinct.size() >= 2);
  }
}

TEST_CASE("mtg seed sweep is non-degenerate",
          "[mtg][nesting-matrix][seeds][seed-sweep][.][slow]") {
  const auto fixture = load_mtg_fixture();
  const auto options = make_brkga_bed1_options();
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const std::size_t expected_count = bed1_piece_count(fixture);
  std::set<std::uint64_t> distinct_hashes;

  for (std::uint64_t seed = 1; seed <= 8; ++seed) {
    SolveControl control{};
    control.random_seed = seed;
    control.seed_mode = SeedProgressionMode::increment;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = expected_count;
    validate_layout(fixture, request, options, solved.value(), expected);

    distinct_hashes.insert(hash_bin_placements(solved.value(), kBed1Id));
  }

  // 8 seeds should yield meaningfully diverse layouts; >= 4 distinct is a
  // strong non-degeneracy signal without being brittle.
  REQUIRE(distinct_hashes.size() >= 4);
}
