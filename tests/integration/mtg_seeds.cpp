// MTG nesting matrix — seed options.
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

[[nodiscard]] auto rectangle(double width_mm, double height_mm)
    -> geom::PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
              {0.0, 0.0},
              {width_mm, 0.0},
              {width_mm, height_mm},
              {0.0, height_mm},
          });
}

[[nodiscard]] auto make_piece(std::uint32_t piece_id, std::string label,
                              std::uint32_t source_bed_id, double width_mm,
                              double height_mm) -> MtgPiece {
  return MtgPiece{
      .piece_id = piece_id,
      .label = std::move(label),
      .source_bed_id = source_bed_id,
      .polygon = rectangle(width_mm, height_mm),
      .width_mm = width_mm,
      .height_mm = height_mm,
  };
}

[[nodiscard]] auto make_seed_fixture() -> MtgFixture {
  auto fixture = make_asymmetric_engine_surface_fixture();
  fixture.source_path = std::filesystem::path{"synthetic://seed_fixture"};
  fixture.pieces.push_back(
      make_piece(3, "bed1-medium-rect", kBed1Id, 9.0, 6.0));
  fixture.pieces.push_back(make_piece(4, "bed2-square-a", kBed2Id, 6.0, 6.0));
  fixture.pieces.push_back(make_piece(5, "bed2-square-b", kBed2Id, 5.0, 5.0));
  fixture.pieces.push_back(make_piece(6, "bed1-strip", kBed1Id, 7.0, 4.0));
  return fixture;
}

auto layout_hashes(const NestingResult &result)
    -> std::array<std::uint64_t, 2> {
  return {
      hash_bin_placements(result, kBed1Id),
      hash_bin_placements(result, kBed2Id),
  };
}

enum class SeedAlgo { bounding_box, sequential_backtrack, brkga };

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
  case SeedAlgo::sequential_backtrack:
    options.strategy = StrategyKind::sequential_backtrack;
    break;
  case SeedAlgo::brkga:
    options.strategy = StrategyKind::metaheuristic_search;
    options.production_optimizer = ProductionOptimizerKind::brkga;
    options.production.population_size = 6;
    // ProductionSearchConfig::is_valid() requires elite + mutant < population
    // strictly; defaults are elite=6/mutant=4, so we must shrink them when we
    // shrink the population to keep the request valid.
    options.production.elite_count = 2;
    options.production.mutant_count = 1;
    options.production.max_iterations = 2;
    break;
  }
  return options;
}

MtgRequestOptions make_brkga_options() {
  return make_seed_options(SeedAlgo::brkga);
}

} // namespace

TEST_CASE("mtg fixed seed produces deterministic layouts",
          "[mtg][nesting-matrix][seeds][slow]") {
  const auto algo = GENERATE(SeedAlgo::bounding_box,
                             SeedAlgo::sequential_backtrack, SeedAlgo::brkga);

  const auto fixture = make_seed_fixture();
  auto options = make_seed_options(algo);

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 42;
  control.seed_mode = SeedProgressionMode::increment;
  control.operation_limit = 1;

  auto solved_a = solve(request, control);
  REQUIRE(solved_a.has_value());
  auto solved_b = solve(request, control);
  REQUIRE(solved_b.has_value());

  REQUIRE(solved_a.value().effective_seed == 42U);
  REQUIRE(solved_b.value().effective_seed == 42U);

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
          "[mtg][nesting-matrix][seeds][seed-mode][slow]") {
  const auto fixture = make_seed_fixture();
  const auto options = make_brkga_options();
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const auto mode =
      GENERATE(SeedProgressionMode::increment, SeedProgressionMode::decrement,
               SeedProgressionMode::random);

  const std::array<std::uint64_t, 2> trial_seeds{7, 11};
  std::array<std::array<std::uint64_t, 2>, 2> trial_hashes{};
  const std::size_t expected_count = fixture.pieces.size();

  for (std::size_t i = 0; i < trial_seeds.size(); ++i) {
    SolveControl control{};
    control.random_seed = trial_seeds[i];
    control.seed_mode = mode;
    control.operation_limit = mode == SeedProgressionMode::random ? 4 : 2;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    REQUIRE(solved.value().effective_seed == trial_seeds[i]);

    ExpectedOutcome expected{};
    expected.expected_placed_count = expected_count;
    validate_layout(fixture, request, options, solved.value(), expected);

    trial_hashes[i] = layout_hashes(solved.value());

    // increment / decrement are deterministic for a given (seed, mode).
    if (mode == SeedProgressionMode::increment ||
        mode == SeedProgressionMode::decrement) {
      auto solved_again = solve(request, control);
      REQUIRE(solved_again.has_value());
      REQUIRE(solved_again.value().effective_seed == trial_seeds[i]);
      REQUIRE(layout_hashes(solved_again.value()) == trial_hashes[i]);
    }
  }

  if (mode == SeedProgressionMode::random) {
    std::set<std::array<std::uint64_t, 2>> distinct(trial_hashes.begin(),
                                                    trial_hashes.end());
    REQUIRE(distinct.size() >= 2);
  }
}

TEST_CASE("mtg seed sweep is non-degenerate",
          "[mtg][nesting-matrix][seeds][seed-sweep][slow]") {
  const auto fixture = make_seed_fixture();
  const auto options = make_brkga_options();
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const std::size_t expected_count = fixture.pieces.size();
  std::set<std::array<std::uint64_t, 2>> distinct_hashes;

  for (std::uint64_t seed = 1; seed <= 6; ++seed) {
    SolveControl control{};
    control.random_seed = seed;
    control.seed_mode = SeedProgressionMode::increment;
    control.operation_limit = 2;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = expected_count;
    validate_layout(fixture, request, options, solved.value(), expected);

    distinct_hashes.insert(layout_hashes(solved.value()));
  }

  // Even on the reduced debug-friendly BRKGA budget, the sweep should still
  // produce more than one layout family on the small two-bed fixture.
  REQUIRE(distinct_hashes.size() >= 2);
}

TEST_CASE("mtg sequential multi-start reports accepted effective seed",
          "[mtg][nesting-matrix][seeds][sequential-backtrack][slow]") {
  const auto fixture = load_mtg_fixture();
  const auto rect = bed1_half_block_exclusion();
  const auto exclusion = make_rect_exclusion(
      99, kBed1Id, rect.min_x, rect.min_y, rect.max_x, rect.max_y);

  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.selected_bin_ids = {};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.part_spacing_mm = 0.0;
  options.bed1_exclusions = {exclusion};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 23;
  control.seed_mode = SeedProgressionMode::increment;
  control.operation_limit = 2;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.exclusions_to_check = {exclusion};
  validate_layout(fixture, request, options, solved.value(), expected);
  REQUIRE(solved.value().effective_seed == 23U);
}
