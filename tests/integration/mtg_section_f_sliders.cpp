// MTG nesting matrix — Section F: slider min/mid/max sweeps.
//
// Each slider gets two TEST_CASEs: one covering the min and mid values
// (default visibility) and one covering the max value (hidden behind the
// `[.][slow]` tag for production-optimizer sweeps so default runs stay
// bounded).

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

constexpr std::uint64_t kSeed = 73;

std::size_t bed1_piece_count(const MtgFixture &fixture) {
  return static_cast<std::size_t>(std::count_if(
      fixture.pieces.begin(), fixture.pieces.end(),
      [](const MtgPiece &p) { return p.source_bed_id == kBed1Id; }));
}

// ---- shared option scaffolds ----------------------------------------------

MtgRequestOptions bbox_all_beds_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.bounding_box_deterministic_attempts = 1;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  return options;
}

MtgRequestOptions irregular_constructive_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_constructive;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  return options;
}

MtgRequestOptions production_brkga_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  // Bounded defaults; sweeps below override individual fields.
  options.production.population_size = 8;
  options.production.max_generations = 4;
  return options;
}

MtgRequestOptions production_sa_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::simulated_annealing;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  options.production.population_size = 8;
  options.production.max_generations = 4;
  options.simulated_annealing.max_iterations = 8;
  return options;
}

MtgRequestOptions production_alns_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::alns;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  options.production.population_size = 8;
  options.production.max_generations = 4;
  options.alns.max_iterations = 8;
  options.alns.destroy_min_count = 1;
  options.alns.destroy_max_count = 2;
  return options;
}

MtgRequestOptions production_gdrr_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::gdrr;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  options.production.population_size = 8;
  options.production.max_generations = 4;
  options.gdrr.max_iterations = 8;
  return options;
}

MtgRequestOptions production_lahc_bed1_defaults() {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::irregular_production;
  options.production_optimizer = ProductionOptimizerKind::lahc;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {kBed1Id};
  options.production.population_size = 8;
  options.production.max_generations = 4;
  options.lahc.max_iterations = 8;
  options.lahc.history_length = 4;
  return options;
}

void run_and_assert_full_bed1(const MtgFixture &fixture,
                              const MtgRequestOptions &options) {
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());
  SolveControl control{};
  control.random_seed = kSeed;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  ExpectedOutcome expected{};
  expected.expected_placed_count = bed1_piece_count(fixture);
  validate_layout(fixture, request, options, solved.value(), expected);
}

void run_and_assert_full_all_beds(const MtgFixture &fixture,
                                  const MtgRequestOptions &options) {
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());
  SolveControl control{};
  control.random_seed = kSeed;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, solved.value(), expected);
}

} // namespace

// ============================================================================
// part_spacing_mm — bounding_box, all beds
// ============================================================================

TEST_CASE("mtg slider part_spacing_mm bounding-box (min/mid)",
          "[mtg][nesting-matrix][sliders][part-spacing]") {
  const double spacing = GENERATE(0.0, 1.0);
  const auto fixture = load_mtg_fixture();
  auto options = bbox_all_beds_defaults();
  options.part_spacing_mm = spacing;
  run_and_assert_full_all_beds(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm bounding-box (max)",
          "[mtg][nesting-matrix][sliders][part-spacing][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = bbox_all_beds_defaults();
  options.part_spacing_mm = 5.0;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// part_spacing_mm — irregular_constructive, bed1
// ============================================================================

TEST_CASE("mtg slider part_spacing_mm irregular-constructive (min/mid)",
          "[mtg][nesting-matrix][sliders][part-spacing][.][slow]") {
  const double spacing = GENERATE(0.0, 1.0);
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.part_spacing_mm = spacing;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm irregular-constructive (max)",
          "[mtg][nesting-matrix][sliders][part-spacing][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.part_spacing_mm = 5.0;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// irregular.max_candidate_points
// ============================================================================

TEST_CASE("mtg slider irregular.max_candidate_points (min/mid)",
          "[mtg][nesting-matrix][sliders][max-candidate-points][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{16}, std::uint32_t{256});
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.irregular.max_candidate_points = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider irregular.max_candidate_points (max)",
          "[mtg][nesting-matrix][sliders][max-candidate-points][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.irregular.max_candidate_points = 1024;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// irregular.candidate_gaussian_sigma
// ============================================================================

TEST_CASE("mtg slider irregular.candidate_gaussian_sigma (min/mid)",
          "[mtg][nesting-matrix][sliders][candidate-gaussian-sigma][.][slow]") {
  const double sigma = GENERATE(0.05, 0.5);
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.irregular.candidate_gaussian_sigma = sigma;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider irregular.candidate_gaussian_sigma (max)",
          "[mtg][nesting-matrix][sliders][candidate-gaussian-sigma][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = irregular_constructive_bed1_defaults();
  options.irregular.candidate_gaussian_sigma = 2.0;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// production.population_size — BRKGA
// ============================================================================

TEST_CASE("mtg slider production.population_size (min/mid)",
          "[mtg][nesting-matrix][sliders][population-size][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{4}, std::uint32_t{24});
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.population_size = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider production.population_size (max)",
          "[mtg][nesting-matrix][sliders][population-size][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.population_size = 64;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// production.elite_count — BRKGA
// ============================================================================

TEST_CASE("mtg slider production.elite_count (min/mid)",
          "[mtg][nesting-matrix][sliders][elite-count][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{6});
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.elite_count = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider production.elite_count (max)",
          "[mtg][nesting-matrix][sliders][elite-count][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.elite_count = 16;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// production.max_generations — BRKGA
// ============================================================================

TEST_CASE("mtg slider production.max_generations (min/mid)",
          "[mtg][nesting-matrix][sliders][max-generations][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{24});
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.max_generations = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider production.max_generations (max)",
          "[mtg][nesting-matrix][sliders][max-generations][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.max_generations = 64;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// production.polishing_passes — BRKGA
// ============================================================================

TEST_CASE("mtg slider production.polishing_passes (min/mid)",
          "[mtg][nesting-matrix][sliders][polishing-passes][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{0}, std::uint32_t{1});
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.polishing_passes = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider production.polishing_passes (max)",
          "[mtg][nesting-matrix][sliders][polishing-passes][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_brkga_bed1_defaults();
  options.production.polishing_passes = 4;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// simulated_annealing.max_iterations
// ============================================================================

TEST_CASE("mtg slider simulated_annealing.max_iterations (min/mid)",
          "[mtg][nesting-matrix][sliders][sa-max-iterations][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{48});
  const auto fixture = load_mtg_fixture();
  auto options = production_sa_bed1_defaults();
  options.simulated_annealing.max_iterations = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider simulated_annealing.max_iterations (max)",
          "[mtg][nesting-matrix][sliders][sa-max-iterations][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_sa_bed1_defaults();
  options.simulated_annealing.max_iterations = 128;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// alns.max_iterations
// ============================================================================

TEST_CASE("mtg slider alns.max_iterations (min/mid)",
          "[mtg][nesting-matrix][sliders][alns-max-iterations][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{48});
  const auto fixture = load_mtg_fixture();
  auto options = production_alns_bed1_defaults();
  options.alns.max_iterations = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider alns.max_iterations (max)",
          "[mtg][nesting-matrix][sliders][alns-max-iterations][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_alns_bed1_defaults();
  options.alns.max_iterations = 128;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// alns.destroy_min/max_count (paired)
// ============================================================================

TEST_CASE("mtg slider alns.destroy_min_max_count paired (min/mid)",
          "[mtg][nesting-matrix][sliders][alns-destroy-counts][.][slow]") {
  const auto pair = GENERATE(std::pair<std::uint32_t, std::uint32_t>{1, 1},
                             std::pair<std::uint32_t, std::uint32_t>{1, 3});
  const auto fixture = load_mtg_fixture();
  auto options = production_alns_bed1_defaults();
  options.alns.destroy_min_count = pair.first;
  options.alns.destroy_max_count = pair.second;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider alns.destroy_min_max_count paired (max)",
          "[mtg][nesting-matrix][sliders][alns-destroy-counts][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_alns_bed1_defaults();
  options.alns.destroy_min_count = 3;
  options.alns.destroy_max_count = 6;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// gdrr.max_iterations
// ============================================================================

TEST_CASE("mtg slider gdrr.max_iterations (min/mid)",
          "[mtg][nesting-matrix][sliders][gdrr-max-iterations][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{48});
  const auto fixture = load_mtg_fixture();
  auto options = production_gdrr_bed1_defaults();
  options.gdrr.max_iterations = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider gdrr.max_iterations (max)",
          "[mtg][nesting-matrix][sliders][gdrr-max-iterations][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_gdrr_bed1_defaults();
  options.gdrr.max_iterations = 128;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// lahc.max_iterations
// ============================================================================

TEST_CASE("mtg slider lahc.max_iterations (min/mid)",
          "[mtg][nesting-matrix][sliders][lahc-max-iterations][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{48});
  const auto fixture = load_mtg_fixture();
  auto options = production_lahc_bed1_defaults();
  options.lahc.max_iterations = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider lahc.max_iterations (max)",
          "[mtg][nesting-matrix][sliders][lahc-max-iterations][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_lahc_bed1_defaults();
  options.lahc.max_iterations = 128;
  run_and_assert_full_bed1(fixture, options);
}

// ============================================================================
// lahc.history_length
// ============================================================================

TEST_CASE("mtg slider lahc.history_length (min/mid)",
          "[mtg][nesting-matrix][sliders][lahc-history-length][.][slow]") {
  const std::uint32_t value = GENERATE(std::uint32_t{1}, std::uint32_t{12});
  const auto fixture = load_mtg_fixture();
  auto options = production_lahc_bed1_defaults();
  options.lahc.history_length = value;
  run_and_assert_full_bed1(fixture, options);
}

TEST_CASE("mtg slider lahc.history_length (max)",
          "[mtg][nesting-matrix][sliders][lahc-history-length][.][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = production_lahc_bed1_defaults();
  options.lahc.history_length = 32;
  run_and_assert_full_bed1(fixture, options);
}
