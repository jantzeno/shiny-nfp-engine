// MTG nesting matrix — slider min/mid/max sweeps.
//
// Each slider gets explicit min/mid/max TEST_CASEs. The max sweeps remain
// tagged `[slow]`, but they are visible in the default suite.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

/* --------------------------------------------------------------------------------
   Configuration min bounds
   --------------------------------------------------------------------------------

    population_size >= 2
    elite_count < population_size
    mutant_count < population_size
    elite_count + mutant_count < population_size

    population_size = 2
    elite_count = 1
    mutant_count = 0
    max_refinements = 1
    history_length = 1
    destroy_min_count = 1
    destroy_max_count = 1
    polishing_passes = 0
    candidate_gaussian_sigma > 0
    max_candidate_points >= 1 //
*/

constexpr std::size_t kSeed = 73;
constexpr std::size_t kMaxControlIterations = 4;
constexpr std::size_t kMaxIterations = 2;
constexpr std::size_t kPopulationSize = 4;
constexpr std::size_t kMaxRefinements = 2;
constexpr std::size_t kEliteCount = 1;
constexpr std::size_t kMutantCount = 1;
constexpr std::size_t kHistoryLength = 4;
constexpr std::size_t kDestroyMinCount = 1;
constexpr std::size_t kDestroyMaxCount = 2;
constexpr std::size_t kPolishingPasses = 1;
constexpr std::size_t kMaxCandidatePoints = 128;
constexpr double kMinCandidateGaussianSigma = 0.25;
constexpr std::size_t kBoundingBoxDeterministicAttempts = 1;
constexpr double kMinSpacingMm = 0.0;
constexpr double kMidSpacingMm = 10.0;
constexpr double kMaxSpacingMm = 20.0;

// ---- shared option scaffolds ----------------------------------------------

MtgRequestOptions baseline_options(bool irregular = false) {
  MtgRequestOptions options{};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;

  if (irregular) {
    options.production.population_size = kPopulationSize;
    options.production.elite_count = kEliteCount;
    options.production.mutant_count = kMutantCount;
    options.production.max_iterations = kMaxIterations;
  }

  return options;
}

MtgRequestOptions bbox_all_beds_defaults() {
  MtgRequestOptions options = baseline_options();
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.bounding_box_deterministic_attempts =
      kBoundingBoxDeterministicAttempts;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  return options;
}

MtgRequestOptions sequential_backtrack_defaults() {
  MtgRequestOptions options = baseline_options();
  options.strategy = StrategyKind::sequential_backtrack;
  return options;
}

MtgRequestOptions metaheuristic_search_brkga_defaults() {
  MtgRequestOptions options = baseline_options(true);
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  return options;
}

MtgRequestOptions metaheuristic_search_sa_defaults() {
  MtgRequestOptions options = baseline_options(true);
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::simulated_annealing;
  options.simulated_annealing.max_refinements = kMaxRefinements;
  return options;
}

MtgRequestOptions metaheuristic_search_alns_defaults() {
  MtgRequestOptions options = baseline_options(true);
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::alns;
  options.alns.max_refinements = kMaxRefinements;
  options.alns.destroy_min_count = kDestroyMinCount;
  options.alns.destroy_max_count = kDestroyMaxCount;
  return options;
}

MtgRequestOptions metaheuristic_search_gdrr_defaults() {
  MtgRequestOptions options = baseline_options(true);
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::gdrr;
  options.gdrr.max_refinements = kMaxRefinements;
  return options;
}

MtgRequestOptions metaheuristic_search_lahc_defaults() {
  MtgRequestOptions options = baseline_options(true);
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::lahc;
  options.lahc.max_refinements = kMaxRefinements;
  options.lahc.history_length = kHistoryLength;
  return options;
}

void run_and_assert_full_all_beds(const MtgFixture &fixture,
                                  const MtgRequestOptions &options) {
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());
  SolveControl control{};
  control.random_seed = kSeed;

  if (options.strategy == StrategyKind::sequential_backtrack ||
      (options.strategy == StrategyKind::metaheuristic_search &&
       options.production_optimizer == ProductionOptimizerKind::brkga)) {
    control.iteration_limit = kMaxControlIterations;
  }

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, solved.value(), expected);
}

} // namespace

// ============================================================================
// part_spacing_mm — bounding_box
// ============================================================================

TEST_CASE("mtg slider part_spacing_mm bounding-box (min)",
          "[mtg][nesting-matrix][sliders][part-spacing]") {
  const auto fixture = load_mtg_fixture();
  auto options = bbox_all_beds_defaults();
  options.part_spacing_mm = kMinSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm bounding-box (mid)",
          "[mtg][nesting-matrix][sliders][part-spacing]") {
  const auto fixture = load_mtg_fixture();
  auto options = bbox_all_beds_defaults();
  options.part_spacing_mm = kMidSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm bounding-box (max)",
          "[mtg][nesting-matrix][sliders][part-spacing][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = bbox_all_beds_defaults();
  options.part_spacing_mm = kMaxSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// part_spacing_mm — sequential_backtrack
// ============================================================================

TEST_CASE("mtg slider part_spacing_mm sequential-backtrack (min)",
          "[mtg][nesting-matrix][sliders][part-spacing][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = sequential_backtrack_defaults();
  options.part_spacing_mm = kMinSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm sequential-backtrack (mid)",
          "[mtg][nesting-matrix][sliders][part-spacing][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = sequential_backtrack_defaults();
  options.part_spacing_mm = kMidSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

TEST_CASE("mtg slider part_spacing_mm sequential-backtrack (max)",
          "[mtg][nesting-matrix][sliders][part-spacing][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = sequential_backtrack_defaults();
  options.part_spacing_mm = kMaxSpacingMm;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// irregular.max_candidate_points
// ============================================================================

TEST_CASE("mtg slider irregular.max_candidate_points",
          "[mtg][nesting-matrix][sliders][max-candidate-points][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = sequential_backtrack_defaults();
  options.irregular.max_candidate_points = kMaxCandidatePoints;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// irregular.candidate_gaussian_sigma
// ============================================================================

TEST_CASE("mtg slider irregular.candidate_gaussian_sigma",
          "[mtg][nesting-matrix][sliders][candidate-gaussian-sigma][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = sequential_backtrack_defaults();
  options.irregular.candidate_gaussian_sigma = kMinCandidateGaussianSigma;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// BRKGA
// ============================================================================

TEST_CASE("mtg slider production.population_size",
          "[mtg][nesting-matrix][sliders][population-size][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_brkga_defaults();
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// production.polishing_passes — BRKGA
// ============================================================================

TEST_CASE("mtg slider production.polishing_passes",
          "[mtg][nesting-matrix][sliders][polishing-passes][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_brkga_defaults();
  options.production.polishing_passes = kPolishingPasses;
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// simulated_annealing.max_refinements
// ============================================================================

TEST_CASE("mtg slider simulated_annealing.max_refinements",
          "[mtg][nesting-matrix][sliders][sa-max-iterations][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_sa_defaults();
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// alns.max_refinements
// ============================================================================

TEST_CASE("mtg slider alns.max_refinements",
          "[mtg][nesting-matrix][sliders][alns-max-iterations][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_alns_defaults();
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// alns.destroy_min/max_count (paired)
// ============================================================================

TEST_CASE("mtg slider alns.destroy_min_max_count paired",
          "[mtg][nesting-matrix][sliders][alns-destroy-counts][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_alns_defaults();
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// gdrr.max_refinements
// ============================================================================

TEST_CASE("mtg slider gdrr.max_refinements",
          "[mtg][nesting-matrix][sliders][gdrr-max-iterations][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_gdrr_defaults();
  run_and_assert_full_all_beds(fixture, options);
}

// ============================================================================
// lahc
// ============================================================================

TEST_CASE("mtg slider lahc.max_refinements",
          "[mtg][nesting-matrix][sliders][lahc-max-iterations][slow]") {
  const auto fixture = load_mtg_fixture();
  auto options = metaheuristic_search_lahc_defaults();
  run_and_assert_full_all_beds(fixture, options);
}