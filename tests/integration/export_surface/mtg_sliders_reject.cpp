// MTG nesting matrix — slider rejection boundary tests.
//
// Complements mtg_sliders.cpp which covers acceptance at min/mid/max values.
// Each test here sets one parameter to an out-of-bounds value and asserts
// is_valid() == false. This verifies the validation gate is live for every
// parameter boundary, not just the happy path.

#include <catch2/catch_test_macros.hpp>

#include "fixtures/export_surface/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

// Returns a complete, valid BRKGA request that passes is_valid().
auto valid_brkga_request() -> NestingRequest {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = ProductionOptimizerKind::brkga;
  options.allow_part_overflow = true;
  options.production.population_size = 4U;
  options.production.elite_count = 1U;
  options.production.mutant_count = 1U;
  options.production.max_iterations = 2U;

  const auto fixture = load_mtg_fixture();
  return make_request(fixture, options);
}

// Returns a complete, valid request for testing irregular config bounds.
auto valid_sequential_request() -> NestingRequest {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.allow_part_overflow = true;

  const auto fixture = load_mtg_fixture();
  return make_request(fixture, options);
}

} // namespace

// ============================================================================
// BRKGA production config boundaries
// ============================================================================

TEST_CASE("mtg slider reject: production.population_size below minimum is invalid",
          "[mtg][nesting-matrix][sliders][reject][population-size]") {
  auto request = valid_brkga_request();
  REQUIRE(request.is_valid());

  // population_size must be >= 2; set to 1 (min - 1).
  request.execution.production.population_size = 1U;
  request.execution.production.elite_count = 1U;
  request.execution.production.mutant_count = 0U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: production.elite_count of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][elite-count]") {
  auto request = valid_brkga_request();
  REQUIRE(request.is_valid());

  // elite_count must be >= 1.
  request.execution.production.elite_count = 0U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: production.max_iterations of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][max-iterations]") {
  auto request = valid_brkga_request();
  REQUIRE(request.is_valid());

  // max_iterations must be >= 1.
  request.execution.production.max_iterations = 0U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: production.separator_worker_count of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][separator-worker-count]") {
  auto request = valid_brkga_request();
  REQUIRE(request.is_valid());

  // separator_worker_count must be >= 1.
  request.execution.production.separator_worker_count = 0U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: production.separator_max_iterations of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][separator-max-iterations]") {
  auto request = valid_brkga_request();
  REQUIRE(request.is_valid());

  // separator_max_iterations must be >= 1.
  request.execution.production.separator_max_iterations = 0U;
  REQUIRE_FALSE(request.is_valid());
}

// ============================================================================
// irregular config boundaries
// ============================================================================

TEST_CASE("mtg slider reject: irregular.max_candidate_points of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][max-candidate-points]") {
  auto request = valid_sequential_request();
  REQUIRE(request.is_valid());

  // max_candidate_points must be >= 1.
  request.execution.irregular.max_candidate_points = 0U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: irregular.max_candidate_points above 100000 is invalid",
          "[mtg][nesting-matrix][sliders][reject][max-candidate-points]") {
  auto request = valid_sequential_request();
  REQUIRE(request.is_valid());

  // max_candidate_points must be <= 100000.
  request.execution.irregular.max_candidate_points = 100001U;
  REQUIRE_FALSE(request.is_valid());
}

TEST_CASE("mtg slider reject: irregular.candidate_gaussian_sigma of zero is invalid",
          "[mtg][nesting-matrix][sliders][reject][candidate-gaussian-sigma]") {
  auto request = valid_sequential_request();
  REQUIRE(request.is_valid());

  // candidate_gaussian_sigma must be > 0.
  request.execution.irregular.candidate_gaussian_sigma = 0.0;
  REQUIRE_FALSE(request.is_valid());
}

// ============================================================================
// Top-level execution boundaries
// ============================================================================

TEST_CASE("mtg slider reject: negative part_spacing is invalid",
          "[mtg][nesting-matrix][sliders][reject][part-spacing]") {
  auto request = valid_sequential_request();
  REQUIRE(request.is_valid());

  // part_spacing must be >= 0.
  request.execution.part_spacing = -1.0;
  REQUIRE_FALSE(request.is_valid());
}
