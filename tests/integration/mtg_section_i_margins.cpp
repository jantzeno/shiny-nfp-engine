// MTG nesting matrix — Section I: bed margins.
//
// Verifies that uniform and asymmetric per-side bed margins (modeled by
// shrinking the bed polygon in `make_request`) still admit full placement
// and visibly affect placed-piece coordinates.

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <string>

#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

enum class AlgorithmKind {
  bounding_box,
  sequential_backtrack,
  metaheuristic_search_brkga,
};

void apply_algorithm(MtgRequestOptions &options, AlgorithmKind kind) {
  switch (kind) {
    case AlgorithmKind::bounding_box:
      options.strategy = StrategyKind::bounding_box;
      options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
      break;
    case AlgorithmKind::sequential_backtrack:
      options.strategy = StrategyKind::sequential_backtrack;
      break;
    case AlgorithmKind::metaheuristic_search_brkga:
      options.strategy = StrategyKind::metaheuristic_search;
      options.production_optimizer = ProductionOptimizerKind::brkga;
      options.production.max_iterations = 2;
      options.production.population_size = 8;
      break;
  }
}

}  // namespace

TEST_CASE("mtg uniform bed margins still place every part",
          "[mtg][nesting-matrix][margins][uniform-margins][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto algorithm = GENERATE(AlgorithmKind::bounding_box,
                                  AlgorithmKind::sequential_backtrack,
                                  AlgorithmKind::metaheuristic_search_brkga);
  const double margin_mm = GENERATE(0.0, 5.0, 25.0);
  const double spacing_mm = GENERATE(0.0, 1.0);

  MtgRequestOptions options{};
  apply_algorithm(options, algorithm);
  options.selected_bin_ids = {};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.part_spacing_mm = spacing_mm;
  options.bed1_margins_mm = {margin_mm, margin_mm, margin_mm, margin_mm};
  options.bed2_margins_mm = {margin_mm, margin_mm, margin_mm, margin_mm};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 17;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg uniform bed margin actually changes the layout",
          "[mtg][nesting-matrix][margins][uniform-margins]") {
  const auto fixture = load_mtg_fixture();

  auto run_with_margin = [&](double margin) {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::bounding_box;
    options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
    options.selected_bin_ids = {};
    options.allow_part_overflow = true;
    options.maintain_bed_assignment = false;
    options.bed1_margins_mm = {margin, margin, margin, margin};
    options.bed2_margins_mm = {margin, margin, margin, margin};

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    SolveControl control{};
    control.random_seed = 17;

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    const auto h1 = hash_bin_placements(solved.value(), kBed1Id);
    const auto h2 = hash_bin_placements(solved.value(), kBed2Id);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  };

  const auto baseline_hash = run_with_margin(0.0);
  const auto inset_hash = run_with_margin(25.0);
  REQUIRE(baseline_hash != inset_hash);
}

TEST_CASE("mtg asymmetric bed margins respect each side",
          "[mtg][nesting-matrix][margins][asymmetric-margins][.][slow]") {
  const auto fixture = load_mtg_fixture();

  const auto algorithm = GENERATE(AlgorithmKind::bounding_box,
                                  AlgorithmKind::sequential_backtrack,
                                  AlgorithmKind::metaheuristic_search_brkga);

  MtgRequestOptions options{};
  apply_algorithm(options, algorithm);
  options.selected_bin_ids = {};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.bed1_margins_mm = {50.0, 10.0, 10.0, 50.0};
  options.bed2_margins_mm = {10.0, 50.0, 50.0, 10.0};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 17;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  const auto &result = solved.value();

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  validate_layout(fixture, request, options, result, expected);

  constexpr double tol = 1e-3;

  // bed1 margins: {left=50, right=10, top=10, bottom=50}
  const double bed1_left_min = options.bed1_margins_mm.left - tol;
  const double bed1_right_max =
      fixture.bed1.width_mm - options.bed1_margins_mm.right + tol;
  const double bed1_bottom_min = options.bed1_margins_mm.bottom - tol;
  const double bed1_top_max =
      fixture.bed1.height_mm - options.bed1_margins_mm.top + tol;

  // bed2 margins: {left=10, right=50, top=50, bottom=10}
  const double bed2_left_min = options.bed2_margins_mm.left - tol;
  const double bed2_right_max =
      fixture.bed2.width_mm - options.bed2_margins_mm.right + tol;
  const double bed2_bottom_min = options.bed2_margins_mm.bottom - tol;
  const double bed2_top_max =
      fixture.bed2.height_mm - options.bed2_margins_mm.top + tol;

  for (const auto &bin : result.layout.bins) {
    if (bin.bin_id == kBed1Id) {
      for (const auto &placed : bin.placements) {
        for (const auto &pt : placed.polygon.outer) {
          INFO("bed1 left margin (>=50) violated");
          REQUIRE(pt.x >= bed1_left_min);
          INFO("bed1 right margin (<=width-10) violated");
          REQUIRE(pt.x <= bed1_right_max);
          INFO("bed1 bottom margin (>=50) violated");
          REQUIRE(pt.y >= bed1_bottom_min);
          INFO("bed1 top margin (<=height-10) violated");
          REQUIRE(pt.y <= bed1_top_max);
        }
      }
    } else if (bin.bin_id == kBed2Id) {
      for (const auto &placed : bin.placements) {
        for (const auto &pt : placed.polygon.outer) {
          INFO("bed2 left margin (>=10) violated");
          REQUIRE(pt.x >= bed2_left_min);
          INFO("bed2 right margin (<=width-50) violated");
          REQUIRE(pt.x <= bed2_right_max);
          INFO("bed2 bottom margin (>=10) violated");
          REQUIRE(pt.y >= bed2_bottom_min);
          INFO("bed2 top margin (<=height-50) violated");
          REQUIRE(pt.y <= bed2_top_max);
        }
      }
    }
  }
}
