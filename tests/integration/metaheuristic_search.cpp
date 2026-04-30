// MTG metaheuristic-search integration coverage.
//
// Keeps production-optimizer surface coverage here: a focused positive matrix
// over the requested engine options plus targeted depth checks for each
// optimizer family.
//
// Grain compatibility is intentionally excluded from this suite (see plan.md).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "geometry/transform.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "runtime/cancellation.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto rectangle(double min_x, double min_y, double max_x,
                             double max_y) -> geom::PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
              {min_x, min_y},
              {max_x, min_y},
              {max_x, max_y},
              {min_x, max_y},
          });
}

[[nodiscard]] auto frame(double min_x, double min_y, double max_x, double max_y,
                         double hole_min_x, double hole_min_y,
                         double hole_max_x, double hole_max_y)
    -> geom::PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles({
              {min_x, min_y},
              {max_x, min_y},
              {max_x, max_y},
              {min_x, max_y},
          }, {{
          {hole_min_x, hole_min_y},
          {hole_min_x, hole_max_y},
          {hole_max_x, hole_max_y},
          {hole_max_x, hole_min_y},
      }});
}

[[nodiscard]] auto four_rotations() -> geom::DiscreteRotationSet {
  return {{0.0, 90.0, 180.0, 270.0}};
}

[[nodiscard]] auto baseline_production_options(ProductionOptimizerKind kind)
    -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = kind;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;

  options.production.population_size = 4;
  options.production.elite_count = 1;
  options.production.mutant_count = 1;
  options.production.max_iterations = 2;
  options.production.polishing_passes = 0;

  options.simulated_annealing.max_refinements = 2;
  options.simulated_annealing.restart_count = 1;

  options.alns.max_refinements = 2;
  options.alns.destroy_min_count = 1;
  options.alns.destroy_max_count = 2;

  options.gdrr.max_refinements = 2;

  options.lahc.max_refinements = 2;
  options.lahc.history_length = 4;
  return options;
}

[[nodiscard]] auto observer_production_options(ProductionOptimizerKind kind)
    -> MtgRequestOptions {
  auto options = baseline_production_options(kind);
  options.production.population_size = 8;
  options.production.elite_count = 2;
  options.production.mutant_count = 2;
  options.production.max_iterations = 4;

  options.simulated_annealing.max_refinements = 8;
  options.simulated_annealing.restart_count = 1;

  options.alns.max_refinements = 8;
  options.gdrr.max_refinements = 8;
  options.lahc.max_refinements = 8;
  options.lahc.history_length = 4;
  return options;
}

[[nodiscard]] auto find_piece_index_by_id(const NestingRequest &request,
                                          std::uint32_t piece_id)
    -> std::size_t {
  for (std::size_t index = 0; index < request.pieces.size(); ++index) {
    if (request.pieces[index].piece_id == piece_id) {
      return index;
    }
  }
  return request.pieces.size();
}

[[nodiscard]] auto find_piece_placement(const NestingResult &result,
                                        std::uint32_t piece_id)
    -> const pack::PlacedPiece * {
  for (const auto &bin : result.layout.bins) {
    for (const auto &placed : bin.placements) {
      if (placed.placement.piece_id == piece_id) {
        return &placed;
      }
    }
  }
  return nullptr;
}

[[nodiscard]] auto
resolve_piece_rotation_degrees(const NestingRequest &request,
                               const pack::PlacedPiece &placed)
    -> std::optional<double> {
  const auto piece_it = std::find_if(
      request.pieces.begin(), request.pieces.end(), [&](const auto &piece) {
        return piece.piece_id == placed.placement.piece_id;
      });
  if (piece_it == request.pieces.end()) {
    return std::nullopt;
  }

  const auto &rotation_source = piece_it->allowed_rotations.has_value()
                                    ? *piece_it->allowed_rotations
                                    : request.execution.default_rotations;
  const auto rotation =
      geom::resolve_rotation(placed.placement.rotation_index, rotation_source);
  if (!rotation.has_value()) {
    return std::nullopt;
  }
  return rotation->degrees;
}

[[nodiscard]] auto count_total_placements(const NestingResult &result)
    -> std::size_t {
  std::size_t total = 0;
  for (const auto &bin : result.layout.bins) {
    total += bin.placements.size();
  }
  return total;
}

[[nodiscard]] auto count_bins_with_lifecycle(const NestingResult &result,
                                             pack::BinLifecycle lifecycle)
    -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(result.layout.bins.begin(), result.layout.bins.end(),
                    [lifecycle](const pack::LayoutBin &bin) {
                      return bin.identity.lifecycle == lifecycle;
                    }));
}

auto seed_priority_values(NestingRequest &request,
                          const PieceOrdering piece_ordering) -> void {
  if (piece_ordering != PieceOrdering::priority || request.pieces.empty()) {
    return;
  }

  request.pieces.front().priority = 100;
  for (std::size_t index = 1; index < request.pieces.size(); ++index) {
    request.pieces[index].priority = 0;
  }
}

[[nodiscard]] auto expanded_piece_ids_for_source(const NestingRequest &request,
                                                 std::uint32_t source_piece_id)
    -> std::vector<std::uint32_t> {
  const auto normalized = normalize_request(request);
  REQUIRE(normalized.has_value());

  std::vector<std::uint32_t> expanded_ids;
  for (const auto &expanded_piece : normalized.value().expanded_pieces) {
    if (expanded_piece.source_piece_id == source_piece_id) {
      expanded_ids.push_back(expanded_piece.expanded_piece_id);
    }
  }
  return expanded_ids;
}

[[nodiscard]] auto make_hole_request(ProductionOptimizerKind kind)
    -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production_optimizer = kind;
  request.execution.default_rotations = {{0.0}};
  request.execution.enable_part_in_part_placement = true;
  request.execution.production.population_size = 6;
  request.execution.production.elite_count = 1;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 2;
  request.execution.simulated_annealing.max_refinements = 4;
  request.execution.simulated_annealing.restart_count = 1;
  request.execution.alns.max_refinements = 4;
  request.execution.alns.destroy_min_count = 1;
  request.execution.alns.destroy_max_count = 2;
  request.execution.gdrr.max_refinements = 4;
  request.execution.lahc.max_refinements = 4;
  request.execution.lahc.history_length = 3;

  request.bins.push_back(BinRequest{
      .bin_id = 950,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 951,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 952,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  return request;
}

[[nodiscard]] auto make_overflow_request(ProductionOptimizerKind kind)
    -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production_optimizer = kind;
  request.execution.default_rotations = {{0.0}};
  request.execution.allow_part_overflow = true;
  request.execution.production.population_size = 4;
  request.execution.production.elite_count = 1;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 2;
  request.execution.production.polishing_passes = 0;
  request.execution.production.infeasible_pool_capacity = 1;
  request.execution.production.infeasible_rollback_after = 1;
  request.execution.simulated_annealing.max_refinements = 2;
  request.execution.simulated_annealing.restart_count = 1;
  request.execution.alns.max_refinements = 2;
  request.execution.alns.destroy_min_count = 1;
  request.execution.alns.destroy_max_count = 2;
  request.execution.gdrr.max_refinements = 2;
  request.execution.lahc.max_refinements = 2;
  request.execution.lahc.history_length = 4;
  request.execution.irregular.piece_ordering = PieceOrdering::input;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 2,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
  });
  return request;
}

[[nodiscard]] auto fast_production_control() -> SolveControl {
  return SolveControl{
      .operation_limit = 2,
      .random_seed = 31,
  };
}

} // namespace

TEST_CASE("metaheuristic-search constructive decodes inherit overflow bins",
          "[metaheuristic-search][overflow][regression]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);

  auto request = make_overflow_request(kind);
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().validation.valid);
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
  REQUIRE(count_total_placements(solved.value()) == 2U);
  REQUIRE(count_bins_with_lifecycle(solved.value(),
                                    pack::BinLifecycle::engine_overflow) == 1U);
}

TEST_CASE("mtg metaheuristic-search positive matrix places every part on the "
          "asymmetric synthetic fixture",
          "[mtg][nesting-matrix][metaheuristic-search][slow]") {
  // This breadth matrix intentionally stays on the small synthetic fixture so
  // it remains fast and optimizer-focused. Real-silhouette underplacement on
  // the MTG artwork is characterized separately in mtg_engine_bug_repros.cpp.
  const auto fixture = make_asymmetric_engine_surface_fixture();

  const auto kind = ProductionOptimizerKind::simulated_annealing;
  const double spacing_mm = GENERATE(0.0, 1.0);
  const auto candidate_strategy = GENERATE(
      CandidateStrategy::anchor_vertex, CandidateStrategy::nfp_perfect,
      CandidateStrategy::nfp_arrangement, CandidateStrategy::nfp_hybrid);
  const auto piece_ordering =
      GENERATE(PieceOrdering::input, PieceOrdering::largest_area_first,
               PieceOrdering::hull_diameter_first, PieceOrdering::priority);
  const bool enable_direct_overlap_check = GENERATE(false, true);
  const bool enable_backtracking = GENERATE(false, true);
  const bool enable_compaction = GENERATE(false, true);

  auto options = baseline_production_options(kind);
  options.part_spacing_mm = spacing_mm;
  options.irregular.candidate_strategy = candidate_strategy;
  options.irregular.piece_ordering = piece_ordering;
  options.irregular.enable_direct_overlap_check = enable_direct_overlap_check;
  options.irregular.enable_backtracking = enable_backtracking;
  options.irregular.max_backtrack_pieces = 1;
  options.irregular.enable_compaction = enable_compaction;
  options.irregular.compaction_passes = 2;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  seed_priority_values(request, piece_ordering);
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg metaheuristic-search optimizer breadth places every part on the "
          "asymmetric synthetic fixture",
          "[mtg][nesting-matrix][metaheuristic-search][slow]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  auto options = baseline_production_options(kind);

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg metaheuristic-search honors per-piece allowed_rotations",
          "[mtg][nesting-matrix][metaheuristic-search][rotations]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_production_options(kind);
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  request.pieces.front().allowed_rotations = geom::DiscreteRotationSet{{90.0}};
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.required_assignments = {{fixture.pieces.front().piece_id, kBed1Id}};
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);

  const auto *placed =
      find_piece_placement(solved.value(), fixture.pieces.front().piece_id);
  REQUIRE(placed != nullptr);
  const auto rotation = resolve_piece_rotation_degrees(request, *placed);
  REQUIRE(rotation.has_value());
  REQUIRE(std::fabs(*rotation - 90.0) <= 1e-9);
}

TEST_CASE(
    "mtg metaheuristic-search allow_mirror toggle preserves full placement",
    "[mtg][nesting-matrix][metaheuristic-search][mirror]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();
  auto options = baseline_production_options(kind);

  auto request_a = make_request(fixture, options);
  request_a.execution.default_rotations = four_rotations();
  REQUIRE(request_a.is_valid());

  auto request_b = make_request(fixture, options);
  request_b.execution.default_rotations = four_rotations();
  for (auto &piece : request_b.pieces) {
    piece.allow_mirror = true;
  }
  REQUIRE(request_b.is_valid());

  const auto solved_a = solve(request_a, fast_production_control());
  const auto solved_b = solve(request_b, fast_production_control());
  REQUIRE(solved_a.has_value());
  REQUIRE(solved_b.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request_a, options, solved_a.value(), expected);
  validate_layout(fixture, request_b, options, solved_b.value(), expected);
}

TEST_CASE("mtg metaheuristic-search expands quantity>1 instances",
          "[mtg][nesting-matrix][metaheuristic-search][quantity]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_production_options(kind);

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  const auto target_piece_id = fixture.pieces.back().piece_id;
  const auto target_index = find_piece_index_by_id(request, target_piece_id);
  REQUIRE(target_index < request.pieces.size());
  request.pieces[target_index].quantity = 3;
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
  REQUIRE(count_total_placements(solved.value()) == fixture.pieces.size() + 2U);

  const auto expanded_ids =
      expanded_piece_ids_for_source(request, target_piece_id);
  REQUIRE(expanded_ids.size() == 3U);

  std::size_t placed_target_instances = 0;
  for (const auto &bin : solved.value().layout.bins) {
    for (const auto &placed : bin.placements) {
      if (std::find(expanded_ids.begin(), expanded_ids.end(),
                    placed.placement.piece_id) != expanded_ids.end()) {
        ++placed_target_instances;
      }
    }
  }
  REQUIRE(placed_target_instances == expanded_ids.size());
}

TEST_CASE("mtg metaheuristic-search enforces allowed_bin_ids",
          "[mtg][nesting-matrix][metaheuristic-search][allowed-bin-ids]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();

  SECTION("non-conflicting allowed_bin_ids stay admissible") {
    const auto options = baseline_production_options(kind);
    auto request = make_request(fixture, options);
    request.execution.default_rotations = four_rotations();
    request.pieces[0].allowed_bin_ids = {kBed1Id};
    request.pieces[1].allowed_bin_ids = {kBed2Id};
    REQUIRE(request.is_valid());

    const auto solved = solve(request, fast_production_control());
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = fixture.pieces.size();
    expected.per_bed_counts = {{kBed1Id, 1}, {kBed2Id, 1}};
    expected.required_assignments = {
        {fixture.pieces[0].piece_id, kBed1Id},
        {fixture.pieces[1].piece_id, kBed2Id},
    };
    expected.require_allowed_bin_admissibility = true;
    expected.require_rotation_admissibility = true;
    validate_layout(fixture, request, options, solved.value(), expected);
  }

  SECTION("selected beds conflicting with allowed_bin_ids keep the piece "
          "unplaced") {
    auto fixture = make_asymmetric_engine_surface_fixture();
    const auto selected_bed_id = GENERATE(kBed1Id, kBed2Id);
    const auto other_bed_id = selected_bed_id == kBed1Id ? kBed2Id : kBed1Id;
    CAPTURE(selected_bed_id);
    fixture.pieces[0].source_bed_id = selected_bed_id;
    fixture.pieces[1].source_bed_id = selected_bed_id;
    auto options = baseline_production_options(kind);
    options.maintain_bed_assignment = true;
    options.allow_part_overflow = false;
    options.selected_bin_ids = {selected_bed_id};

    auto request = make_request(fixture, options);
    request.execution.default_rotations = four_rotations();
    request.pieces[0].allowed_bin_ids = {other_bed_id};
    request.pieces[1].allowed_bin_ids = {selected_bed_id};
    REQUIRE(request.is_valid());

    const auto solved = solve(request, fast_production_control());
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = 1;
    expected.per_bed_counts = {{selected_bed_id, 1}};
    expected.required_assignments = {
        {fixture.pieces[1].piece_id, selected_bed_id}};
    expected.require_allowed_bin_admissibility = true;
    validate_layout(fixture, request, options, solved.value(), expected);

    REQUIRE(find_piece_placement(solved.value(), fixture.pieces[0].piece_id) ==
            nullptr);
  }
}

TEST_CASE(
    "mtg metaheuristic-search maintain_bed_assignment pins pieces to source "
    "beds",
    "[mtg][nesting-matrix][metaheuristic-search][maintain-bed-assignment]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_production_options(kind);
  options.maintain_bed_assignment = true;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.per_bed_counts = {{kBed1Id, 1}, {kBed2Id, 1}};
  expected.required_assignments = {
      {fixture.pieces[0].piece_id, kBed1Id},
      {fixture.pieces[1].piece_id, kBed2Id},
  };
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE(
    "mtg metaheuristic-search observer and cancellation work for every "
    "optimizer",
    "[mtg][nesting-matrix][metaheuristic-search][observer][cancellation]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = load_mtg_fixture();

  const auto options = observer_production_options(kind);
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  runtime::CancellationSource cancel_source{};
  std::vector<ProgressSnapshot> observed;

  SolveControl control{};
  control.random_seed = 31;
  control.cancellation = cancel_source.token();
  control.on_progress = [&](const ProgressSnapshot &snapshot) {
    observed.push_back(snapshot);
    if (observed.size() >= 3U) {
      cancel_source.request_stop();
    }
  };

  const auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  REQUIRE(observed.size() >= 2U);
  for (std::size_t index = 1; index < observed.size(); ++index) {
  }

  REQUIRE(solved.value().stop_reason == StopReason::cancelled);
}

TEST_CASE("metaheuristic-search enable_part_in_part_placement fills the hole",
          "[mtg][nesting-matrix][metaheuristic-search][part-in-part]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto request = make_hole_request(kind);
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
  REQUIRE(solved.value().layout.bins.size() == 1U);
  REQUIRE(solved.value().layout.bins.front().placements.size() == 2U);

  const auto &placements = solved.value().layout.bins.front().placements;
  REQUIRE(placements[1].inside_hole);

  const auto overlap =
      poly::intersection_polygons(placements[0].polygon, placements[1].polygon);
  REQUIRE(overlap.empty());
}

TEST_CASE("mtg metaheuristic-search explore_concave_candidates still places "
          "every part on the asymmetric synthetic fixture",
          "[mtg][nesting-matrix][metaheuristic-search][concave-candidates]") {
  const auto kind =
      GENERATE(ProductionOptimizerKind::brkga,
               ProductionOptimizerKind::simulated_annealing,
               ProductionOptimizerKind::alns, ProductionOptimizerKind::gdrr,
               ProductionOptimizerKind::lahc);
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_production_options(kind);
  options.irregular.candidate_strategy = CandidateStrategy::nfp_hybrid;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  request.execution.explore_concave_candidates = true;
  REQUIRE(request.is_valid());

  const auto solved = solve(request, fast_production_control());
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}
