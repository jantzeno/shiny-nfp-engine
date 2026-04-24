// MTG sequential-backtrack integration coverage.
//
// Keeps constructive-specific surface coverage here: a focused positive matrix
// over the requested option breadth plus targeted depth checks for the
// irregular solver contract.
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

[[nodiscard]] auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> geom::PolygonWithHoles {
  return {
      .outer = {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
  };
}

[[nodiscard]] auto frame(double min_x, double min_y, double max_x, double max_y,
                         double hole_min_x, double hole_min_y, double hole_max_x,
                         double hole_max_y) -> geom::PolygonWithHoles {
  return {
      .outer = {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
      .holes = {{
          {hole_min_x, hole_min_y},
          {hole_min_x, hole_max_y},
          {hole_max_x, hole_max_y},
          {hole_max_x, hole_min_y},
      }},
  };
}

[[nodiscard]] auto four_rotations() -> geom::DiscreteRotationSet {
  return {{0.0, 90.0, 180.0, 270.0}};
}

[[nodiscard]] auto baseline_constructive_options() -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
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

[[nodiscard]] auto resolve_piece_rotation_degrees(
    const NestingRequest &request, const pack::PlacedPiece &placed)
    -> std::optional<double> {
  const auto piece_it =
      std::find_if(request.pieces.begin(), request.pieces.end(), [&](const auto &piece) {
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

[[nodiscard]] auto make_hole_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {{0.0}};
  request.execution.enable_part_in_part_placement = true;

  request.bins.push_back(BinRequest{
      .bin_id = 900,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 901,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 902,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  return request;
}

}  // namespace

TEST_CASE("mtg sequential-backtrack positive matrix places every part on the asymmetric synthetic fixture",
          "[mtg][nesting-matrix][sequential-backtrack][slow]") {
  // This breadth matrix intentionally stays on the small synthetic fixture so it
  // remains fast and shape-independent. Real-silhouette underplacement on the
  // MTG artwork is characterized separately in mtg_engine_bug_repros.cpp.
  const auto fixture = make_asymmetric_engine_surface_fixture();

  const double spacing_mm = GENERATE(0.0, 1.0);
  const auto candidate_strategy = GENERATE(CandidateStrategy::anchor_vertex,
                                           CandidateStrategy::nfp_perfect,
                                           CandidateStrategy::nfp_arrangement,
                                           CandidateStrategy::nfp_hybrid);
  const auto piece_ordering = GENERATE(PieceOrdering::input,
                                       PieceOrdering::largest_area_first,
                                       PieceOrdering::hull_diameter_first,
                                       PieceOrdering::priority);
  const bool enable_direct_overlap_check = GENERATE(false, true);
  const bool enable_backtracking = GENERATE(false, true);
  const bool enable_compaction = GENERATE(false, true);

  MtgRequestOptions options = baseline_constructive_options();
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

  const auto solved = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}

TEST_CASE("mtg sequential-backtrack honors per-piece allowed_rotations",
          "[mtg][nesting-matrix][sequential-backtrack][rotations]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_constructive_options();
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  request.pieces.front().allowed_rotations = geom::DiscreteRotationSet{{90.0}};
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
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

TEST_CASE("mtg sequential-backtrack allow_mirror toggle preserves full placement",
          "[mtg][nesting-matrix][sequential-backtrack][mirror]") {
  const auto fixture = load_mtg_fixture();
  const auto options = baseline_constructive_options();

  auto request_a = make_request(fixture, options);
  request_a.execution.default_rotations = four_rotations();
  REQUIRE(request_a.is_valid());

  auto request_b = make_request(fixture, options);
  request_b.execution.default_rotations = four_rotations();
  for (auto &piece : request_b.pieces) {
    piece.allow_mirror = true;
  }
  REQUIRE(request_b.is_valid());

  const auto solved_a = solve(request_a, SolveControl{.random_seed = 0});
  const auto solved_b = solve(request_b, SolveControl{.random_seed = 0});
  REQUIRE(solved_a.has_value());
  REQUIRE(solved_b.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request_a, options, solved_a.value(), expected);
  validate_layout(fixture, request_b, options, solved_b.value(), expected);
}

TEST_CASE("mtg sequential-backtrack expands quantity>1 instances",
          "[mtg][nesting-matrix][sequential-backtrack][quantity]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_constructive_options();

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  const auto target_piece_id = fixture.pieces.back().piece_id;
  const auto target_index = find_piece_index_by_id(request, target_piece_id);
  REQUIRE(target_index < request.pieces.size());
  request.pieces[target_index].quantity = 3;
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().stop_reason == StopReason::completed);
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
  REQUIRE(count_total_placements(solved.value()) == fixture.pieces.size() + 2U);

  const auto expanded_ids = expanded_piece_ids_for_source(request, target_piece_id);
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

TEST_CASE("mtg sequential-backtrack enforces allowed_bin_ids",
          "[mtg][nesting-matrix][sequential-backtrack][allowed-bin-ids]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  SECTION("non-conflicting allowed_bin_ids stay admissible") {
    const auto options = baseline_constructive_options();
    auto request = make_request(fixture, options);
    request.execution.default_rotations = four_rotations();
    request.pieces[0].allowed_bin_ids = {kBed1Id};
    request.pieces[1].allowed_bin_ids = {kBed2Id};
    REQUIRE(request.is_valid());

    const auto solved = solve(request, SolveControl{.random_seed = 0});
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

  SECTION("selected beds conflicting with allowed_bin_ids keep the piece unplaced") {
    auto fixture = make_asymmetric_engine_surface_fixture();
    const auto selected_bed_id = GENERATE(kBed1Id, kBed2Id);
    const auto other_bed_id = selected_bed_id == kBed1Id ? kBed2Id : kBed1Id;
    CAPTURE(selected_bed_id);
    fixture.pieces[0].source_bed_id = selected_bed_id;
    fixture.pieces[1].source_bed_id = selected_bed_id;
    auto options = baseline_constructive_options();
    options.maintain_bed_assignment = true;
    options.allow_part_overflow = false;
    options.selected_bin_ids = {selected_bed_id};

    auto request = make_request(fixture, options);
    request.execution.default_rotations = four_rotations();
    request.pieces[0].allowed_bin_ids = {other_bed_id};
    request.pieces[1].allowed_bin_ids = {selected_bed_id};
    REQUIRE(request.is_valid());

    const auto solved = solve(request, SolveControl{.random_seed = 0});
    REQUIRE(solved.has_value());

    ExpectedOutcome expected{};
    expected.expected_placed_count = 1;
    expected.per_bed_counts = {{selected_bed_id, 1}};
    expected.required_assignments = {{fixture.pieces[1].piece_id, selected_bed_id}};
    expected.require_allowed_bin_admissibility = true;
    validate_layout(fixture, request, options, solved.value(), expected);

    REQUIRE(find_piece_placement(solved.value(), fixture.pieces[0].piece_id) == nullptr);
  }
}

TEST_CASE("mtg sequential-backtrack maintain_bed_assignment pins pieces to source beds",
          "[mtg][nesting-matrix][sequential-backtrack][maintain-bed-assignment]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_constructive_options();
  options.maintain_bed_assignment = true;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
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

TEST_CASE("mtg sequential-backtrack multi-start observer keeps the best full layout",
          "[mtg][nesting-matrix][sequential-backtrack][observer][cancellation]") {
  const auto fixture = load_mtg_fixture();

  auto options = baseline_constructive_options();
  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  runtime::CancellationSource cancel_source{};
  std::vector<ProgressSnapshot> observed;
  bool saw_empty_search_pulse = false;
  bool saw_full_layout_snapshot = false;
  std::size_t best_observed_layout = 0;

  SolveControl control{};
  control.iteration_limit = 4;
  control.random_seed = 99;
  control.cancellation = cancel_source.token();
  control.on_progress = [&](const ProgressSnapshot &snapshot) {
    observed.push_back(snapshot);

    const std::size_t observed_placed = snapshot.layout.placement_trace.size();
    best_observed_layout = std::max(best_observed_layout, observed_placed);
    saw_full_layout_snapshot |= observed_placed == fixture.pieces.size();

    if (saw_full_layout_snapshot && snapshot.sequence >= 2U &&
        snapshot.layout.placement_trace.empty()) {
      saw_empty_search_pulse = true;
      cancel_source.request_stop();
    }
  };

  const auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;
  expected.expected_stop_reason = StopReason::cancelled;
  validate_layout(fixture, request, options, solved.value(), expected);

  REQUIRE(observed.size() >= 2U);
  REQUIRE(saw_empty_search_pulse);
  REQUIRE(saw_full_layout_snapshot);
  REQUIRE(best_observed_layout == kBaselinePieceCount);
  REQUIRE(solved.value().layout.placement_trace.size() == best_observed_layout);
  REQUIRE(observed.back().layout.placement_trace.size() <
          solved.value().layout.placement_trace.size());
  REQUIRE(solved.value().budget.cancellation_requested);
  REQUIRE(solved.value().budget.iterations_completed >= 2U);
}

TEST_CASE("sequential-backtrack enable_part_in_part_placement fills the hole",
          "[mtg][nesting-matrix][sequential-backtrack][part-in-part]") {
  const auto request = make_hole_request();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().stop_reason == StopReason::completed);
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
  REQUIRE(solved.value().layout.bins.size() == 1U);
  REQUIRE(solved.value().layout.bins.front().placements.size() == 2U);

  const auto &placements = solved.value().layout.bins.front().placements;
  REQUIRE(placements[1].inside_hole);

  const auto overlap = poly::intersection_polygons(placements[0].polygon,
                                                   placements[1].polygon);
  REQUIRE(overlap.empty());
}

TEST_CASE("mtg sequential-backtrack explore_concave_candidates still places every part on the asymmetric synthetic fixture",
          "[mtg][nesting-matrix][sequential-backtrack][concave-candidates]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  auto options = baseline_constructive_options();
  options.irregular.candidate_strategy = CandidateStrategy::nfp_hybrid;

  auto request = make_request(fixture, options);
  request.execution.default_rotations = four_rotations();
  request.execution.explore_concave_candidates = true;
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);
}
