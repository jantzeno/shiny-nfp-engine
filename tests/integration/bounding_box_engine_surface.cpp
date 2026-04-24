// MTG bounding-box engine-surface coverage.
//
// Exercises low-level NestingRequest / SolveControl knobs against the
// bounding-box strategy: rotation sets, mirror, quantity, priority,
// ProgressObserver, and the part-in-part / explore-concave execution flags.
//
// This file intentionally stays in the fast MTG regression lane
// (`[mtg]~[slow]~[broken]`). Geometry-sensitive contract checks should use
// small focused fixtures here instead of relying only on the broad rectangle
// MTG matrix.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "geometry/transform.hpp"
#include "observer.hpp"
#include "solve.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto baseline_bb_options() -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {};
  return options;
}

[[nodiscard]] auto count_total_placements(const NestingResult &result)
    -> std::size_t {
  std::size_t total = 0;
  for (const auto &bin : result.layout.bins) {
    total += bin.placements.size();
  }
  return total;
}

[[nodiscard]] auto find_piece_index_by_id(const NestingRequest &request,
                                          std::uint32_t piece_id)
    -> std::size_t {
  for (std::size_t i = 0; i < request.pieces.size(); ++i) {
    if (request.pieces[i].piece_id == piece_id) {
      return i;
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

}  // namespace

TEST_CASE("focused asymmetric fixture makes default_rotations directly observable",
          "[mtg][nesting-matrix][engine-surface][rotations]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();
  auto options = baseline_bb_options();
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;

  // Solve A — only 0°. The wide piece cannot fit its source bed.
  auto request_a = make_request(fixture, options);
  request_a.execution.default_rotations = geom::DiscreteRotationSet{{0.0}};
  REQUIRE(request_a.is_valid());
  SolveControl control_a{};
  control_a.random_seed = 11;
  auto result_a = solve(request_a, control_a);
  REQUIRE(result_a.has_value());

  ExpectedOutcome expected_a{};
  expected_a.expected_placed_count = 1;
  expected_a.require_rotation_admissibility = true;
  validate_layout(fixture, request_a, options, result_a.value(), expected_a);
  REQUIRE(find_piece_placement(result_a.value(), fixture.pieces.front().piece_id) ==
          nullptr);

  // Solve B — 0° + 90°. The same piece now fits its source bed only when
  // rotated, so the contract is visible without relying on hash drift.
  auto request_b = make_request(fixture, options);
  request_b.execution.default_rotations =
      geom::DiscreteRotationSet{{0.0, 90.0}};
  REQUIRE(request_b.is_valid());
  SolveControl control_b{};
  control_b.random_seed = 11;
  auto result_b = solve(request_b, control_b);
  REQUIRE(result_b.has_value());

  ExpectedOutcome expected_b{};
  expected_b.expected_placed_count = fixture.pieces.size();
  expected_b.required_assignments = {{fixture.pieces.front().piece_id, kBed1Id}};
  expected_b.require_rotation_admissibility = true;
  validate_layout(fixture, request_b, options, result_b.value(), expected_b);

  const auto *placed =
      find_piece_placement(result_b.value(), fixture.pieces.front().piece_id);
  REQUIRE(placed != nullptr);
  const auto rotation = resolve_piece_rotation_degrees(request_b, *placed);
  REQUIRE(rotation.has_value());
  REQUIRE(std::fabs(*rotation - 90.0) <= 1e-9);
}

TEST_CASE("focused asymmetric fixture honors non-conflicting allowed_bin_ids",
          "[mtg][nesting-matrix][engine-surface][allowed-bin-ids]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();
  const auto options = baseline_bb_options();

  auto request = make_request(fixture, options);
  request.execution.default_rotations = geom::DiscreteRotationSet{{0.0, 90.0}};
  REQUIRE(request.pieces.size() == fixture.pieces.size());
  request.pieces[0].allowed_bin_ids = {kBed1Id};
  request.pieces[1].allowed_bin_ids = {kBed2Id};
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 11;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.per_bed_counts = {{kBed1Id, 1}, {kBed2Id, 1}};
  expected.required_assignments = {
      {fixture.pieces[0].piece_id, kBed1Id},
      {fixture.pieces[1].piece_id, kBed2Id},
  };
  expected.require_rotation_admissibility = true;
  expected.require_allowed_bin_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);

  const auto *placed = find_piece_placement(solved.value(), fixture.pieces[0].piece_id);
  REQUIRE(placed != nullptr);
  const auto rotation = resolve_piece_rotation_degrees(request, *placed);
  REQUIRE(rotation.has_value());
  REQUIRE(std::fabs(*rotation - 90.0) <= 1e-9);
}

TEST_CASE("focused asymmetric fixture keeps pieces unplaced when selected bins conflict",
          "[mtg][nesting-matrix][engine-surface][allowed-bin-ids]") {
  auto fixture = make_asymmetric_engine_surface_fixture();
  const auto selected_bed_id = GENERATE(kBed1Id, kBed2Id);
  const auto other_bed_id = selected_bed_id == kBed1Id ? kBed2Id : kBed1Id;
  CAPTURE(selected_bed_id);
  fixture.pieces[0].source_bed_id = selected_bed_id;
  fixture.pieces[1].source_bed_id = selected_bed_id;
  auto options = baseline_bb_options();
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;
  options.selected_bin_ids = {selected_bed_id};

  auto request = make_request(fixture, options);
  request.execution.default_rotations = geom::DiscreteRotationSet{{0.0, 90.0}};
  REQUIRE(request.pieces.size() == fixture.pieces.size());
  request.pieces[0].allowed_bin_ids = {other_bed_id};
  request.pieces[1].allowed_bin_ids = {selected_bed_id};
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 11;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = 1;
  expected.per_bed_counts = {{selected_bed_id, 1}};
  expected.required_assignments = {{fixture.pieces[1].piece_id, selected_bed_id}};
  expected.require_allowed_bin_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);

  REQUIRE(find_piece_placement(solved.value(), fixture.pieces[0].piece_id) == nullptr);
  REQUIRE(find_piece_placement(solved.value(), fixture.pieces[1].piece_id) != nullptr);
}

TEST_CASE("mtg per-piece allowed_rotations is honored",
          "[mtg][nesting-matrix][engine-surface][rotations]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();
  auto options = baseline_bb_options();
  options.maintain_bed_assignment = true;
  options.allow_part_overflow = false;

  auto request = make_request(fixture, options);
  REQUIRE(!request.pieces.empty());
  request.execution.default_rotations = geom::DiscreteRotationSet{{0.0, 90.0}};
  request.pieces[0].allowed_rotations = geom::DiscreteRotationSet{{90.0}};
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 11;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.required_assignments = {{fixture.pieces.front().piece_id, kBed1Id}};
  expected.require_rotation_admissibility = true;
  validate_layout(fixture, request, options, solved.value(), expected);

  const auto *placed =
      find_piece_placement(solved.value(), fixture.pieces.front().piece_id);
  REQUIRE(placed != nullptr);
  REQUIRE(placed->placement.rotation_index.value == 0);
  const auto rotation = resolve_piece_rotation_degrees(request, *placed);
  REQUIRE(rotation.has_value());
  REQUIRE(std::fabs(*rotation - 90.0) <= 1e-9);
}

TEST_CASE("mtg allow_mirror toggle preserves placement count",
          "[mtg][nesting-matrix][engine-surface][mirror]") {
  const auto fixture = load_mtg_fixture();
  const auto options = baseline_bb_options();
  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;

  // Solve A — allow_mirror=false (default from make_request).
  auto request_a = make_request(fixture, options);
  REQUIRE(request_a.is_valid());
  SolveControl control_a{};
  control_a.random_seed = 11;
  auto result_a = solve(request_a, control_a);
  REQUIRE(result_a.has_value());
  validate_layout(fixture, request_a, options, result_a.value(), expected);

  // Solve B — allow_mirror=true on every piece.
  auto request_b = make_request(fixture, options);
  for (auto &piece : request_b.pieces) {
    piece.allow_mirror = true;
  }
  REQUIRE(request_b.is_valid());
  SolveControl control_b{};
  control_b.random_seed = 11;
  auto result_b = solve(request_b, control_b);
  REQUIRE(result_b.has_value());
  validate_layout(fixture, request_b, options, result_b.value(), expected);
}

TEST_CASE("mtg quantity>1 places duplicate pieces",
          "[mtg][nesting-matrix][engine-surface][quantity]") {
  const auto fixture = load_mtg_fixture();

  // Find smallest piece in the fixture by AABB area.
  REQUIRE(!fixture.pieces.empty());
  std::size_t smallest_idx = 0;
  double smallest_area = fixture.pieces[0].width_mm * fixture.pieces[0].height_mm;
  for (std::size_t i = 1; i < fixture.pieces.size(); ++i) {
    const auto area = fixture.pieces[i].width_mm * fixture.pieces[i].height_mm;
    if (area < smallest_area) {
      smallest_area = area;
      smallest_idx = i;
    }
  }
  const auto smallest_piece_id = fixture.pieces[smallest_idx].piece_id;

  const auto options = baseline_bb_options();
  auto request = make_request(fixture, options);
  const auto target_idx = find_piece_index_by_id(request, smallest_piece_id);
  REQUIRE(target_idx < request.pieces.size());
  request.pieces[target_idx].quantity = 3;
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 11;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  const auto total_placed = count_total_placements(solved.value());
  REQUIRE(total_placed == static_cast<std::size_t>(kBaselinePieceCount) + 2);

  // Quantity expansion derives new expanded_piece_ids for instances 1+;
  // map source -> expanded ids via normalize_request to count placements.
  auto normalized = normalize_request(request);
  REQUIRE(normalized.has_value());
  std::vector<std::uint32_t> expanded_ids;
  for (const auto &exp : normalized.value().expanded_pieces) {
    if (exp.source_piece_id == smallest_piece_id) {
      expanded_ids.push_back(exp.expanded_piece_id);
    }
  }
  REQUIRE(expanded_ids.size() == 3);

  std::size_t target_count = 0;
  for (const auto &bin : solved.value().layout.bins) {
    for (const auto &placed : bin.placements) {
      if (std::find(expanded_ids.begin(), expanded_ids.end(),
                    placed.placement.piece_id) != expanded_ids.end()) {
        ++target_count;
      }
    }
  }
  REQUIRE(target_count == 3);
}

TEST_CASE("mtg piece priority survives partial placement",
          "[mtg][nesting-matrix][engine-surface][priority]") {
  const auto fixture = load_mtg_fixture();

  // Note: PieceOrdering::priority is only consulted by the irregular
  // constructive / production strategies, which are exercised in the
  // dedicated sequential-backtrack / metaheuristic-search suites with full
  // ordering depth. Here we verify the bounding-box contract: a priority-marked
  // piece must always appear in the placed set when any other pieces are placed,
  // regardless of strategy. The deeper priority-sequence contract (piece order)
  // lives in the dedicated irregular suites.
  auto options = baseline_bb_options();
  options.irregular.piece_ordering = PieceOrdering::priority;

  // Pick the largest piece overall and bump its priority.
  std::size_t largest_idx = 0;
  double largest_area =
      fixture.pieces[0].width_mm * fixture.pieces[0].height_mm;
  for (std::size_t i = 1; i < fixture.pieces.size(); ++i) {
    const auto area = fixture.pieces[i].width_mm * fixture.pieces[i].height_mm;
    if (area > largest_area) {
      largest_area = area;
      largest_idx = i;
    }
  }
  const auto priority_piece_id = fixture.pieces[largest_idx].piece_id;

  auto request = make_request(fixture, options);
  for (auto &piece : request.pieces) {
    piece.priority = (piece.piece_id == priority_piece_id) ? 100 : 0;
  }
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 11;
  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  const auto &unplaced = solved.value().layout.unplaced_piece_ids;
  const bool priority_unplaced =
      std::find(unplaced.begin(), unplaced.end(), priority_piece_id) !=
      unplaced.end();
  INFO("unplaced count=" << unplaced.size()
                         << " priority_piece_id=" << priority_piece_id);
  REQUIRE_FALSE(priority_unplaced);

  bool found_priority = false;
  for (const auto &bin : solved.value().layout.bins) {
    for (const auto &placed : bin.placements) {
      if (placed.placement.piece_id == priority_piece_id) {
        found_priority = true;
      }
    }
  }
  REQUIRE(found_priority);
}

TEST_CASE("mtg ProgressObserver receives monotonic sequences",
          "[mtg][nesting-matrix][engine-surface][observer]") {
  const auto fixture = load_mtg_fixture();

  // Bounding-box (constructive) emits per-placement progress snapshots —
  // sufficient to validate the observer contract without a brkga run.
  auto options = baseline_bb_options();

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  std::vector<ProgressSnapshot> observed;
  SolveControl control{};
  control.random_seed = 5;
  control.on_progress = [&](const ProgressSnapshot &snap) {
    observed.push_back(snap);
  };

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  REQUIRE(observed.size() >= 1);
  for (std::size_t i = 1; i < observed.size(); ++i) {
    INFO("seq[" << i - 1 << "]=" << observed[i - 1].sequence << " seq[" << i
                << "]=" << observed[i].sequence);
    REQUIRE(observed[i].sequence == observed[i - 1].sequence + 1U);
    REQUIRE(observed[i].budget.iterations_completed >=
            observed[i - 1].budget.iterations_completed);
  }
  REQUIRE(observed.front().sequence == 1U);
  REQUIRE(observed.back().budget.iterations_completed ==
          solved.value().budget.iterations_completed);
  REQUIRE(observed.back().budget.iterations_completed <=
          request.execution.deterministic_attempts.max_attempts);
  REQUIRE(solved.value().stop_reason == StopReason::completed);
}

TEST_CASE("mtg engine flag toggles still place all parts",
          "[mtg][nesting-matrix][engine-surface][toggles]") {
  const auto fixture = load_mtg_fixture();
  const auto options = baseline_bb_options();
  ExpectedOutcome expected{};
  expected.expected_placed_count = kBaselinePieceCount;

  SECTION("enable_part_in_part_placement=true") {
    auto request = make_request(fixture, options);
    request.execution.enable_part_in_part_placement = true;
    REQUIRE(request.is_valid());
    SolveControl control{};
    control.random_seed = 11;
    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    validate_layout(fixture, request, options, solved.value(), expected);
  }

  SECTION("explore_concave_candidates=true") {
    auto request = make_request(fixture, options);
    request.execution.explore_concave_candidates = true;
    REQUIRE(request.is_valid());
    SolveControl control{};
    control.random_seed = 11;
    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    validate_layout(fixture, request, options, solved.value(), expected);
  }
}
