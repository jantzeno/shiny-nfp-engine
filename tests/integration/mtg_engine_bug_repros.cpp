// MTG engine regressions.
//
// Each TEST_CASE here pins a single, minimal regression for an engine bug
// surfaced by the MTG nesting matrix work. These cases remain targeted
// diagnostics even after the broader matrix absorbs the same contract.
//
// Milestone-4 classification:
// - long-term product contracts live in the fast MTG lane
//   (bounding_box_engine_surface plus the named algorithm suites and broad
//   matrix) for rotations, mirror, quantity, priority, progress,
//   cancellation, and engine-flag toggles
// - this file keeps one minimal passing regression per formerly broken bug
// - test/documentation clarifications already locked in elsewhere:
//   the small-population BRKGA validity rule is current contract,
//   `CancellationSource::request_stop()` is the real API, and the rectangle MTG
//   fixture is breadth-first rather than the only rotation-sensitive proof
//   surface

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "geometry/types.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto aabb_of(const geom::PolygonWithHoles &p) -> geom::Box2 {
  geom::Box2 box{};
  if (p.outer.empty()) {
    return box;
  }
  box.min = p.outer.front();
  box.max = p.outer.front();
  for (const auto &pt : p.outer) {
    box.min.x = std::min(box.min.x, pt.x);
    box.min.y = std::min(box.min.y, pt.y);
    box.max.x = std::max(box.max.x, pt.x);
    box.max.y = std::max(box.max.y, pt.y);
  }
  return box;
}

[[nodiscard]] auto total_placed(const NestingResult &result) -> std::size_t {
  std::size_t n = 0;
  for (const auto &bin : result.layout.bins) {
    n += bin.placements.size();
  }
  return n;
}

[[nodiscard]] auto actual_polygon_layout_summary(const NestingResult &result)
    -> std::string {
  std::ostringstream out;
  out << "bins=";
  for (std::size_t index = 0; index < result.layout.bins.size(); ++index) {
    const auto &bin = result.layout.bins[index];
    if (index > 0U) {
      out << ",";
    }
    out << bin.bin_id << ":" << bin.placements.size();
  }
  out << " unplaced=";
  for (std::size_t index = 0; index < result.layout.unplaced_piece_ids.size();
       ++index) {
    if (index > 0U) {
      out << ",";
    }
    out << result.layout.unplaced_piece_ids[index];
  }
  return out.str();
}

// Asserts the per-bin geometric invariants (bin containment + pair-wise
// AABB spacing) without requiring full conservation. Used by the
// time-limited actual-polygon repros, where engine-level conservation
// under `stop_reason=time_limit_reached` is a separately-tracked finding
// (the engine occasionally drops a piece from `unplaced_piece_ids` when
// the time limit fires mid-piece).
auto assert_placed_layout_invariants(const NestingResult &result,
                                     double spacing_mm) -> void {
  for (const auto &bin : result.layout.bins) {
    std::vector<geom::Box2> boxes;
    boxes.reserve(bin.placements.size());
    for (const auto &p : bin.placements) {
      boxes.push_back(aabb_of(p.polygon));
    }
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      for (std::size_t j = i + 1; j < boxes.size(); ++j) {
        INFO("bin " << bin.bin_id << " spacing-violation pair "
                    << bin.placements[i].placement.piece_id << " vs "
                    << bin.placements[j].placement.piece_id);
        REQUIRE_FALSE(
            boxes_violate_spacing(boxes[i], boxes[j], spacing_mm, 1e-3));
      }
    }
  }
}

inline constexpr std::uint64_t kActualPolygonTimeCapMs = 60'000U;
inline constexpr std::uint32_t kActualPolygonSeed = 1234U;

[[nodiscard]] constexpr auto
candidate_strategy_name(const CandidateStrategy strategy) -> std::string_view {
  switch (strategy) {
  case CandidateStrategy::anchor_vertex:
    return "anchor_vertex";
  case CandidateStrategy::nfp_perfect:
    return "nfp_perfect";
  case CandidateStrategy::nfp_arrangement:
    return "nfp_arrangement";
  case CandidateStrategy::nfp_hybrid:
    return "nfp_hybrid";
  case CandidateStrategy::count:
    return "count";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto stop_reason_name(const StopReason reason)
    -> std::string_view {
  switch (reason) {
  case StopReason::none:
    return "none";
  case StopReason::completed:
    return "completed";
  case StopReason::cancelled:
    return "cancelled";
  case StopReason::operation_limit_reached:
    return "operation_limit_reached";
  case StopReason::time_limit_reached:
    return "time_limit_reached";
  case StopReason::invalid_request:
    return "invalid_request";
  }
  return "unknown";
}

[[nodiscard]] constexpr auto
start_corner_name(const place::PlacementStartCorner corner)
    -> std::string_view {
  switch (corner) {
  case place::PlacementStartCorner::bottom_left:
    return "bottom_left";
  case place::PlacementStartCorner::bottom_right:
    return "bottom_right";
  case place::PlacementStartCorner::top_left:
    return "top_left";
  case place::PlacementStartCorner::top_right:
    return "top_right";
  }
  return "unknown";
}

[[nodiscard]] auto make_actual_polygon_constructive_options(
    const CandidateStrategy candidate_strategy,
    const place::PlacementStartCorner start_corner) -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.irregular.candidate_strategy = candidate_strategy;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.part_spacing_mm = 0.0;
  options.bed1_start_corner = start_corner;
  options.bed2_start_corner = start_corner;
  return options;
}

struct ActualPolygonSolveOutcome {
  std::optional<NestingResult> result{};
  std::string summary{};
  std::string exception_message{};
};

[[nodiscard]] auto solve_actual_polygon_constructive_case(
    const CandidateStrategy candidate_strategy,
    const place::PlacementStartCorner start_corner =
        place::PlacementStartCorner::bottom_left) -> ActualPolygonSolveOutcome {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto options = make_actual_polygon_constructive_options(
      candidate_strategy, start_corner);
  auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  try {
    SolveControl control{};
    control.time_limit_milliseconds = kActualPolygonTimeCapMs;
    control.random_seed = 0;

    const auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    const auto &result = solved.value();
    const auto placed = total_placed(result);
    const std::uint64_t slack =
        std::max<std::uint64_t>(500U, kActualPolygonTimeCapMs);
    std::ostringstream summary;
    summary << "strategy=" << candidate_strategy_name(candidate_strategy)
            << " start_corner=" << start_corner_name(start_corner)
            << " placed=" << placed << "/" << kBaselinePieceCount
            << " unplaced=" << result.layout.unplaced_piece_ids.size()
            << " stop_reason=" << stop_reason_name(result.stop_reason)
            << " elapsed_ms=" << result.budget.elapsed_milliseconds
            << " iterations=" << result.budget.operations_completed << " "
            << actual_polygon_layout_summary(result);
    REQUIRE(result.budget.elapsed_milliseconds <=
            kActualPolygonTimeCapMs + slack);
    assert_placed_layout_invariants(result, options.part_spacing_mm);
    return ActualPolygonSolveOutcome{
        .result = result,
        .summary = summary.str(),
    };
  } catch (const std::exception &ex) {
    std::ostringstream summary;
    summary << "strategy=" << candidate_strategy_name(candidate_strategy)
            << " threw exception: " << ex.what();
    return ActualPolygonSolveOutcome{
        .result = std::nullopt,
        .summary = summary.str(),
        .exception_message = ex.what(),
    };
  }
}

} // namespace

// -----------------------------------------------------------------------------
// Repro 1: bb-shelf with pinned-to-source-bed model leaves one piece unplaced.
//
// Engine model: maintain_bed_assignment=false + allow_part_overflow=false
// pins each piece to its source bed (per nesting_adapter.cpp). Each MTG
// fixture piece individually fits its source bed, yet bb-shelf ends with
// 17/18 placed.
// -----------------------------------------------------------------------------
TEST_CASE(
    "REGRESSION: bb-shelf pinned packing places every source-pinned piece",
    "[mtg][engine-bug-repro][bb-pinned]") {
  const auto fixture = load_mtg_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.part_spacing_mm = 0.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = false;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 1234;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  const auto placed = total_placed(solved.value());
  INFO("bb-shelf pinned placed "
       << placed << " of " << kBaselinePieceCount << " (unplaced "
       << solved.value().layout.unplaced_piece_ids.size() << ")");
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}

// -----------------------------------------------------------------------------
// Regression 2: historical allowed_bin_ids/selected_bin_ids asymmetry. This is
// the Milestone-2 ownership boundary for request-surface semantics: per-piece
// allowed_bin_ids must be honored after selected-bin filtering, and an empty
// effective intersection must leave the piece unplaced rather than silently
// relocating it to the selected bed.
// -----------------------------------------------------------------------------
TEST_CASE("REGRESSION: allowed_bin_ids remains enforced under selected_bin_ids",
          "[mtg][engine-bug-repro][allowed-bin-ids]") {
  const auto fixture = load_mtg_fixture();

  SECTION(
      "selected={bed1}, override one bed-2-source piece to allowed={bed2}") {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::bounding_box;
    options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
    options.maintain_bed_assignment = true;
    options.allow_part_overflow = false;
    options.selected_bin_ids = {kBed1Id};

    auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    std::uint32_t target_piece_id = 0;
    for (const auto &p : fixture.pieces) {
      if (p.source_bed_id == kBed2Id) {
        target_piece_id = p.piece_id;
        break;
      }
    }
    REQUIRE(target_piece_id != 0);

    bool overridden = false;
    for (auto &pr : request.pieces) {
      if (pr.piece_id == target_piece_id) {
        pr.allowed_bin_ids = {kBed2Id};
        overridden = true;
        break;
      }
    }
    REQUIRE(overridden);

    SolveControl control{};
    control.random_seed = 1234;
    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    INFO("Target piece " << target_piece_id
                         << " has allowed_bin_ids={bed2} but selected={bed1}; "
                            "engine must NOT place it on bed1.");
    for (const auto &bin : solved.value().layout.bins) {
      if (bin.bin_id != kBed1Id) {
        continue;
      }
      for (const auto &pp : bin.placements) {
        REQUIRE(pp.placement.piece_id != target_piece_id);
      }
    }
  }

  SECTION(
      "selected={bed2}, override one bed-1-source piece to allowed={bed1}") {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::bounding_box;
    options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
    options.maintain_bed_assignment = true;
    options.allow_part_overflow = false;
    options.selected_bin_ids = {kBed2Id};

    auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    std::uint32_t target_piece_id = 0;
    for (const auto &p : fixture.pieces) {
      if (p.source_bed_id == kBed1Id) {
        target_piece_id = p.piece_id;
        break;
      }
    }
    REQUIRE(target_piece_id != 0);

    bool overridden = false;
    for (auto &pr : request.pieces) {
      if (pr.piece_id == target_piece_id) {
        pr.allowed_bin_ids = {kBed1Id};
        overridden = true;
        break;
      }
    }
    REQUIRE(overridden);

    SolveControl control{};
    control.random_seed = 1234;
    auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    INFO("Target piece " << target_piece_id
                         << " has allowed_bin_ids={bed1} but selected={bed2}; "
                            "engine must NOT place it on bed2.");
    for (const auto &bin : solved.value().layout.bins) {
      if (bin.bin_id != kBed2Id) {
        continue;
      }
      for (const auto &pp : bin.placements) {
        REQUIRE(pp.placement.piece_id != target_piece_id);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Repro 3: BB heuristic emits placements that violate the requested spacing.
//
// Historical spacing regression: in ~9 of 108 BB-heuristic cells
// (heuristic × spacing × policy × deterministic_attempts) two placements on
// bin 1 were too close — the gap between their AABBs was less than the
// requested `part_spacing_mm`. Keep the canonical tuple below as a hidden
// passing regression so the skyline clearance model stays pinned:
//
//   heuristic = skyline, spacing = 1.0 mm, deterministic_attempts = 1,
//   placement_policy = bottom_left, all beds, overflow=true, maintain=false,
//   seed = 12345.
//
// The other 8 cells extend across {bottom_left, minimum_length,
// maximum_utilization} × {1, 4, 8} attempts (always skyline + spacing=1.0).
// If the build/run environment differs the tuple may need updating; the
// goal is to pin one canonical repro per known engine bug.
//
// NOTE: the original work-summary phrasing mentioned "exact AABB overlap";
// the actual invariant violated is the spacing/clearance one — pieces are
// not raw-overlapping, but the gap between their AABBs is < spacing. This
// matches the fixture's `boxes_overlap(a, b, effective_spacing)` check.
// -----------------------------------------------------------------------------
TEST_CASE("REGRESSION: bb heuristic honors requested spacing",
          "[mtg][engine-bug-repro][bb-overlap]") {
  const auto fixture = load_mtg_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::skyline;
  options.bounding_box_deterministic_attempts = 1;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.part_spacing_mm = 1.0;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = 12345;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  for (const auto &bin : solved.value().layout.bins) {
    if (bin.bin_id != kBed1Id) {
      continue;
    }
    for (std::size_t i = 0; i < bin.placements.size(); ++i) {
      const auto box_a = aabb_of(bin.placements[i].polygon);
      for (std::size_t j = i + 1; j < bin.placements.size(); ++j) {
        const auto box_b = aabb_of(bin.placements[j].polygon);
        INFO("bin1 spacing-violation pair pieces "
             << bin.placements[i].placement.piece_id << " [" << box_a.min.x
             << "," << box_a.min.y << "]-[" << box_a.max.x << "," << box_a.max.y
             << "] vs " << bin.placements[j].placement.piece_id << " ["
             << box_b.min.x << "," << box_b.min.y << "]-[" << box_b.max.x << ","
             << box_b.max.y << "] (required spacing " << options.part_spacing_mm
             << " mm)");
        REQUIRE_FALSE(
            boxes_violate_spacing(box_a, box_b, options.part_spacing_mm, 1e-3));
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Repro 4: actual-polygon anchor_vertex should place the full MTG set.
//
// This is intentionally normative. Today it fails by hitting the time cap and
// placing only a subset of the real silhouettes.
// -----------------------------------------------------------------------------
TEST_CASE(
    "REGRESSION: anchor_vertex places every actual-polygon MTG silhouette",
    "[mtg][engine-bug-repro][actual-polygons][anchor-vertex]") {
  const auto outcome =
      solve_actual_polygon_constructive_case(CandidateStrategy::anchor_vertex);
  INFO(outcome.summary);
  REQUIRE(outcome.result.has_value());
  const auto &result = *outcome.result;
  const auto placed = total_placed(result);

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}

// -----------------------------------------------------------------------------
// Repro 5: actual-polygon NFP strategies should solve the full MTG set.
//
// This is intentionally normative. Today the actual-polygon NFP family fails in
// multiple ways under the fixed solve setup: some strategies throw CGAL
// exceptions, and `nfp_hybrid` reaches the time cap with only 8/18 placed.
// -----------------------------------------------------------------------------
TEST_CASE(
    "REGRESSION: NFP strategies place every actual-polygon MTG silhouette",
    "[mtg][engine-bug-repro][actual-polygons][nfp-strategies]") {
  const auto candidate_strategy = GENERATE(CandidateStrategy::nfp_perfect,
                                           CandidateStrategy::nfp_arrangement,
                                           CandidateStrategy::nfp_hybrid);

  const auto outcome =
      solve_actual_polygon_constructive_case(candidate_strategy);
  INFO(outcome.summary);
  REQUIRE(outcome.result.has_value());
  const auto &result = *outcome.result;
  const auto placed = total_placed(result);

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}

TEST_CASE("REGRESSION: actual-polygon anchor and hybrid placement stay "
          "complete across opposite start corners",
          "[mtg][engine-bug-repro][actual-polygons][start-corner][slow]") {
  const auto candidate_strategy =
      GENERATE(CandidateStrategy::anchor_vertex, CandidateStrategy::nfp_hybrid);
  const auto start_corner = GENERATE(place::PlacementStartCorner::bottom_left,
                                     place::PlacementStartCorner::top_right);

  const auto outcome =
      solve_actual_polygon_constructive_case(candidate_strategy, start_corner);
  INFO(outcome.summary);
  REQUIRE(outcome.result.has_value());
  const auto &result = *outcome.result;
  const auto placed = total_placed(result);

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}
