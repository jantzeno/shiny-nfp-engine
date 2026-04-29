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
#include "packing/irregular/sequential/packer.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto rect_polygon(const double width, const double height)
    -> geom::PolygonWithHoles {
  return geom::PolygonWithHoles{
      .outer = {{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}}};
}

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

[[nodiscard]] auto placement_trace_bin_summary(const NestingResult &result)
    -> std::string {
  std::ostringstream out;
  for (std::size_t index = 0; index < result.layout.placement_trace.size();
       ++index) {
    const auto &entry = result.layout.placement_trace[index];
    if (index > 0U) {
      out << " -> ";
    }
    out << entry.piece_id << "@" << entry.bin_id;
    if (entry.opened_new_bin) {
      out << "*";
    }
  }
  return out.str();
}

auto require_monotonic_frontier_progression(const NestingResult &result)
    -> void {
  REQUIRE_FALSE(result.layout.placement_trace.empty());
  REQUIRE(result.layout.placement_trace.front().bin_id == kBed1Id);

  const auto first_bin2 =
      std::find_if(result.layout.placement_trace.begin(),
                   result.layout.placement_trace.end(),
                   [](const auto &entry) { return entry.bin_id == kBed2Id; });
  if (first_bin2 == result.layout.placement_trace.end()) {
    return;
  }

  const auto later_bin1 =
      std::find_if(std::next(first_bin2), result.layout.placement_trace.end(),
                   [](const auto &entry) { return entry.bin_id == kBed1Id; });
  REQUIRE(later_bin1 == result.layout.placement_trace.end());
}

// Asserts the per-bin geometric invariants (bin containment + pair-wise
// exact spacing/overlap) without requiring full conservation. Used by the
// time-limited actual-polygon repros, where engine-level conservation
// under `stop_reason=time_limit_reached` is a separately-tracked finding
// (the engine occasionally drops a piece from `unplaced_piece_ids` when
// the time limit fires mid-piece).
auto assert_placed_layout_invariants(const NestingResult &result,
                                     double spacing_mm) -> void {
  for (const auto &bin : result.layout.bins) {
    for (std::size_t i = 0; i < bin.placements.size(); ++i) {
      for (std::size_t j = i + 1; j < bin.placements.size(); ++j) {
        INFO("bin " << bin.bin_id << " spacing-violation pair "
                    << bin.placements[i].placement.piece_id << " vs "
                    << bin.placements[j].placement.piece_id);
        const auto overlap_area =
            geom::polygon_area_sum(poly::intersection_polygons(
                bin.placements[i].polygon, bin.placements[j].polygon));
        REQUIRE(overlap_area <= 1e-6);
        if (spacing_mm > 0.0) {
          const auto clearance = poly::polygon_distance(
              bin.placements[i].polygon, bin.placements[j].polygon);
          REQUIRE(clearance + 1e-3 >= spacing_mm);
        }
      }
    }
  }
}

inline constexpr std::uint64_t kActualPolygonTimeCapMs = 60'000U;
inline constexpr std::uint32_t kActualPolygonSeed = 1234U;
inline constexpr std::size_t kBed1Parts = 13;
inline constexpr std::size_t kBed2Parts = 5;

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
    const std::size_t max_candidate_points,
    const place::PlacementStartCorner start_corner) -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::sequential_backtrack;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.irregular.candidate_strategy = candidate_strategy;
  options.irregular.max_candidate_points = max_candidate_points;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.part_spacing_mm = 0.0;
  options.bed1_start_corner = start_corner;
  options.bed2_start_corner = start_corner;
  return options;
}

[[nodiscard]] auto make_actual_polygon_metaheuristic_options(
    const CandidateStrategy candidate_strategy,
    const ProductionOptimizerKind optimizer,
    const place::PlacementStartCorner start_corner) -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::metaheuristic_search;
  options.production_optimizer = optimizer;
  options.placement_policy = place::PlacementPolicy::bottom_left;
  options.irregular.candidate_strategy = candidate_strategy;
  options.irregular.piece_ordering = PieceOrdering::largest_area_first;
  options.irregular.enable_backtracking = true;
  options.irregular.max_backtrack_pieces = 3;
  options.irregular.enable_compaction = true;
  options.irregular.compaction_passes = 2;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.part_spacing_mm = 0.0;
  options.bed1_start_corner = start_corner;
  options.bed2_start_corner = start_corner;

  options.production.population_size = 24;
  options.production.elite_count = 6;
  options.production.mutant_count = 4;
  options.production.max_iterations = 24;
  options.production.polishing_passes = 1;
  return options;
}

struct ActualPolygonSolveOutcome {
  std::optional<NestingResult> result{};
  std::string summary{};
  std::string exception_message{};
};

[[nodiscard]] auto solve_actual_polygon_constructive_case(
    const CandidateStrategy candidate_strategy,
    const std::size_t max_candidate_points = 1200U,
    const std::uint64_t time_limit_milliseconds = kActualPolygonTimeCapMs,
    const place::PlacementStartCorner start_corner =
        place::PlacementStartCorner::bottom_left) -> ActualPolygonSolveOutcome {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto options = make_actual_polygon_constructive_options(
      candidate_strategy, max_candidate_points, start_corner);
  auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  try {
    SolveControl control{};
    control.time_limit_milliseconds = time_limit_milliseconds;
    control.random_seed = 0;

    const auto solved = solve(request, control);
    REQUIRE(solved.has_value());

    const auto &result = solved.value();
    const auto placed = total_placed(result);
    std::ostringstream summary;
    summary << "strategy=" << candidate_strategy_name(candidate_strategy)
            << " max_candidate_points=" << max_candidate_points
            << " start_corner=" << start_corner_name(start_corner)
            << " placed=" << placed << "/" << kBaselinePieceCount
            << " unplaced=" << result.layout.unplaced_piece_ids.size()
            << " stop_reason=" << stop_reason_name(result.stop_reason)
            << actual_polygon_layout_summary(result);
    assert_placed_layout_invariants(result, options.part_spacing_mm);
    return ActualPolygonSolveOutcome{
        .result = result,
        .summary = summary.str(),
    };
  } catch (const std::exception &ex) {
    std::ostringstream summary;
    summary << "strategy=" << candidate_strategy_name(candidate_strategy)
            << " max_candidate_points=" << max_candidate_points
            << " threw exception: " << ex.what();
    return ActualPolygonSolveOutcome{
        .result = std::nullopt,
        .summary = summary.str(),
        .exception_message = ex.what(),
    };
  }
}

[[nodiscard]] auto make_aspect_ratio_exhaustion_request() -> NestingRequest {
  NestingRequest request{};
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.placement_policy = place::PlacementPolicy::bottom_left;
  request.execution.irregular = {};
  request.execution.irregular.candidate_strategy =
      CandidateStrategy::anchor_vertex;
  request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;
  request.execution.part_spacing = 0.0;

  request.bins = {
      BinRequest{
          .bin_id = kBed1Id,
          .polygon = rect_polygon(6.0, 8.0),
          .stock = 1,
          .geometry_revision = 1,
      },
      BinRequest{
          .bin_id = kBed2Id,
          .polygon = rect_polygon(6.0, 8.0),
          .stock = 1,
          .geometry_revision = 2,
      },
  };
  request.pieces = {
      PieceRequest{
          .piece_id = 1,
          .polygon = rect_polygon(4.0, 8.0),
          .quantity = 1,
          .geometry_revision = 11,
      },
      PieceRequest{
          .piece_id = 2,
          .polygon = rect_polygon(4.0, 4.0),
          .quantity = 1,
          .geometry_revision = 12,
      },
  };
  return request;
}

[[nodiscard]] auto make_skipped_exhausted_bed_request() -> NestingRequest {
  NestingRequest request{};
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.placement_policy = place::PlacementPolicy::bottom_left;
  request.execution.irregular = {};
  request.execution.irregular.candidate_strategy =
      CandidateStrategy::anchor_vertex;
  request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;

  request.bins = {
      BinRequest{
          .bin_id = kBed1Id,
          .polygon = rect_polygon(6.0, 8.0),
          .stock = 1,
      },
      BinRequest{
          .bin_id = kBed2Id,
          .polygon = rect_polygon(6.0, 4.0),
          .stock = 1,
          .exclusion_zones = {make_rect_exclusion(1, kBed2Id, 2.0, 0.0, 4.0,
                                                  4.0)},
      },
      BinRequest{
          .bin_id = 3,
          .polygon = rect_polygon(4.0, 4.0),
          .stock = 1,
      },
  };
  request.pieces = {
      PieceRequest{
          .piece_id = 1,
          .polygon = rect_polygon(4.0, 8.0),
          .quantity = 1,
      },
      PieceRequest{
          .piece_id = 2,
          .polygon = rect_polygon(4.0, 4.0),
          .quantity = 1,
      },
  };
  return request;
}

} // namespace

// -----------------------------------------------------------------------------
// Repro 1: bb-shelf with explicit maintain-bed-assignment pinning leaves one
// piece unplaced. Each MTG fixture piece individually fits its source bed, yet
// bb-shelf historically ended with 17/18 placed.
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
  options.maintain_bed_assignment = true;
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
  REQUIRE(solved.value().placed_parts() == kBaselinePieceCount);
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
    REQUIRE(solved.value().placed_parts() == kBed1Parts);

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
    REQUIRE(solved.value().placed_parts() == kBed2Parts);
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
  REQUIRE(solved.value().placed_parts() == kBaselinePieceCount);

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

  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
}

TEST_CASE("REGRESSION: actual-polygon MTG candidate-point coverage plateaus "
          "at 1200",
          "[mtg][engine-bug-repro][actual-polygons][max-candidate-points]"
          "[slow]") {
  const auto strategy = GENERATE(CandidateStrategy::anchor_vertex,
                                 CandidateStrategy::nfp_perfect);

  // BUG: 6/18 parts placed
  const auto low_coverage =
      solve_actual_polygon_constructive_case(strategy, 640U, 20'000U);
  // BUG: 7/18 parts placed
  const auto default_coverage =
      solve_actual_polygon_constructive_case(strategy, 1200U, 20'000U);

  INFO(low_coverage.summary);
  INFO(default_coverage.summary);

  REQUIRE(low_coverage.result.has_value());
  REQUIRE(default_coverage.result.has_value());

  const auto low_placed = total_placed(*low_coverage.result);
  const auto default_placed = total_placed(*default_coverage.result);

  REQUIRE(default_placed > low_placed);
  REQUIRE(default_coverage.result->placed_parts() == kBaselinePieceCount);
  REQUIRE(default_placed == static_cast<std::size_t>(kBaselinePieceCount));
}

TEST_CASE("REGRESSION: actual-polygon MTG constructive does not return to bed1 "
          "after bed2 opens",
          "[mtg][engine-bug-repro][actual-polygons][anchor-vertex][bin-order]"
          "[strict-fill-first]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto options = make_actual_polygon_constructive_options(
      CandidateStrategy::anchor_vertex, 1200U,
      place::PlacementStartCorner::bottom_left);
  auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());
  request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;

  auto normalized = normalize_request(request);
  REQUIRE(normalized.ok());

  pack::SequentialBacktrackPacker packer;
  const auto solved =
      packer.solve(normalized.value(),
                   SolveControl{
                       .time_limit_milliseconds = kActualPolygonTimeCapMs,
                       .random_seed = 0,
                   });
  REQUIRE(solved.ok());
  const auto &result = solved.value();

  INFO(actual_polygon_layout_summary(result));
  INFO(placement_trace_bin_summary(result));
  require_monotonic_frontier_progression(result);

  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  const auto bed1_placements = static_cast<std::size_t>(
      std::count_if(result.layout.placement_trace.begin(),
                    result.layout.placement_trace.end(),
                    [](const auto &entry) { return entry.bin_id == kBed1Id; }));
  const auto bed2_placements = static_cast<std::size_t>(
      std::count_if(result.layout.placement_trace.begin(),
                    result.layout.placement_trace.end(),
                    [](const auto &entry) { return entry.bin_id == kBed2Id; }));
  REQUIRE(bed1_placements >= bed2_placements);
  REQUIRE(bed1_placements + bed2_placements ==
          static_cast<std::size_t>(kBaselinePieceCount));
}

TEST_CASE("REGRESSION: actual-polygon MTG nfp_perfect does not return to bed1 "
          "after bed2 opens",
          "[mtg][engine-bug-repro][actual-polygons][nfp-perfect][bin-order]"
          "[strict-fill-first]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto options = make_actual_polygon_constructive_options(
      CandidateStrategy::nfp_perfect, 1200U,
      place::PlacementStartCorner::bottom_left);
  auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());
  request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;

  auto normalized = normalize_request(request);
  REQUIRE(normalized.ok());

  pack::SequentialBacktrackPacker packer;
  const auto solved =
      packer.solve(normalized.value(),
                   SolveControl{
                       .time_limit_milliseconds = kActualPolygonTimeCapMs,
                       .random_seed = 0,
                   });
  REQUIRE(solved.ok());
  const auto &result = solved.value();

  INFO(actual_polygon_layout_summary(result));
  INFO(placement_trace_bin_summary(result));
  INFO(stop_reason_name(result.stop_reason));
  REQUIRE(result.stop_reason != StopReason::time_limit_reached);
  require_monotonic_frontier_progression(result);

  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  const auto bed1_placements = static_cast<std::size_t>(
      std::count_if(result.layout.placement_trace.begin(),
                    result.layout.placement_trace.end(),
                    [](const auto &entry) { return entry.bin_id == kBed1Id; }));
  const auto bed2_placements = static_cast<std::size_t>(
      std::count_if(result.layout.placement_trace.begin(),
                    result.layout.placement_trace.end(),
                    [](const auto &entry) { return entry.bin_id == kBed2Id; }));
  REQUIRE(bed1_placements >= bed2_placements);
  REQUIRE(bed1_placements + bed2_placements ==
          static_cast<std::size_t>(kBaselinePieceCount));
}

TEST_CASE(
    "REGRESSION: strict fill-first opens the next bed only after "
    "geometry-aware frontier exhaustion",
    "[engine-bug-repro][synthetic][strict-fill-first][frontier-exhaustion]") {
  const auto request = make_aspect_ratio_exhaustion_request();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0U});
  REQUIRE(solved.has_value());

  const auto &result = solved.value();
  INFO(placement_trace_bin_summary(result));

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.bins.size() == 2U);
  REQUIRE(result.layout.placement_trace.size() == 2U);
  REQUIRE(result.layout.placement_trace[0].bin_id == kBed1Id);
  REQUIRE(result.layout.placement_trace[1].bin_id == kBed2Id);
  REQUIRE(solved.value().placed_parts() == request.pieces.size());

  require_monotonic_frontier_progression(result);
}

TEST_CASE("REGRESSION: strict fill-first skips an exhausted unopened bed "
          "before opening the next one",
          "[engine-bug-repro][synthetic][strict-fill-first][unopened-bed-"
          "exhaustion]") {
  const auto request = make_skipped_exhausted_bed_request();
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 0U});
  REQUIRE(solved.has_value());

  const auto &result = solved.value();
  INFO(placement_trace_bin_summary(result));

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.layout.placement_trace.size() == 2U);
  REQUIRE(result.layout.placement_trace[0].bin_id == kBed1Id);
  REQUIRE(result.layout.placement_trace[1].bin_id == 3U);
  REQUIRE(solved.value().placed_parts() == request.pieces.size());
}

TEST_CASE("REGRESSION: BRKGA places every actual-polygon MTG silhouette with "
          "strict fill-first ordering",
          "[mtg][engine-bug-repro][actual-polygons][metaheuristic-search]"
          "[strict-fill-first][slow]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto options = make_actual_polygon_metaheuristic_options(
      CandidateStrategy::anchor_vertex, ProductionOptimizerKind::brkga,
      place::PlacementStartCorner::bottom_left);
  auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const auto solved =
      solve(request, SolveControl{
                         .time_limit_milliseconds = kActualPolygonTimeCapMs,
                         .random_seed = kActualPolygonSeed,
                     });
  REQUIRE(solved.has_value());

  const auto &result = solved.value();
  const auto placed = total_placed(result);
  INFO(actual_polygon_layout_summary(result));
  INFO(placement_trace_bin_summary(result));
  INFO(stop_reason_name(result.stop_reason));

  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
  REQUIRE(result.strategy == StrategyKind::metaheuristic_search);
  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
  REQUIRE(result.layout.bins.size() == 2U);
  REQUIRE_FALSE(result.layout.placement_trace.empty());
  REQUIRE(result.layout.placement_trace.front().bin_id == kBed1Id);

  const auto first_bin2 =
      std::find_if(result.layout.placement_trace.begin(),
                   result.layout.placement_trace.end(),
                   [](const auto &entry) { return entry.bin_id == kBed2Id; });
  REQUIRE(first_bin2 != result.layout.placement_trace.end());
  REQUIRE(first_bin2->opened_new_bin);

  const auto later_bin1 =
      std::find_if(std::next(first_bin2), result.layout.placement_trace.end(),
                   [](const auto &entry) { return entry.bin_id == kBed1Id; });
  REQUIRE(later_bin1 == result.layout.placement_trace.end());
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
  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}

TEST_CASE("REGRESSION: actual-polygon anchor and hybrid placement stay "
          "complete across opposite start corners",
          "[mtg][engine-bug-repro][actual-polygons][start-corner][slow]") {
  const auto candidate_strategy =
      GENERATE(CandidateStrategy::anchor_vertex, CandidateStrategy::nfp_hybrid);
  const auto start_corner = GENERATE(place::PlacementStartCorner::bottom_left,
                                     place::PlacementStartCorner::top_right);

  const auto outcome = solve_actual_polygon_constructive_case(
      candidate_strategy, 1200U, kActualPolygonTimeCapMs, start_corner);
  INFO(outcome.summary);
  REQUIRE(outcome.result.has_value());
  const auto &result = *outcome.result;
  const auto placed = total_placed(result);

  REQUIRE(result.stop_reason == StopReason::completed);
  REQUIRE(result.layout.unplaced_piece_ids.empty());
  REQUIRE(result.placed_parts() == kBaselinePieceCount);
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}
