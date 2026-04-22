// MTG engine bug reproductions.
//
// Each TEST_CASE here pins a single, minimal repro for a real engine bug
// surfaced by the MTG nesting matrix work. They are hidden from the default
// lane (`[.]`) and tagged `[broken]` so they can be selected on demand and
// CI can later mark them as expected-failures. Asserting the EXPECTED
// behavior means each case currently FAILS — that failure IS the repro.
// When an engine fix lands, the case will pass; at that point remove the
// `[broken]` tag (and the `[.]` if appropriate) to bring it into the default
// lane.

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

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

}  // namespace

// -----------------------------------------------------------------------------
// Repro 1: bb-shelf with pinned-to-source-bed model leaves one piece unplaced.
//
// Engine model: maintain_bed_assignment=false + allow_part_overflow=false
// pins each piece to its source bed (per nesting_adapter.cpp). Each MTG
// fixture piece individually fits its source bed, yet bb-shelf ends with
// 17/18 placed.
// -----------------------------------------------------------------------------
TEST_CASE("REPRO: bb-shelf pinned packing leaves one piece unplaced",
          "[mtg][engine-bug-repro][bb-pinned][.][broken]") {
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
  INFO("bb-shelf pinned placed " << placed << " of "
                                 << kBaselinePieceCount
                                 << " (unplaced "
                                 << solved.value().layout.unplaced_piece_ids.size()
                                 << ")");
  INFO("If this passes, the engine bug has been fixed; please remove the "
       "[broken] tag.");
  // Expected: 18; observed: 17.
  REQUIRE(placed == static_cast<std::size_t>(kBaselinePieceCount));
}

// -----------------------------------------------------------------------------
// Repro 2: allowed_bin_ids violated when selected_bin_ids restricts to the
// other bed. Per-piece allowed_bin_ids must be honored regardless of the
// bin-selection filter; the offending piece should land in
// `unplaced_piece_ids`, not be silently relocated to the selected bed.
// -----------------------------------------------------------------------------
TEST_CASE("REPRO: allowed_bin_ids violated when selected_bin_ids restricts to other bed",
          "[mtg][engine-bug-repro][allowed-bin-ids][.][broken]") {
  const auto fixture = load_mtg_fixture();

  SECTION("selected={bed1}, override one bed-2-source piece to allowed={bed2}") {
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

  SECTION("selected={bed2}, override one bed-1-source piece to allowed={bed1}") {
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
// In ~9 of 108 BB-heuristic cells (heuristic × spacing × policy ×
// deterministic_attempts) two placements on bin 1 are too close — the gap
// between their AABBs is less than the requested `part_spacing_mm`. This
// repros the cluster of 9 spacing-violation failures with the canonical
// tuple identified by enumerating Section C cells:
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
TEST_CASE("REPRO: bb heuristic emits overlapping placements",
          "[mtg][engine-bug-repro][bb-overlap][.][broken]") {
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

  // Spacing-aware overlap: pieces overlap if the gap between their AABBs
  // is less than `part_spacing_mm`. Mirrors the fixture's `boxes_overlap`
  // helper. A small slack (1e-3 mm) accounts for floating-point noise.
  const double effective_spacing = std::max(0.0, options.part_spacing_mm - 1e-3);
  auto overlap_with_spacing = [effective_spacing](const geom::Box2 &a,
                                                  const geom::Box2 &b) {
    const double half = effective_spacing * 0.5;
    return !(a.max.x + half <= b.min.x - half ||
             b.max.x + half <= a.min.x - half ||
             a.max.y + half <= b.min.y - half ||
             b.max.y + half <= a.min.y - half);
  };

  for (const auto &bin : solved.value().layout.bins) {
    if (bin.bin_id != kBed1Id) {
      continue;
    }
    for (std::size_t i = 0; i < bin.placements.size(); ++i) {
      const auto box_a = aabb_of(bin.placements[i].polygon);
      for (std::size_t j = i + 1; j < bin.placements.size(); ++j) {
        const auto box_b = aabb_of(bin.placements[j].polygon);
        INFO("bin1 spacing-violation pair pieces "
             << bin.placements[i].placement.piece_id << " ["
             << box_a.min.x << "," << box_a.min.y << "]-[" << box_a.max.x
             << "," << box_a.max.y << "] vs "
             << bin.placements[j].placement.piece_id << " ["
             << box_b.min.x << "," << box_b.min.y << "]-[" << box_b.max.x
             << "," << box_b.max.y << "] (required spacing "
             << options.part_spacing_mm << " mm)");
        REQUIRE_FALSE(overlap_with_spacing(box_a, box_b));
      }
    }
  }
}
