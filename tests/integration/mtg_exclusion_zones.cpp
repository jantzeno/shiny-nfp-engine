// MTG nesting matrix — exclusion zone forces redistribution.
//
// A canonical "half of bed1" exclusion is asserted to (a) push parts onto
// bed2 when overflow is allowed and (b) leave originally-pinned bed1 parts
// unplaced when overflow is denied (without ever migrating them to bed2).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <unordered_set>
#include <vector>

#include "geometry/polygon.hpp"
#include "geometry/types.hpp"
#include "polygon_ops/boolean_ops.hpp"

#include "solve.hpp"
#include "support/mtg_fixture.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

constexpr std::uint64_t kSeed = 23;
constexpr std::size_t kIrregularIterationLimit = 2;
constexpr std::uint32_t kOverlapPairPieceA = 11U;
constexpr std::uint32_t kOverlapPairPieceB = 14U;

[[nodiscard]] auto count_placements_on_bed(const NestingResult &result,
                                           std::uint32_t bin_id)
    -> std::size_t {
  for (const auto &bin : result.layout.bins) {
    if (bin.bin_id == bin_id) {
      return bin.placements.size();
    }
  }
  return 0;
}

[[nodiscard]] auto baseline_bed2_count(const MtgFixture &fixture)
    -> std::size_t {
  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
  options.selected_bin_ids = {};
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  SolveControl control{};
  control.random_seed = kSeed;

  auto solved = solve(request, control);
  REQUIRE(solved.has_value());
  return count_placements_on_bed(solved.value(), kBed2Id);
}

[[nodiscard]] auto polygon_aabb_local(const geom::PolygonWithHoles &polygon)
    -> geom::Box2 {
  geom::Box2 box{{0.0, 0.0}, {0.0, 0.0}};
  if (polygon.outer.empty()) {
    return box;
  }
  box.min = polygon.outer.front();
  box.max = polygon.outer.front();
  for (const auto &p : polygon.outer) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
  }
  return box;
}

[[nodiscard]] auto rects_strictly_overlap(const geom::Box2 &a,
                                          const geom::Box2 &b) -> bool {
  return !(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y ||
           b.max.y <= a.min.y);
}

[[nodiscard]] auto base_solve_control(const MtgRequestOptions &options)
    -> SolveControl {
  SolveControl control{};
  control.random_seed = kSeed;

  if (options.strategy == StrategyKind::sequential_backtrack) {
    control.seed_mode = SeedProgressionMode::increment;
  }

  if (options.strategy == StrategyKind::sequential_backtrack ||
      options.strategy == StrategyKind::metaheuristic_search) {
    control.operation_limit = kIrregularIterationLimit;
  }
  return control;
}

[[nodiscard]] auto exact_intersection_area(const geom::PolygonWithHoles &lhs,
                                           const geom::PolygonWithHoles &rhs)
    -> double {
  return geom::polygon_area_sum(poly::intersection_polygons(lhs, rhs));
}

[[nodiscard]] auto find_piece_on_bin(const NestingResult &result,
                                     std::uint32_t piece_id,
                                     std::uint32_t bin_id)
    -> const pack::PlacedPiece * {
  for (const auto &bin : result.layout.bins) {
    if (bin.bin_id != bin_id) {
      continue;
    }
    const auto it = std::find_if(bin.placements.begin(), bin.placements.end(),
                                 [piece_id](const pack::PlacedPiece &placed) {
                                   return placed.placement.piece_id == piece_id;
                                 });
    if (it != bin.placements.end()) {
      return &*it;
    }
  }
  return nullptr;
}

} // namespace

TEST_CASE("mtg exclusion zone on bed1 forces overflow to bed2",
          "[mtg][nesting-matrix][exclusion-zones][slow]") {
  const auto fixture = load_mtg_fixture();
  const auto rect = bed1_half_block_exclusion();
  const auto exclusion = make_rect_exclusion(
      99, kBed1Id, rect.min_x, rect.min_y, rect.max_x, rect.max_y);

  const auto baseline_bed2 = baseline_bed2_count(fixture);

  auto run_overflow_case = [&](double spacing_mm, auto configure_algorithm) {
    MtgRequestOptions options{};
    configure_algorithm(options);
    options.selected_bin_ids = {};
    options.allow_part_overflow = true;
    options.maintain_bed_assignment = false;
    options.part_spacing_mm = spacing_mm;
    options.bed1_exclusions = {exclusion};

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    const SolveControl control = base_solve_control(options);

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    const auto &result = solved.value();

    ExpectedOutcome expected{};
    expected.expected_placed_count = kBaselinePieceCount;
    expected.exclusions_to_check = {exclusion};
    validate_layout(fixture, request, options, result, expected);

    const auto bed2_count = count_placements_on_bed(result, kBed2Id);
    REQUIRE(bed2_count > baseline_bed2);

    return result;
  };

  auto run_blocked_case = [&](bool maintain) {
    std::unordered_set<std::uint32_t> expected_blocked;
    {
      MtgRequestOptions baseline_opts{};
      baseline_opts.strategy = StrategyKind::bounding_box;
      baseline_opts.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
      baseline_opts.selected_bin_ids = {};
      baseline_opts.allow_part_overflow = false;
      baseline_opts.maintain_bed_assignment = maintain;
      baseline_opts.part_spacing_mm = 0.0;

      const auto baseline_request = make_request(fixture, baseline_opts);
      REQUIRE(baseline_request.is_valid());

      const SolveControl baseline_control = base_solve_control(baseline_opts);

      auto baseline_solved = solve(baseline_request, baseline_control);
      REQUIRE(baseline_solved.has_value());
      const auto &baseline = baseline_solved.value();

      const geom::Box2 excl_box{{rect.min_x, rect.min_y},
                                {rect.max_x, rect.max_y}};
      for (const auto &bin : baseline.layout.bins) {
        if (bin.bin_id != kBed1Id) {
          continue;
        }
        for (const auto &placed : bin.placements) {
          const auto pid = placed.placement.piece_id;
          const auto it = std::find_if(
              fixture.pieces.begin(), fixture.pieces.end(),
              [pid](const MtgPiece &p) { return p.piece_id == pid; });
          if (it == fixture.pieces.end() || it->source_bed_id != kBed1Id) {
            continue;
          }
          const auto piece_box = polygon_aabb_local(placed.polygon);
          if (rects_strictly_overlap(piece_box, excl_box)) {
            expected_blocked.insert(pid);
          }
        }
      }
    }

    MtgRequestOptions options{};
    options.strategy = StrategyKind::bounding_box;
    options.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
    options.selected_bin_ids = {};
    options.allow_part_overflow = false;
    options.maintain_bed_assignment = maintain;
    options.part_spacing_mm = 0.0;
    options.bed1_exclusions = {exclusion};

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    const SolveControl control = base_solve_control(options);

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    const auto &result = solved.value();

    ExpectedOutcome expected{};
    expected.exclusions_to_check = {exclusion};
    validate_layout(fixture, request, options, result, expected);

    REQUIRE_FALSE(result.layout.unplaced_piece_ids.empty());

    const std::unordered_set<std::uint32_t> unplaced(
        result.layout.unplaced_piece_ids.begin(),
        result.layout.unplaced_piece_ids.end());
    INFO("expected_blocked.size()=" << expected_blocked.size()
                                    << " unplaced.size()=" << unplaced.size());
    for (const auto pid : expected_blocked) {
      INFO("expected-blocked piece_id=" << pid);
      REQUIRE(unplaced.contains(pid));
    }

    for (const auto &bin : result.layout.bins) {
      for (const auto &placed : bin.placements) {
        const auto pid = placed.placement.piece_id;
        INFO("placement piece_id=" << pid << " on bin=" << bin.bin_id);
        REQUIRE(!expected_blocked.contains(pid));
      }
    }

    for (const auto &bin : result.layout.bins) {
      if (bin.bin_id != kBed2Id) {
        continue;
      }
      for (const auto &placed : bin.placements) {
        const auto pid = placed.placement.piece_id;
        const auto it = std::find_if(
            fixture.pieces.begin(), fixture.pieces.end(),
            [pid](const MtgPiece &p) { return p.piece_id == pid; });
        REQUIRE(it != fixture.pieces.end());
        REQUIRE(it->source_bed_id == kBed2Id);
      }
    }
  };

  SECTION("overflow + bounding_box, spacing 0mm") {
    run_overflow_case(0.0, [](MtgRequestOptions &o) {
      o.strategy = StrategyKind::bounding_box;
      o.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
      o.allow_part_overflow = true;
    });
  }

  SECTION("overflow + bounding_box, spacing 1mm") {
    run_overflow_case(1.0, [](MtgRequestOptions &o) {
      o.strategy = StrategyKind::bounding_box;
      o.bounding_box.heuristic = pack::BoundingBoxHeuristic::shelf;
      o.allow_part_overflow = true;
    });
  }

  SECTION("overflow + sequential_backtrack, spacing 0mm") {
    // Test fails with seed 23
    // Test succeeds with seed 24
    const NestingResult result = run_overflow_case(0.0, [](MtgRequestOptions &o) {
      o.strategy = StrategyKind::sequential_backtrack;
      o.allow_part_overflow = true;
    });
    REQUIRE(result.effective_seed == 24U);
  }

  SECTION("overflow + metaheuristic_search-brkga, spacing 0mm") {
    run_overflow_case(0.0, [](MtgRequestOptions &o) {
      o.strategy = StrategyKind::metaheuristic_search;
      o.production_optimizer = ProductionOptimizerKind::brkga;
      o.production.max_iterations = 2;
      o.production.population_size = 4;
      o.production.elite_count = 1;
      o.production.mutant_count = 1;
    });
  }

  SECTION("no-overflow + bounding_box leaves blocked pieces unplaced") {
    run_blocked_case(/*maintain=*/false);
  }

  SECTION(
      "maintain + no-overflow + bounding_box leaves blocked pieces unplaced") {
    run_blocked_case(/*maintain=*/true);
  }
}

TEST_CASE("sequential exclusion-zone overlap attempt advances to next seed",
          "[mtg][nesting-matrix][seeds][sequential-backtrack][exclusion-zones][slow]") {
  const auto fixture = load_mtg_fixture();
  const auto rect = bed1_half_block_exclusion();
  const auto exclusion = make_rect_exclusion(
      99, kBed1Id, rect.min_x, rect.min_y, rect.max_x, rect.max_y);

  auto run_repro = [&](const char *label, const SolveControl &control,
                       const bool expect_overlap) {
    MtgRequestOptions options{};
    options.strategy = StrategyKind::sequential_backtrack;
    options.selected_bin_ids = {};
    options.allow_part_overflow = true;
    options.maintain_bed_assignment = false;
    options.part_spacing_mm = 0.0;
    options.bed1_exclusions = {exclusion};

    const auto request = make_request(fixture, options);
    REQUIRE(request.is_valid());

    auto solved = solve(request, control);
    REQUIRE(solved.has_value());
    REQUIRE(solved.value().layout.unplaced_piece_ids.empty());
    REQUIRE(solved.value().layout.placement_trace.size() ==
            kBaselinePieceCount);

    const auto *piece11 =
        find_piece_on_bin(solved.value(), kOverlapPairPieceA, kBed2Id);
    const auto *piece14 =
        find_piece_on_bin(solved.value(), kOverlapPairPieceB, kBed2Id);
    REQUIRE(piece11 != nullptr);
    REQUIRE(piece14 != nullptr);

    const double overlap_area =
        exact_intersection_area(piece11->polygon, piece14->polygon);
    INFO(label << " piece11 tx=" << piece11->placement.translation.x
               << " ty=" << piece11->placement.translation.y
               << " rot=" << piece11->resolved_rotation.degrees
               << " piece14 tx=" << piece14->placement.translation.x
               << " ty=" << piece14->placement.translation.y
               << " rot=" << piece14->resolved_rotation.degrees
               << " overlap_area=" << overlap_area);
    if (expect_overlap) {
      REQUIRE(overlap_area > 0.0);
    } else {
      REQUIRE(overlap_area <= 1e-9);
      REQUIRE(solved.value().effective_seed == 24U);
    }
  };

  SECTION("default multi-start control") {
    MtgRequestOptions control_options{};
    control_options.strategy = StrategyKind::sequential_backtrack;
    run_repro("multi-start", base_solve_control(control_options),
              /*expect_overlap=*/false);
  }

  SECTION("multi-start disabled via random_seed=0") {
    MtgRequestOptions control_options{};
    control_options.strategy = StrategyKind::sequential_backtrack;
    auto control = base_solve_control(control_options);
    control.random_seed = 0;
    control.operation_limit = 0;
    run_repro("single-start", control, /*expect_overlap=*/true);
  }
}
