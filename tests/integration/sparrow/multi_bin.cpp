#include <algorithm>
#include <set>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "internal/request_normalization.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "packing/sparrow/adapters/layout_adapter.hpp"
#include "packing/sparrow/adapters/request_adapter.hpp"
#include "packing/sparrow/multi_bin.hpp"
#include "support/sparrow_harness.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::ProductionSearchConfig;
using shiny::nesting::ProfileProgressSnapshot;
using shiny::nesting::ProfileRequest;
using shiny::nesting::ProfileSolveControl;
using shiny::nesting::SolveControl;
using shiny::nesting::SolveProfile;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto rectangle(double width, double height) -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}});
}

auto make_assignment_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.execution.allow_part_overflow = false;
  request.execution.irregular.piece_ordering =
      shiny::nesting::PieceOrdering::input;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(4.0, 4.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 2,
      .polygon = rectangle(4.0, 4.0),
  });

  request.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = rectangle(4.0, 4.0),
      .allowed_bin_ids = {1},
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 2,
      .polygon = rectangle(4.0, 4.0),
      .allowed_bin_ids = {2},
  });
  return request;
}

auto make_overflow_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};
  request.execution.allow_part_overflow = true;
  request.execution.irregular.piece_ordering =
      shiny::nesting::PieceOrdering::input;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(4.0, 4.0),
      .stock = 1,
  });

  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 11,
      .polygon = rectangle(4.0, 4.0),
  });
  return request;
}

auto make_stock3_request(StrategyKind strategy) -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = strategy;
  request.execution.default_rotations = {{0.0}};
  request.execution.allow_part_overflow = false;
  request.execution.irregular.piece_ordering =
      shiny::nesting::PieceOrdering::input;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(4.0, 4.0),
      .stock = 3,
  });

  for (std::uint32_t id = 20; id <= 22; ++id) {
    request.pieces.push_back(PieceRequest{
        .piece_id = id,
        .polygon = rectangle(4.0, 4.0),
    });
  }
  return request;
}

auto make_profile_assignment_request(const SolveProfile profile)
    -> ProfileRequest {
  ProfileRequest request;
  request.profile = profile;
  request.time_limit_milliseconds = 1'000U;
  request.maintain_bed_assignment = true;
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(4.0, 4.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 2,
      .polygon = rectangle(4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = rectangle(4.0, 4.0),
      .assigned_bin_id = 1,
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 2,
      .polygon = rectangle(4.0, 4.0),
      .assigned_bin_id = 2,
  });
  return request;
}

auto find_seed_placement(
    const shiny::nesting::pack::sparrow::SeedSolution &seed,
    const std::uint32_t piece_id)
    -> const shiny::nesting::pack::sparrow::PortPlacement * {
  const auto it = std::find_if(seed.placements.begin(), seed.placements.end(),
                               [piece_id](const auto &placement) {
                                 return placement.piece_id == piece_id;
                               });
  return it == seed.placements.end() ? nullptr : &*it;
}

} // namespace

TEST_CASE("fill-first warm-start export preserves explicit assignment bin ids",
          "[sparrow][multi-bin][warm-start]") {
  const auto solved = shiny::nesting::solve(make_assignment_request(),
                                            SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().validation.valid);
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());

  const auto seed =
      shiny::nesting::pack::constructive::export_sparrow_seed(solved.value());
  const auto round_trip =
      shiny::nesting::pack::sparrow::adapters::to_layout(seed);

  REQUIRE(seed.placements.size() == 2U);
  const auto *first = find_seed_placement(seed, 1U);
  const auto *second = find_seed_placement(seed, 2U);
  REQUIRE(first != nullptr);
  REQUIRE(second != nullptr);
  CHECK(first->bin_id == 1U);
  CHECK(second->bin_id == 2U);
  CHECK(seed.multi_bin.active_bin_ids == std::vector<std::uint32_t>({1U, 2U}));
  CHECK(seed.multi_bin.unplaced_piece_ids.empty());
  CHECK(seed.multi_bin.overflow_lineage.empty());
  CHECK(shiny::nesting::test::sparrow::same_layout(solved.value().layout,
                                                   round_trip));
}

TEST_CASE("fill-first warm-start export preserves overflow lineage in the seed solution",
          "[sparrow][multi-bin][warm-start]") {
  const auto solved = shiny::nesting::solve(make_overflow_request(),
                                            SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().validation.valid);
  REQUIRE(solved.value().layout.unplaced_piece_ids.empty());

  const auto seed =
      shiny::nesting::pack::constructive::export_sparrow_seed(solved.value());
  const auto round_trip =
      shiny::nesting::pack::sparrow::adapters::to_layout(seed);

  CHECK(seed.constructive.overflow_events.size() == 1U);
  REQUIRE(seed.multi_bin.active_bin_ids.size() == 2U);
  REQUIRE(seed.multi_bin.overflow_lineage.size() == 1U);
  CHECK(seed.multi_bin.unplaced_piece_ids.empty());
  CHECK(seed.multi_bin.overflow_lineage.front().template_bin_id == 1U);
  CHECK(seed.multi_bin.overflow_lineage.front().source_request_bin_id == 1U);
  REQUIRE(round_trip.bins.size() == 2U);
  const auto overflow_it = std::find_if(
      round_trip.bins.begin(), round_trip.bins.end(), [](const auto &bin) {
        return bin.identity.lifecycle ==
               shiny::nesting::pack::BinLifecycle::engine_overflow;
      });
  REQUIRE(overflow_it != round_trip.bins.end());
  REQUIRE(overflow_it->identity.template_bin_id.has_value());
  CHECK(*overflow_it->identity.template_bin_id == 1U);
  CHECK(seed.multi_bin.overflow_lineage.front().overflow_bin_id ==
        overflow_it->bin_id);
  CHECK(shiny::nesting::test::sparrow::same_layout(solved.value().layout,
                                                   round_trip));
}

TEST_CASE("multi-bin legality classifier gates inter-bin overlap work by assignment and lineage rules",
          "[sparrow][multi-bin][legality]") {
  const auto normalized =
      shiny::nesting::normalize_request(make_overflow_request());
  REQUIRE(normalized.has_value());

  const auto adapted =
      shiny::nesting::pack::sparrow::adapters::to_port_instance(
          normalized.value());
  const auto solved = shiny::nesting::solve(make_overflow_request(),
                                            SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  const auto seed =
      shiny::nesting::pack::constructive::export_sparrow_seed(solved.value());

  REQUIRE(seed.multi_bin.overflow_lineage.size() == 1U);
  const auto overflow_bin_id =
      seed.multi_bin.overflow_lineage.front().overflow_bin_id;

  const auto regular = shiny::nesting::pack::sparrow::classify_bin_legality(
      adapted, seed.multi_bin, 10U, 1U);
  const auto overflow = shiny::nesting::pack::sparrow::classify_bin_legality(
      adapted, seed.multi_bin, 11U, overflow_bin_id);

  CHECK(regular.allowed);
  CHECK(regular.selected_bin);
  CHECK(regular.assignment_allowed);
  CHECK(overflow.allowed);
  CHECK(overflow.selected_bin);
  CHECK(overflow.assignment_allowed);
  CHECK(overflow.overflow_lineage_allowed);
  CHECK_FALSE(shiny::nesting::pack::sparrow::inter_bin_collision_relevant(
      1U, overflow_bin_id));

  const auto pinned_request =
      make_profile_assignment_request(SolveProfile::maximum_search);
  const auto pinned_or = shiny::nesting::pack::sparrow::adapters::adapt_request(
      pinned_request, ProfileSolveControl{.random_seed = 17U});
  REQUIRE(pinned_or.has_value());

  const auto pinned_decision =
      shiny::nesting::pack::sparrow::classify_bin_legality(
          pinned_or.value().instance,
          shiny::nesting::pack::sparrow::PortMultiBinState{
              .active_bin_ids = {1U, 2U},
              .overflow_lineage = {{.template_bin_id = 1U,
                                    .overflow_bin_id = 1001U,
                                    .source_request_bin_id = 1U}},
          },
          1U, 1001U);
  CHECK_FALSE(pinned_decision.allowed);
  CHECK_FALSE(pinned_decision.overflow_lineage_allowed);
}

TEST_CASE("profile solve warm-starts multi-bin Sparrow search from the fill-first constructive assignment",
          "[sparrow][multi-bin][warm-start]") {
  const auto profile_request =
      make_profile_assignment_request(SolveProfile::balanced);
  const auto translated = shiny::nesting::to_nesting_request(profile_request);
  REQUIRE(translated.has_value());

  const auto normalized = shiny::nesting::normalize_request(translated.value());
  REQUIRE(normalized.has_value());

  shiny::nesting::pack::constructive::FillFirstEngine constructive_engine;
  const auto constructive = constructive_engine.solve(
      normalized.value(), SolveControl{.random_seed = 21U});
  REQUIRE(constructive.has_value());

  std::vector<ProfileProgressSnapshot> progress;
  const auto solved = shiny::nesting::solve(
      profile_request, ProfileSolveControl{
                           .on_progress =
                               [&](const ProfileProgressSnapshot &snapshot) {
                                 progress.push_back(snapshot);
                               },
                           .random_seed = 21U,
                       });

  REQUIRE(solved.has_value());
  REQUIRE_FALSE(progress.empty());
  CHECK(shiny::nesting::test::sparrow::same_layout(
      constructive.value().layout, progress.front().current_layout));
  CHECK(progress.front().placed_count == constructive.value().placed_parts());
  CHECK(solved.value().validation.valid);
}

TEST_CASE("multi-bin move classifier enumerates legal move classes and weak-bin targets",
          "[sparrow][multi-bin][legality]") {
  const auto normalized =
      shiny::nesting::normalize_request(make_overflow_request());
  REQUIRE(normalized.has_value());
  const auto instance =
      shiny::nesting::pack::sparrow::adapters::to_port_instance(
          normalized.value());
  const auto solved = shiny::nesting::solve(make_overflow_request(),
                                            SolveControl{.random_seed = 0});
  REQUIRE(solved.has_value());
  const auto seed =
      shiny::nesting::pack::constructive::export_sparrow_seed(solved.value());

  const auto active_bin_id = seed.multi_bin.active_bin_ids.front();
  const auto overflow_bin_id =
      seed.multi_bin.overflow_lineage.front().overflow_bin_id;
  const auto moves = shiny::nesting::pack::sparrow::legal_move_classes(
      instance, seed.multi_bin, 11U, overflow_bin_id);
  const auto weak_bins =
      shiny::nesting::pack::sparrow::weak_bin_candidates(solved.value().layout);

  CHECK(std::find(moves.begin(), moves.end(),
                  shiny::nesting::pack::sparrow::MultiBinMoveClass::
                      intra_bin_translate) != moves.end());
  CHECK(std::find(moves.begin(), moves.end(),
                  shiny::nesting::pack::sparrow::MultiBinMoveClass::
                      consolidate_overflow) != moves.end());
  CHECK(
      std::find(moves.begin(), moves.end(),
                shiny::nesting::pack::sparrow::MultiBinMoveClass::empty_bin) !=
      moves.end());
  REQUIRE(weak_bins.size() == 2U);
  CHECK((weak_bins.front() == active_bin_id ||
         weak_bins.front() == overflow_bin_id));
}

TEST_CASE("multi-bin objective scores bin-count reductions ahead of local compaction tradeoffs",
          "[sparrow][multi-bin][objective]") {
  const auto fewer_bins =
      shiny::nesting::pack::sparrow::optimize::ObjectiveValue{
          .metrics = {.placed_parts = 2U,
                      .bin_count = 1U,
                      .strip_length = 12.0,
                      .utilization = 0.40},
          .multi_bin = {.active_bin_count = 1U,
                        .active_bin_compaction = 12.0,
                        .active_bin_utilization = 0.40},
      };
  const auto tighter_two_bin =
      shiny::nesting::pack::sparrow::optimize::ObjectiveValue{
          .metrics = {.placed_parts = 2U,
                      .bin_count = 2U,
                      .strip_length = 8.0,
                      .utilization = 0.80},
          .multi_bin = {.active_bin_count = 2U,
                        .active_bin_compaction = 8.0,
                        .active_bin_utilization = 0.80},
      };

  CHECK(shiny::nesting::pack::sparrow::bin_count_tradeoff_better(
      fewer_bins, tighter_two_bin));
  CHECK_FALSE(shiny::nesting::pack::sparrow::bin_count_tradeoff_better(
      tighter_two_bin, fewer_bins));
}

TEST_CASE("stock=3 bin definition is expanded to 3 copies with pieces distributed across all copies",
          "[sparrow][multi-bin][stock]") {
  auto run_and_check = [](NestingRequest request) {
    REQUIRE(request.is_valid());
    const auto result =
        shiny::nesting::solve(request, SolveControl{.operation_limit = 200U,
                                                    .random_seed = 7U});
    REQUIRE(result.has_value());
    CHECK(result.value().all_parts_placed());
    CHECK(result.value().validation.valid);

    // Collect distinct bin ids across all placements — stock=3 with 3
    // exactly-fitting pieces must span all 3 bin copies.
    std::set<std::uint32_t> used_bins;
    for (const auto &entry : result.value().layout.placement_trace) {
      used_bins.insert(entry.bin_id);
    }
    CHECK(used_bins.size() == 3U);
    CHECK(result.value().layout.bins.size() == 3U);
  };

  SECTION("bounding_box distributes pieces across all 3 stock copies") {
    run_and_check(make_stock3_request(StrategyKind::bounding_box));
  }

  SECTION("metaheuristic_search brkga distributes pieces across all 3 stock copies") {
    auto request = make_stock3_request(StrategyKind::metaheuristic_search);
    request.execution.production_optimizer = ProductionOptimizerKind::brkga;
    // Keep all defaults (population_size=24, elite_count=6, etc. are all valid)
    // but cap iterations to keep the test fast. max_iterations must be >= the
    // default infeasible_rollback_after (2), so use 5.
    ProductionSearchConfig brkga{};
    brkga.max_iterations = 5U;
    request.execution.production = brkga;
    run_and_check(std::move(request));
  }
}