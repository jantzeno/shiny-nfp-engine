#include <catch2/catch_test_macros.hpp>

#include "packing/bin_identity.hpp"
#include "packing/sparrow/adapters/layout_adapter.hpp"
#include "packing/sparrow/adapters/progress_adapter.hpp"
#include "packing/sparrow/eval/constructive_seed_evaluator.hpp"
#include "support/sparrow_harness.hpp"

namespace {

auto make_layout() -> shiny::nesting::pack::Layout {
  shiny::nesting::pack::Layout layout;
  layout.bins.push_back({
      .bin_id = 12,
      .identity =
          {
              .lifecycle = shiny::nesting::pack::BinLifecycle::user_created,
              .source_request_bin_id = 12,
          },
      .utilization =
          {
              .bin_id = 12,
              .placement_count = 1,
              .occupied_area = 16.0,
              .container_area = 64.0,
              .utilization = 0.25,
          },
  });
  shiny::nesting::pack::PlacementTraceEntry entry;
  entry.piece_id = 501;
  entry.bin_id = 12;
  entry.resolved_rotation = shiny::nesting::geom::ResolvedRotation{90.0};
  entry.translation = shiny::nesting::geom::Point2{1.0, 2.0};
  entry.mirrored = true;
  entry.phase = shiny::nesting::pack::ConstructivePlacementPhase::gap_fill;
  layout.placement_trace.push_back(entry);
  return layout;
}

auto make_constructive_replay() -> shiny::nesting::ConstructiveReplay {
  shiny::nesting::ConstructiveReplay constructive;
  constructive.frontier_changes.push_back({
      .previous_bin_id = std::nullopt,
      .next_bin_id = 12,
      .piece_id = 501,
  });
  return constructive;
}

} // namespace

TEST_CASE("layout adapter converts fill-first result to a Sparrow seed solution",
          "[sparrow][constructive-seed]") {
  const auto layout = make_layout();
  const auto constructive = make_constructive_replay();

  const auto seed = shiny::nesting::pack::sparrow::adapters::to_seed_solution(
      layout, constructive);
  REQUIRE(seed.placements.size() == 1U);
  CHECK(seed.placements.front().piece_id == 501U);
  CHECK(seed.placements.front().phase ==
        shiny::nesting::pack::ConstructivePlacementPhase::gap_fill);

  shiny::nesting::pack::sparrow::PortSolution port_solution{
      .layout = layout,
      .constructive = constructive,
      .stop_reason = shiny::nesting::StopReason::completed,
      .effective_seed = 77,
  };
  const auto nesting_result =
      shiny::nesting::pack::sparrow::adapters::to_nesting_result(
          port_solution, shiny::nesting::StrategyKind::metaheuristic_search,
          1U);
  CHECK(nesting_result.effective_seed == 77U);
  CHECK(nesting_result.constructive.frontier_changes.size() == 1U);

  const auto progress =
      shiny::nesting::pack::sparrow::adapters::to_profile_progress_snapshot({
          .current_layout = layout,
          .best_layout = layout,
          .active_bin_id = 12,
          .elapsed_time_milliseconds = 5,
          .improved = true,
      });
  CHECK(progress.placed_count == 1U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      progress.utilization_percent, 25.0));
}

TEST_CASE("layout helper confirms identical layouts are equal under the same-layout predicate",
          "[sparrow][constructive-seed]") {
  const auto layout = make_layout();
  CHECK(shiny::nesting::test::sparrow::same_layout(layout, layout));
}

TEST_CASE("layout adapter round-trips warm-start seed layout without data loss",
          "[sparrow][constructive-seed]") {
  auto layout = make_layout();
  layout.unplaced_piece_ids.push_back(900);
  const auto seed = shiny::nesting::pack::sparrow::adapters::to_seed_solution(
      layout, make_constructive_replay());

  const auto round_trip =
      shiny::nesting::pack::sparrow::adapters::to_layout(seed);

  CHECK(shiny::nesting::test::sparrow::same_layout(layout, round_trip));
  REQUIRE(round_trip.placement_trace.size() == 1U);
  CHECK(round_trip.placement_trace.front().resolved_rotation.degrees == 90.0);
  CHECK(round_trip.placement_trace.front().mirrored);
  REQUIRE(round_trip.unplaced_piece_ids.size() == 1U);
  CHECK(round_trip.unplaced_piece_ids.front() == 900U);
}

TEST_CASE("constructive-seed evaluator scores complete warm starts higher than partial ones",
          "[sparrow][constructive-seed]") {
  const auto constructive = make_constructive_replay();
  const auto full_seed =
      shiny::nesting::pack::sparrow::adapters::to_seed_solution(make_layout(),
                                                                constructive);

  auto partial_layout = make_layout();
  partial_layout.unplaced_piece_ids.push_back(777);
  partial_layout.bins.front().utilization.placement_count = 0;
  partial_layout.bins.front().utilization.occupied_area = 8.0;
  partial_layout.bins.front().utilization.utilization = 0.125;
  const auto partial_seed =
      shiny::nesting::pack::sparrow::adapters::to_seed_solution(partial_layout,
                                                                constructive);

  const auto full_eval =
      shiny::nesting::pack::sparrow::eval::evaluate_constructive_seed(
          full_seed);
  const auto partial_eval =
      shiny::nesting::pack::sparrow::eval::evaluate_constructive_seed(
          partial_seed);

  CHECK(full_eval.complete_warm_start);
  CHECK_FALSE(partial_eval.complete_warm_start);
  CHECK(full_eval.replay.placement_count == 1U);
  CHECK(partial_eval.replay.unplaced_count == 1U);
  CHECK(full_eval.score > partial_eval.score);
}

TEST_CASE("constructive-seed evaluator reports replay metadata with deterministic warm-start signatures",
          "[sparrow][constructive-seed]") {
  auto layout = make_layout();
  layout.unplaced_piece_ids.push_back(808);
  auto constructive = make_constructive_replay();
  constructive.overflow_events.push_back({
      .template_bin_id = 12,
      .overflow_bin_id = 19,
      .source_request_bin_id = 12,
  });
  constructive.exhaustion_events.push_back({
      .piece_id = 808,
      .frontier_bin_id = 12,
      .decision =
          shiny::nesting::ConstructiveFrontierExhaustionDecision::exhausted,
  });

  const auto seed = shiny::nesting::pack::sparrow::adapters::to_seed_solution(
      layout, constructive);
  const auto first =
      shiny::nesting::pack::sparrow::eval::evaluate_constructive_seed(seed);
  const auto second =
      shiny::nesting::pack::sparrow::eval::evaluate_constructive_seed(seed);

  auto altered_layout = layout;
  altered_layout.placement_trace.front().translation =
      shiny::nesting::geom::Point2{2.0, 2.0};
  const auto altered_seed =
      shiny::nesting::pack::sparrow::adapters::to_seed_solution(altered_layout,
                                                                constructive);
  const auto altered =
      shiny::nesting::pack::sparrow::eval::evaluate_constructive_seed(
          altered_seed);

  CHECK(first.replay.frontier_change_count == 1U);
  CHECK(first.replay.overflow_event_count == 1U);
  CHECK(first.replay.exhaustion_event_count == 1U);
  REQUIRE(first.replay.terminal_bin_id.has_value());
  CHECK(*first.replay.terminal_bin_id == 12U);
  CHECK(first.warm_start_signature == second.warm_start_signature);
  CHECK(first.warm_start_signature != altered.warm_start_signature);
}