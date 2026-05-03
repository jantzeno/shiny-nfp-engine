#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

#include "api/dto.hpp"
#include "internal/request_normalization.hpp"
#include "packing/bin_identity.hpp"
#include "packing/sparrow/adapters/progress_adapter.hpp"
#include "packing/sparrow/adapters/request_adapter.hpp"
#include "packing/sparrow/optimize/disruption.hpp"
#include "packing/sparrow/optimize/exploration_phase.hpp"
#include "packing/sparrow/runtime/cancellation.hpp"
#include "packing/sparrow/runtime/progress.hpp"
#include "runtime/cancellation.hpp"
#include "runtime/timing.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"
#include "support/sparrow_harness.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProfileProgressSnapshot;
using shiny::nesting::ProfileRequest;
using shiny::nesting::ProfileSolveControl;
using shiny::nesting::SolveProfile;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto rectangle(double width, double height) -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}});
}

auto make_exploration_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy =
      shiny::nesting::StrategyKind::bounding_box;
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
      .piece_id = 101,
      .polygon = rectangle(4.0, 4.0),
      .allowed_bin_ids = {1U},
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(4.0, 4.0),
      .allowed_bin_ids = {2U},
  });
  return request;
}

auto make_exploration_seed(const NestingRequest &request,
                           const shiny::nesting::SolveControl &control)
    -> shiny::nesting::search::SolutionPoolEntry {
  const auto normalized = shiny::nesting::normalize_request(request);
  CHECK(normalized.has_value());
  if (!normalized.has_value()) {
    return {};
  }

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  shiny::nesting::search::detail::OrderEvaluator evaluator(
      normalized.value(), control, time_budget, stopwatch);
  return evaluator.evaluate(
      shiny::nesting::search::detail::original_order(normalized.value()),
      shiny::nesting::search::detail::original_forced_rotations(
          normalized.value()));
}

auto make_ranked_entry(std::vector<std::size_t> order,
                       const std::size_t placed_parts,
                       const std::size_t bin_count, const double strip_length,
                       const double utilization)
    -> shiny::nesting::search::SolutionPoolEntry {
  return {
      .order = std::move(order),
      .metrics =
          {
              .placed_parts = placed_parts,
              .bin_count = bin_count,
              .strip_length = strip_length,
              .utilization = utilization,
          },
  };
}

auto make_progress_layout(const std::uint32_t bin_id,
                          const std::uint32_t piece_id,
                          const double occupied_area,
                          const double container_area)
    -> shiny::nesting::pack::Layout {
  shiny::nesting::pack::Layout layout;
  layout.bins.push_back({
      .bin_id = bin_id,
      .identity =
          {
              .lifecycle = shiny::nesting::pack::BinLifecycle::user_created,
              .source_request_bin_id = bin_id,
          },
      .utilization =
          {
              .bin_id = bin_id,
              .placement_count = 1,
              .occupied_area = occupied_area,
              .container_area = container_area,
              .utilization =
                  container_area <= 0.0 ? 0.0 : occupied_area / container_area,
          },
  });
  layout.placement_trace.push_back({
      .piece_id = piece_id,
      .bin_id = bin_id,
      .translation = shiny::nesting::geom::Point2{1.0, 2.0},
      .phase = shiny::nesting::pack::ConstructivePlacementPhase::primary_order,
  });
  return layout;
}

auto make_profile_request(const SolveProfile profile) -> ProfileRequest {
  ProfileRequest request;
  request.profile = profile;
  request.time_limit_milliseconds = 1'000U;
  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(12.0, 12.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(12.0, 12.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 601,
      .polygon = rectangle(8.0, 8.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 602,
      .polygon = rectangle(4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 603,
      .polygon = rectangle(4.0, 4.0),
  });
  return request;
}

} // namespace

TEST_CASE(
    "adapts balanced profile requests into the Sparrow harness",
    "[sparrow][integration][balanced]") {
  ProfileRequest request;
  request.profile = SolveProfile::balanced;
  request.time_limit_milliseconds = 60'000;
  request.selected_bin_ids = {10U};
  request.allow_part_overflow = true;
  request.bins.push_back(BinRequest{
      .bin_id = 10,
      .polygon = rectangle(12.0, 12.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(12.0, 12.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 601,
      .polygon = rectangle(4.0, 4.0),
  });

  const auto adapted_or =
      shiny::nesting::pack::sparrow::adapters::adapt_request(
          request, shiny::nesting::test::sparrow::make_profile_control(123));
  REQUIRE(adapted_or.has_value());

  const auto &adapted = adapted_or.value();
  CHECK(adapted.seed_flow.profile == SolveProfile::balanced);
  CHECK(adapted.seed_flow.public_seed == 123U);
  CHECK(adapted.instance.selected_bin_ids == std::vector<std::uint32_t>{10U});
  CHECK(adapted.instance.allow_part_overflow);
  REQUIRE(adapted.instance.pieces.size() == 1U);
  REQUIRE(adapted.instance.bins.size() == 1U);
  CHECK(adapted.instance.bins.front().selected);
  CHECK(adapted.instance.frontier_bin_ids == std::vector<std::uint32_t>{10U});
}

TEST_CASE(
    "routes balanced Sparrow progress through one runtime surface",
    "[sparrow][integration][balanced]") {
  const auto layout = make_progress_layout(10U, 601U, 16.0, 64.0);
  const shiny::nesting::pack::sparrow::runtime::PortProgressSnapshot snapshot{
      .current_layout = layout,
      .best_layout = layout,
      .active_bin_id = 10U,
      .sampled_placements = 3U,
      .accepted_moves = 1U,
      .elapsed_time_milliseconds = 250U,
      .remaining_time_milliseconds = 750U,
      .phase = shiny::nesting::pack::sparrow::SparrowPhase::profile_balanced,
      .improved = true,
  };

  CHECK(shiny::nesting::pack::sparrow::runtime_phase_name(snapshot.phase) ==
        "profile_balanced");

  const auto summary =
      shiny::nesting::pack::sparrow::runtime::summarize_progress(snapshot);
  const auto progress =
      shiny::nesting::pack::sparrow::adapters::to_profile_progress_snapshot(
          snapshot);

  CHECK(summary.placed_count == 1U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(summary.utilization_percent,
                                                    25.0));
  CHECK(progress.active_bin_id == 10U);
  CHECK(progress.placed_count == summary.placed_count);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      progress.utilization_percent, summary.utilization_percent));
  CHECK(progress.elapsed_time_milliseconds == 250U);
  REQUIRE(progress.remaining_time_milliseconds.has_value());
  CHECK(*progress.remaining_time_milliseconds == 750U);
  CHECK(progress.improved);
}

TEST_CASE("balanced Sparrow cancellation token takes precedence over time and operation limits",
          "[sparrow][integration][balanced]") {
  shiny::nesting::runtime::CancellationSource cancellation_source;
  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(1'000U);

  cancellation_source.request_stop();
  const auto status =
      shiny::nesting::pack::sparrow::runtime::capture_termination_status({
          .cancellation = cancellation_source.token(),
          .time_budget = &time_budget,
          .stopwatch = &stopwatch,
          .operation_limit = 8U,
      });
  const auto decision =
      shiny::nesting::pack::sparrow::runtime::classify_stop(status, false);

  CHECK(shiny::nesting::pack::sparrow::runtime::interruption_requested(status));
  CHECK(decision.trigger ==
        shiny::nesting::pack::sparrow::runtime::StopTrigger::cancellation);
  CHECK(shiny::nesting::pack::sparrow::runtime::stop_trigger_name(
            decision.trigger) == "cancellation");
  CHECK(decision.stop_reason == shiny::nesting::StopReason::cancelled);
  REQUIRE(decision.remaining_time_milliseconds.has_value());
  CHECK(*decision.remaining_time_milliseconds <= 1'000U);
}

TEST_CASE("balanced Sparrow exploration phase stays bounded by the configured iteration budget",
          "[sparrow][integration][balanced]") {
  const auto request = make_exploration_request();
  const shiny::nesting::SolveControl control{.random_seed = 11U};
  const auto seed = make_exploration_seed(request, control);
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  const auto result =
      shiny::nesting::pack::sparrow::optimize::run_exploration_phase(
          normalized.value(), control, time_budget, stopwatch, seed,
          {.iteration_budget = 3U,
           .plateau_limit = 0U,
           .acceptance_window_max_ratio = 0.25,
           .acceptance_window_min_ratio = 0.02,
           .enable_rotation_moves = false});

  CHECK(result.iterations == 3U);
  CHECK(result.phase_metrics.exploration_iteration_budget == 3U);
  CHECK(result.phase_metrics.exploration_iterations == 3U);
  CHECK(result.steps.size() == 3U);
  CHECK_FALSE(result.steps.front().operator_name.empty());
}

TEST_CASE("balanced Sparrow exploration accepts escape moves inside the record acceptance window",
          "[sparrow][integration][balanced]") {
  const auto request = make_exploration_request();
  const shiny::nesting::SolveControl control{.random_seed = 17U};
  const auto seed = make_exploration_seed(request, control);
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  const auto result =
      shiny::nesting::pack::sparrow::optimize::run_exploration_phase(
          normalized.value(), control, time_budget, stopwatch, seed,
          {.iteration_budget = 4U,
           .plateau_limit = 4U,
           .acceptance_window_max_ratio = 0.25,
           .acceptance_window_min_ratio = 0.02,
           .enable_rotation_moves = false});

  CHECK(result.accepted_moves > 0U);
  CHECK(result.escape_moves > 0U);
  CHECK(std::any_of(result.steps.begin(), result.steps.end(),
                    [](const auto &step) { return step.escape_move; }));
}

TEST_CASE("balanced Sparrow large-item disruption is deterministic and shape-aware",
          "[sparrow][integration][balanced]") {
  const auto request = make_exploration_request();
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());
  shiny::nesting::runtime::DeterministicRng rng(23U);

  const auto result =
      shiny::nesting::pack::sparrow::optimize::disrupt_large_items(
          shiny::nesting::search::detail::original_order(normalized.value()),
          normalized.value(),
          shiny::nesting::search::detail::piece_areas_for(normalized.value()),
          rng);

  CHECK(result.applied);
  CHECK(result.first_position != result.second_position);
  CHECK(result.order !=
        shiny::nesting::search::detail::original_order(normalized.value()));
}

TEST_CASE("balanced Sparrow rollback ranking selects from the infeasible pool by rank",
          "[sparrow][integration][balanced]") {
  const auto incumbent = make_ranked_entry({0U, 1U, 2U}, 3U, 1U, 8.0, 0.9);
  const auto rollback = make_ranked_entry({0U, 2U, 1U}, 3U, 1U, 8.2, 0.88);
  const auto degraded = make_ranked_entry({2U, 1U, 0U}, 2U, 1U, 8.5, 0.8);
  const std::array entries{incumbent, rollback, degraded};
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;

  const auto selected =
      shiny::nesting::pack::sparrow::optimize::select_rollback_candidate(
          entries, incumbent, 77U, &trace);
  REQUIRE(selected.has_value());
  CHECK(selected->rank == 2U);
  REQUIRE(trace.infeasible_pool_selections.size() == 1U);
  CHECK(trace.infeasible_pool_selections.front().rank == 2U);
  CHECK(trace.infeasible_pool_selections.front().pool_size == 3U);

  const auto adopt =
      shiny::nesting::pack::sparrow::optimize::resolve_rollback_candidate(
          incumbent, selected);
  CHECK(adopt.rolled_back);
  CHECK_FALSE(adopt.restored_previous_incumbent);

  const auto restored =
      shiny::nesting::pack::sparrow::optimize::resolve_rollback_candidate(
          incumbent,
          shiny::nesting::pack::sparrow::optimize::RankedRollbackCandidate{
              .entry = degraded,
              .rank = 3U,
          });
  CHECK_FALSE(restored.rolled_back);
  CHECK(restored.restored_previous_incumbent);
}

TEST_CASE("balanced profile solve routes through Sparrow while quick profile stays constructive-only",
          "[sparrow][integration][balanced]") {
  const auto quick_request = make_profile_request(SolveProfile::quick);
  const auto balanced_request = make_profile_request(SolveProfile::balanced);
  std::vector<ProfileProgressSnapshot> balanced_progress;

  const auto quick =
      shiny::nesting::solve(quick_request, ProfileSolveControl{});
  const auto balanced = shiny::nesting::solve(
      balanced_request, ProfileSolveControl{
                            .on_progress =
                                [&](const ProfileProgressSnapshot &snapshot) {
                                  balanced_progress.push_back(snapshot);
                                },
                            .random_seed = 19U,
                        });

  REQUIRE(quick.has_value());
  REQUIRE(balanced.has_value());
  CHECK(quick.value().strategy == StrategyKind::bounding_box);
  CHECK_FALSE(quick.value().search.sparrow_polished);
  CHECK(balanced.value().strategy == StrategyKind::metaheuristic_search);
  CHECK(balanced.value().search.sparrow_polished);
  CHECK(balanced.value().search.optimizer ==
        shiny::nesting::OptimizerKind::none);
  CHECK(balanced.value().search.phase_metrics.exploration_iteration_budget >
        0U);
  CHECK(balanced.value().search.phase_metrics.compression_iteration_budget >
        0U);
  REQUIRE_FALSE(balanced_progress.empty());
  CHECK(balanced_progress.front().placed_count > 0U);
  CHECK(balanced_progress.back().stop_reason == balanced.value().stop_reason);

  const auto dto = shiny::nesting::api::to_dto(balanced.value());
  CHECK(dto.sparrow_polished);
  CHECK(dto.optimizer == shiny::nesting::OptimizerKind::none);
}