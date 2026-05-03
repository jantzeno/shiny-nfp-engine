#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <optional>
#include <vector>

#include "api/dto.hpp"
#include "internal/request_normalization.hpp"
#include "packing/bin_identity.hpp"
#include "packing/constructive/fill_first_engine.hpp"
#include "packing/sparrow/adapters/progress_adapter.hpp"
#include "packing/sparrow/adapters/request_adapter.hpp"
#include "packing/sparrow/optimize/compression_phase.hpp"
#include "packing/sparrow/optimize/objective.hpp"
#include "packing/sparrow/optimize/optimize.hpp"
#include "packing/sparrow/runtime/cancellation.hpp"
#include "packing/sparrow/runtime/progress.hpp"
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

auto strip_case_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.strategy =
      shiny::nesting::StrategyKind::bounding_box;
  request.execution.irregular.enable_direct_overlap_check = true;

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(10.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 11,
      .polygon = rectangle(4.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 12,
      .polygon = rectangle(6.0, 4.0),
  });
  return request;
}

auto make_compression_seed(const NestingRequest &request,
                           const shiny::nesting::SolveControl &control)
    -> shiny::nesting::search::SolutionPoolEntry {
  const auto normalized = shiny::nesting::normalize_request(request);
  CHECK(normalized.has_value());
  if (!normalized.has_value()) {
    return {};
  }

  shiny::nesting::pack::constructive::FillFirstEngine packer;
  const auto seed = packer.solve(normalized.value(), control);
  CHECK(seed.has_value());
  if (!seed.has_value()) {
    return {};
  }

  return {
      .order =
          shiny::nesting::search::detail::original_order(normalized.value()),
      .piece_indexed_forced_rotations =
          shiny::nesting::search::detail::original_forced_rotations(
              normalized.value()),
      .metrics =
          shiny::nesting::search::metrics_for_layout(seed.value().layout),
      .result = seed.value(),
  };
}

auto make_ranked_entry(
    std::vector<std::size_t> order, std::size_t placed_parts,
    std::size_t bin_count, double strip_length, double utilization,
    std::vector<std::optional<shiny::nesting::geom::RotationIndex>>
        forced_rotations = {}) -> shiny::nesting::search::SolutionPoolEntry {
  return {
      .order = std::move(order),
      .piece_indexed_forced_rotations = std::move(forced_rotations),
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
      .translation = shiny::nesting::geom::Point2{2.0, 3.0},
      .phase = shiny::nesting::pack::ConstructivePlacementPhase::gap_fill,
  });
  return layout;
}

auto make_profile_request(const SolveProfile profile) -> ProfileRequest {
  ProfileRequest request;
  request.profile = profile;
  request.time_limit_milliseconds = 1'000U;
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 701,
      .polygon = rectangle(6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 702,
      .polygon = rectangle(4.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 703,
      .polygon = rectangle(4.0, 4.0),
  });
  return request;
}

} // namespace

TEST_CASE("maximum-search request adapter maps profile request into the Sparrow harness",
          "[sparrow][integration][maximum-search]") {
  ProfileRequest request;
  request.profile = SolveProfile::maximum_search;
  request.time_limit_milliseconds = 300'000;
  request.maintain_bed_assignment = true;
  request.bins.push_back(BinRequest{
      .bin_id = 20,
      .polygon = rectangle(10.0, 10.0),
  });
  request.bins.push_back(BinRequest{
      .bin_id = 30,
      .polygon = rectangle(10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 701,
      .polygon = rectangle(3.0, 3.0),
      .assigned_bin_id = 20,
  });

  const auto adapted_or =
      shiny::nesting::pack::sparrow::adapters::adapt_request(
          request, shiny::nesting::test::sparrow::make_profile_control(321));
  REQUIRE(adapted_or.has_value());

  const auto &adapted = adapted_or.value();
  CHECK(adapted.seed_flow.profile == SolveProfile::maximum_search);
  CHECK(adapted.seed_flow.public_seed == 321U);
  CHECK(adapted.instance.maintain_bed_assignment);
  CHECK(adapted.instance.frontier_bin_ids ==
        std::vector<std::uint32_t>({20U, 30U}));
  REQUIRE(adapted.instance.pieces.size() == 1U);
  REQUIRE(adapted.instance.assignments.size() == 1U);
  REQUIRE(adapted.instance.pieces.front().instance.source_piece_id == 701U);
  REQUIRE(adapted.instance.pieces.front().pinned_bin_id.has_value());
  CHECK(*adapted.instance.pieces.front().pinned_bin_id == 20U);
  CHECK(adapted.instance.assignments.front().piece_id == 701U);
  REQUIRE(adapted.instance.assignments.front().pinned_bin_id.has_value());
  CHECK(*adapted.instance.assignments.front().pinned_bin_id == 20U);
}

TEST_CASE("maximum-search Sparrow progress routes through a single runtime surface",
          "[sparrow][integration][maximum-search]") {
  const auto layout = make_progress_layout(20U, 701U, 21.0, 84.0);
  const shiny::nesting::pack::sparrow::runtime::PortProgressSnapshot snapshot{
      .current_layout = layout,
      .best_layout = layout,
      .active_bin_id = 20U,
      .sampled_placements = 5U,
      .accepted_moves = 2U,
      .elapsed_time_milliseconds = 500U,
      .stop_reason = shiny::nesting::StopReason::time_limit_reached,
      .phase =
          shiny::nesting::pack::sparrow::SparrowPhase::profile_maximum_search,
      .improved = false,
  };

  CHECK(shiny::nesting::pack::sparrow::runtime_phase_name(snapshot.phase) ==
        "profile_maximum_search");

  const auto summary =
      shiny::nesting::pack::sparrow::runtime::summarize_progress(snapshot);
  const auto progress =
      shiny::nesting::pack::sparrow::adapters::to_profile_progress_snapshot(
          snapshot);

  CHECK(summary.placed_count == 1U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(summary.utilization_percent,
                                                    25.0));
  CHECK(progress.active_bin_id == 20U);
  CHECK(progress.placed_count == summary.placed_count);
  CHECK(progress.stop_reason == shiny::nesting::StopReason::time_limit_reached);
  CHECK_FALSE(progress.remaining_time_milliseconds.has_value());
  CHECK_FALSE(progress.improved);
}

TEST_CASE(
    "maximum-search Sparrow stop classifier distinguishes time-limit and operation-limit stops",
    "[sparrow][integration][maximum-search]") {
  const auto time_limited =
      shiny::nesting::pack::sparrow::runtime::TerminationStatus{
          .time_limit_reached = true,
          .operation_limit_enabled = true,
          .remaining_time_milliseconds = 0U,
      };
  const auto time_limited_decision =
      shiny::nesting::pack::sparrow::runtime::classify_stop(time_limited, true);
  CHECK(time_limited_decision.trigger ==
        shiny::nesting::pack::sparrow::runtime::StopTrigger::time_limit);
  CHECK(time_limited_decision.stop_reason ==
        shiny::nesting::StopReason::time_limit_reached);
  REQUIRE(time_limited_decision.remaining_time_milliseconds.has_value());
  CHECK(*time_limited_decision.remaining_time_milliseconds == 0U);

  const auto operation_limited =
      shiny::nesting::pack::sparrow::runtime::TerminationStatus{
          .operation_limit_enabled = true,
      };
  const auto operation_limited_decision =
      shiny::nesting::pack::sparrow::runtime::classify_stop(operation_limited,
                                                            true);
  CHECK(operation_limited_decision.trigger ==
        shiny::nesting::pack::sparrow::runtime::StopTrigger::operation_limit);
  CHECK(operation_limited_decision.stop_reason ==
        shiny::nesting::StopReason::operation_limit_reached);
  CHECK_FALSE(shiny::nesting::pack::sparrow::runtime::interruption_requested(
      operation_limited));
}

TEST_CASE("maximum-search compression phase produces a deterministic shrink schedule",
          "[sparrow][integration][maximum-search]") {
  const auto request = strip_case_request();
  const shiny::nesting::SolveControl control{.random_seed = 31U};
  const auto seed = make_compression_seed(request, control);
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;
  const auto result =
      shiny::nesting::pack::sparrow::optimize::run_compression_phase(
          normalized.value(), control, time_budget, stopwatch, seed,
          {.iteration_budget = 4U,
           .plateau_limit = 0U,
           .shrink_max_ratio = 0.01,
           .shrink_min_ratio = 0.001},
          &trace);

  REQUIRE(result.steps.size() == 4U);
  REQUIRE(trace.compression_attempts.size() == 4U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      result.steps.front().shrink_ratio, 0.01));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      result.steps.back().shrink_ratio, 0.001));
  CHECK(trace.compression_attempts.front().accepted ==
        result.steps.front().accepted);
}

TEST_CASE("maximum-search compression phase exposes accepted-move and iteration counts explicitly",
          "[sparrow][integration][maximum-search]") {
  const auto request = strip_case_request();
  const shiny::nesting::SolveControl control{.random_seed = 37U};
  const auto seed = make_compression_seed(request, control);
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::runtime::Stopwatch stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;
  const auto result =
      shiny::nesting::pack::sparrow::optimize::run_compression_phase(
          normalized.value(), control, time_budget, stopwatch, seed,
          {.iteration_budget = 6U,
           .plateau_limit = 6U,
           .shrink_max_ratio = 0.01,
           .shrink_min_ratio = 0.001},
          &trace);

  CHECK(result.phase_metrics.accepted_moves == result.accepted_moves);
  CHECK(result.phase_metrics.compression_iterations == result.iterations);
  CHECK(trace.compression_attempts.size() == result.steps.size());
}

TEST_CASE("maximum-search objective ranking aligns with search metrics hierarchy",
          "[sparrow][integration][maximum-search]") {
  const auto more_parts = make_ranked_entry({0U, 1U, 2U}, 3U, 2U, 10.0, 0.60);
  const auto fewer_bins = make_ranked_entry({0U, 2U, 1U}, 3U, 1U, 10.0, 0.50);
  const auto shorter_strip = make_ranked_entry({1U, 0U, 2U}, 3U, 1U, 8.0, 0.45);
  const auto better_utilization =
      make_ranked_entry({1U, 2U, 0U}, 3U, 1U, 8.0, 0.70);

  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            fewer_bins, more_parts) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            shorter_strip, fewer_bins) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            better_utilization, shorter_strip) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
}

TEST_CASE("maximum-search objective tie-breaks are resolved deterministically by order and rotation signatures",
          "[sparrow][integration][maximum-search]") {
  const auto first =
      make_ranked_entry({0U, 1U, 2U}, 2U, 1U, 9.0, 0.50,
                        {std::nullopt, std::nullopt, std::nullopt});
  const auto second =
      make_ranked_entry({2U, 1U, 0U}, 2U, 1U, 9.0, 0.50,
                        {std::nullopt, std::nullopt, std::nullopt});
  const auto rotated =
      make_ranked_entry({0U, 1U, 2U}, 2U, 1U, 9.0, 0.50,
                        {shiny::nesting::geom::RotationIndex{0}, std::nullopt,
                         shiny::nesting::geom::RotationIndex{1}});

  const auto first_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(first);
  const auto second_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(second);
  const auto rotated_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(rotated);

  CHECK(first_value.order_signature != second_value.order_signature);
  CHECK(first_value.rotation_signature != rotated_value.rotation_signature);

  const auto ordering =
      shiny::nesting::pack::sparrow::optimize::compare_objective(first_value,
                                                                 second_value);
  REQUIRE(
      ordering !=
      shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::equivalent);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            second_value, first_value) != ordering);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            first_value, rotated_value) !=
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::equivalent);
}

TEST_CASE("multi-bin Sparrow objective breakdown exposes active bin count compaction and utilization",
          "[sparrow][integration][maximum-search]") {
  const auto fewer_bins = make_ranked_entry({0U, 2U, 1U}, 3U, 1U, 10.0, 0.50);
  const auto tighter_compaction =
      make_ranked_entry({1U, 0U, 2U}, 3U, 1U, 8.0, 0.45);
  const auto better_utilization =
      make_ranked_entry({1U, 2U, 0U}, 3U, 1U, 8.0, 0.70);

  const auto fewer_bins_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(fewer_bins);
  const auto tighter_compaction_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(
          tighter_compaction);
  const auto better_utilization_value =
      shiny::nesting::pack::sparrow::optimize::evaluate_objective(
          better_utilization);

  CHECK(fewer_bins_value.multi_bin.active_bin_count == 1U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      fewer_bins_value.multi_bin.active_bin_compaction, 10.0));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      fewer_bins_value.multi_bin.active_bin_utilization, 0.50));
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            tighter_compaction_value, fewer_bins_value) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            better_utilization_value, tighter_compaction_value) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
}

TEST_CASE("maximum-search optimize driver produces deterministic exploration and compression results",
          "[sparrow][integration][maximum-search]") {
  const auto request = strip_case_request();
  const shiny::nesting::SolveControl control{.random_seed = 41U};
  const auto seed = make_compression_seed(request, control);
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  const shiny::nesting::pack::sparrow::optimize::OptimizeConfig config{
      .exploration = {.iteration_budget = 4U,
                      .plateau_limit = 4U,
                      .acceptance_window_max_ratio = 0.25,
                      .acceptance_window_min_ratio = 0.02,
                      .enable_rotation_moves = true,
                      .enable_disruption_moves = true,
                      .infeasible_pool_capacity = 2U,
                      .rollback_after = 2U},
      .compression = {.iteration_budget = 4U,
                      .plateau_limit = 4U,
                      .shrink_max_ratio = 0.01,
                      .shrink_min_ratio = 0.001},
  };
  shiny::nesting::runtime::Stopwatch first_stopwatch;
  const shiny::nesting::runtime::TimeBudget time_budget(0U);
  shiny::nesting::pack::sparrow::runtime::TraceCapture first_trace;
  const auto first = shiny::nesting::pack::sparrow::optimize::run_optimize(
      normalized.value(), control, time_budget, first_stopwatch, seed, config,
      &first_trace);

  shiny::nesting::runtime::Stopwatch second_stopwatch;
  shiny::nesting::pack::sparrow::runtime::TraceCapture second_trace;
  const auto second = shiny::nesting::pack::sparrow::optimize::run_optimize(
      normalized.value(), control, time_budget, second_stopwatch, seed, config,
      &second_trace);

  CHECK(first.accepted_moves == first.phase_metrics.accepted_moves);
  CHECK(first.phase_metrics.accepted_moves ==
        first.exploration.accepted_moves + first.compression.accepted_moves);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            first.best_solution, first.exploration.best_solution) !=
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::worse);
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            first.best_solution, first.compression.best_solution) !=
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::worse);
  CHECK(first.best_objective.order_signature ==
        shiny::nesting::pack::sparrow::optimize::evaluate_objective(
            first.best_solution)
            .order_signature);
  CHECK(first.best_solution.metrics.placed_parts ==
        second.best_solution.metrics.placed_parts);
  CHECK(first.best_solution.metrics.bin_count ==
        second.best_solution.metrics.bin_count);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      first.best_solution.metrics.strip_length,
      second.best_solution.metrics.strip_length));
  CHECK(first.phase_metrics.accepted_moves ==
        second.phase_metrics.accepted_moves);
  CHECK(first_trace.compression_attempts.size() ==
        second_trace.compression_attempts.size());
}

TEST_CASE("maximum-search profile solve routes through the deeper Sparrow path and emits progress",
          "[sparrow][integration][maximum-search]") {
  const auto balanced_request = make_profile_request(SolveProfile::balanced);
  const auto maximum_request =
      make_profile_request(SolveProfile::maximum_search);
  std::vector<ProfileProgressSnapshot> maximum_progress;

  const auto balanced = shiny::nesting::solve(
      balanced_request, ProfileSolveControl{.random_seed = 23U});
  const auto maximum = shiny::nesting::solve(
      maximum_request, ProfileSolveControl{
                           .on_progress =
                               [&](const ProfileProgressSnapshot &snapshot) {
                                 maximum_progress.push_back(snapshot);
                               },
                           .random_seed = 23U,
                       });

  REQUIRE(balanced.has_value());
  REQUIRE(maximum.has_value());
  CHECK(maximum.value().strategy == StrategyKind::metaheuristic_search);
  CHECK(maximum.value().search.sparrow_polished);
  CHECK(maximum.value().search.optimizer ==
        shiny::nesting::OptimizerKind::none);
  CHECK(maximum.value().search.phase_metrics.exploration_iteration_budget >
        balanced.value().search.phase_metrics.exploration_iteration_budget);
  CHECK(maximum.value().search.phase_metrics.compression_iteration_budget >
        balanced.value().search.phase_metrics.compression_iteration_budget);
  REQUIRE_FALSE(maximum_progress.empty());
  CHECK(maximum_progress.front().placed_count > 0U);
  CHECK(maximum_progress.back().stop_reason == maximum.value().stop_reason);

  const auto dto = shiny::nesting::api::to_dto(maximum.value());
  CHECK(dto.sparrow_polished);
  CHECK(dto.optimizer == shiny::nesting::OptimizerKind::none);
}