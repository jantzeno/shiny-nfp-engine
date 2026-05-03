#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "internal/legacy_solve.hpp"
#include "internal/request_normalization.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "runtime/cancellation.hpp"
#include "solve.hpp"
#include "validation/layout_validation.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProgressPhase;
using shiny::nesting::ProgressSnapshot;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

auto frame(double min_x, double min_y, double max_x, double max_y,
           double hole_min_x, double hole_min_y, double hole_max_x,
           double hole_max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(
      {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
      {{
          {hole_min_x, hole_min_y},
          {hole_min_x, hole_max_y},
          {hole_max_x, hole_max_y},
          {hole_max_x, hole_min_y},
      }});
}

auto improvement_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.enable_part_in_part_placement = true;
  request.execution.production.population_size = 12;
  request.execution.production.elite_count = 3;
  request.execution.production.mutant_count = 2;
  request.execution.production.max_iterations = 6;
  request.execution.production.polishing_passes = 1;
  request.execution.production.diversification_swaps = 1;

  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  return request;
}

auto duplicate_piece_request() -> NestingRequest {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production.population_size = 8;
  request.execution.production.elite_count = 1;
  request.execution.production.mutant_count = 1;
  request.execution.production.max_iterations = 4;
  request.execution.production.polishing_passes = 0;

  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 40.0, 40.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      .quantity = 8,
  });
  return request;
}

void require_summary_consistency(const shiny::nesting::NestingResult &result) {
  const auto summary = result.summary();
  REQUIRE(summary.placed_parts == result.placed_parts());
  REQUIRE(summary.unplaced_parts == result.unplaced_parts());
  REQUIRE(summary.layout_valid == result.layout_valid());
  REQUIRE(summary.all_parts_placed == result.all_parts_placed());
  REQUIRE(summary.full_success == result.is_full_success());
  REQUIRE(summary.partial_layout == result.is_partial_layout());
  REQUIRE(summary.stop_reason == result.stop_reason);
  REQUIRE(summary.placed_parts + summary.unplaced_parts == result.total_parts);
}

auto has_issue(const shiny::nesting::LayoutValidationReport &report,
               const shiny::nesting::LayoutValidationIssueKind issue_kind)
    -> bool {
  return std::any_of(report.issues.begin(), report.issues.end(),
                     [issue_kind](const auto &issue) {
                       return issue.issue_kind == issue_kind;
                     });
}

} // namespace

TEST_CASE("shared search layout validation rejects non-conserving layouts",
          "[solve][production][validation]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult result;
  result.total_parts = 2;
  result.layout.bins.push_back({
      .bin_id = 50,
      .container = rectangle(0.0, 0.0, 20.0, 20.0),
      .placements =
          {
              {
                  .placement = {.piece_id = 101, .bin_id = 50},
                  .resolved_rotation = {.degrees = 0.0},
                  .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
              },
              {
                  .placement = {.piece_id = 101, .bin_id = 50},
                  .resolved_rotation = {.degrees = 0.0},
                  .polygon = rectangle(3.0, 0.0, 5.0, 2.0),
              },
          },
  });

  const auto report =
      shiny::nesting::validation::validate_layout(normalized.value(), result);
  REQUIRE_FALSE(report.valid);
  REQUIRE(report.issues.size() >= 2U);
  REQUIRE(std::any_of(
      report.issues.begin(), report.issues.end(), [](const auto &issue) {
        return issue.issue_kind ==
               shiny::nesting::LayoutValidationIssueKind::duplicate_piece;
      }));
  REQUIRE(std::any_of(
      report.issues.begin(), report.issues.end(), [](const auto &issue) {
        return issue.issue_kind ==
               shiny::nesting::LayoutValidationIssueKind::missing_piece;
      }));
}

TEST_CASE("result finalization repairs conservation and exposes "
          "partial-summary helpers",
          "[solve][production][result]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult result;
  result.total_parts = 2;
  result.stop_reason = StopReason::time_limit_reached;
  result.layout.placement_trace.push_back({
      .piece_id = 101,
      .bin_id = 50,
      .rotation_index = {.value = 0},
      .resolved_rotation = {.degrees = 0.0},
      .translation = {0.0, 0.0},
  });
  result.layout.bins.push_back({
      .bin_id = 50,
      .container = rectangle(0.0, 0.0, 20.0, 20.0),
      .placements = {{
          .placement = {.piece_id = 101, .bin_id = 50},
          .resolved_rotation = {.degrees = 0.0},
          .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
      }},
  });

  shiny::nesting::validation::finalize_result(normalized.value(), result);

  REQUIRE(result.layout.unplaced_piece_ids == std::vector<std::uint32_t>{102U});
  REQUIRE(result.layout_valid());
  REQUIRE(has_issue(
      result.validation,
      shiny::nesting::LayoutValidationIssueKind::conservation_repair));
  REQUIRE_FALSE(result.all_parts_placed());
  REQUIRE_FALSE(result.is_full_success());
  REQUIRE(result.is_partial_layout());
  require_summary_consistency(result);
}

TEST_CASE("result finalization preserves invalid conservation evidence",
          "[solve][production][result][validation]") {
  auto request = duplicate_piece_request();
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult result;
  result.total_parts = normalized.value().expanded_pieces.size();
  result.layout.bins.push_back({
      .bin_id = 50,
      .container = rectangle(0.0, 0.0, 40.0, 40.0),
      .placements = {{
          .placement =
              {.piece_id =
                   normalized.value().expanded_pieces[0].expanded_piece_id,
               .bin_id = 50},
          .resolved_rotation = {.degrees = 0.0},
          .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      }},
  });
  result.layout.unplaced_piece_ids = {
      normalized.value().expanded_pieces[0].expanded_piece_id,
      normalized.value().expanded_pieces[0].expanded_piece_id,
      999999U,
  };

  shiny::nesting::validation::finalize_result(normalized.value(), result);
  const auto first_unplaced = result.layout.unplaced_piece_ids;
  shiny::nesting::validation::finalize_result(normalized.value(), result);

  REQUIRE(result.layout.unplaced_piece_ids == first_unplaced);
  REQUIRE_FALSE(result.layout_valid());
  REQUIRE(
      has_issue(result.validation,
                shiny::nesting::LayoutValidationIssueKind::duplicate_piece));
  REQUIRE(has_issue(result.validation,
                    shiny::nesting::LayoutValidationIssueKind::unknown_piece));
}

TEST_CASE("solution pool validates layouts instead of trusting stale metadata",
          "[solve][production][validation][pool]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult stale_valid;
  stale_valid.total_parts = 1;
  stale_valid.validation.valid = true;
  stale_valid.layout.unplaced_piece_ids = {999999U};

  shiny::nesting::search::SolutionPool pool(4U, &normalized.value());
  pool.insert(shiny::nesting::search::SolutionPoolEntry{
      .order = {0U},
      .piece_indexed_forced_rotations = {std::nullopt},
      .metrics = {.placed_parts = 1U},
      .result = stale_valid,
  });

  REQUIRE(pool.empty());
}

TEST_CASE("solution pool signatures distinguish identical order with rotations",
          "[solve][production][rotation][pool]") {
  shiny::nesting::NestingResult valid;
  valid.total_parts = 1;
  valid.validation.valid = true;

  shiny::nesting::search::SolutionPool pool(4U);
  pool.insert(shiny::nesting::search::SolutionPoolEntry{
      .order = {0U},
      .piece_indexed_forced_rotations = {shiny::nesting::geom::RotationIndex{
          0}},
      .metrics = {.placed_parts = 1U, .bin_count = 1U, .utilization = 0.4},
      .result = valid,
  });
  pool.insert(shiny::nesting::search::SolutionPoolEntry{
      .order = {0U},
      .piece_indexed_forced_rotations = {shiny::nesting::geom::RotationIndex{
          1}},
      .metrics = {.placed_parts = 1U, .bin_count = 1U, .utilization = 0.5},
      .result = valid,
  });

  REQUIRE(pool.size() == 2U);
}

TEST_CASE("BRKGA rotation genes solve a rotation-critical fixture",
          "[solve][production][brkga][rotation]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production_optimizer =
      shiny::nesting::ProductionOptimizerKind::brkga;
  request.execution.default_rotations = {{0.0, 90.0}};
  request.execution.production.population_size = 10;
  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 2;
  request.execution.production.max_iterations = 6;
  request.execution.production.polishing_passes = 0;
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(0.0, 0.0, 2.0, 4.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 10,
      .polygon = rectangle(0.0, 0.0, 3.0, 1.0),
  });

  const auto solved =
      shiny::nesting::solve(request, SolveControl{.random_seed = 7});
  REQUIRE(solved.has_value());
  REQUIRE(solved.value().placed_parts() == 1U);
  REQUIRE(solved.value().layout.placement_trace.front().rotation_index.value ==
          1U);
  REQUIRE(solved.value().layout_valid());
}

TEST_CASE("shared validation reports every issue kind",
          "[solve][production][validation]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.part_spacing = 1.0;
  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
      .exclusion_zones = {shiny::nesting::place::BedExclusionZone{
          .zone_id = 1,
          .bin_id = 50,
          .region = shiny::nesting::geom::Polygon(
              rectangle(0.0, 0.0, 4.0, 4.0).outer()),
      }},
  });
  request.bins.push_back(BinRequest{
      .bin_id = 51,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
      .allow_mirror = false,
      .allowed_rotations = shiny::nesting::geom::DiscreteRotationSet{{0.0}},
      .allowed_bin_ids = {51},
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult result;
  result.total_parts = normalized.value().expanded_pieces.size();
  result.layout.bins.push_back({
      .bin_id = 50,
      .container = rectangle(0.0, 0.0, 10.0, 10.0),
      .placements =
          {
              {
                  .placement = {.piece_id = 101,
                                .bin_id = 999,
                                .rotation_index = {.value = 1},
                                .mirrored = true},
                  .resolved_rotation = {.degrees = 90.0},
                  .polygon = rectangle(-1.0, -1.0, 3.0, 3.0),
              },
              {
                  .placement = {.piece_id = 101, .bin_id = 50},
                  .resolved_rotation = {.degrees = 0.0},
                  .polygon = rectangle(2.0, 2.0, 6.0, 6.0),
              },
          },
  });
  result.layout.bins.push_back({
      .bin_id = 999,
      .container = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  result.layout.unplaced_piece_ids = {101U, 999999U};

  auto report =
      shiny::nesting::validation::validate_layout(normalized.value(), result);
  shiny::nesting::validation::finalize_result(normalized.value(), result);
  report.issues.insert(report.issues.end(), result.validation.issues.begin(),
                       result.validation.issues.end());

  for (const auto issue_kind : std::array{
           shiny::nesting::LayoutValidationIssueKind::missing_piece,
           shiny::nesting::LayoutValidationIssueKind::duplicate_piece,
           shiny::nesting::LayoutValidationIssueKind::unknown_piece,
           shiny::nesting::LayoutValidationIssueKind::unknown_bin,
           shiny::nesting::LayoutValidationIssueKind::bin_mismatch,
           shiny::nesting::LayoutValidationIssueKind::disallowed_bin,
           shiny::nesting::LayoutValidationIssueKind::disallowed_rotation,
           shiny::nesting::LayoutValidationIssueKind::disallowed_mirror,
           shiny::nesting::LayoutValidationIssueKind::outside_container,
           shiny::nesting::LayoutValidationIssueKind::inside_exclusion_zone,
           shiny::nesting::LayoutValidationIssueKind::piece_overlap,
           shiny::nesting::LayoutValidationIssueKind::spacing_violation,
           shiny::nesting::LayoutValidationIssueKind::conservation_repair,
       }) {
    INFO(static_cast<int>(issue_kind));
    REQUIRE(has_issue(report, issue_kind));
  }
}

TEST_CASE("shared validation uses exact spacing instead of AABB-only rejection",
          "[solve][production][validation][spacing]") {
  NestingRequest request;
  request.execution.default_rotations = {{0.0}};
  request.execution.part_spacing = 0.25;
  request.bins.push_back(BinRequest{
      .bin_id = 50,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 101,
      .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 102,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.has_value());

  shiny::nesting::NestingResult result;
  result.total_parts = 2;
  result.layout.bins.push_back({
      .bin_id = 50,
      .container = rectangle(0.0, 0.0, 20.0, 20.0),
      .placements =
          {
              {
                  .placement = {.piece_id = 101, .bin_id = 50},
                  .resolved_rotation = {.degrees = 0.0},
                  .polygon = frame(0.0, 0.0, 8.0, 8.0, 2.0, 2.0, 6.0, 6.0),
              },
              {
                  .placement = {.piece_id = 102,
                                .bin_id = 50,
                                .translation = {2.5, 2.5}},
                  .resolved_rotation = {.degrees = 0.0},
                  .polygon = rectangle(2.5, 2.5, 5.5, 5.5),
              },
          },
  });

  const auto valid_report =
      shiny::nesting::validation::validate_layout(normalized.value(), result);
  REQUIRE(valid_report.valid);

  request.execution.part_spacing = 0.75;
  const auto tighter = shiny::nesting::normalize_request(request);
  REQUIRE(tighter.has_value());
  const auto invalid_report =
      shiny::nesting::validation::validate_layout(tighter.value(), result);
  REQUIRE_FALSE(invalid_report.valid);
  REQUIRE(std::any_of(
      invalid_report.issues.begin(), invalid_report.issues.end(),
      [](const auto &issue) {
        return issue.issue_kind ==
               shiny::nesting::LayoutValidationIssueKind::spacing_violation;
      }));
}

TEST_CASE("production search improves the constructive seed and records "
          "replayable progress",
          "[solve][production][brkga]") {
  auto constructive_request = improvement_request();
  constructive_request.execution.strategy = StrategyKind::bounding_box;
  const auto constructive = shiny::nesting::solve(constructive_request);

  auto production_request = improvement_request();
  production_request.execution.strategy = StrategyKind::metaheuristic_search;

  std::vector<ProgressSnapshot> snapshots;
  const auto production = shiny::nesting::solve(
      production_request, SolveControl{
                              .on_progress =
                                  [&](const ProgressSnapshot &snapshot) {
                                    snapshots.push_back(snapshot);
                                  },
                              .operation_limit = 4,
                              .random_seed = 17,
                          });

  REQUIRE(constructive.has_value());
  REQUIRE(production.has_value());
  REQUIRE(constructive.value().layout.placement_trace.size() == 1U);
  REQUIRE(production.value().layout.placement_trace.size() == 2U);
  REQUIRE(production.value().layout.unplaced_piece_ids.empty());
  REQUIRE(production.value().search.optimizer == OptimizerKind::brkga);
  REQUIRE(production.value().search.sparrow_polished);
  REQUIRE_FALSE(production.value().search.progress.empty());
  REQUIRE(production.value().search.cache_metrics.exact_nfp_computations +
              production.value().search.cache_metrics.exact_nfp_cache_hits >
          0U);
  REQUIRE(production.value().is_full_success());
  REQUIRE(production.value().layout_valid());
  REQUIRE(production.value().search.progress.front().iteration == 1U);
  REQUIRE(
      production.value().search.progress.back().layout.placement_trace.size() ==
      production.value().layout.placement_trace.size());
  require_summary_consistency(production.value());
  REQUIRE_FALSE(snapshots.empty());
}

TEST_CASE("production search respects operation limits and cancellation",
          "[solve][production][budget]") {
  auto request = improvement_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;

  std::vector<ProgressSnapshot> snapshots;
  const auto limited = shiny::nesting::solve(
      request, SolveControl{
                   .on_progress =
                       [&](const ProgressSnapshot &snapshot) {
                         snapshots.push_back(snapshot);
                       },
                   .operation_limit = 1,
                   .random_seed = 3,
               });
  REQUIRE(limited.has_value());
  REQUIRE(limited.value().stop_reason == StopReason::operation_limit_reached);
  REQUIRE(limited.value().layout_valid());
  require_summary_consistency(limited.value());
  for (const auto &snapshot : snapshots) {
    REQUIRE(snapshot.phase != ProgressPhase::refinement);
  }

  const auto unlimited = shiny::nesting::solve(
      request, SolveControl{.operation_limit = 0, .random_seed = 3});
  REQUIRE(unlimited.has_value());
  REQUIRE(unlimited.value().stop_reason != StopReason::operation_limit_reached);
  require_summary_consistency(unlimited.value());

  shiny::nesting::runtime::CancellationSource source;
  source.request_stop();
  const auto cancelled = shiny::nesting::solve(
      request, SolveControl{.cancellation = source.token()});
  REQUIRE(cancelled.has_value());
  REQUIRE(cancelled.value().stop_reason == StopReason::cancelled);
  REQUIRE(cancelled.value().layout_valid());
  require_summary_consistency(cancelled.value());
}

TEST_CASE("production search progress remains monotonic when duplicate "
          "chromosomes are skipped",
          "[solve][production][budget]") {
  auto request = duplicate_piece_request();

  std::vector<ProgressSnapshot> snapshots;
  const auto solved = shiny::nesting::solve(
      request, SolveControl{
                   .on_progress =
                       [&](const ProgressSnapshot &snapshot) {
                         snapshots.push_back(snapshot);
                       },
                   .operation_limit = 4,
                   .random_seed = 11,
               });

  REQUIRE(solved.has_value());
  const auto &result = solved.value();
  REQUIRE(result.search.optimizer == OptimizerKind::brkga);
  REQUIRE_FALSE(result.search.progress.empty());
  REQUIRE(result.search.cache_metrics.exact_nfp_computations +
              result.search.cache_metrics.exact_nfp_cache_hits >
          0U);
  require_summary_consistency(result);
}

TEST_CASE("production search obeys time budgets under search",
          "[solve][production][time]") {
  NestingRequest request;
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.default_rotations = {{0.0}};
  request.execution.production.population_size = 48;
  request.execution.production.elite_count = 8;
  request.execution.production.mutant_count = 8;
  request.execution.production.max_iterations = 64;

  request.bins.push_back(BinRequest{
      .bin_id = 70,
      .polygon = rectangle(0.0, 0.0, 20.0, 20.0),
  });
  for (std::size_t index = 0; index < 18U; ++index) {
    request.pieces.push_back(PieceRequest{
        .piece_id = static_cast<std::uint32_t>(200U + index),
        .polygon = rectangle(0.0, 0.0, 4.0 + (index % 3U), 2.0 + (index % 2U)),
    });
  }

  std::vector<ProgressSnapshot> snapshots;
  const auto result = shiny::nesting::solve(
      request, SolveControl{.on_progress =
                                [&](const ProgressSnapshot &snapshot) {
                                  snapshots.push_back(snapshot);
                                },
                            .time_limit_milliseconds = 1});
  REQUIRE(result.has_value());
  REQUIRE(result.value().stop_reason == StopReason::time_limit_reached);
  REQUIRE(result.value().layout_valid());
  require_summary_consistency(result.value());
  for (const auto &snapshot : snapshots) {
    REQUIRE(snapshot.phase != ProgressPhase::refinement);
  }
}

TEST_CASE("public strategies conserve layouts under limits and cancellation",
          "[solve][production][limits][cancellation][time]") {
  struct StrategyCase {
    StrategyKind strategy{StrategyKind::bounding_box};
    shiny::nesting::ProductionOptimizerKind optimizer{
        shiny::nesting::ProductionOptimizerKind::brkga};
  };

  const std::array cases{
      StrategyCase{.strategy = StrategyKind::metaheuristic_search,
                   .optimizer = shiny::nesting::ProductionOptimizerKind::brkga},

  };

  for (const auto &strategy_case : cases) {
    auto request = improvement_request();
    request.execution.strategy = strategy_case.strategy;
    request.execution.production_optimizer = strategy_case.optimizer;
    request.execution.production.max_iterations = 2;

    CAPTURE(static_cast<int>(strategy_case.strategy));
    CAPTURE(static_cast<int>(strategy_case.optimizer));
    const auto limited = shiny::nesting::solve(
        request, SolveControl{.operation_limit = 1, .random_seed = 5});
    REQUIRE(limited.has_value());
    REQUIRE(limited.value().layout_valid());
    require_summary_consistency(limited.value());

    shiny::nesting::runtime::CancellationSource source;
    source.request_stop();
    const auto cancelled = shiny::nesting::solve(
        request, SolveControl{.cancellation = source.token(),
                              .operation_limit = 1,
                              .random_seed = 5});
    REQUIRE(cancelled.has_value());
    REQUIRE(cancelled.value().layout_valid());
    require_summary_consistency(cancelled.value());
  }
}

TEST_CASE("production search keeps strict small-population BRKGA validation",
          "[solve][production][config]") {
  auto request = improvement_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.production.population_size = 4;
  request.execution.production.elite_count = 1;
  request.execution.production.mutant_count = 1;
  REQUIRE(request.execution.production.is_valid());
  REQUIRE(request.is_valid());

  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 2;
  REQUIRE_FALSE(request.execution.production.is_valid());
  REQUIRE_FALSE(request.is_valid());
  REQUIRE_FALSE(shiny::nesting::solve(request).has_value());
}
