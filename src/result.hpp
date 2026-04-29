#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "observer.hpp"
#include "request.hpp"

namespace shiny::nesting {

enum class OptimizerKind : std::uint8_t {
  none = 0,
  brkga = 1,
  simulated_annealing = 2,
  alns = 3,
  gdrr = 4,
  lahc = 5,
};

struct SearchProgressEntry {
  std::size_t iteration{0};
  bool improved{false};
  pack::Layout layout{};
  bool layout_valid{false};
  std::size_t validation_issue_count{0};
};

struct SearchPhaseMetrics {
  std::size_t exploration_iteration_budget{0};
  std::size_t exploration_iterations{0};
  std::size_t compression_iteration_budget{0};
  std::size_t compression_iterations{0};
  std::size_t accepted_moves{0};
  std::size_t infeasible_candidates{0};
  std::size_t infeasible_rollbacks{0};
};

struct SeparatorReplayMetrics {
  std::size_t attempts{0};
  std::size_t converged_runs{0};
  std::size_t accepted_compactions{0};
  std::size_t total_iterations{0};
  std::size_t worker_count{0};
  std::size_t max_iterations{0};
  std::size_t iter_no_improvement_limit{0};
  std::size_t strike_limit{0};
  std::size_t global_samples{0};
  std::size_t focused_samples{0};
  std::size_t coordinate_descent_iterations{0};
};

struct SearchCacheMetrics {
  std::size_t exact_nfp_cache_hits{0};
  std::size_t conservative_bbox_cache_hits{0};
  std::size_t exact_nfp_computations{0};
  std::size_t conservative_bbox_fallbacks{0};
};

struct SearchFallbackMetrics {
  std::size_t conservative_bbox_candidate_points{0};
  std::size_t selected_fallback_placements{0};
};

struct SearchReplay {
  OptimizerKind optimizer{OptimizerKind::none};
  bool sparrow_polished{false};
  std::vector<SearchProgressEntry> progress{};
  SearchPhaseMetrics phase_metrics{};
  SeparatorReplayMetrics separator_metrics{};
  SearchCacheMetrics cache_metrics{};
  SearchFallbackMetrics fallback_metrics{};
};

enum class LayoutValidationIssueKind : std::uint8_t {
  missing_piece = 0,
  duplicate_piece = 1,
  unknown_piece = 2,
  unknown_bin = 3,
  bin_mismatch = 4,
  disallowed_bin = 5,
  disallowed_rotation = 6,
  disallowed_mirror = 7,
  outside_container = 8,
  inside_exclusion_zone = 9,
  piece_overlap = 10,
  spacing_violation = 11,
  conservation_repair = 12,
};

struct LayoutValidationIssue {
  LayoutValidationIssueKind issue_kind{
      LayoutValidationIssueKind::missing_piece};
  std::optional<std::uint32_t> expanded_piece_id{};
  std::optional<std::uint32_t> expanded_bin_id{};
  std::optional<std::uint32_t> other_piece_id{};
  double measured_value{0.0};
  double tolerance{0.0};
};

struct LayoutValidationReport {
  bool valid{false};
  std::vector<LayoutValidationIssue> issues{};
};

struct NestingSummary {
  std::size_t placed_parts{0};
  std::size_t unplaced_parts{0};
  bool all_parts_placed{false};
  bool layout_valid{false};
  bool partial_layout{false};
  bool full_success{false};
  double utilization_percent{0.0};
  StopReason stop_reason{StopReason::none};
};

struct NestingResult {
  StrategyKind strategy{StrategyKind::bounding_box};
  pack::Layout layout{};
  std::size_t total_parts{0};
  std::uint64_t effective_seed{0};
  StopReason stop_reason{StopReason::none};
  SearchReplay search{};
  LayoutValidationReport validation{};

  [[nodiscard]] auto placed_parts() const -> std::size_t {
    return layout.placement_trace.size();
  }

  [[nodiscard]] auto unplaced_parts() const -> std::size_t {
    return layout.unplaced_piece_ids.size();
  }

  [[nodiscard]] auto all_parts_placed() const -> bool {
    return placed_parts() == total_parts && unplaced_parts() == 0U;
  }

  [[nodiscard]] auto layout_valid() const -> bool { return validation.valid; }

  [[nodiscard]] auto is_full_success() const -> bool {
    return stop_reason == StopReason::completed && all_parts_placed() &&
           layout_valid();
  }

  [[nodiscard]] auto is_partial_layout() const -> bool {
    return placed_parts() > 0U && !is_full_success();
  }

  [[nodiscard]] auto utilization_percent() const -> double {
    double total_occupied_area = 0.0;
    double total_container_area = 0.0;
    for (const auto &bin : layout.bins) {
      total_occupied_area += bin.utilization.occupied_area;
      total_container_area += bin.utilization.container_area;
    }
    if (total_container_area <= 0.0) {
      return 0.0;
    }
    return (total_occupied_area / total_container_area) * 100.0;
  }

  [[nodiscard]] auto summary() const -> NestingSummary {
    return {
        .placed_parts = placed_parts(),
        .unplaced_parts = unplaced_parts(),
        .all_parts_placed = all_parts_placed(),
        .layout_valid = layout_valid(),
        .partial_layout = is_partial_layout(),
        .full_success = is_full_success(),
        .utilization_percent = utilization_percent(),
        .stop_reason = stop_reason,
    };
  }
};

} // namespace shiny::nesting
