#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace shiny::nesting::api {

struct SolveControlDto {
  std::size_t operation_limit{0};
  std::uint64_t time_limit_milliseconds{0};
  std::uint64_t random_seed{0};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
};

struct SolveRequestDto {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  ExecutionPolicy execution{};
  SolveControlDto control{};
};

struct LayoutDto {
  std::size_t bin_count{0};
  std::size_t placement_count{0};
  std::vector<std::uint32_t> unplaced_piece_ids{};
};

struct LayoutValidationIssueDto {
  LayoutValidationIssueKind issue_kind{
      LayoutValidationIssueKind::missing_piece};
  std::optional<std::uint32_t> expanded_piece_id{};
  std::optional<std::uint32_t> expanded_bin_id{};
  std::optional<std::uint32_t> other_piece_id{};
  double measured_value{0.0};
  double tolerance{0.0};
};

struct LayoutValidationReportDto {
  bool valid{false};
  std::vector<LayoutValidationIssueDto> issues{};
};

struct NestingSummaryDto {
  std::size_t placed_parts{0};
  std::size_t unplaced_parts{0};
  bool all_parts_placed{false};
  bool layout_valid{false};
  bool partial_layout{false};
  bool full_success{false};
  double utilization_percent{0.0};
  StopReason stop_reason{StopReason::none};
};

struct SolveResultDto {
  StrategyKind strategy{StrategyKind::bounding_box};
  LayoutDto layout{};
  std::size_t total_parts{0};
  std::uint64_t effective_seed{0};
  StopReason stop_reason{StopReason::none};
  OptimizerKind optimizer{OptimizerKind::none};
  bool sparrow_polished{false};
  LayoutValidationReportDto validation{};
  NestingSummaryDto summary{};
};

[[nodiscard]] inline auto to_dto(const SolveControl &control)
    -> SolveControlDto {
  return {
      .operation_limit = control.operation_limit,
      .time_limit_milliseconds = control.time_limit_milliseconds,
      .random_seed = control.random_seed,
      .seed_mode = control.seed_mode,
  };
}

[[nodiscard]] inline auto to_solve_control(const SolveControlDto &dto)
    -> SolveControl {
  return {
      .operation_limit = dto.operation_limit,
      .time_limit_milliseconds = dto.time_limit_milliseconds,
      .random_seed = dto.random_seed,
      .seed_mode = dto.seed_mode,
  };
}

[[nodiscard]] inline auto to_dto(const NestingRequest &request,
                                 const SolveControl &control = {})
    -> SolveRequestDto {
  return {
      .bins = request.bins,
      .pieces = request.pieces,
      .preprocess = request.preprocess,
      .execution = request.execution,
      .control = to_dto(control),
  };
}

[[nodiscard]] inline auto to_request(const SolveRequestDto &dto)
    -> NestingRequest {
  return {
      .bins = dto.bins,
      .pieces = dto.pieces,
      .preprocess = dto.preprocess,
      .execution = dto.execution,
  };
}

[[nodiscard]] inline auto to_dto(const LayoutValidationIssue &issue)
    -> LayoutValidationIssueDto {
  return {
      .issue_kind = issue.issue_kind,
      .expanded_piece_id = issue.expanded_piece_id,
      .expanded_bin_id = issue.expanded_bin_id,
      .other_piece_id = issue.other_piece_id,
      .measured_value = issue.measured_value,
      .tolerance = issue.tolerance,
  };
}

[[nodiscard]] inline auto to_dto(const LayoutValidationReport &report)
    -> LayoutValidationReportDto {
  LayoutValidationReportDto dto;
  dto.valid = report.valid;
  dto.issues.reserve(report.issues.size());
  for (const auto &issue : report.issues) {
    dto.issues.push_back(to_dto(issue));
  }
  return dto;
}

[[nodiscard]] inline auto to_dto(const NestingSummary &summary)
    -> NestingSummaryDto {
  return {
      .placed_parts = summary.placed_parts,
      .unplaced_parts = summary.unplaced_parts,
      .all_parts_placed = summary.all_parts_placed,
      .layout_valid = summary.layout_valid,
      .partial_layout = summary.partial_layout,
      .full_success = summary.full_success,
      .utilization_percent = summary.utilization_percent,
      .stop_reason = summary.stop_reason,
  };
}

[[nodiscard]] inline auto to_dto(const pack::Layout &layout) -> LayoutDto {
  return {
      .bin_count = layout.bins.size(),
      .placement_count = layout.placement_trace.size(),
      .unplaced_piece_ids = layout.unplaced_piece_ids,
  };
}

[[nodiscard]] inline auto to_dto(const NestingResult &result)
    -> SolveResultDto {
  return {
      .strategy = result.strategy,
      .layout = to_dto(result.layout),
      .total_parts = result.total_parts,
      .effective_seed = result.effective_seed,
      .stop_reason = result.stop_reason,
      .optimizer = result.search.optimizer,
      .sparrow_polished = result.search.sparrow_polished,
      .validation = to_dto(result.validation),
      .summary = to_dto(result.summary()),
  };
}

} // namespace shiny::nesting::api
