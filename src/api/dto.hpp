#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "api/solve_control.hpp"
#include "request.hpp"
#include "result.hpp"

namespace shiny::nesting::api {

struct BinProgressSummaryDto {
  std::uint32_t bin_id{0};
  std::size_t placed_count{0};
  double utilization_percent{0.0};
  bool active{false};
};

struct ProfileSolveControlDto {
  std::size_t operation_limit{0};
  std::uint64_t random_seed{0};
  SeedProgressionMode seed_mode{SeedProgressionMode::increment};
};

struct ProfileSolveRequestDto {
  std::vector<BinRequest> bins{};
  std::vector<PieceRequest> pieces{};
  PreprocessPolicy preprocess{};
  SolveProfile profile{SolveProfile::balanced};
  ObjectiveMode objective_mode{ObjectiveMode::placement_count};
  std::optional<std::uint64_t> time_limit_milliseconds{};
  std::vector<std::uint32_t> selected_bin_ids{};
  bool allow_part_overflow{true};
  bool maintain_bed_assignment{false};
  ProfileSolveControlDto control{};
};

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

struct ProfileProgressSnapshotDto {
  SolveProfile profile{SolveProfile::balanced};
  ProgressPhase phase{ProgressPhase::none};
  std::string phase_detail{};
  LayoutDto current_layout{};
  LayoutDto best_layout{};
  std::optional<std::uint32_t> active_bin_id{};
  std::vector<BinProgressSummaryDto> bin_summary{};
  std::size_t placed_count{0};
  double utilization_percent{0.0};
  std::uint64_t elapsed_time_milliseconds{0};
  std::optional<std::uint64_t> remaining_time_milliseconds{};
  StopReason stop_reason{StopReason::none};
  bool improved{false};
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

[[nodiscard]] inline auto to_dto(const ProfileSolveControl &control)
    -> ProfileSolveControlDto {
  return {
      .operation_limit = control.operation_limit,
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

[[nodiscard]] inline auto to_solve_control(const ProfileSolveControlDto &dto)
    -> ProfileSolveControl {
  return {
      .operation_limit = dto.operation_limit,
      .random_seed = dto.random_seed,
      .seed_mode = dto.seed_mode,
  };
}

[[nodiscard]] inline auto to_dto(const ProfileRequest &request,
                                 const ProfileSolveControl &control = {})
    -> ProfileSolveRequestDto {
  return {
      .bins = request.bins,
      .pieces = request.pieces,
      .preprocess = request.preprocess,
      .profile = request.profile,
      .objective_mode = request.objective_mode,
      .time_limit_milliseconds = request.time_limit_milliseconds,
      .selected_bin_ids = request.selected_bin_ids,
      .allow_part_overflow = request.allow_part_overflow,
      .maintain_bed_assignment = request.maintain_bed_assignment,
      .control = to_dto(control),
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

[[nodiscard]] inline auto to_request(const ProfileSolveRequestDto &dto)
    -> ProfileRequest {
  return {
      .bins = dto.bins,
      .pieces = dto.pieces,
      .preprocess = dto.preprocess,
      .profile = dto.profile,
      .objective_mode = dto.objective_mode,
      .time_limit_milliseconds = dto.time_limit_milliseconds,
      .selected_bin_ids = dto.selected_bin_ids,
      .allow_part_overflow = dto.allow_part_overflow,
      .maintain_bed_assignment = dto.maintain_bed_assignment,
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

[[nodiscard]] inline auto to_dto(const BinProgressSummary &summary)
    -> BinProgressSummaryDto {
  return {
      .bin_id = summary.bin_id,
      .placed_count = summary.placed_count,
      .utilization_percent = summary.utilization_percent,
      .active = summary.active,
  };
}

[[nodiscard]] inline auto to_dto(const ProfileProgressSnapshot &snapshot)
    -> ProfileProgressSnapshotDto {
  std::vector<BinProgressSummaryDto> bin_summary;
  bin_summary.reserve(snapshot.bin_summary.size());
  for (const auto &entry : snapshot.bin_summary) {
    bin_summary.push_back(to_dto(entry));
  }

  return {
      .profile = snapshot.profile,
      .phase = snapshot.phase,
      .phase_detail = snapshot.phase_detail,
      .current_layout = to_dto(snapshot.current_layout),
      .best_layout = to_dto(snapshot.best_layout),
      .active_bin_id = snapshot.active_bin_id,
      .bin_summary = std::move(bin_summary),
      .placed_count = snapshot.placed_count,
      .utilization_percent = snapshot.utilization_percent,
      .elapsed_time_milliseconds = snapshot.elapsed_time_milliseconds,
      .remaining_time_milliseconds = snapshot.remaining_time_milliseconds,
      .stop_reason = snapshot.stop_reason,
      .improved = snapshot.improved,
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
