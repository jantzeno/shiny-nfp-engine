#include "packing/irregular/sequential/detail.hpp"

#include <algorithm>
#include <cstddef>
#include <format>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "runtime/timing.hpp"

#include "logging/shiny_log.hpp"
#include "packing/constructive/ordering.hpp"
#include "packing/irregular/workspace.hpp"
#include "validation/layout_validation.hpp"

namespace shiny::nesting::pack::detail {

namespace {} // namespace

// auto make_budget(const SolveControl &control,
//                  const runtime::TimeBudget &time_budget,
//                  const runtime::Stopwatch &stopwatch,
//                  const std::size_t operations_completed) -> BudgetState {
//   return {
//       .operation_limit_enabled = control.operation_limit > 0U,
//       .operation_limit = control.operation_limit,
//       .operations_completed = operations_completed,
//       .time_limit_enabled = time_budget.enabled(),
//       .time_limit_milliseconds = time_budget.limit_milliseconds(),
//       .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
//       .cancellation_requested = control.cancellation.stop_requested(),
//   };
// }

auto emit_progress(const SolveControl &control, const std::size_t sequence,
                   const std::size_t placement_attempts_completed,
                   const std::size_t placements_successful,
                   const std::size_t total_requested_parts,
                   const std::size_t candidate_evaluations_completed,
                   const std::span<const WorkingBin> bins,
                   const std::vector<PlacementTraceEntry> &trace,
                   const std::vector<std::uint32_t> &unplaced,
                   const StopReason stop_reason,
                   const std::string &phase_detail, const bool lightweight)
    -> void {
  if (!control.on_progress) {
    return;
  }

  SHINY_DEBUG(
      "constructive emit_progress: seq={} placed={}/{} bins={} lightweight={} ",
      sequence, placements_successful, total_requested_parts, bins.size(),
      lightweight);

  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placement_attempts_completed = placement_attempts_completed,
      .placements_successful = placements_successful,
      .total_requested_parts = total_requested_parts,
      .candidate_evaluations_completed = candidate_evaluations_completed,
      .layout = lightweight ? build_lightweight_layout(bins, trace, unplaced)
                            : build_layout(bins, trace, unplaced),
      .stop_reason = stop_reason,
      .phase = ProgressPhase::placement,
      .phase_detail = phase_detail,
      .utilization_percent = compute_utilization_percent(bins),
      .improved = !lightweight,
  });
}

auto emit_search_progress(const SolveControl &control,
                          const std::size_t placements_successful,
                          const std::size_t total_requested_parts,
                          const std::size_t candidate_evaluations_completed,
                          const std::string &phase_detail) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placements_successful = placements_successful,
      .total_requested_parts = total_requested_parts,
      .candidate_evaluations_completed = candidate_evaluations_completed,
      .layout = {},
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::placement,
      .phase_detail = phase_detail,
      .utilization_percent = 0.0,
      .improved = false,
  });
}

// auto emit_search_progress(const SolveControl &control,
//                           const std::size_t placements_successful,
//                           const std::size_t total_requested_parts,
//                           const std::size_t candidate_evaluations_completed,
//                           const BudgetState &budget,
//                           const std::string &phase_detail) -> void {
//   if (!control.on_progress) {
//     return;
//   }
//   control.on_progress(ProgressSnapshot{
//       .sequence = 0,
//       .placements_successful = placements_successful,
//       .total_requested_parts = total_requested_parts,
//       .candidate_evaluations_completed = candidate_evaluations_completed,
//       .layout = {},
//       .budget = budget,
//       .stop_reason = StopReason::none,
//       .phase = ProgressPhase::placement,
//       .phase_detail = phase_detail,
//       .utilization_percent = 0.0,
//       .improved = false,
//   });
// }

auto interrupted(const SolveControl &control) -> bool {
  return control.cancellation.stop_requested();
}

auto solve_irregular_constructive(const NormalizedRequest &request,
                                  const SolveControl &control)
    -> std::expected<NestingResult, util::Status> {
  if (!request.request.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }

  const auto piece_instances = order_piece_instances(
      build_piece_instances(request), request.request.execution);
  auto bin_instances = build_bin_instances(request);
  if (piece_instances.size() != request.expanded_pieces.size() ||
      bin_instances.size() != request.expanded_bins.size()) {
    return std::unexpected(util::Status::invalid_input);
  }

  std::unordered_map<std::uint32_t, const PieceInstance *> piece_by_id;
  piece_by_id.reserve(piece_instances.size());
  for (const auto &piece : piece_instances) {
    piece_by_id.emplace(piece.expanded.expanded_piece_id, &piece);
  }

  // runtime::Stopwatch stopwatch;
  // const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);

  std::optional<runtime::DeterministicRng> rng_storage;
  runtime::DeterministicRng *rng_ptr = nullptr;
  if (control.random_seed != 0U) {
    rng_storage.emplace(control.random_seed);
    rng_ptr = &*rng_storage;
  }

  std::optional<PackerWorkspace> fallback_workspace;
  PackerWorkspace *workspace = control.workspace;
  if (workspace == nullptr) {
    fallback_workspace.emplace();
    workspace = &*fallback_workspace;
  }

  ProgressThrottle throttle;
  ProgressThrottle search_throttle;
  std::vector<WorkingBin> opened_bins;
  std::vector<bool> opened_flags(bin_instances.size(), false);
  std::vector<PlacementTraceEntry> trace;
  std::vector<std::uint32_t> unplaced_piece_ids;
  std::size_t placement_attempts_completed = 0;
  std::size_t placements_completed = 0;
  std::size_t processed_pieces = 0;
  std::size_t candidate_evaluations_completed = 0;
  std::size_t sequence = 0;
  StopReason stop_reason = StopReason::completed;
  ConstructiveReplay constructive_replay{};

  for (std::size_t piece_index = 0; piece_index < piece_instances.size();
       ++piece_index) {
    if (control.cancellation.stop_requested()) {
      stop_reason = StopReason::cancelled;
      break;
    }
    // if (time_budget.expired(stopwatch)) {
    //   stop_reason = StopReason::time_limit_reached;
    //   break;
    // }
    if (time_budget.expired(stopwatch)) {
      stop_reason = StopReason::time_limit_reached;
      break;
    }
    const auto &piece = piece_instances[piece_index];
    ++processed_pieces;

    PlacementAttemptContext attempt_context{};

    auto placement_status = try_place_piece(
        piece, request, bin_instances, opened_bins, opened_flags, trace,
        control, search_throttle, placements_completed, piece_instances.size(),
        attempt_context, &workspace->nfp_cache, &workspace->search_metrics,
        rng_ptr, &constructive_replay,
        ConstructivePlacementPhase::primary_order);
    candidate_evaluations_completed +=
        attempt_context.candidate_evaluations_completed;

    if (placement_status == PlacementSearchStatus::interrupted) {
      ++placement_attempts_completed;
      unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
      stop_reason = control.cancellation_requested
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    if (placement_status == PlacementSearchStatus::no_candidate &&
        request.request.execution.irregular.enable_backtracking) {
      attempt_context = {};
      placement_status = try_backtrack_place_piece(
          piece, request, bin_instances, opened_bins, opened_flags, trace,
          piece_by_id, control, search_throttle, piece_instances.size(),
          attempt_context, &workspace->nfp_cache, &workspace->search_metrics,
          rng_ptr, request.request.execution.irregular.max_backtrack_pieces);
      candidate_evaluations_completed +=
          attempt_context.candidate_evaluations_completed;
    }

    if (placement_status == PlacementSearchStatus::interrupted) {
      ++placement_attempts_completed;
      unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
      stop_reason = control.cancellation.stop_requested()
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    ++placement_attempts_completed;
    if (placement_status == PlacementSearchStatus::found) {
      placements_completed = trace.size();
      ++sequence;
      if (throttle.should_emit()) {
        emit_progress(control, sequence, placement_attempts_completed,
                      placements_completed, piece_instances.size(),
                      candidate_evaluations_completed, opened_bins, trace,
                      unplaced_piece_ids, StopReason::none,
                      std::format("Placing part {}/{}", placements_completed,
                                  piece_instances.size()),
                      true);
      }
      continue;
    }

    unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
  }

  if (stop_reason != StopReason::completed) {
    for (std::size_t piece_index = processed_pieces;
         piece_index < piece_instances.size(); ++piece_index) {
      unplaced_piece_ids.push_back(
          piece_instances[piece_index].expanded.expanded_piece_id);
    }
  }

  if (stop_reason == StopReason::completed &&
      request.request.execution.irregular.enable_compaction) {
    for (std::uint32_t pass = 0;
         pass < request.request.execution.irregular.compaction_passes; ++pass) {
      bool moved_any = false;
      for (auto &bin : opened_bins) {
        std::size_t placement_index = 0;
        while (placement_index < bin.state.placements.size()) {
          const auto original = bin.state.placements[placement_index];
          const auto piece_it = piece_by_id.find(original.placement.piece_id);
          if (piece_it == piece_by_id.end()) {
            ++placement_index;
            continue;
          }

          CandidatePlacement current_candidate{
              .placement = original.placement,
              .piece_geometry_revision = original.piece_geometry_revision,
              .resolved_rotation = original.resolved_rotation,
              .polygon = original.polygon,
              .bounds = geom::compute_bounds(original.polygon),
              .source = original.source,
              .nfp_accuracy = original.nfp_accuracy,
              .inside_hole = original.inside_hole,
              .hole_index = original.hole_index,
              .score = original.score,
          };

          const auto original_trace_index =
              std::find_if(trace.begin(), trace.end(), [&](const auto &entry) {
                return entry.piece_id == original.placement.piece_id;
              });
          const auto restore_trace_position = [&](const std::size_t index) {
            if (trace.empty() || index >= trace.size()) {
              return;
            }
            std::rotate(trace.begin() + static_cast<std::ptrdiff_t>(index),
                        trace.end() - 1, trace.end());
          };
          const auto restore_placement_position = [&](const std::size_t index) {
            if (bin.state.placements.empty() ||
                index >= bin.state.placements.size()) {
              return;
            }
            std::rotate(bin.state.placements.begin() +
                            static_cast<std::ptrdiff_t>(index),
                        bin.state.placements.end() - 1,
                        bin.state.placements.end());
            std::rotate(bin.placement_bounds.begin() +
                            static_cast<std::ptrdiff_t>(index),
                        bin.placement_bounds.end() - 1,
                        bin.placement_bounds.end());
          };
          const auto trace_index =
              original_trace_index == trace.end()
                  ? std::nullopt
                  : std::optional<std::size_t>(static_cast<std::size_t>(
                        std::distance(trace.begin(), original_trace_index)));
          const bool opened_new_bin = original_trace_index != trace.end() &&
                                      original_trace_index->opened_new_bin;

          bin.state.placements.erase(
              bin.state.placements.begin() +
              static_cast<std::ptrdiff_t>(placement_index));
          remove_trace_entries_for_piece(trace, original.placement.piece_id);
          rebuild_working_bin(bin, request.request.execution);

          PlacementAttemptContext compaction_attempt{};
          PlacementSearchResult search_result;
          if (request.request.execution.irregular.candidate_strategy ==
              CandidateStrategy::nfp_perfect) {
            search_result = try_exact_fit_candidate(
                bin, *piece_it->second, request, control, compaction_attempt);
          }
          if (!search_result.candidate.has_value() &&
              search_result.status != PlacementSearchStatus::interrupted) {
            search_result = find_best_for_bin(
                bin, *piece_it->second, request, control, search_throttle,
                placements_completed, piece_instances.size(),
                compaction_attempt, &workspace->nfp_cache,
                &workspace->search_metrics, nullptr);
          }
          candidate_evaluations_completed +=
              compaction_attempt.candidate_evaluations_completed;

          if (search_result.candidate.has_value() &&
              better_candidate(bin, request.request.execution.placement_policy,
                               *search_result.candidate, current_candidate)) {
            apply_candidate(bin, *search_result.candidate, trace,
                            opened_new_bin, request.request.execution);
            restore_placement_position(placement_index);
            if (trace_index.has_value()) {
              restore_trace_position(*trace_index);
            }
            moved_any = true;
            continue;
          }

          apply_candidate(bin, current_candidate, trace, opened_new_bin,
                          request.request.execution);
          restore_placement_position(placement_index);
          if (trace_index.has_value()) {
            restore_trace_position(*trace_index);
          }
          ++placement_index;
        }
      }

      if (!moved_any) {
        break;
      }
    }
  }

  // Multi-pass gap-fill: attempt to place pieces that failed during the primary
  // ordering pass by retrying them after each pass that makes progress.
  //
  // Deferred vs. unplaceable distinction:
  //   - A piece is *deferred* when it fails in a pass but a subsequent
  //     placement in the same pass opens a new bin, giving it a retry
  //     opportunity in the next pass.
  //   - A piece is *unplaceable* when it survives a full pass in which nothing
  //     was placed — no new bin was opened, so no retry can help.
  //
  // This loop is naturally frontier-monotonic: each pass can only open bins
  // forward, never revisiting already-exhausted frontier positions.
  //
  // kMaxGapFillPasses caps worst-case iterations to keep termination obvious.
  static constexpr std::uint32_t kMaxGapFillPasses{64};

  if (stop_reason == StopReason::completed &&
      request.request.execution.irregular.enable_gap_fill &&
      !unplaced_piece_ids.empty()) {
    for (std::uint32_t gap_pass = 0;
         gap_pass < kMaxGapFillPasses && !unplaced_piece_ids.empty();
         ++gap_pass) {
      bool made_progress = false;

      std::vector<std::uint32_t> remaining_unplaced;
      remaining_unplaced.reserve(unplaced_piece_ids.size());

      const auto gap_fill_order = constructive::build_fill_first_gap_fill_order(
          request, unplaced_piece_ids);
      for (const auto &ordered_piece : gap_fill_order) {
        const auto piece_id = ordered_piece.expanded_piece_id;
        const auto piece_it = piece_by_id.find(piece_id);
        if (piece_it == piece_by_id.end()) {
          remaining_unplaced.push_back(piece_id);
          continue;
        }

        PlacementAttemptContext retry_attempt{};
        const auto placement_status = try_place_piece(
            *piece_it->second, request, bin_instances, opened_bins,
            opened_flags, trace, control, search_throttle, placements_completed,
            piece_instances.size(), retry_attempt, &workspace->nfp_cache,
            &workspace->search_metrics, nullptr, &constructive_replay,
            ConstructivePlacementPhase::gap_fill);
        candidate_evaluations_completed +=
            retry_attempt.candidate_evaluations_completed;

        if (placement_status == PlacementSearchStatus::found) {
          ++placements_completed;
          made_progress = true;
          continue;
        }

        remaining_unplaced.push_back(piece_id);
      }

      unplaced_piece_ids = std::move(remaining_unplaced);

      // If nothing was placed this pass, every remaining candidate is
      // unplaceable on the current frontier — stop retrying.
      if (!made_progress) {
        break;
      }
    }
  }

  NestingResult result{
      .strategy = StrategyKind::metaheuristic_search,
      .layout = build_layout(opened_bins, trace, unplaced_piece_ids),
      .total_parts = piece_instances.size(),
      .effective_seed = control.random_seed,
      .stop_reason = stop_reason,
      .constructive = std::move(constructive_replay),
  };
  validation::finalize_result(request, result);

  ++sequence;
  emit_progress(
      control, sequence, placement_attempts_completed, placements_completed,
      piece_instances.size(), candidate_evaluations_completed, opened_bins,
      trace, result.layout.unplaced_piece_ids, stop_reason,
      std::format("Placed {}/{}", placements_completed, piece_instances.size()),
      false);
  return result;
}

// auto solve_irregular_constructive_bak(const NormalizedRequest &request,
//                                   const SolveControl &control)
//     -> std::expected<NestingResult, util::Status> {
//   if (!request.request.is_valid()) {
//     return std::unexpected(util::Status::invalid_input);
//   }

//   const auto piece_instances = order_piece_instances(
//       build_piece_instances(request), request.request.execution);
//   auto bin_instances = build_bin_instances(request);
//   if (piece_instances.size() != request.expanded_pieces.size() ||
//       bin_instances.size() != request.expanded_bins.size()) {
//     return std::unexpected(util::Status::invalid_input);
//   }

//   std::unordered_map<std::uint32_t, const PieceInstance *> piece_by_id;
//   piece_by_id.reserve(piece_instances.size());
//   for (const auto &piece : piece_instances) {
//     piece_by_id.emplace(piece.expanded.expanded_piece_id, &piece);
//   }

//   runtime::Stopwatch stopwatch;
//   const runtime::TimeBudget time_budget(control.time_limit_milliseconds);

//   std::optional<runtime::DeterministicRng> rng_storage;
//   runtime::DeterministicRng *rng_ptr = nullptr;
//   if (control.random_seed != 0U) {
//     rng_storage.emplace(control.random_seed);
//     rng_ptr = &*rng_storage;
//   }

//   std::optional<PackerWorkspace> fallback_workspace;
//   PackerWorkspace *workspace = control.workspace;
//   if (workspace == nullptr) {
//     fallback_workspace.emplace();
//     workspace = &*fallback_workspace;
//   }

//   ProgressThrottle throttle;
//   ProgressThrottle search_throttle;
//   std::vector<WorkingBin> opened_bins;
//   std::vector<bool> opened_flags(bin_instances.size(), false);
//   std::vector<PlacementTraceEntry> trace;
//   std::vector<std::uint32_t> unplaced_piece_ids;
//   std::size_t placement_attempts_completed = 0;
//   std::size_t placements_completed = 0;
//   std::size_t processed_pieces = 0;
//   std::size_t candidate_evaluations_completed = 0;
//   std::size_t sequence = 0;
//   StopReason stop_reason = StopReason::completed;

//   for (std::size_t piece_index = 0; piece_index < piece_instances.size();
//        ++piece_index) {
//     if (control.cancellation.stop_requested()) {
//       stop_reason = StopReason::cancelled;
//       break;
//     }
//     if (time_budget.expired(stopwatch)) {
//       stop_reason = StopReason::time_limit_reached;
//       break;
//     }
//     const auto &piece = piece_instances[piece_index];
//     ++processed_pieces;

//     const auto remaining_pieces = piece_instances.size() - piece_index;
//     const auto per_piece_budget_ms =
//         derive_per_piece_budget_ms(time_budget, stopwatch, remaining_pieces);

//     PlacementAttemptContext attempt_context{
//         .started_milliseconds = stopwatch.elapsed_milliseconds(),
//         .budget_milliseconds = per_piece_budget_ms,
//     };

//     auto placement_status = try_place_piece(
//         piece, request, bin_instances, opened_bins, opened_flags, trace,
//         time_budget, stopwatch, control, search_throttle,
//         placements_completed, piece_instances.size(), attempt_context,
//         &workspace->nfp_cache, &workspace->search_metrics, rng_ptr);
//     candidate_evaluations_completed +=
//         attempt_context.candidate_evaluations_completed;

//     if (placement_status == PlacementSearchStatus::interrupted ||
//         interrupted(control, time_budget, stopwatch)) {
//       ++placement_attempts_completed;
//       unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
//       stop_reason = control.cancellation.stop_requested()
//                         ? StopReason::cancelled
//                         : StopReason::time_limit_reached;
//       break;
//     }

//     if (placement_status == PlacementSearchStatus::no_candidate &&
//         request.request.execution.irregular.enable_backtracking) {
//       attempt_context.candidate_evaluations_completed = 0;
//       placement_status = try_backtrack_place_piece(
//           piece, request, bin_instances, opened_bins, opened_flags, trace,
//           piece_by_id, time_budget, stopwatch, control, search_throttle,
//           piece_instances.size(), attempt_context, &workspace->nfp_cache,
//           &workspace->search_metrics, rng_ptr,
//           request.request.execution.irregular.max_backtrack_pieces);
//       candidate_evaluations_completed +=
//           attempt_context.candidate_evaluations_completed;
//     }

//     if (placement_status == PlacementSearchStatus::interrupted ||
//         interrupted(control, time_budget, stopwatch)) {
//       ++placement_attempts_completed;
//       unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
//       stop_reason = control.cancellation.stop_requested()
//                         ? StopReason::cancelled
//                         : StopReason::time_limit_reached;
//       break;
//     }

//     ++placement_attempts_completed;
//     if (placement_status == PlacementSearchStatus::found) {
//       placements_completed = trace.size();
//       ++sequence;
//       if (throttle.should_emit()) {
//         emit_progress(control, sequence, placement_attempts_completed,
//                       placements_completed, piece_instances.size(),
//                       candidate_evaluations_completed, opened_bins, trace,
//                       unplaced_piece_ids,
//                       make_budget(control, time_budget, stopwatch, 0U),
//                       StopReason::none,
//                       std::format("Placing part {}/{}", placements_completed,
//                                   piece_instances.size()),
//                       true);
//       }
//       continue;
//     }

//     unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
//   }

//   if (stop_reason != StopReason::completed) {
//     for (std::size_t piece_index = processed_pieces;
//          piece_index < piece_instances.size(); ++piece_index) {
//       unplaced_piece_ids.push_back(
//           piece_instances[piece_index].expanded.expanded_piece_id);
//     }
//   }

//   if (stop_reason == StopReason::completed &&
//       request.request.execution.irregular.enable_compaction) {
//     for (std::uint32_t pass = 0;
//          pass < request.request.execution.irregular.compaction_passes;
//          ++pass) {
//       bool moved_any = false;
//       for (auto &bin : opened_bins) {
//         std::size_t placement_index = 0;
//         while (placement_index < bin.state.placements.size()) {
//           const auto original = bin.state.placements[placement_index];
//           const auto piece_it =
//           piece_by_id.find(original.placement.piece_id); if (piece_it ==
//           piece_by_id.end()) {
//             ++placement_index;
//             continue;
//           }

//           CandidatePlacement current_candidate{
//               .placement = original.placement,
//               .piece_geometry_revision = original.piece_geometry_revision,
//               .resolved_rotation = original.resolved_rotation,
//               .polygon = original.polygon,
//               .bounds = geom::compute_bounds(original.polygon),
//               .source = original.source,
//               .nfp_accuracy = original.nfp_accuracy,
//               .inside_hole = original.inside_hole,
//               .hole_index = original.hole_index,
//               .score = original.score,
//           };

//           bin.state.placements.erase(
//               bin.state.placements.begin() +
//               static_cast<std::ptrdiff_t>(placement_index));
//           remove_trace_entries_for_piece(trace, original.placement.piece_id);
//           rebuild_working_bin(bin, request.request.execution);

//           PlacementAttemptContext compaction_attempt{
//               .started_milliseconds = stopwatch.elapsed_milliseconds(),
//               .budget_milliseconds = derive_per_piece_budget_ms(
//                   time_budget, stopwatch,
//                   std::max<std::size_t>(1U, piece_instances.size())),
//           };
//           const auto search_result = find_best_for_bin(
//               bin, *piece_it->second, request, time_budget, stopwatch,
//               control, search_throttle, placements_completed,
//               piece_instances.size(), compaction_attempt,
//               &workspace->nfp_cache, &workspace->search_metrics, nullptr);
//           candidate_evaluations_completed +=
//               compaction_attempt.candidate_evaluations_completed;

//           if (search_result.candidate.has_value() &&
//               better_candidate(bin,
//               request.request.execution.placement_policy,
//                                *search_result.candidate, current_candidate))
//                                {
//             apply_candidate(bin, *search_result.candidate, trace, false,
//                             request.request.execution);
//             moved_any = true;
//             continue;
//           }

//           apply_candidate(bin, current_candidate, trace, false,
//                           request.request.execution);
//           ++placement_index;
//         }
//       }

//       if (!moved_any) {
//         break;
//       }
//     }

//     std::vector<std::uint32_t> remaining_unplaced;
//     remaining_unplaced.reserve(unplaced_piece_ids.size());
//     for (const auto piece_id : unplaced_piece_ids) {
//       const auto piece_it = piece_by_id.find(piece_id);
//       if (piece_it == piece_by_id.end()) {
//         remaining_unplaced.push_back(piece_id);
//         continue;
//       }

//       PlacementAttemptContext retry_attempt{
//           .started_milliseconds = stopwatch.elapsed_milliseconds(),
//           .budget_milliseconds = derive_per_piece_budget_ms(
//               time_budget, stopwatch,
//               std::max<std::size_t>(1U, remaining_unplaced.size())),
//       };
//       const auto placement_status = try_place_piece(
//           *piece_it->second, request, bin_instances, opened_bins,
//           opened_flags, trace, time_budget, stopwatch, control,
//           search_throttle, placements_completed, piece_instances.size(),
//           retry_attempt, &workspace->nfp_cache, &workspace->search_metrics,
//           nullptr);
//       candidate_evaluations_completed +=
//           retry_attempt.candidate_evaluations_completed;

//       if (placement_status == PlacementSearchStatus::found) {
//         ++placements_completed;
//         continue;
//       }

//       remaining_unplaced.push_back(piece_id);
//     }
//     unplaced_piece_ids = std::move(remaining_unplaced);
//   }

//   NestingResult result{
//       .strategy = StrategyKind::sequential_backtrack,
//       .layout = build_layout(opened_bins, trace, unplaced_piece_ids),
//       .total_parts = piece_instances.size(),
//       .effective_seed = control.random_seed,
//       .budget = make_budget(control, time_budget, stopwatch,
//                             stop_reason == StopReason::completed ? 1U : 0U),
//       .stop_reason = stop_reason,
//   };
//   validation::finalize_result(request, result);

//   ++sequence;
//   emit_progress(
//       control, sequence, placement_attempts_completed, placements_completed,
//       piece_instances.size(), candidate_evaluations_completed, opened_bins,
//       trace, result.layout.unplaced_piece_ids, result.budget, stop_reason,
//       std::format("Placed {}/{}", placements_completed,
//       piece_instances.size()), false);
//   return result;
// }

} // namespace shiny::nesting::pack::detail
