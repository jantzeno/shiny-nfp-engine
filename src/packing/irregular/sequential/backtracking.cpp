// Backtracking placement helper for the irregular constructive packer
// (Plan §10.7 / review Phase 10). Two entry points:
//
//   * `try_place_piece` — first tries every already-opened bin, picking
//     the best-scoring candidate, and falls back to opening a fresh bin
//     when nothing fits.
//   * `try_backtrack_place_piece` — when straight placement fails,
//     speculatively removes the most-recently-placed K pieces (for
//     K = 1..max_backtrack_pieces) and retries with the failing piece
//     pushed to the front of the retry order. Each speculative trial is
//     scoped by `BinTrialGuard`, which restores the bin states / opened
//     flags / placement trace if the trial does not commit.
#include "packing/irregular/core.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "logging/shiny_log.hpp"

namespace shiny::nesting::pack::detail {
namespace {

// RAII rollback scope for one speculative backtracking trial. Captures
// references to the live bins / opened flags / placement trace and
// records every mutation made during the trial. On destruction, if
// `commit()` has not been called, the recorded mutations are reversed
// in LIFO order so the caller observes the same state it started with.
//
// Trace handling note:
//   We track `appended_trace_entries_` as a simple counter rather than
//   diffing the trace at rollback time. The trial may both REMOVE
//   existing entries (recorded individually so we can re-insert them
//   at their original indices) AND APPEND new entries from successful
//   speculative placements. Reconstructing the appended count from a
//   pre/post diff would be ambiguous because removals can shrink the
//   trace before the appends happen, leaving the post-trial size
//   smaller than the pre-trial size even though new entries were
//   pushed. The explicit counter avoids that ambiguity.
class BinTrialGuard {
public:
  BinTrialGuard(std::vector<BinInstance> &bin_instances,
                std::vector<WorkingBin> &bins, std::vector<bool> &flags,
                std::vector<PlacementTraceEntry> &trace)
      : bin_instances_{bin_instances}, bins_{bins}, flags_{flags},
        trace_{trace}, original_bin_instance_count_{bin_instances.size()},
        original_bin_count_{bins.size()}, original_flag_count_{flags.size()} {}

  ~BinTrialGuard() {
    if (!committed_) {
      rollback();
    }
  }

  auto snapshot_bin(const std::size_t bin_index) -> void {
    const auto already_saved =
        std::find_if(snapshots_.begin(), snapshots_.end(),
                     [bin_index](const BinSnapshot &snapshot) {
                       return snapshot.bin_index == bin_index;
                     });
    if (already_saved == snapshots_.end()) {
      snapshots_.push_back({.bin_index = bin_index, .state = bins_[bin_index]});
    }
  }

  auto record_opened_bin(const std::size_t flag_index) -> void {
    opened_flag_indices_.push_back(flag_index);
  }

  auto record_removed_trace_entry(const std::size_t index,
                                  const PlacementTraceEntry &entry) -> void {
    removed_trace_entries_.push_back({.index = index, .entry = entry});
  }

  auto record_appended_trace_entry() -> void { ++appended_trace_entries_; }

  auto commit() -> void { committed_ = true; }

private:
  struct BinSnapshot {
    std::size_t bin_index{0};
    WorkingBin state{};
  };

  struct RemovedTraceEntry {
    std::size_t index{0};
    PlacementTraceEntry entry{};
  };

  auto rollback() -> void {
    // The appended counter must never exceed the live trace size: every
    // increment corresponds to an append we made into `trace_` during
    // this trial, and only this guard removes appended entries on
    // rollback. A mismatch would indicate someone else truncated the
    // trace behind our back and we'd resize past zero.
    assert(appended_trace_entries_ <= trace_.size());
    if (appended_trace_entries_ > 0U &&
        appended_trace_entries_ <= trace_.size()) {
      trace_.resize(trace_.size() - appended_trace_entries_);
    }
    for (auto it = removed_trace_entries_.rbegin();
         it != removed_trace_entries_.rend(); ++it) {
      trace_.insert(trace_.begin() + static_cast<std::ptrdiff_t>(it->index),
                    it->entry);
    }

    bin_instances_.resize(original_bin_instance_count_);
    flags_.resize(original_flag_count_);
    for (const auto flag_index : opened_flag_indices_) {
      if (flag_index < flags_.size()) {
        flags_[flag_index] = false;
      }
    }
    bins_.resize(original_bin_count_);
    for (const auto &snapshot : snapshots_) {
      bins_[snapshot.bin_index] = snapshot.state;
    }
  }

  std::vector<BinInstance> &bin_instances_;
  std::vector<WorkingBin> &bins_;
  std::vector<bool> &flags_;
  std::vector<PlacementTraceEntry> &trace_;
  std::size_t original_bin_instance_count_{0};
  std::size_t original_bin_count_{0};
  std::size_t original_flag_count_{0};
  std::vector<BinSnapshot> snapshots_{};
  std::vector<std::size_t> opened_flag_indices_{};
  std::vector<RemovedTraceEntry> removed_trace_entries_{};
  std::size_t appended_trace_entries_{0};
  bool committed_{false};
};

[[nodiscard]] auto
find_bin_instance_index(const std::vector<BinInstance> &bin_instances,
                        const std::uint32_t bin_id)
    -> std::optional<std::size_t> {
  for (std::size_t index = 0; index < bin_instances.size(); ++index) {
    if (bin_instances[index].expanded.expanded_bin_id == bin_id) {
      return index;
    }
  }
  return std::nullopt;
}

[[nodiscard]] auto
next_overflow_bin_id(const std::vector<BinInstance> &bin_instances)
    -> std::uint32_t {
  std::uint32_t max_id = 0;
  for (const auto &bin : bin_instances) {
    max_id = std::max(max_id, bin.expanded.expanded_bin_id);
  }
  return max_id + 1U;
}

auto append_overflow_bin_instance(std::vector<BinInstance> &bin_instances,
                                  std::vector<bool> &opened_flags,
                                  const BinInstance &template_instance,
                                  const std::uint32_t template_bin_id)
    -> std::size_t {
  BinInstance overflow_instance = template_instance;
  overflow_instance.expanded.expanded_bin_id =
      next_overflow_bin_id(bin_instances);
  overflow_instance.expanded.stock_index =
      static_cast<std::uint32_t>(bin_instances.size());
  overflow_instance.expanded.identity = {
      .lifecycle = pack::BinLifecycle::engine_overflow,
      .source_request_bin_id =
          template_instance.expanded.identity.source_request_bin_id != 0U
              ? template_instance.expanded.identity.source_request_bin_id
              : template_instance.expanded.source_bin_id,
      .template_bin_id = template_bin_id,
  };

  bin_instances.push_back(std::move(overflow_instance));
  opened_flags.push_back(false);
  return bin_instances.size() - 1U;
}

} // namespace

auto try_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    std::vector<BinInstance> &bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace, const SolveControl &control,
    ProgressThrottle &search_throttle, const std::size_t placed_parts,
    const std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, PackerSearchMetrics *search_metrics,
    runtime::DeterministicRng *rng_ptr, ConstructiveReplay *constructive_replay,
    const ConstructivePlacementPhase placement_phase,
    const TrialStateRecorder *trial_recorder) -> PlacementSearchStatus {
  const auto candidate_strategy =
      request.request.execution.irregular.candidate_strategy;

  // Shared per-bin candidate evaluator: both the existing-bin loop and
  // the open-new-bin loop need to call `find_best_for_bin` with the
  // same long argument list. Capture once so the call sites stay short
  // and any future signature change happens in one place.
  const auto find_best = [&](WorkingBin &bin) {
    if (candidate_strategy == CandidateStrategy::nfp_perfect) {
      PlacementAttemptContext exact_fit_attempt{};
      auto exact_fit_result = try_exact_fit_candidate(
          bin, piece, request, control, exact_fit_attempt);
      attempt_context.candidate_evaluations_completed +=
          exact_fit_attempt.candidate_evaluations_completed;
      if (exact_fit_result.status == PlacementSearchStatus::interrupted ||
          exact_fit_result.candidate.has_value()) {
        return exact_fit_result;
      }
    }

    return find_best_for_bin(bin, piece, request, control, search_throttle,
                             placed_parts, total_parts, attempt_context,
                             nfp_cache, search_metrics, rng_ptr);
  };

  const bool use_strict_frontier = !piece.restricted_to_allowed_bins;
  std::optional<std::size_t> overflow_template_index;

  if (!opened_bins.empty()) {
    if (use_strict_frontier) {
      const std::size_t frontier_index = opened_bins.size() - 1U;
      overflow_template_index = find_bin_instance_index(
          bin_instances, opened_bins[frontier_index].state.bin_id);
      auto result = find_best(opened_bins[frontier_index]);
      if (result.status == PlacementSearchStatus::interrupted) {
        return result.status;
      }
      if (result.candidate.has_value()) {
        if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
          trial_recorder->snapshot_bin(frontier_index);
        }
        apply_candidate(opened_bins[frontier_index], *result.candidate, trace,
                        false, request.request.execution, placement_phase,
                        trial_recorder);
        return PlacementSearchStatus::found;
      }

      if (candidate_strategy != CandidateStrategy::nfp_perfect) {
        PlacementAttemptContext exact_fit_attempt{};
        auto exact_fit_result =
            try_exact_fit_candidate(opened_bins[frontier_index], piece, request,
                                    control, exact_fit_attempt);
        attempt_context.candidate_evaluations_completed +=
            exact_fit_attempt.candidate_evaluations_completed;
        if (exact_fit_result.status == PlacementSearchStatus::interrupted) {
          return exact_fit_result.status;
        }
        if (exact_fit_result.candidate.has_value()) {
          if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
            trial_recorder->snapshot_bin(frontier_index);
          }
          apply_candidate(opened_bins[frontier_index],
                          *exact_fit_result.candidate, trace, false,
                          request.request.execution, placement_phase,
                          trial_recorder);
          return PlacementSearchStatus::found;
        }
      }

      const auto frontier_status = frontier_exhaustion_status_for_piece(
          opened_bins[frontier_index], piece, request.request.execution);
      if (constructive_replay != nullptr) {
        constructive_replay->exhaustion_events.push_back({
            .piece_id = piece.expanded.expanded_piece_id,
            .frontier_bin_id = opened_bins[frontier_index].state.bin_id,
            .decision =
                frontier_status == FrontierExhaustionStatus::exhausted
                    ? ConstructiveFrontierExhaustionDecision::exhausted
                    : ConstructiveFrontierExhaustionDecision::fit_may_exist,
        });
      }
      if (frontier_status != FrontierExhaustionStatus::exhausted) {
        return PlacementSearchStatus::no_candidate;
      }
    } else {
      for (std::size_t bin_index = 0; bin_index < opened_bins.size();
           ++bin_index) {
        auto result = find_best(opened_bins[bin_index]);
        if (result.status == PlacementSearchStatus::interrupted) {
          return result.status;
        }
        if (!result.candidate.has_value()) {
          continue;
        }
        if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
          trial_recorder->snapshot_bin(bin_index);
        }
        apply_candidate(opened_bins[bin_index], *result.candidate, trace, false,
                        request.request.execution, placement_phase,
                        trial_recorder);
        return PlacementSearchStatus::found;
      }
    }
  }

  for (std::size_t bin_index = 0; bin_index < bin_instances.size();
       ++bin_index) {
    if (opened_flags[bin_index]) {
      continue;
    }
    auto working_bin = make_working_bin(bin_instances[bin_index]);
    refresh_bin_state(working_bin, request.request.execution);
    auto result = find_best(working_bin);
    if (result.status == PlacementSearchStatus::interrupted) {
      return result.status;
    }
    if (!result.candidate.has_value()) {
      if (use_strict_frontier) {
        const auto frontier_status = frontier_exhaustion_status_for_piece(
            working_bin, piece, request.request.execution);
        if (constructive_replay != nullptr) {
          constructive_replay->exhaustion_events.push_back({
              .piece_id = piece.expanded.expanded_piece_id,
              .frontier_bin_id = working_bin.state.bin_id,
              .decision =
                  frontier_status == FrontierExhaustionStatus::exhausted
                      ? ConstructiveFrontierExhaustionDecision::exhausted
                      : ConstructiveFrontierExhaustionDecision::fit_may_exist,
          });
        }
        if (frontier_status != FrontierExhaustionStatus::exhausted) {
          return PlacementSearchStatus::no_candidate;
        }
      }
      overflow_template_index = bin_index;
      continue;
    }

    apply_candidate(working_bin, *result.candidate, trace, true,
                    request.request.execution, placement_phase, trial_recorder);
    opened_flags[bin_index] = true;
    if (constructive_replay != nullptr) {
      constructive_replay->frontier_changes.push_back({
          .previous_bin_id = opened_bins.empty()
                                 ? std::nullopt
                                 : std::optional<std::uint32_t>(
                                       opened_bins.back().state.bin_id),
          .next_bin_id = working_bin.state.bin_id,
          .piece_id = piece.expanded.expanded_piece_id,
          .reason = opened_bins.empty()
                        ? ConstructiveFrontierAdvanceReason::initial_bin_opened
                        : ConstructiveFrontierAdvanceReason::next_bin_opened,
      });
    }
    opened_bins.push_back(std::move(working_bin));
    if (trial_recorder != nullptr && trial_recorder->record_opened_bin) {
      trial_recorder->record_opened_bin(bin_index);
    }
    return PlacementSearchStatus::found;
  }

  if (use_strict_frontier && request.request.execution.allow_part_overflow &&
      overflow_template_index.has_value()) {
    const auto template_instance = bin_instances[*overflow_template_index];
    const auto new_bin_index = append_overflow_bin_instance(
        bin_instances, opened_flags, template_instance,
        template_instance.expanded.expanded_bin_id);
    auto working_bin = make_working_bin(bin_instances[new_bin_index]);
    refresh_bin_state(working_bin, request.request.execution);
    auto result = find_best(working_bin);
    if (result.status == PlacementSearchStatus::interrupted) {
      return result.status;
    }
    if (result.candidate.has_value()) {
      apply_candidate(working_bin, *result.candidate, trace, true,
                      request.request.execution, placement_phase,
                      trial_recorder);
      opened_flags[new_bin_index] = true;
      if (constructive_replay != nullptr) {
        constructive_replay->overflow_events.push_back({
            .template_bin_id = template_instance.expanded.expanded_bin_id,
            .overflow_bin_id = working_bin.state.bin_id,
            .source_request_bin_id =
                template_instance.expanded.identity.source_request_bin_id != 0U
                    ? template_instance.expanded.identity.source_request_bin_id
                    : template_instance.expanded.source_bin_id,
        });
        constructive_replay->frontier_changes.push_back({
            .previous_bin_id = opened_bins.empty()
                                   ? std::nullopt
                                   : std::optional<std::uint32_t>(
                                         opened_bins.back().state.bin_id),
            .next_bin_id = working_bin.state.bin_id,
            .piece_id = piece.expanded.expanded_piece_id,
            .reason = ConstructiveFrontierAdvanceReason::overflow_bin_opened,
        });
      }
      opened_bins.push_back(std::move(working_bin));
      if (trial_recorder != nullptr && trial_recorder->record_opened_bin) {
        trial_recorder->record_opened_bin(new_bin_index);
      }
      return PlacementSearchStatus::found;
    }
  }

  return PlacementSearchStatus::no_candidate;
}

// auto try_place_piece(
//     const PieceInstance &piece, const NormalizedRequest &request,
//     std::vector<BinInstance> &bin_instances,
//     std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
//     std::vector<PlacementTraceEntry> &trace,
//     const runtime::TimeBudget &time_budget, const runtime::Stopwatch
//     &stopwatch, const SolveControl &control, ProgressThrottle
//     &search_throttle, const std::size_t placed_parts, const std::size_t
//     total_parts, PlacementAttemptContext &attempt_context, cache::NfpCache
//     *nfp_cache, PackerSearchMetrics *search_metrics,
//     runtime::DeterministicRng *rng_ptr, const TrialStateRecorder
//     *trial_recorder) -> PlacementSearchStatus {
//   // Shared per-bin candidate evaluator: both the existing-bin loop and
//   // the open-new-bin loop need to call `find_best_for_bin` with the
//   // same long argument list. Capture once so the call sites stay short
//   // and any future signature change happens in one place.
//   const auto find_best = [&](WorkingBin &bin) {
//     return find_best_for_bin(bin, piece, request, time_budget, stopwatch,
//                              control, search_throttle, placed_parts,
//                              total_parts, attempt_context, nfp_cache,
//                              search_metrics, rng_ptr);
//   };

//   const bool use_strict_frontier = !piece.restricted_to_allowed_bins;
//   std::optional<std::size_t> overflow_template_index;

//   if (!opened_bins.empty()) {
//     if (use_strict_frontier) {
//       const std::size_t frontier_index = opened_bins.size() - 1U;
//       overflow_template_index = find_bin_instance_index(
//           bin_instances, opened_bins[frontier_index].state.bin_id);
//       auto result = find_best(opened_bins[frontier_index]);
//       if (result.status == PlacementSearchStatus::interrupted ||
//           result.status == PlacementSearchStatus::placement_budget_exhausted)
//           {
//         return result.status;
//       }
//       if (result.candidate.has_value()) {
//         if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
//           trial_recorder->snapshot_bin(frontier_index);
//         }
//         apply_candidate(opened_bins[frontier_index], *result.candidate,
//         trace,
//                         false, request.request.execution, trial_recorder);
//         return PlacementSearchStatus::found;
//       }

//       if (frontier_exhaustion_status_for_piece(
//               opened_bins[frontier_index], piece, request.request.execution)
//               !=
//           FrontierExhaustionStatus::exhausted) {
//         return PlacementSearchStatus::no_candidate;
//       }
//     } else {
//       for (std::size_t bin_index = 0; bin_index < opened_bins.size();
//            ++bin_index) {
//         auto result = find_best(opened_bins[bin_index]);
//         if (result.status == PlacementSearchStatus::interrupted ||
//             result.status ==
//                 PlacementSearchStatus::placement_budget_exhausted) {
//           return result.status;
//         }
//         if (!result.candidate.has_value()) {
//           continue;
//         }
//         if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
//           trial_recorder->snapshot_bin(bin_index);
//         }
//         apply_candidate(opened_bins[bin_index], *result.candidate, trace,
//         false,
//                         request.request.execution, trial_recorder);
//         return PlacementSearchStatus::found;
//       }
//     }
//   }

//   for (std::size_t bin_index = 0; bin_index < bin_instances.size();
//        ++bin_index) {
//     if (opened_flags[bin_index]) {
//       continue;
//     }
//     auto working_bin = make_working_bin(bin_instances[bin_index]);
//     refresh_bin_state(working_bin, request.request.execution);
//     auto result = find_best(working_bin);
//     if (result.status == PlacementSearchStatus::interrupted ||
//         result.status == PlacementSearchStatus::placement_budget_exhausted) {
//       return result.status;
//     }
//     if (!result.candidate.has_value()) {
//       if (use_strict_frontier &&
//           frontier_exhaustion_status_for_piece(working_bin, piece,
//                                                request.request.execution) !=
//               FrontierExhaustionStatus::exhausted) {
//         return PlacementSearchStatus::no_candidate;
//       }
//       overflow_template_index = bin_index;
//       continue;
//     }

//     apply_candidate(working_bin, *result.candidate, trace, true,
//                     request.request.execution, trial_recorder);
//     opened_flags[bin_index] = true;
//     opened_bins.push_back(std::move(working_bin));
//     if (trial_recorder != nullptr && trial_recorder->record_opened_bin) {
//       trial_recorder->record_opened_bin(bin_index);
//     }
//     return PlacementSearchStatus::found;
//   }

//   if (use_strict_frontier && request.request.execution.allow_part_overflow &&
//       overflow_template_index.has_value()) {
//     const auto new_bin_index = append_overflow_bin_instance(
//         bin_instances, opened_flags, bin_instances[*overflow_template_index],
//         bin_instances[*overflow_template_index].expanded.expanded_bin_id);
//     auto working_bin = make_working_bin(bin_instances[new_bin_index]);
//     refresh_bin_state(working_bin, request.request.execution);
//     auto result = find_best(working_bin);
//     if (result.status == PlacementSearchStatus::interrupted ||
//         result.status == PlacementSearchStatus::placement_budget_exhausted) {
//       return result.status;
//     }
//     if (result.candidate.has_value()) {
//       apply_candidate(working_bin, *result.candidate, trace, true,
//                       request.request.execution, trial_recorder);
//       opened_flags[new_bin_index] = true;
//       opened_bins.push_back(std::move(working_bin));
//       if (trial_recorder != nullptr && trial_recorder->record_opened_bin) {
//         trial_recorder->record_opened_bin(new_bin_index);
//       }
//       return PlacementSearchStatus::found;
//     }
//   }

//   return PlacementSearchStatus::no_candidate;
// }

auto try_backtrack_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    std::vector<BinInstance> &bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace,
    const std::unordered_map<std::uint32_t, const PieceInstance *> &piece_by_id,
    const SolveControl &control, ProgressThrottle &search_throttle,
    const std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, PackerSearchMetrics *search_metrics,
    runtime::DeterministicRng *rng_ptr,
    const std::uint32_t max_backtrack_pieces) -> PlacementSearchStatus {
  const auto recent_piece_ids =
      collect_recent_piece_ids(trace, max_backtrack_pieces);
  for (std::size_t removal_count = 1; removal_count <= recent_piece_ids.size();
       ++removal_count) {
    BinTrialGuard trial_guard(bin_instances, opened_bins, opened_flags, trace);
    const TrialStateRecorder trial_recorder{
        .snapshot_bin =
            [&](const std::size_t bin_index) {
              trial_guard.snapshot_bin(bin_index);
            },
        .record_opened_bin =
            [&](const std::size_t flag_index) {
              trial_guard.record_opened_bin(flag_index);
            },
        .record_removed_trace_entry =
            [&](const std::size_t index, const PlacementTraceEntry &entry) {
              trial_guard.record_removed_trace_entry(index, entry);
            },
        .record_appended_trace_entry =
            [&]() { trial_guard.record_appended_trace_entry(); },
    };

    std::vector<std::uint32_t> removed_ids(
        recent_piece_ids.begin(),
        recent_piece_ids.begin() + static_cast<std::ptrdiff_t>(removal_count));
    for (const auto piece_id : removed_ids) {
      remove_piece_from_bins(opened_bins, piece_id, request.request.execution,
                             &trial_recorder);
      remove_trace_entries_for_piece(trace, piece_id, &trial_recorder);
    }

    std::vector<const PieceInstance *> retry_order;
    retry_order.reserve(removed_ids.size() + 1U);
    retry_order.push_back(&piece);
    for (auto it = removed_ids.rbegin(); it != removed_ids.rend(); ++it) {
      const auto piece_it = piece_by_id.find(*it);
      if (piece_it == piece_by_id.end()) {
        // The trace referenced a piece id that the caller no longer
        // recognises (e.g. a stale piece map after a request mutation).
        // Abort this trial size and let the rollback restore state;
        // surface the inconsistency so it doesn't go unnoticed.
        SHINY_WARN("backtracking: piece id {} from trace not found in "
                   "piece_by_id; skipping trial of size {}",
                   *it, removal_count);
        retry_order.clear();
        break;
      }
      retry_order.push_back(piece_it->second);
    }
    if (retry_order.empty()) {
      continue;
    }

    bool all_placed = true;
    for (const auto *retry_piece : retry_order) {
      const auto status = try_place_piece(
          *retry_piece, request, bin_instances, opened_bins, opened_flags,
          trace, control, search_throttle, trace.size(), total_parts,
          attempt_context, nfp_cache, search_metrics, rng_ptr, nullptr,
          ConstructivePlacementPhase::primary_order, &trial_recorder);
      if (status == PlacementSearchStatus::interrupted) {
        return status;
      }
      if (status != PlacementSearchStatus::found) {
        all_placed = false;
        break;
      }
    }

    if (all_placed) {
      trial_guard.commit();
      return PlacementSearchStatus::found;
    }
  }

  return PlacementSearchStatus::no_candidate;
}

} // namespace shiny::nesting::pack::detail
