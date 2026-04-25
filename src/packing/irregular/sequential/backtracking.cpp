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
  BinTrialGuard(std::vector<WorkingBin> &bins, std::vector<bool> &flags,
                std::vector<PlacementTraceEntry> &trace)
      : bins_{bins}, flags_{flags}, trace_{trace},
        original_bin_count_{bins.size()} {}

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

    for (const auto flag_index : opened_flag_indices_) {
      flags_[flag_index] = false;
    }
    bins_.resize(original_bin_count_);
    for (const auto &snapshot : snapshots_) {
      bins_[snapshot.bin_index] = snapshot.state;
    }
  }

  std::vector<WorkingBin> &bins_;
  std::vector<bool> &flags_;
  std::vector<PlacementTraceEntry> &trace_;
  std::size_t original_bin_count_{0};
  std::vector<BinSnapshot> snapshots_{};
  std::vector<std::size_t> opened_flag_indices_{};
  std::vector<RemovedTraceEntry> removed_trace_entries_{};
  std::size_t appended_trace_entries_{0};
  bool committed_{false};
};

} // namespace

auto try_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    const std::span<const BinInstance> bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace,
    const runtime::TimeBudget &time_budget, const runtime::Stopwatch &stopwatch,
    const SolveControl &control, ProgressThrottle &search_throttle,
    const std::size_t placed_parts, const std::size_t total_parts,
    PlacementAttemptContext &attempt_context, cache::NfpCache *nfp_cache,
    runtime::DeterministicRng *rng_ptr,
    const TrialStateRecorder *trial_recorder) -> PlacementSearchStatus {
  // Shared per-bin candidate evaluator: both the existing-bin loop and
  // the open-new-bin loop need to call `find_best_for_bin` with the
  // same long argument list. Capture once so the call sites stay short
  // and any future signature change happens in one place.
  const auto find_best = [&](WorkingBin &bin) {
    return find_best_for_bin(bin, piece, request, time_budget, stopwatch,
                             control, search_throttle, placed_parts,
                             total_parts, attempt_context, nfp_cache, rng_ptr);
  };

  std::optional<CandidatePlacement> best_existing;
  std::size_t best_existing_index = 0;
  for (std::size_t bin_index = 0; bin_index < opened_bins.size(); ++bin_index) {
    auto result = find_best(opened_bins[bin_index]);
    if (result.status == PlacementSearchStatus::interrupted ||
        result.status == PlacementSearchStatus::placement_budget_exhausted) {
      return result.status;
    }
    if (!result.candidate.has_value()) {
      continue;
    }
    if (!best_existing.has_value() ||
        better_candidate(opened_bins[best_existing_index],
                         request.request.execution.placement_policy,
                         *result.candidate, *best_existing)) {
      best_existing = std::move(result.candidate);
      best_existing_index = bin_index;
    }
  }

  if (best_existing.has_value()) {
    if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
      trial_recorder->snapshot_bin(best_existing_index);
    }
    apply_candidate(opened_bins[best_existing_index], *best_existing, trace,
                    false, request.request.execution, trial_recorder);
    return PlacementSearchStatus::found;
  }

  for (std::size_t bin_index = 0; bin_index < bin_instances.size();
       ++bin_index) {
    if (opened_flags[bin_index]) {
      continue;
    }
    auto working_bin = make_working_bin(bin_instances[bin_index]);
    refresh_bin_state(working_bin, request.request.execution);
    auto result = find_best(working_bin);
    if (result.status == PlacementSearchStatus::interrupted ||
        result.status == PlacementSearchStatus::placement_budget_exhausted) {
      return result.status;
    }
    if (!result.candidate.has_value()) {
      continue;
    }

    apply_candidate(working_bin, *result.candidate, trace, true,
                    request.request.execution, trial_recorder);
    opened_flags[bin_index] = true;
    opened_bins.push_back(std::move(working_bin));
    if (trial_recorder != nullptr && trial_recorder->record_opened_bin) {
      trial_recorder->record_opened_bin(bin_index);
    }
    return PlacementSearchStatus::found;
  }

  return PlacementSearchStatus::no_candidate;
}

auto try_backtrack_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    const std::span<const BinInstance> bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace,
    const std::unordered_map<std::uint32_t, const PieceInstance *> &piece_by_id,
    const runtime::TimeBudget &time_budget, const runtime::Stopwatch &stopwatch,
    const SolveControl &control, ProgressThrottle &search_throttle,
    const std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, runtime::DeterministicRng *rng_ptr,
    const std::uint32_t max_backtrack_pieces) -> PlacementSearchStatus {
  const auto recent_piece_ids =
      collect_recent_piece_ids(trace, max_backtrack_pieces);
  for (std::size_t removal_count = 1; removal_count <= recent_piece_ids.size();
       ++removal_count) {
    BinTrialGuard trial_guard(opened_bins, opened_flags, trace);
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
          trace, time_budget, stopwatch, control, search_throttle, trace.size(),
          total_parts, attempt_context, nfp_cache, rng_ptr, &trial_recorder);
      if (status == PlacementSearchStatus::interrupted ||
          status == PlacementSearchStatus::placement_budget_exhausted) {
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
