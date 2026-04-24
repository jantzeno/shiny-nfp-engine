#include "packing/irregular/sequential/detail.hpp"

#include <algorithm>
#include <cstddef>
#include <format>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "logging/shiny_log.hpp"
#include "packing/irregular/workspace.hpp"

namespace shiny::nesting::pack::detail {
auto make_budget(const SolveControl &control,
                 const runtime::TimeBudget &time_budget,
                 const runtime::Stopwatch &stopwatch,
                 const std::size_t iterations_completed) -> BudgetState {
  return {
      .iteration_limit_enabled = control.iteration_limit > 0U,
      .iteration_limit = control.iteration_limit,
      .iterations_completed = iterations_completed,
      .time_limit_enabled = time_budget.enabled(),
      .time_limit_milliseconds = time_budget.limit_milliseconds(),
      .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
      .cancellation_requested = control.cancellation.stop_requested(),
  };
}

auto emit_progress(const SolveControl &control, const std::size_t sequence,
                   const std::size_t placed_parts, const std::size_t total_parts,
                   const std::span<const WorkingBin> bins,
                   const std::vector<PlacementTraceEntry> &trace,
                   const std::vector<std::uint32_t> &unplaced,
                   const BudgetState &budget, const StopReason stop_reason,
                   const std::string &phase_detail,
                   const bool lightweight) -> void {
  if (!control.on_progress) {
    return;
  }

  SHINY_DEBUG("constructive emit_progress: seq={} placed={}/{} bins={} lightweight={} "
              "elapsed_ms={}",
              sequence, placed_parts, total_parts, bins.size(), lightweight,
              budget.elapsed_milliseconds);

  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placed_parts = placed_parts,
      .total_parts = total_parts,
      .layout = lightweight ? build_lightweight_layout(bins, trace, unplaced)
                            : build_layout(bins, trace, unplaced),
      .budget = budget,
      .stop_reason = stop_reason,
      .phase = ProgressPhase::part_placement,
      .phase_detail = phase_detail,
      .utilization_percent = compute_utilization_percent(bins),
      .improved = !lightweight,
  });
}

auto emit_search_progress(const SolveControl &control, const std::size_t placed_parts,
                          const std::size_t total_parts, const BudgetState &budget,
                          const std::string &phase_detail) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placed_parts = placed_parts,
      .total_parts = total_parts,
      .layout = {},
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::part_placement,
      .phase_detail = phase_detail,
      .utilization_percent = 0.0,
      .improved = false,
  });
}

auto interrupted(const SolveControl &control,
                 const runtime::TimeBudget &time_budget,
                 const runtime::Stopwatch &stopwatch) -> bool {
  return control.cancellation.stop_requested() || time_budget.expired(stopwatch);
}

auto solve_irregular_constructive(const NormalizedRequest &request,
                                  const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  const auto piece_instances =
      order_piece_instances(build_piece_instances(request), request.request.execution);
  const auto bin_instances = build_bin_instances(request);
  if (piece_instances.size() != request.expanded_pieces.size() ||
      bin_instances.size() != request.expanded_bins.size()) {
    return util::Status::invalid_input;
  }

  std::unordered_map<std::uint32_t, const PieceInstance *> piece_by_id;
  piece_by_id.reserve(piece_instances.size());
  for (const auto &piece : piece_instances) {
    piece_by_id.emplace(piece.expanded.expanded_piece_id, &piece);
  }

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
  std::size_t placements_completed = 0;
  std::size_t processed_pieces = 0;
  std::size_t sequence = 0;
  StopReason stop_reason = StopReason::completed;

  for (std::size_t piece_index = 0; piece_index < piece_instances.size(); ++piece_index) {
    if (control.cancellation.stop_requested()) {
      stop_reason = StopReason::cancelled;
      break;
    }
    if (time_budget.expired(stopwatch)) {
      stop_reason = StopReason::time_limit_reached;
      break;
    }
    if (control.iteration_limit > 0U && processed_pieces >= control.iteration_limit) {
      stop_reason = StopReason::iteration_limit_reached;
      break;
    }

    const auto &piece = piece_instances[piece_index];
    ++processed_pieces;

    const auto remaining_pieces = piece_instances.size() - piece_index;
    std::uint64_t per_piece_budget_ms = kDefaultPerPieceBudgetMs;
    if (time_budget.enabled() && remaining_pieces > 0U) {
      const auto elapsed = stopwatch.elapsed_milliseconds();
      const auto limit = time_budget.limit_milliseconds();
      if (elapsed < limit) {
        const auto computed = (limit - elapsed) / remaining_pieces;
        per_piece_budget_ms = std::max<std::uint64_t>(
            1U, std::min<std::uint64_t>(per_piece_budget_ms, computed));
      }
    }

    bool placed = try_place_piece(
        piece, request, bin_instances, opened_bins, opened_flags, trace, time_budget,
        stopwatch, control, search_throttle, placements_completed,
        piece_instances.size(), per_piece_budget_ms, &workspace->nfp_cache, rng_ptr);

    if (interrupted(control, time_budget, stopwatch)) {
      stop_reason = control.cancellation.stop_requested()
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    if (!placed && request.request.execution.irregular.enable_backtracking) {
      placed = try_backtrack_place_piece(
          piece, request, bin_instances, opened_bins, opened_flags, trace,
          piece_by_id, time_budget, stopwatch, control, search_throttle,
          piece_instances.size(), per_piece_budget_ms, &workspace->nfp_cache, rng_ptr,
          request.request.execution.irregular.max_backtrack_pieces);
    }

    if (interrupted(control, time_budget, stopwatch)) {
      stop_reason = control.cancellation.stop_requested()
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    if (placed) {
      placements_completed = trace.size();
      ++sequence;
      if (throttle.should_emit()) {
        emit_progress(control, sequence, placements_completed, piece_instances.size(),
                      opened_bins, trace, unplaced_piece_ids,
                      make_budget(control, time_budget, stopwatch, processed_pieces),
                      StopReason::none,
                      std::format("Placing part {}/{}", placements_completed,
                                  piece_instances.size()),
                      true);
      }
      continue;
    }

    unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
  }

  if (stop_reason != StopReason::completed) {
    for (std::size_t piece_index = processed_pieces; piece_index < piece_instances.size();
         ++piece_index) {
      unplaced_piece_ids.push_back(piece_instances[piece_index].expanded.expanded_piece_id);
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
              .inside_hole = original.inside_hole,
              .hole_index = original.hole_index,
              .score = original.score,
          };

          bin.state.placements.erase(
              bin.state.placements.begin() +
              static_cast<std::ptrdiff_t>(placement_index));
          remove_trace_entries_for_piece(trace, original.placement.piece_id);
          rebuild_working_bin(bin, request.request.execution);

          const auto candidate = find_best_for_bin(
              bin, *piece_it->second, request, time_budget, stopwatch, control,
              search_throttle, placements_completed, piece_instances.size(), 0U,
              &workspace->nfp_cache, nullptr);

          if (candidate.has_value() &&
              better_candidate(bin, request.request.execution.placement_policy,
                               *candidate, current_candidate)) {
            apply_candidate(bin, *candidate, trace, false, request.request.execution);
            moved_any = true;
            continue;
          }

          apply_candidate(bin, current_candidate, trace, false,
                          request.request.execution);
          ++placement_index;
        }
      }

      if (!moved_any) {
        break;
      }
    }

    std::vector<std::uint32_t> remaining_unplaced;
    remaining_unplaced.reserve(unplaced_piece_ids.size());
    for (const auto piece_id : unplaced_piece_ids) {
      const auto piece_it = piece_by_id.find(piece_id);
      if (piece_it == piece_by_id.end()) {
        remaining_unplaced.push_back(piece_id);
        continue;
      }

      std::optional<CandidatePlacement> best_existing;
      std::size_t best_existing_index = 0;
      for (std::size_t bin_index = 0; bin_index < opened_bins.size(); ++bin_index) {
        const auto candidate = find_best_for_bin(
            opened_bins[bin_index], *piece_it->second, request, time_budget,
            stopwatch, control, search_throttle, placements_completed,
            piece_instances.size(), 0U, &workspace->nfp_cache, nullptr);
        if (!candidate.has_value()) {
          continue;
        }
        if (!best_existing.has_value() ||
            better_candidate(opened_bins[best_existing_index],
                             request.request.execution.placement_policy, *candidate,
                             *best_existing)) {
          best_existing = candidate;
          best_existing_index = bin_index;
        }
      }

      if (best_existing.has_value()) {
        apply_candidate(opened_bins[best_existing_index], *best_existing, trace, false,
                        request.request.execution);
        ++placements_completed;
        continue;
      }

      bool placed = false;
      for (std::size_t bin_index = 0; bin_index < bin_instances.size(); ++bin_index) {
        if (opened_flags[bin_index]) {
          continue;
        }
        auto working_bin = make_working_bin(bin_instances[bin_index]);
        refresh_bin_state(working_bin, request.request.execution);
        const auto candidate = find_best_for_bin(
            working_bin, *piece_it->second, request, time_budget, stopwatch, control,
            search_throttle, placements_completed, piece_instances.size(), 0U,
            &workspace->nfp_cache, nullptr);
        if (!candidate.has_value()) {
          continue;
        }

        apply_candidate(working_bin, *candidate, trace, true, request.request.execution);
        opened_flags[bin_index] = true;
        opened_bins.push_back(std::move(working_bin));
        ++placements_completed;
        placed = true;
        break;
      }

      if (!placed) {
        remaining_unplaced.push_back(piece_id);
      }
    }
    unplaced_piece_ids = std::move(remaining_unplaced);
  }

  NestingResult result{
      .strategy = StrategyKind::sequential_backtrack,
      .layout = build_layout(opened_bins, trace, unplaced_piece_ids),
      .total_parts = piece_instances.size(),
      .budget = make_budget(control, time_budget, stopwatch, processed_pieces),
      .stop_reason = stop_reason,
  };

  ++sequence;
  emit_progress(control, sequence, placements_completed, piece_instances.size(),
                opened_bins, trace, unplaced_piece_ids, result.budget, stop_reason,
                std::format("Placed {}/{}", placements_completed,
                            piece_instances.size()),
                false);
  return result;
}

} // namespace shiny::nesting::pack::detail
