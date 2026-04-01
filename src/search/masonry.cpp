#include "search/masonry.hpp"

#include <chrono>
#include <cstdint>
#include <utility>

#include "search/detail/run_event_sink.hpp"

namespace shiny::nfp::search {
namespace {

constexpr auto kAlgorithmKind = AlgorithmKind::masonry_builder;

using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

struct RunTiming {
  runtime::ExecutionControlConfig control{};
  SteadyClock::time_point started_steady{SteadyClock::now()};

  [[nodiscard]] auto elapsed_ms() const -> std::uint64_t {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - started_steady)
            .count());
  }

  [[nodiscard]] auto event_elapsed_ms() const -> std::uint64_t {
    return control.capture_timestamps ? elapsed_ms() : 0U;
  }

  [[nodiscard]] auto timestamp_unix_ms() const -> std::uint64_t {
    if (!control.capture_timestamps) {
      return 0U;
    }

    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            SystemClock::now().time_since_epoch())
            .count());
  }
};

[[nodiscard]] auto
extract_piece_order(const std::vector<pack::PieceInput> &pieces)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> piece_order;
  piece_order.reserve(pieces.size());
  for (const auto &piece : pieces) {
    piece_order.push_back(piece.piece_id);
  }
  return piece_order;
}

[[nodiscard]] auto
improvement_found(const SearchProgressEntry *previous,
                  const pack::MasonryProgressSnapshot &snapshot) -> bool {
  if (previous == nullptr) {
    return snapshot.placed_piece_count > 0U;
  }
  return snapshot.placed_piece_count > previous->best_placed_piece_count;
}

[[nodiscard]] auto
to_progress_entry(const MasonryRunRequest &request,
                  const pack::MasonryProgressSnapshot &snapshot, bool improved,
                  const RunTiming &timing) -> SearchProgressEntry {
  return {
      .algorithm_kind = kAlgorithmKind,
      .iteration = static_cast<std::uint32_t>(snapshot.processed_piece_count),
      .iteration_budget = static_cast<std::uint32_t>(snapshot.piece_count),
      .move_kind = SearchMoveKind::none,
      .improved = improved,
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
      .evaluated_layout_count = snapshot.processed_piece_count,
      .reevaluation_cache_hits = 0,
      .best_bin_count = snapshot.bin_count,
      .best_placed_piece_count = snapshot.placed_piece_count,
      .best_unplaced_piece_count = snapshot.unplaced_piece_count,
      .best_total_utilization = snapshot.total_utilization,
      .best_piece_order =
          extract_piece_order(request.masonry_request.decoder_request.pieces),
  };
}

[[nodiscard]] auto make_run_summary(const MasonryRunRequest &request,
                                    const MasonryRunResult &result,
                                    const RunTiming &timing)
    -> SearchRunSummary {
  const auto piece_order =
      extract_piece_order(request.masonry_request.decoder_request.pieces);
  const auto piece_count =
      request.masonry_request.decoder_request.pieces.size();
  const auto iterations_completed =
      static_cast<std::uint32_t>(result.progress.size());

  std::size_t best_bin_count = result.masonry.bins.size();
  std::size_t best_placed_piece_count =
      result.masonry.layout.placement_trace.size();
  std::size_t best_unplaced_piece_count =
      result.masonry.layout.unplaced_piece_ids.size();
  double best_total_utilization = 0.0;
  for (const auto &bin : result.masonry.bins) {
    best_total_utilization += bin.utilization.utilization;
  }

  return {
      .algorithm_kind = kAlgorithmKind,
      .iterations_completed = iterations_completed,
      .iteration_budget = static_cast<std::uint32_t>(piece_count),
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
      .evaluated_layout_count = result.evaluated_layout_count,
      .reevaluation_cache_hits = 0,
      .best_bin_count = best_bin_count,
      .best_placed_piece_count = best_placed_piece_count,
      .best_unplaced_piece_count = best_unplaced_piece_count,
      .best_total_utilization = best_total_utilization,
      .best_piece_order = std::move(piece_order),
  };
}

} // namespace

auto MasonryRunner::run(const MasonryRunRequest &request) -> MasonryRunResult {
  MasonryRunResult result{};
  result.algorithm = kAlgorithmKind;

  if (!request.masonry_request.masonry.is_valid() ||
      !request.masonry_request.decoder_request.config.is_valid() ||
      request.masonry_request.decoder_request.bin.polygon.outer.empty() ||
      !request.execution.control.is_valid()) {
    return result;
  }

  result.status = SearchRunStatus::completed;
  const RunTiming timing{.control = request.execution.control};
  detail::RunEventSink event_sink{request.execution, result.events};

  event_sink.emit(SearchRunStartedEvent{
      .algorithm_kind = kAlgorithmKind,
      .deterministic_seed = 0,
      .iteration_budget = static_cast<std::uint32_t>(
          request.masonry_request.decoder_request.pieces.size()),
      .piece_count = request.masonry_request.decoder_request.pieces.size(),
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
  });

  auto adapted_request = request.masonry_request;
  adapted_request.observer.on_progress =
      [&](const pack::MasonryProgressSnapshot &snapshot) {
        const SearchProgressEntry *previous =
            result.progress.empty() ? nullptr : &result.progress.back();
        const bool improved = improvement_found(previous, snapshot);
        result.progress.push_back(
            to_progress_entry(request, snapshot, improved, timing));
        result.evaluated_layout_count = snapshot.processed_piece_count;

        const auto &progress = result.progress.back();
        event_sink.emit(SearchStepProgressEvent{.progress = progress});
        if (improved) {
          event_sink.emit(SearchImprovementFoundEvent{.progress = progress});
        }
      };

  result.masonry = builder_.build(adapted_request);
  result.evaluated_layout_count = result.progress.size();

  event_sink.emit(SearchRunCompletedEvent{
      .summary = make_run_summary(request, result, timing),
  });
  return result;
}

auto MasonryRunner::clear_caches() -> void { builder_.clear_caches(); }

} // namespace shiny::nfp::search