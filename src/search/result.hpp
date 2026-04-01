#pragma once

#include <vector>

#include "search/evaluation.hpp"
#include "search/observer.hpp"

namespace shiny::nfp::search {

/**
 * @brief Describes how one search run terminated.
 */
enum class SearchRunStatus : std::uint8_t {
  invalid_request = 0,
  completed = 1,
  timed_out = 2,
  cancelled = 3,
};

/**
 * @brief Full output of one local-search run.
 *
 * Carries the baseline decode, best improved decode, per-iteration progress,
 * retained live-event history, and aggregate reevaluation-cache statistics.
 *
 * @par Invariants
 * - `baseline` is the identity-order decode and `best` is the best evaluation
 *   observed during the run when `status` is not `invalid_request`.
 *
 * @par Performance Notes
 * - Progress snapshots and retained events make deterministic replay testing
 *   straightforward when timestamps are disabled.
 */
struct SearchResult {
  AlgorithmKind algorithm{AlgorithmKind::jostle_search};
  LayoutEvaluation baseline{};
  LayoutEvaluation best{};
  std::vector<SearchProgressEntry> progress{};
  std::vector<SearchEvent> events{};
  SearchRunStatus status{SearchRunStatus::invalid_request};
  std::uint32_t deterministic_seed{0};
  std::uint32_t iterations_completed{0};
  std::size_t evaluated_layout_count{0};
  std::size_t reevaluation_cache_hits{0};

  /**
   * @brief Reports whether the best evaluation beats the baseline evaluation.
   *
   * @return `true` when search reduced unplaced pieces or bins, or improved
   *   utilization under the objective ordering.
   */
  [[nodiscard]] auto improved() const -> bool {
    if (status == SearchRunStatus::invalid_request) {
      return false;
    }

    return best.unplaced_piece_count < baseline.unplaced_piece_count ||
           (best.unplaced_piece_count == baseline.unplaced_piece_count &&
            (best.bin_count < baseline.bin_count ||
             (best.bin_count == baseline.bin_count &&
              best.total_utilization > baseline.total_utilization)));
  }
};

} // namespace shiny::nfp::search