#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "algorithm_kind.hpp"
#include "packing/decoder.hpp"

namespace shiny::nfp::search {

/**
 * @brief Identifies which neighborhood move produced one search step.
 */
enum class SearchMoveKind : std::uint8_t {
  none = 0,
  jostle_oscillation = 1,
  one_piece_insert = 2,
  genetic_generation = 3,
};

/**
 * @brief Summary of one evaluated piece order.
 *
 * Short paragraph describing the decoder output and objective metrics cached
 * for one candidate ordering.
 *
 * @par Invariants
 * - `decode` and the summary metrics describe the same evaluated order.
 *
 * @par Performance Notes
 * - Cached directly in the search reevaluation store.
 */
struct LayoutEvaluation {
  std::vector<std::uint32_t> piece_order{};
  pack::DecoderResult decode{};
  std::size_t bin_count{0};
  std::size_t placed_piece_count{0};
  std::size_t unplaced_piece_count{0};
  double total_utilization{0.0};
  double average_utilization{0.0};
  double last_bin_utilization{0.0};
};

/**
 * @brief Snapshot of one iteration or generation in the search trace.
 *
 * Short paragraph describing the best-known search state after one emitted
 * progress step.
 *
 * @par Invariants
 * - `best_*` fields describe the best evaluation known after this iteration.
 * - `algorithm_kind` uses the canonical outward-facing vocabulary.
 *
 * @par Performance Notes
 * - Shared by final result traces and live progress observer payloads.
 */
struct SearchProgressEntry {
  AlgorithmKind algorithm_kind{AlgorithmKind::jostle_search};
  std::uint32_t iteration{0};
  std::uint32_t iteration_budget{0};
  SearchMoveKind move_kind{SearchMoveKind::none};
  bool improved{false};
  std::uint64_t timestamp_unix_ms{0};
  std::uint64_t elapsed_ms{0};
  std::size_t evaluated_layout_count{0};
  std::size_t reevaluation_cache_hits{0};
  std::size_t best_bin_count{0};
  std::size_t best_placed_piece_count{0};
  std::size_t best_unplaced_piece_count{0};
  double best_total_utilization{0.0};
  std::vector<std::uint32_t> best_piece_order{};
};

} // namespace shiny::nfp::search