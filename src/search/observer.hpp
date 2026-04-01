#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "search/evaluation.hpp"

namespace shiny::nfp::search {

/**
 * @brief Identifies one stable family of emitted search events.
 */
enum class SearchEventKind : std::uint8_t {
  run_started = 0,
  step_progress = 1,
  improvement_found = 2,
  run_completed = 3,
  timeout_reached = 4,
  cancellation_acknowledged = 5,
};

/**
 * @brief Returns the canonical serialization label for one search event kind.
 *
 * Short paragraph describing the stable snake_case vocabulary used by
 * fixtures, examples, tools, and serialized observer artifacts.
 *
 * @param kind Stable search event family identifier.
 * @return Canonical snake_case label for `kind`.
 *
 * @pre `kind` must be one of the supported enumerators.
 * @post Returns the same label for the same event family on every call.
 * @par Determinism
 * - Deterministic for a fixed `kind`.
 */
[[nodiscard]] constexpr auto to_string(SearchEventKind kind)
    -> std::string_view {
  switch (kind) {
  case SearchEventKind::run_started:
    return "run_started";
  case SearchEventKind::step_progress:
    return "step_progress";
  case SearchEventKind::improvement_found:
    return "improvement_found";
  case SearchEventKind::run_completed:
    return "run_completed";
  case SearchEventKind::timeout_reached:
    return "timeout_reached";
  case SearchEventKind::cancellation_acknowledged:
    return "cancellation_acknowledged";
  }

  return "unknown";
}

/**
 * @brief Parses one canonical search event label.
 *
 * Short paragraph describing conversion from serialized observer artifacts to
 * the stable event-family enum.
 *
 * @param value Canonical snake_case event label.
 * @return Matching event-family identifier when `value` is recognized.
 *
 * @pre `value` should come from trusted repository artifacts or validated
 *   input.
 * @post Returns an empty optional when `value` is not part of the canonical
 *   event vocabulary.
 * @par Determinism
 * - Deterministic for a fixed input string.
 */
[[nodiscard]] constexpr auto parse_search_event_kind(std::string_view value)
    -> std::optional<SearchEventKind> {
  if (value == "run_started") {
    return SearchEventKind::run_started;
  }
  if (value == "step_progress") {
    return SearchEventKind::step_progress;
  }
  if (value == "improvement_found") {
    return SearchEventKind::improvement_found;
  }
  if (value == "run_completed") {
    return SearchEventKind::run_completed;
  }
  if (value == "timeout_reached") {
    return SearchEventKind::timeout_reached;
  }
  if (value == "cancellation_acknowledged") {
    return SearchEventKind::cancellation_acknowledged;
  }

  return std::nullopt;
}

/**
 * @brief Announces that one run has started.
 *
 * Short paragraph describing the immutable run-scoped metadata emitted before
 * iterative search work begins.
 *
 * @par Invariants
 * - `algorithm_kind` uses the canonical outward-facing vocabulary.
 *
 * @par Performance Notes
 * - Small value payload suitable for direct observer delivery and retention.
 */
struct SearchRunStartedEvent {
  AlgorithmKind algorithm_kind{AlgorithmKind::jostle_search};
  std::uint32_t deterministic_seed{0};
  std::uint32_t iteration_budget{0};
  std::size_t piece_count{0};
  std::uint64_t timestamp_unix_ms{0};
  std::uint64_t elapsed_ms{0};
};

/**
 * @brief Carries one live progress snapshot for an active run.
 *
 * Short paragraph describing the per-step payload used by observers and final
 * retained progress history.
 *
 * @par Invariants
 * - `progress` matches the same step stored in the final `SearchResult`.
 *
 * @par Performance Notes
 * - Reuses `SearchProgressEntry` to keep live and retained progress aligned.
 */
struct SearchStepProgressEvent {
  SearchProgressEntry progress{};
};

/**
 * @brief Announces that one step improved the best-known evaluation.
 *
 * Short paragraph describing the improvement snapshot observers can use for
 * milestone notifications or replay checks.
 *
 * @par Invariants
 * - `progress.improved` is `true`.
 *
 * @par Performance Notes
 * - Reuses `SearchProgressEntry` to avoid duplicate metric definitions.
 */
struct SearchImprovementFoundEvent {
  SearchProgressEntry progress{};
};

/**
 * @brief Summary metrics for one terminated run.
 *
 * Short paragraph describing the best-known objective state when a run ends,
 * times out, or acknowledges cancellation.
 *
 * @par Invariants
 * - `algorithm_kind` uses the canonical outward-facing vocabulary.
 * - `best_*` fields describe the final best-known evaluation.
 *
 * @par Performance Notes
 * - Avoids storing full decoder layouts in retained event history.
 */
struct SearchRunSummary {
  AlgorithmKind algorithm_kind{AlgorithmKind::jostle_search};
  std::uint32_t iterations_completed{0};
  std::uint32_t iteration_budget{0};
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

/**
 * @brief Announces successful completion of one run.
 *
 * Short paragraph describing the terminal summary emitted when search ends
 * without timing out or being cancelled.
 *
 * @par Invariants
 * - `summary` describes the final best-known evaluation.
 *
 * @par Performance Notes
 * - Small summary payload intended for logging, tooling, and replay tests.
 */
struct SearchRunCompletedEvent {
  SearchRunSummary summary{};
};

/**
 * @brief Announces that one run reached its time budget.
 *
 * Short paragraph describing the best-known state at the safe interruption
 * point where the time budget was observed.
 *
 * @par Invariants
 * - `summary` describes the final best-known evaluation at timeout.
 *
 * @par Performance Notes
 * - Uses the same compact summary shape as the completion payload.
 */
struct SearchTimeoutReachedEvent {
  SearchRunSummary summary{};
};

/**
 * @brief Announces that one run acknowledged cancellation.
 *
 * Short paragraph describing the best-known state at the safe interruption
 * point where cancellation was observed.
 *
 * @par Invariants
 * - `summary` describes the final best-known evaluation at cancellation.
 *
 * @par Performance Notes
 * - Uses the same compact summary shape as the completion payload.
 */
struct SearchCancellationAcknowledgedEvent {
  SearchRunSummary summary{};
};

using SearchEvent =
    std::variant<SearchRunStartedEvent, SearchStepProgressEvent,
                 SearchImprovementFoundEvent, SearchRunCompletedEvent,
                 SearchTimeoutReachedEvent,
                 SearchCancellationAcknowledgedEvent>;

/**
 * @brief Reports which stable event family one payload belongs to.
 *
 * Short paragraph describing the variant-to-enum bridge used by tests and
 * artifact emitters.
 *
 * @param event Variant payload emitted by search execution.
 * @return Stable event family identifier for `event`.
 *
 * @pre `event` must hold one of the supported payload alternatives.
 * @post Returns the same event kind for the same payload alternative.
 * @par Determinism
 * - Deterministic for a fixed payload alternative.
 */
[[nodiscard]] inline auto search_event_kind(const SearchEvent &event)
    -> SearchEventKind {
  return std::visit(
      [](const auto &payload) -> SearchEventKind {
        using Payload = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<Payload, SearchRunStartedEvent>) {
          return SearchEventKind::run_started;
        }
        if constexpr (std::is_same_v<Payload, SearchStepProgressEvent>) {
          return SearchEventKind::step_progress;
        }
        if constexpr (std::is_same_v<Payload, SearchImprovementFoundEvent>) {
          return SearchEventKind::improvement_found;
        }
        if constexpr (std::is_same_v<Payload, SearchRunCompletedEvent>) {
          return SearchEventKind::run_completed;
        }
        if constexpr (std::is_same_v<Payload, SearchTimeoutReachedEvent>) {
          return SearchEventKind::timeout_reached;
        }
        return SearchEventKind::cancellation_acknowledged;
      },
      event);
}

/**
 * @brief Reports which algorithm family emitted one event payload.
 *
 * Short paragraph describing extraction of the canonical algorithm vocabulary
 * from any event alternative.
 *
 * @param event Variant payload emitted by search execution.
 * @return Canonical algorithm family for `event`.
 *
 * @pre `event` must hold one of the supported payload alternatives.
 * @post Returns the same algorithm kind for the same payload contents.
 * @par Determinism
 * - Deterministic for a fixed payload value.
 */
[[nodiscard]] inline auto search_event_algorithm_kind(const SearchEvent &event)
    -> AlgorithmKind {
  return std::visit(
      [](const auto &payload) -> AlgorithmKind {
        using Payload = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<Payload, SearchRunStartedEvent>) {
          return payload.algorithm_kind;
        } else if constexpr (std::is_same_v<Payload, SearchStepProgressEvent> ||
                             std::is_same_v<Payload,
                                            SearchImprovementFoundEvent>) {
          return payload.progress.algorithm_kind;
        } else {
          return payload.summary.algorithm_kind;
        }
      },
      event);
}

} // namespace shiny::nfp::search