#pragma once

#include <cstddef>
#include <vector>

#include "cache/cache_policy.hpp"
#include "packing/masonry.hpp"
#include "search/execution.hpp"
#include "search/observer.hpp"
#include "search/result.hpp"

namespace shiny::nfp::search {

/**
 * @brief Full input bundle for one masonry execution run.
 *
 * Wraps the packing-layer masonry request with the shared run-scoped observer
 * and execution-control surfaces used by outward-facing solver paths.
 *
 * @par Invariants
 * - `masonry_request.decoder_request.config` and `execution.control` must both
 *   be valid before execution.
 *
 * @par Performance Notes
 * - Keeps masonry-specific geometry surfaces in the packing layer while search
 *   owns canonical event emission.
 */
struct MasonryRunRequest {
  pack::MasonryRequest masonry_request{};
  SearchExecutionConfig execution{};
};

/**
 * @brief Full output of one masonry execution run.
 *
 * Carries the final masonry layout together with retained progress snapshots,
 * retained canonical observer events, and terminal run status.
 *
 * @par Invariants
 * - `algorithm` always reports `AlgorithmKind::masonry_builder`.
 *
 * @par Performance Notes
 * - Retained event history uses the same `SearchEvent` vocabulary as local
 *   search so examples, tests, and tools stay aligned.
 */
struct MasonryRunResult {
  AlgorithmKind algorithm{AlgorithmKind::masonry_builder};
  pack::MasonryResult masonry{};
  std::vector<SearchProgressEntry> progress{};
  std::vector<SearchEvent> events{};
  SearchRunStatus status{SearchRunStatus::invalid_request};
  std::size_t evaluated_layout_count{0};
};

/**
 * @brief Emits canonical observer events for one deterministic masonry run.
 *
 * Adapts the packing-layer masonry progress snapshots to the shared search
 * event vocabulary without changing the constructive layout result.
 *
 * @par Invariants
 * - Live observer delivery must not change geometric output for a fixed
 *   request.
 *
 * @par Performance Notes
 * - Owns one masonry builder instance so geometry caches can be reused across
 *   repeated runs.
 */
class MasonryRunner {
public:
  explicit MasonryRunner(cache::CachePolicyConfig cache_policy = {})
      : builder_(cache_policy) {}

  /**
   * @brief Executes one masonry run and emits canonical observer events.
   *
   * @param request Masonry request plus shared observer and execution-control
   *   settings.
   * @return Final masonry layout, retained progress snapshots, retained events,
   *   and terminal run status.
   */
  [[nodiscard]] auto run(const MasonryRunRequest &request) -> MasonryRunResult;

  /**
   * @brief Clears geometry caches owned by the underlying masonry builder.
   *
   * Short paragraph describing the explicit cache reset boundary used between
   * repeated masonry runs.
   *
   * @pre None.
   * @post Owned masonry-side geometry caches are empty.
   * @par Determinism
   * - Deterministic for a fixed cache state.
   */
  auto clear_caches() -> void;

private:
  pack::MasonryBuilder builder_{};
};

} // namespace shiny::nfp::search