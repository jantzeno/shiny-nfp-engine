#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "algorithm_kind.hpp"
#include "cache/cache_policy.hpp"
#include "nfp/engine.hpp"
#include "packing/bin_state.hpp"
#include "packing/decoder.hpp"

namespace shiny::nfp::pack {

/**
 * @brief Configures deterministic shelf-style masonry placement.
 *
 * Prefers filling existing shelves before opening higher shelves, which makes
 * this builder a better fit than the greedy constructive decoder for
 * row-oriented rectangular or near-rectangular nests.
 *
 * @par Invariants
 * - `shelf_alignment_tolerance` must be finite and non-negative.
 *
 * @par Performance Notes
 * - Tolerance only affects shelf grouping and does not bypass legality checks.
 */
struct MasonryConfig {
  bool fill_existing_shelves_first{true};
  double shelf_alignment_tolerance{1e-6};

  /**
   * @brief Reports whether the masonry-specific settings are internally
   * consistent.
   *
   * Short paragraph describing the shelf-grouping constraints required before
   * one masonry run can start.
   *
   * @return `true` when shelf grouping can safely run with this config.
   *
   * @pre None.
   * @post Does not modify the config.
   * @par Determinism
   * - Deterministic for a fixed config state.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Snapshot of one masonry build step.
 *
 * Summarizes the best-known partial layout after one input piece has been
 * processed so callers can observe constructive progress without reconstructing
 * intermediate bin state.
 *
 * @par Invariants
 * - `algorithm_kind` always reports `AlgorithmKind::masonry_builder`.
 * - `processed_piece_count` never exceeds `piece_count`.
 *
 * @par Performance Notes
 * - Small immutable value intended for synchronous observer callbacks.
 */
struct MasonryProgressSnapshot {
  AlgorithmKind algorithm_kind{AlgorithmKind::masonry_builder};
  std::uint32_t current_piece_id{0};
  std::size_t processed_piece_count{0};
  std::size_t piece_count{0};
  std::size_t bin_count{0};
  std::size_t placed_piece_count{0};
  std::size_t unplaced_piece_count{0};
  double total_utilization{0.0};
};

/**
 * @brief Registers one run-scoped masonry progress callback.
 *
 * @par Invariants
 * - `on_progress` may be empty when live callbacks are not needed.
 *
 * @par Performance Notes
 * - Delivery happens synchronously on the masonry execution thread.
 */
struct MasonryObserver {
  std::function<void(const MasonryProgressSnapshot &progress)> on_progress{};

  /**
   * @brief Reports whether a masonry observer callback is installed.
   *
   * Short paragraph describing the presence check used before live progress
   * delivery.
   *
   * @return `true` when `on_progress` contains a callable target.
   *
   * @pre None.
   * @post Does not modify the observer.
   * @par Determinism
   * - Deterministic for a fixed observer state.
   */
  [[nodiscard]] auto installed() const -> bool {
    return static_cast<bool>(on_progress);
  }
};

/**
 * @brief Full input bundle for one masonry layout run.
 *
 * Wraps the ordinary constructive decode request with masonry-specific shelf
 * behavior without duplicating solve-run geometry and manufacturing inputs.
 *
 * @par Invariants
 * - `decoder_request.config` and `masonry` must both be valid before use.
 *
 * @par Performance Notes
 * - Reuses the decoder request surface so future search layers can share one
 *   constructive request contract.
 */
struct MasonryRequest {
  DecoderRequest decoder_request{};
  MasonryConfig masonry{};
  MasonryObserver observer{};
};

/**
 * @brief One masonry-specific placement step.
 *
 * Records shelf assignment together with the ordinary placement pose so callers
 * can inspect when the builder filled an existing shelf versus starting a new
 * one.
 *
 * @par Invariants
 * - `shelf_index` is stable within one bin for one finished result.
 *
 * @par Performance Notes
 * - Stored alongside the generic layout trace instead of replacing it.
 */
struct MasonryTraceEntry {
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  std::size_t shelf_index{0};
  geom::RotationIndex rotation_index{};
  geom::ResolvedRotation resolved_rotation{};
  geom::Point2 translation{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::nfp_boundary};
  bool opened_new_bin{false};
  bool started_new_shelf{false};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
  double score{0.0};
};

/**
 * @brief Final result of one masonry layout run.
 *
 * Carries the same geometric layout views as the decoder plus masonry-specific
 * trace metadata and canonical algorithm identity.
 *
 * @par Invariants
 * - `algorithm` always reports `AlgorithmKind::masonry_builder`.
 *
 * @par Performance Notes
 * - Exposes both bin-state and export-oriented layout views to avoid
 *   recomputing summaries.
 */
struct MasonryResult {
  AlgorithmKind algorithm{AlgorithmKind::masonry_builder};
  std::vector<BinState> bins{};
  Layout layout{};
  std::vector<MasonryTraceEntry> trace{};
  std::vector<MasonryProgressSnapshot> progress{};
};

/**
 * @brief Deterministic shelf-style constructive packing engine.
 *
 * Uses the existing NFP and legality stack to build row-oriented layouts while
 * preferring to fill lower shelves before opening new vertical bands.
 *
 * @par Invariants
 * - Cache contents are tied to the current geometry and algorithm revisions.
 *
 * @par Performance Notes
 * - Owns convex, nonconvex, and decomposition caches through `NfpEngine`.
 */
class MasonryBuilder {
public:
  explicit MasonryBuilder(cache::CachePolicyConfig cache_policy = {});

  /**
   * @brief Builds one deterministic masonry layout.
   *
   * Uses shelf-style candidate selection above the existing placement legality
   * machinery so retained rotations, hole-aware placement, grain direction, and
   * exclusion zones are enforced by the same geometric rules as the decoder.
   *
   * @param request Constructive request plus masonry shelf settings.
   * @return Final masonry result with layout, bin summaries, and shelf trace.
   *
   * @pre `request.decoder_request.config` and `request.masonry` must be valid.
   * @post Returned placements satisfy the active legality checks for the run.
   * @par Determinism
   * - Deterministic for fixed request geometry, ordering, and config.
   */
  [[nodiscard]] auto build(const MasonryRequest &request) -> MasonryResult;

  /**
   * @brief Clears all geometry caches owned by the builder.
   *
   * Short paragraph describing the explicit cache reset boundary used by tests,
   * tools, and repeated runs.
   *
   * @pre None.
   * @post Owned NFP and decomposition caches are empty.
   * @par Determinism
   * - Deterministic for a fixed cache state.
   */
  auto clear_caches() -> void;

  /**
   * @brief Returns the current convex NFP cache size.
   *
   * Short paragraph describing the diagnostic surface exposed for tooling and
   * verification.
   *
   * @return Number of cached convex NFP entries currently owned by the builder.
   *
   * @pre None.
   * @post Does not modify the builder.
   * @par Determinism
   * - Deterministic for a fixed cache state.
   */
  [[nodiscard]] auto convex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the current nonconvex NFP cache size.
   *
   * Short paragraph describing the diagnostic surface exposed for tooling and
   * verification.
   *
   * @return Number of cached nonconvex NFP entries currently owned by the
   * builder.
   *
   * @pre None.
   * @post Does not modify the builder.
   * @par Determinism
   * - Deterministic for a fixed cache state.
   */
  [[nodiscard]] auto nonconvex_cache_size() const -> std::size_t;

  /**
   * @brief Returns the current decomposition cache size.
   *
   * Short paragraph describing the diagnostic surface exposed for tooling and
   * verification.
   *
   * @return Number of cached decomposition entries currently owned by the
   * builder.
   *
   * @pre None.
   * @post Does not modify the builder.
   * @par Determinism
   * - Deterministic for a fixed cache state.
   */
  [[nodiscard]] auto decomposition_cache_size() const -> std::size_t;

private:
  NfpEngine nfp_engine_{};
  std::uint64_t build_generation_{0};
};

} // namespace shiny::nfp::pack