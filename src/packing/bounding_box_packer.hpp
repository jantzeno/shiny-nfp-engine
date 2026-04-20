#pragma once

#include "packing/decoder.hpp"

namespace shiny::nesting::pack {

/**
 * @brief Deterministic AABB-based constructive packing engine.
 *
 * Places pieces by rotated axis-aligned bounding boxes using a simple shelf
 * heuristic while preserving the shared bounding-box request and result
 * contract.
 */
class BoundingBoxPacker {
public:
  using AttemptObserver =
      std::function<void(std::size_t attempt_index, const DecoderResult &)>;

  /**
   * @brief Packs a bounded set of deterministic piece-order variants.
   *
   * Uses the request's deterministic-attempt budget to evaluate a stable set
   * of constructive orderings and returns the results in attempt order.
   *
   * @param request Bin prototype, piece order, and packing config.
   * @param interruption_requested Optional run-scoped interruption probe used
   *   to stop between or during attempts.
   * @return One decode result per realized deterministic attempt.
   */
  [[nodiscard]] auto
  decode_attempts(const DecoderRequest &request,
                  const InterruptionProbe &interruption_requested = {},
                  const AttemptObserver &on_attempt_complete = {})
      -> std::vector<DecoderResult>;

  /**
   * @brief Packs the requested piece order into one or more bins.
   *
   * @param request Bin prototype, piece order, and packing config.
   * @param interruption_requested Optional run-scoped interruption probe used
   *   to stop at safe piece boundaries.
   * @return Concrete packing result containing bin states and exported layout.
   */
  [[nodiscard]] auto
  decode(const DecoderRequest &request,
         const InterruptionProbe &interruption_requested = {}) -> DecoderResult;
};

} // namespace shiny::nesting::pack
