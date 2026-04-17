#pragma once

#include "packing/decoder.hpp"

namespace shiny::nfp::pack {

/**
 * @brief Deterministic AABB-based constructive packing engine.
 *
 * Places pieces by rotated axis-aligned bounding boxes using a simple shelf
 * heuristic. This avoids NFP generation while keeping the same request and
 * result contract as `ConstructiveDecoder`.
 */
class BoundingBoxPacker {
public:
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
                  const InterruptionProbe &interruption_requested = {})
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

} // namespace shiny::nfp::pack
