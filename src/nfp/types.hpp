#pragma once

#include <cstdint>
#include <vector>

#include "algorithm_kind.hpp"
#include "geometry/types.hpp"

namespace shiny::nfp {

/**
 * @brief Labels how an extracted loop participates in the NFP arrangement.
 */
enum class NfpFeatureKind : std::int8_t {
  outer_loop = 0,
  hole = 1,
  perfect_fit_point = 2,
  perfect_sliding_segment = 3,
};

/**
 * @brief One extracted loop from an NFP arrangement.
 *
 * @par Invariants
 * - `vertices` is expected to encode a closed normalized ring.
 */
struct NfpLoop {
  geom::Ring vertices{};
  NfpFeatureKind kind{NfpFeatureKind::outer_loop};
};

/**
 * @brief Canonical NFP result shared by convex, nonconvex, and verifier paths.
 *
 * @par Invariants
 * - Loop and feature collections describe the same translational arrangement.
 *
 * @par Performance Notes
 * - One common shape keeps higher layers independent from the concrete NFP
 *   algorithm used.
 */
struct NfpResult {
  std::vector<NfpLoop> loops{};
  std::vector<geom::Point2> perfect_fit_points{};
  std::vector<geom::Segment2> perfect_sliding_segments{};
  AlgorithmKind algorithm{AlgorithmKind::convex_nfp};
  bool normalized{false};
};

} // namespace shiny::nfp