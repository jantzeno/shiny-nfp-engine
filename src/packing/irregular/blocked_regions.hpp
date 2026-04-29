#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "packing/irregular/candidate_generation.hpp"
#include "util/status.hpp"

namespace shiny::nesting::pack {

struct BlockedRegions {
  std::vector<geom::PolygonWithHoles> polygons{};
  cache::NfpCacheAccuracy accuracy{cache::NfpCacheAccuracy::exact};
};

[[nodiscard]] auto
build_blocked_regions(std::span<const CandidateGenerationObstacle> obstacles,
                      const geom::PolygonWithHoles &moving_piece,
                      std::uint64_t moving_piece_revision,
                      geom::ResolvedRotation moving_rotation,
                      cache::NfpCache *cache_ptr,
                      CandidateGenerationDiagnostics *diagnostics)
    -> util::StatusOr<BlockedRegions>;

} // namespace shiny::nesting::pack
