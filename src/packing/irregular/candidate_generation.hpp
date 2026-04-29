#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "geometry/transform.hpp"
#include "geometry/types.hpp"
#include "placement/types.hpp"
#include "request.hpp"
#include "util/status.hpp"

namespace shiny::nesting::runtime {
class DeterministicRng;
}

namespace shiny::nesting::pack {

struct CandidateGenerationObstacle {
  std::uint64_t geometry_revision{0};
  geom::PolygonWithHoles polygon{};
  geom::Vector2 translation{};
  geom::ResolvedRotation rotation{};
};

struct GeneratedCandidatePoint {
  geom::Point2 translation{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::constructive_boundary};
  cache::NfpCacheAccuracy nfp_accuracy{cache::NfpCacheAccuracy::exact};
};

struct CandidateGenerationDiagnostics {
  CandidateStrategy strategy{CandidateStrategy::anchor_vertex};
  std::size_t constructive_boundary_candidates{0};
  std::size_t perfect_fit_candidates{0};
  std::size_t perfect_slide_candidates{0};
  std::size_t conservative_bbox_fallback_candidates{0};
  std::size_t exact_nfp_cache_hits{0};
  std::size_t conservative_bbox_cache_hits{0};
  std::size_t exact_nfp_computations{0};
  std::size_t conservative_bbox_fallbacks{0};

  [[nodiscard]] auto used_conservative_bbox_fallback() const -> bool {
    return conservative_bbox_cache_hits > 0 ||
           conservative_bbox_fallbacks > 0 ||
           conservative_bbox_fallback_candidates > 0;
  }
};

struct CandidateDiagnosticsTotals {
  std::size_t exact_nfp_cache_hits{0};
  std::size_t conservative_bbox_cache_hits{0};
  std::size_t exact_nfp_computations{0};
  std::size_t conservative_bbox_fallbacks{0};
  std::size_t conservative_bbox_candidate_points{0};
};

class CandidateDiagnosticsRecorder {
public:
  auto reset() -> void { totals_ = {}; }

  auto record(const CandidateGenerationDiagnostics &diagnostics) -> void {
    totals_.exact_nfp_cache_hits += diagnostics.exact_nfp_cache_hits;
    totals_.conservative_bbox_cache_hits +=
        diagnostics.conservative_bbox_cache_hits;
    totals_.exact_nfp_computations += diagnostics.exact_nfp_computations;
    totals_.conservative_bbox_fallbacks +=
        diagnostics.conservative_bbox_fallbacks;
    totals_.conservative_bbox_candidate_points +=
        diagnostics.conservative_bbox_fallback_candidates;
  }

  [[nodiscard]] auto totals() const -> const CandidateDiagnosticsTotals & {
    return totals_;
  }

private:
  CandidateDiagnosticsTotals totals_{};
};

auto limit_candidate_points(std::vector<GeneratedCandidatePoint> &points,
                            const ExecutionPolicy &execution,
                            place::PlacementStartCorner start_corner,
                            runtime::DeterministicRng *rng) -> void;

[[nodiscard]] auto generate_nfp_candidate_points(
    const geom::PolygonWithHoles &container,
    std::span<const geom::PolygonWithHoles> exclusion_regions,
    std::span<const CandidateGenerationObstacle> obstacles,
    const geom::PolygonWithHoles &moving_piece,
    std::uint64_t moving_piece_revision, geom::ResolvedRotation moving_rotation,
    CandidateStrategy strategy, cache::NfpCache *cache = nullptr,
    CandidateGenerationDiagnostics *diagnostics = nullptr)
    -> util::StatusOr<std::vector<GeneratedCandidatePoint>>;

} // namespace shiny::nesting::pack
