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
    CandidateStrategy strategy, cache::NfpCache *cache = nullptr)
    -> util::StatusOr<std::vector<GeneratedCandidatePoint>>;

} // namespace shiny::nesting::pack
