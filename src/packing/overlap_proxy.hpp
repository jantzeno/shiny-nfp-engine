#pragma once

#include <cstdint>

#include "cache/penetration_depth_cache.hpp"
#include "cache/pole_cache.hpp"
#include "geometry/types.hpp"

namespace shiny::nesting::pack {

inline constexpr double kDefaultPoleSearchEpsilon = 1e-3;

[[nodiscard]] auto compute_pole_of_inaccessibility(
    const geom::PolygonWithHoles &polygon,
    double epsilon = kDefaultPoleSearchEpsilon,
    cache::PoleCache *pole_cache = nullptr,
    cache::PenetrationDepthCache *pd_cache = nullptr) -> PoleOfInaccessibility;

[[nodiscard]] auto compute_pole_of_inaccessibility(
    const geom::PolygonWithHoles &polygon, std::uint64_t polygon_revision,
    cache::PoleCache *pole_cache = nullptr,
    cache::PenetrationDepthCache *pd_cache = nullptr,
    double epsilon = kDefaultPoleSearchEpsilon) -> PoleOfInaccessibility;

[[nodiscard]] auto
overlap_proxy_loss(const geom::PolygonWithHoles &lhs,
                   const geom::PolygonWithHoles &rhs,
                   cache::PoleCache *pole_cache = nullptr,
                   cache::PenetrationDepthCache *pd_cache = nullptr,
                   double epsilon = kDefaultPoleSearchEpsilon) -> double;

[[nodiscard]] auto overlap_proxy_loss(
    const geom::PolygonWithHoles &lhs, std::uint64_t lhs_revision,
    const geom::PolygonWithHoles &rhs, std::uint64_t rhs_revision,
    cache::PoleCache *pole_cache = nullptr,
    cache::PenetrationDepthCache *pd_cache = nullptr,
    double epsilon = kDefaultPoleSearchEpsilon) -> double;

} // namespace shiny::nesting::pack
