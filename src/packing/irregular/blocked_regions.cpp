#include "packing/irregular/blocked_regions.hpp"

#include <exception>
#include <memory>
#include <utility>

#include "logging/shiny_log.hpp"
#include "nfp/nfp.hpp"

namespace shiny::nesting::pack {
namespace {

[[nodiscard]] auto
obstacle_base_polygon(const CandidateGenerationObstacle &obstacle)
    -> geom::PolygonWithHoles {
  return geom::translate(
      obstacle.polygon,
      geom::Vector2(-obstacle.translation.x(), -obstacle.translation.y()));
}

[[nodiscard]] auto
build_bbox_blocked_region_fallback(const geom::PolygonWithHoles &fixed_polygon,
                                   const geom::PolygonWithHoles &moving_piece,
                                   const util::Status fallback_status)
    -> cache::NfpCacheValue {
  const auto fixed_bounds = geom::compute_bounds(fixed_polygon);
  const auto moving_bounds = geom::compute_bounds(moving_piece);
  const geom::Box2 overlap_domain{
      .min = geom::Point2(fixed_bounds.min.x() - moving_bounds.max.x(),
                          fixed_bounds.min.y() - moving_bounds.max.y()),
      .max = geom::Point2(fixed_bounds.max.x() - moving_bounds.min.x(),
                          fixed_bounds.max.y() - moving_bounds.min.y()),
  };
  return {
      .polygons = {geom::box_to_polygon(overlap_domain)},
      .accuracy = cache::NfpCacheAccuracy::conservative_bbox_fallback,
      .status = fallback_status,
  };
}

} // namespace

auto build_blocked_regions(
    std::span<const CandidateGenerationObstacle> obstacles,
    const geom::PolygonWithHoles &moving_piece,
    const std::uint64_t moving_piece_revision,
    const geom::ResolvedRotation moving_rotation, cache::NfpCache *cache_ptr,
    CandidateGenerationDiagnostics *diagnostics)
    -> std::expected<BlockedRegions, util::Status> {
  BlockedRegions blocked;

  for (const auto &obstacle : obstacles) {
    const auto fixed_polygon = obstacle_base_polygon(obstacle);
    const auto fixed_revision = obstacle.geometry_revision != 0U
                                    ? obstacle.geometry_revision
                                    : geom::polygon_revision(fixed_polygon);
    const auto exact_key = cache::make_nfp_cache_key(
        fixed_revision, moving_piece_revision, obstacle.rotation.degrees,
        moving_rotation.degrees, cache::NfpCacheEntryKind::exact);
    const auto fallback_key = cache::make_nfp_cache_key(
        fixed_revision, moving_piece_revision, obstacle.rotation.degrees,
        moving_rotation.degrees,
        cache::NfpCacheEntryKind::conservative_bbox_fallback);

    std::shared_ptr<const cache::NfpCacheValue> exact_cached;
    if (cache_ptr != nullptr) {
      exact_cached = cache_ptr->get(exact_key);
      if (exact_cached != nullptr && diagnostics != nullptr) {
        ++diagnostics->exact_nfp_cache_hits;
      }
    }

    const cache::NfpCacheValue *base_nfp_ptr = exact_cached.get();
    cache::NfpCacheValue exact_storage;
    cache::NfpCacheValue fallback_storage;
    if (base_nfp_ptr == nullptr) {
      auto computed = std::expected<std::vector<geom::PolygonWithHoles>, util::Status>(
          std::unexpected(util::Status::computation_failed));
      try {
        computed = nfp::compute_nfp(fixed_polygon, moving_piece);
      } catch (const std::exception &ex) {
        SHINY_DEBUG("candidate_generation: build_blocked_regions NFP threw "
                    "fixed_rev={} moving_rev={} fixed_rot={} moving_rot={} "
                    "error={}",
                    fixed_revision, moving_piece_revision,
                    obstacle.rotation.degrees, moving_rotation.degrees,
                    ex.what());
      } catch (...) {
        SHINY_DEBUG("candidate_generation: build_blocked_regions NFP threw "
                    "fixed_rev={} moving_rev={} fixed_rot={} moving_rot={} "
                    "with unknown exception",
                    fixed_revision, moving_piece_revision,
                    obstacle.rotation.degrees, moving_rotation.degrees);
      }

      if (computed.has_value()) {
        if (diagnostics != nullptr) {
          ++diagnostics->exact_nfp_computations;
        }
        exact_storage = {
            .polygons = std::move(computed).value(),
            .accuracy = cache::NfpCacheAccuracy::exact,
            .status = util::Status::ok,
        };
        if (cache_ptr != nullptr) {
          cache_ptr->put(exact_key, exact_storage);
        }
        base_nfp_ptr = &exact_storage;
      } else {
        const auto exact_status = computed.error();

        std::shared_ptr<const cache::NfpCacheValue> fallback_cached;
        if (cache_ptr != nullptr) {
          fallback_cached = cache_ptr->get(fallback_key);
          if (fallback_cached != nullptr && diagnostics != nullptr) {
            ++diagnostics->conservative_bbox_cache_hits;
          }
        }

        base_nfp_ptr = fallback_cached.get();
        if (base_nfp_ptr == nullptr) {
          SHINY_DEBUG("candidate_generation: build_blocked_regions falling "
                      "back to bbox NFP status={} fixed_rev={} moving_rev={} "
                      "fixed_rot={} moving_rot={}",
                      util::status_name(exact_status), fixed_revision,
                      moving_piece_revision, obstacle.rotation.degrees,
                      moving_rotation.degrees);
          fallback_storage = build_bbox_blocked_region_fallback(
              fixed_polygon, moving_piece, exact_status);
          if (cache_ptr != nullptr) {
            cache_ptr->put(fallback_key, fallback_storage);
          }
          if (diagnostics != nullptr) {
            ++diagnostics->conservative_bbox_fallbacks;
          }
          base_nfp_ptr = &fallback_storage;
        }
      }
    }

    if (base_nfp_ptr->accuracy ==
        cache::NfpCacheAccuracy::conservative_bbox_fallback) {
      blocked.accuracy = cache::NfpCacheAccuracy::conservative_bbox_fallback;
    }
    for (const auto &polygon : base_nfp_ptr->polygons) {
      blocked.polygons.push_back(
          geom::translate(polygon, geom::Vector2(obstacle.translation.x(),
                                                 obstacle.translation.y())));
    }
  }

  return blocked;
}

} // namespace shiny::nesting::pack
