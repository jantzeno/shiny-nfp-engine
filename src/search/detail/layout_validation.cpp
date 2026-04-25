#include "search/detail/layout_validation.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>

#include "packing/common.hpp"
#include "polygon_ops/boolean_ops.hpp"

namespace shiny::nesting::search::detail {
namespace {

constexpr double kConstructiveOverlapAreaEpsilon = 1e-6;

[[nodiscard]] auto exact_overlap_area(const geom::PolygonWithHoles &lhs,
                                      const geom::PolygonWithHoles &rhs)
    -> double {
  return geom::polygon_area_sum(poly::intersection_polygons(lhs, rhs));
}

[[nodiscard]] auto
source_bin_id_for_expanded_bin(const NormalizedRequest &request,
                               const std::uint32_t expanded_bin_id)
    -> std::optional<std::uint32_t> {
  for (const auto &expanded_bin : request.expanded_bins) {
    if (expanded_bin.expanded_bin_id == expanded_bin_id) {
      return expanded_bin.source_bin_id;
    }
  }
  return std::nullopt;
}

[[nodiscard]] auto
source_bin_for_expanded_bin(const NormalizedRequest &request,
                            const std::uint32_t expanded_bin_id)
    -> const BinRequest * {
  const auto source_bin_id =
      source_bin_id_for_expanded_bin(request, expanded_bin_id);
  if (!source_bin_id.has_value()) {
    return nullptr;
  }

  const auto &bins = request.request.bins;
  const auto it = std::find_if(bins.begin(), bins.end(),
                               [source_bin_id](const BinRequest &bin) {
                                 return bin.bin_id == *source_bin_id;
                               });
  return it == bins.end() ? nullptr : &*it;
}

[[nodiscard]] auto
overlaps_configured_exclusion_zone(const NormalizedRequest &request,
                                   const std::uint32_t expanded_bin_id,
                                   const pack::PlacedPiece &placed) -> bool {
  const auto *source_bin =
      source_bin_for_expanded_bin(request, expanded_bin_id);
  if (source_bin == nullptr || source_bin->exclusion_zones.empty()) {
    return false;
  }

  const auto placed_bounds = pack::compute_bounds(placed.polygon);
  for (const auto &zone : source_bin->exclusion_zones) {
    if (pack::overlaps_exclusion_zone(placed.polygon, placed_bounds, zone)) {
      return true;
    }
  }
  return false;
}

} // namespace

auto constructive_layout_has_geometry_violation(
    const NormalizedRequest &request, const NestingResult &result) -> bool {
  for (const auto &bin : result.layout.bins) {
    for (const auto &placed : bin.placements) {
      if (overlaps_configured_exclusion_zone(request, bin.bin_id, placed)) {
        return true;
      }
    }

    for (std::size_t lhs_index = 0; lhs_index < bin.placements.size();
         ++lhs_index) {
      const auto &lhs = bin.placements[lhs_index];
      const auto lhs_bounds = pack::compute_bounds(lhs.polygon);
      for (std::size_t rhs_index = lhs_index + 1;
           rhs_index < bin.placements.size(); ++rhs_index) {
        const auto &rhs = bin.placements[rhs_index];
        const auto rhs_bounds = pack::compute_bounds(rhs.polygon);
        const bool part_in_part_pair =
            request.request.execution.enable_part_in_part_placement &&
            (lhs.inside_hole || rhs.inside_hole);
        if (!part_in_part_pair && pack::boxes_violate_spacing(
                                      lhs_bounds, rhs_bounds,
                                      request.request.execution.part_spacing)) {
          return true;
        }
        if (!pack::boxes_overlap(lhs_bounds, rhs_bounds)) {
          continue;
        }
        if (exact_overlap_area(lhs.polygon, rhs.polygon) >
            kConstructiveOverlapAreaEpsilon) {
          return true;
        }
      }
    }
  }
  return false;
}

} // namespace shiny::nesting::search::detail
