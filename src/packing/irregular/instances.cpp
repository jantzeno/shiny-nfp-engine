#include "packing/irregular/core.hpp"

#include <algorithm>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry/polygon.hpp"
#include "polygon_ops/convex_hull.hpp"

namespace shiny::nesting::pack::detail {
namespace {

struct PieceOrderingMetrics {
  std::size_t original_index{0};
  double polygon_area{0.0};
  double hull_area{0.0};
  double max_dimension{0.0};
  double hull_diameter_score{0.0};
  std::int32_t priority{0};
};

[[nodiscard]] auto piece_order_metrics_for(const PieceInstance &piece,
                                           const std::size_t original_index)
    -> PieceOrderingMetrics {
  const auto bounds = geom::compute_bounds(piece.source->polygon);
  const auto hull = poly::compute_convex_hull(piece.source->polygon);
  const auto hull_area = geom::polygon_area(hull);
  const auto max_dimension =
      std::max(geom::box_width(bounds), geom::box_height(bounds));
  return {
      .original_index = original_index,
      .polygon_area = geom::polygon_area(piece.source->polygon),
      .hull_area = hull_area,
      .max_dimension = max_dimension,
      .hull_diameter_score = hull_area * max_dimension,
      .priority = piece.source->priority,
  };
}

} // namespace

auto order_piece_instances(std::vector<PieceInstance> pieces,
                           const ExecutionPolicy &execution)
    -> std::vector<PieceInstance> {
  if (execution.irregular.piece_ordering == PieceOrdering::input) {
    return pieces;
  }

  std::unordered_map<std::uint32_t, PieceOrderingMetrics> metrics_by_id;
  metrics_by_id.reserve(pieces.size());
  for (std::size_t index = 0; index < pieces.size(); ++index) {
    metrics_by_id.emplace(pieces[index].expanded.expanded_piece_id,
                          piece_order_metrics_for(pieces[index], index));
  }

  std::stable_sort(pieces.begin(), pieces.end(),
                   [&execution, &metrics_by_id](const PieceInstance &lhs,
                                                const PieceInstance &rhs) {
                     const auto &lhs_metrics =
                         metrics_by_id.at(lhs.expanded.expanded_piece_id);
                     const auto &rhs_metrics =
                         metrics_by_id.at(rhs.expanded.expanded_piece_id);

                     switch (execution.irregular.piece_ordering) {
                     case PieceOrdering::input:
                       return lhs_metrics.original_index < rhs_metrics.original_index;
                     case PieceOrdering::largest_area_first:
                       if (!almost_equal(lhs_metrics.polygon_area,
                                         rhs_metrics.polygon_area)) {
                         return lhs_metrics.polygon_area > rhs_metrics.polygon_area;
                       }
                       break;
                     case PieceOrdering::hull_diameter_first:
                       if (!almost_equal(lhs_metrics.hull_diameter_score,
                                         rhs_metrics.hull_diameter_score)) {
                         return lhs_metrics.hull_diameter_score >
                                rhs_metrics.hull_diameter_score;
                       }
                       break;
                     case PieceOrdering::priority:
                       if (lhs_metrics.priority != rhs_metrics.priority) {
                         return lhs_metrics.priority > rhs_metrics.priority;
                       }
                       if (!almost_equal(lhs_metrics.polygon_area,
                                         rhs_metrics.polygon_area)) {
                         return lhs_metrics.polygon_area > rhs_metrics.polygon_area;
                       }
                       break;
                     }

                     return lhs_metrics.original_index < rhs_metrics.original_index;
                   });
  return pieces;
}

auto build_piece_instances(const NormalizedRequest &request)
    -> std::vector<PieceInstance> {
  std::unordered_map<std::uint32_t, const PieceRequest *> pieces_by_id;
  std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>
      expanded_bin_ids_by_source;
  for (const auto &piece : request.request.pieces) {
    pieces_by_id.emplace(piece.piece_id, &piece);
  }
  for (const auto &expanded_bin : request.expanded_bins) {
    expanded_bin_ids_by_source[expanded_bin.source_bin_id].push_back(
        expanded_bin.expanded_bin_id);
  }

  std::vector<PieceInstance> instances;
  instances.reserve(request.expanded_pieces.size());
  for (std::size_t index = 0; index < request.expanded_pieces.size(); ++index) {
    const auto &expanded = request.expanded_pieces[index];
    const auto it = pieces_by_id.find(expanded.source_piece_id);
    if (it == pieces_by_id.end()) {
      continue;
    }

    PieceInstance instance{
        .expanded = expanded,
        .source = it->second,
        .restricted_to_allowed_bins = !it->second->allowed_bin_ids.empty(),
        .forced_rotation_index =
            index < request.forced_rotations.size() ? request.forced_rotations[index]
                                                    : std::nullopt,
    };
    for (const auto source_bin_id : it->second->allowed_bin_ids) {
      const auto bins_it = expanded_bin_ids_by_source.find(source_bin_id);
      if (bins_it == expanded_bin_ids_by_source.end()) {
        continue;
      }
      instance.allowed_expanded_bin_ids.insert(instance.allowed_expanded_bin_ids.end(),
                                               bins_it->second.begin(),
                                               bins_it->second.end());
    }
    std::sort(instance.allowed_expanded_bin_ids.begin(),
              instance.allowed_expanded_bin_ids.end());
    instance.allowed_expanded_bin_ids.erase(
        std::unique(instance.allowed_expanded_bin_ids.begin(),
                    instance.allowed_expanded_bin_ids.end()),
        instance.allowed_expanded_bin_ids.end());
    instances.push_back(std::move(instance));
  }
  return instances;
}

auto build_bin_instances(const NormalizedRequest &request)
    -> std::vector<BinInstance> {
  std::unordered_map<std::uint32_t, const BinRequest *> bins_by_id;
  for (const auto &bin : request.request.bins) {
    bins_by_id.emplace(bin.bin_id, &bin);
  }

  std::vector<BinInstance> instances;
  instances.reserve(request.expanded_bins.size());
  for (const auto &expanded : request.expanded_bins) {
    const auto it = bins_by_id.find(expanded.source_bin_id);
    if (it != bins_by_id.end()) {
      instances.push_back({.expanded = expanded, .source = it->second});
    }
  }
  return instances;
}

} // namespace shiny::nesting::pack::detail
