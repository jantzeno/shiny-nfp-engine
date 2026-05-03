#include "packing/sparrow/search/disruption.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>

#include "geometry/polygon.hpp"

namespace shiny::nesting::search {
namespace {

struct PieceShapeProfile {
  double area{0.0};
  double aspect_ratio{1.0};
  double diameter{0.0};
};

[[nodiscard]] auto build_shape_profiles(const NormalizedRequest &request)
    -> std::unordered_map<std::uint32_t, PieceShapeProfile> {
  std::unordered_map<std::uint32_t, PieceShapeProfile> profiles;
  profiles.reserve(request.request.pieces.size());

  for (const auto &piece : request.request.pieces) {
    const auto bounds = geom::compute_bounds(piece.polygon);
    const double width = std::max(geom::box_width(bounds), 1e-9);
    const double height = std::max(geom::box_height(bounds), 1e-9);
    profiles.emplace(piece.piece_id,
                     PieceShapeProfile{
                         .area = geom::polygon_area(piece.polygon),
                         .aspect_ratio = width / height,
                         .diameter = std::hypot(width, height),
                     });
  }

  return profiles;
}

// Symmetric area floor. The earlier `+1e-9` additive trick skewed the
// log ratio for very small pieces (1e-12 vs 1e-9 produced log(1000)
// ~= 6.9, dwarfing the aspect/diameter terms). Clamping each operand
// to `kAreaFloor` independently keeps the comparison symmetric and
// caps the worst-case log ratio at log(area_max / kAreaFloor).
constexpr double kAreaFloor = 1e-6;

[[nodiscard]] auto dissimilarity_score(const PieceShapeProfile &lhs,
                                       const PieceShapeProfile &rhs) -> double {
  const double area_ratio = std::abs(std::log(std::max(lhs.area, kAreaFloor) /
                                              std::max(rhs.area, kAreaFloor)));
  const double aspect_delta = std::abs(lhs.aspect_ratio - rhs.aspect_ratio);
  const double diameter_delta = std::abs(lhs.diameter - rhs.diameter) /
                                std::max({lhs.diameter, rhs.diameter, 1e-9});
  return area_ratio + aspect_delta + diameter_delta;
}

} // namespace

// Pick two "large item" positions whose pieces differ most by
// shape-profile (log-area, aspect ratio, normalised diameter); swap
// them. Implements Sparrow §7.1's "large-item swap" disruption
// operator.
//
// Candidate set: pieces with area at or above the mean piece area,
// with a floor of the two largest positions so small fixtures still
// perturb something. Pair score is purely pairwise; O(c²).
auto disrupt_large_items(std::span<const std::size_t> order,
                         const NormalizedRequest &request,
                         std::span<const double> piece_areas,
                         runtime::DeterministicRng &rng) -> DisruptionResult {
  (void)rng;
  DisruptionResult result;
  result.order.assign(order.begin(), order.end());
  if (order.size() < 2U || piece_areas.size() < order.size()) {
    return result;
  }
  assert(piece_areas.size() >= order.size());

  const auto shape_profiles = build_shape_profiles(request);

  std::vector<std::size_t> candidate_positions(order.size());
  for (std::size_t position = 0; position < order.size(); ++position) {
    candidate_positions[position] = position;
  }
  std::stable_sort(candidate_positions.begin(), candidate_positions.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (piece_areas[order[lhs]] != piece_areas[order[rhs]]) {
                       return piece_areas[order[lhs]] > piece_areas[order[rhs]];
                     }
                     return lhs < rhs;
                   });

  // Kahan-compensated mean over piece areas — large catalogues with
  // wide area dynamic range otherwise lose the contribution of the
  // smallest pieces in a naive double sum.
  double area_sum = 0.0;
  double area_compensation = 0.0;
  for (const auto piece_index : order) {
    const double y = piece_areas[piece_index] - area_compensation;
    const double t = area_sum + y;
    area_compensation = (t - area_sum) - y;
    area_sum = t;
  }
  const double mean_area = area_sum / static_cast<double>(order.size());
  const auto ranked_positions = candidate_positions;
  candidate_positions.erase(
      std::remove_if(candidate_positions.begin(), candidate_positions.end(),
                     [&](const std::size_t position) {
                       return piece_areas[order[position]] + 1e-9 < mean_area;
                     }),
      candidate_positions.end());
  if (candidate_positions.size() < 2U) {
    // Fall back to the top two distinct ranked positions; positions are
    // unique by construction so dedupe is implicit, but we guard
    // explicitly to avoid an out-of-range access on degenerate inputs.
    candidate_positions.clear();
    for (const auto position : ranked_positions) {
      if (candidate_positions.empty() ||
          candidate_positions.front() != position) {
        candidate_positions.push_back(position);
      }
      if (candidate_positions.size() == 2U) {
        break;
      }
    }
    if (candidate_positions.size() < 2U) {
      return result;
    }
  }

  double best_score = -std::numeric_limits<double>::infinity();
  std::size_t best_first = candidate_positions.front();
  std::size_t best_second = candidate_positions.back();
  for (std::size_t lhs_index = 0; lhs_index < candidate_positions.size();
       ++lhs_index) {
    for (std::size_t rhs_index = lhs_index + 1U;
         rhs_index < candidate_positions.size(); ++rhs_index) {
      const auto lhs_piece =
          request.expanded_pieces[order[candidate_positions[lhs_index]]];
      const auto rhs_piece =
          request.expanded_pieces[order[candidate_positions[rhs_index]]];
      const auto lhs_profile = shape_profiles.find(lhs_piece.source_piece_id);
      const auto rhs_profile = shape_profiles.find(rhs_piece.source_piece_id);
      const double score =
          lhs_profile == shape_profiles.end() ||
                  rhs_profile == shape_profiles.end()
              ? std::abs(piece_areas[order[candidate_positions[lhs_index]]] -
                         piece_areas[order[candidate_positions[rhs_index]]])
              : dissimilarity_score(lhs_profile->second, rhs_profile->second);
      if (score > best_score) {
        best_score = score;
        best_first = candidate_positions[lhs_index];
        best_second = candidate_positions[rhs_index];
      }
    }
  }

  // Candidate positions are unique by construction and the pair search
  // above only considers rhs_index > lhs_index, so best_first and
  // best_second are guaranteed distinct whenever the inner loop ran at
  // least once. Guard explicitly only for the empty-pair edge case.
  if (best_first == best_second) {
    return result;
  }

  std::swap(result.order[best_first], result.order[best_second]);
  result.first_position = best_first;
  result.second_position = best_second;
  result.applied = best_first != best_second;
  return result;
}

} // namespace shiny::nesting::search
