#include "packing/constructive/ordering.hpp"

#include <algorithm>
#include <iterator>
#include <unordered_map>

#include "packing/irregular/core.hpp"

namespace shiny::nesting::pack::constructive {
namespace {

auto primary_ordered_piece_ids(const NormalizedRequest &request)
    -> std::vector<std::uint32_t> {
  const auto ordered = detail::order_piece_instances(
      detail::build_piece_instances(request), request.request.execution);
  std::vector<std::uint32_t> ids;
  ids.reserve(ordered.size());
  std::ranges::transform(ordered, std::back_inserter(ids),
                         [](const auto &piece) {
                           return piece.expanded.expanded_piece_id;
                         });
  return ids;
}

auto source_piece_id_by_expanded(const NormalizedRequest &request)
    -> std::unordered_map<std::uint32_t, std::uint32_t> {
  std::unordered_map<std::uint32_t, std::uint32_t> ids;
  ids.reserve(request.expanded_pieces.size());
  for (const auto &piece : request.expanded_pieces) {
    ids.emplace(piece.expanded_piece_id, piece.source_piece_id);
  }
  return ids;
}

} // namespace

auto build_fill_first_primary_order(const NormalizedRequest &request)
    -> std::vector<FillFirstOrderedPiece> {
  const auto ordered_ids = primary_ordered_piece_ids(request);
  const auto source_ids = source_piece_id_by_expanded(request);

  std::vector<FillFirstOrderedPiece> ordered;
  ordered.reserve(ordered_ids.size());
  for (std::size_t index = 0; index < ordered_ids.size(); ++index) {
    const auto expanded_piece_id = ordered_ids[index];
    ordered.push_back({
        .source_piece_id = source_ids.at(expanded_piece_id),
        .expanded_piece_id = expanded_piece_id,
        .sequence = index,
        .phase = ConstructivePlacementPhase::primary_order,
    });
  }
  return ordered;
}

auto build_fill_first_gap_fill_order(
    const NormalizedRequest &request,
    std::span<const std::uint32_t> unplaced_piece_ids)
    -> std::vector<FillFirstOrderedPiece> {
  const auto ordered_ids = primary_ordered_piece_ids(request);
  const auto source_ids = source_piece_id_by_expanded(request);

  std::unordered_map<std::uint32_t, std::size_t> priority_by_piece;
  priority_by_piece.reserve(ordered_ids.size());
  for (std::size_t index = 0; index < ordered_ids.size(); ++index) {
    priority_by_piece.emplace(ordered_ids[index], index);
  }

  std::vector<std::uint32_t> retry_ids(unplaced_piece_ids.begin(),
                                       unplaced_piece_ids.end());
  // Sort descending by priority index so that the smallest/lowest-priority
  // pieces (highest index) are tried first in gap fill.  Unplaced pieces are
  // retried in the reverse of the primary ordering: smaller pieces are more
  // likely to fit into gaps left by the earlier pass.
  std::ranges::sort(retry_ids,
                     [&](const std::uint32_t lhs, const std::uint32_t rhs) {
                       return priority_by_piece.at(lhs) >
                              priority_by_piece.at(rhs);
                     });

  std::vector<FillFirstOrderedPiece> ordered;
  ordered.reserve(retry_ids.size());
  for (std::size_t index = 0; index < retry_ids.size(); ++index) {
    const auto expanded_piece_id = retry_ids[index];
    ordered.push_back({
        .source_piece_id = source_ids.at(expanded_piece_id),
        .expanded_piece_id = expanded_piece_id,
        .sequence = index,
        .phase = ConstructivePlacementPhase::gap_fill,
    });
  }
  return ordered;
}

} // namespace shiny::nesting::pack::constructive