#include "packing/sparrow/multi_bin.hpp"

#include <algorithm>

#include "packing/sparrow/optimize/objective.hpp"

namespace shiny::nesting::pack::sparrow {
namespace detail {

constexpr double kValueTolerance = 1e-9;

[[nodiscard]] auto contains_bin_id(const std::vector<std::uint32_t> &bin_ids,
                                   const std::uint32_t bin_id) -> bool {
  return std::find(bin_ids.begin(), bin_ids.end(), bin_id) != bin_ids.end();
}

[[nodiscard]] auto find_assignment(const PortInstance &instance,
                                   const std::uint32_t piece_id)
    -> const PortBinAssignment * {
  const auto it =
      std::find_if(instance.assignments.begin(), instance.assignments.end(),
                   [piece_id](const PortBinAssignment &assignment) {
                     return assignment.piece_id == piece_id;
                   });
  return it == instance.assignments.end() ? nullptr : &*it;
}

[[nodiscard]] auto find_piece(const PortInstance &instance,
                              const std::uint32_t piece_id)
    -> const PortPiece * {
  const auto it =
      std::find_if(instance.pieces.begin(), instance.pieces.end(),
                   [piece_id](const PortPiece &piece) {
                     return piece.instance.expanded_piece_id == piece_id;
                   });
  return it == instance.pieces.end() ? nullptr : &*it;
}

[[nodiscard]] auto find_bin(const PortInstance &instance,
                            const std::uint32_t bin_id) -> const PortBin * {
  const auto it = std::find_if(instance.bins.begin(), instance.bins.end(),
                               [bin_id](const PortBin &bin) {
                                 return bin.instance.expanded_bin_id == bin_id;
                               });
  return it == instance.bins.end() ? nullptr : &*it;
}

[[nodiscard]] auto find_overflow_lineage(const PortMultiBinState &state,
                                         const std::uint32_t bin_id)
    -> const PortOverflowLineage * {
  const auto it =
      std::find_if(state.overflow_lineage.begin(), state.overflow_lineage.end(),
                   [bin_id](const PortOverflowLineage &overflow) {
                     return overflow.overflow_bin_id == bin_id;
                   });
  return it == state.overflow_lineage.end() ? nullptr : &*it;
}

[[nodiscard]] auto
assignment_allows_source_bin(const PortBinAssignment &assignment,
                             const std::uint32_t source_bin_id) -> bool {
  return assignment.allowed_bin_ids.empty() ||
         contains_bin_id(assignment.allowed_bin_ids, source_bin_id);
}

} // namespace detail

auto inter_bin_collision_relevant(const std::uint32_t lhs_bin_id,
                                  const std::uint32_t rhs_bin_id) -> bool {
  return lhs_bin_id == rhs_bin_id;
}

auto classify_bin_legality(const PortInstance &instance,
                           const PortMultiBinState &state,
                           const std::uint32_t piece_id,
                           const std::uint32_t bin_id) -> BinLegalityDecision {
  BinLegalityDecision decision;

  const auto *assignment = detail::find_assignment(instance, piece_id);
  if (assignment == nullptr) {
    return decision;
  }

  if (const auto *bin = detail::find_bin(instance, bin_id); bin != nullptr) {
    decision.selected_bin = bin->selected;
    if (!decision.selected_bin) {
      return decision;
    }

    if (assignment->pinned_bin_id.has_value()) {
      decision.assignment_allowed =
          bin->instance.source_bin_id == *assignment->pinned_bin_id;
    } else {
      decision.assignment_allowed = detail::assignment_allows_source_bin(
          *assignment, bin->instance.source_bin_id);
    }
    decision.allowed = decision.assignment_allowed;
    return decision;
  }

  const auto *overflow = detail::find_overflow_lineage(state, bin_id);
  if (overflow == nullptr) {
    return decision;
  }

  decision.selected_bin =
      instance.selected_bin_ids.empty() ||
      detail::contains_bin_id(instance.selected_bin_ids,
                              overflow->source_request_bin_id);
  if (!instance.allow_part_overflow || instance.maintain_bed_assignment ||
      assignment->pinned_bin_id.has_value()) {
    return decision;
  }

  decision.assignment_allowed = detail::assignment_allows_source_bin(
      *assignment, overflow->source_request_bin_id);
  decision.overflow_lineage_allowed =
      decision.selected_bin && decision.assignment_allowed;
  decision.allowed = decision.overflow_lineage_allowed;
  return decision;
}

auto legal_move_classes(const PortInstance &instance,
                        const PortMultiBinState &state,
                        const std::uint32_t piece_id,
                        const std::uint32_t current_bin_id)
    -> std::vector<MultiBinMoveClass> {
  std::vector<MultiBinMoveClass> moves{
      MultiBinMoveClass::intra_bin_translate,
  };

  if (const auto *piece = detail::find_piece(instance, piece_id);
      piece != nullptr && !piece->forced_rotation.has_value()) {
    moves.push_back(MultiBinMoveClass::intra_bin_rotate);
  }

  const auto *assignment = detail::find_assignment(instance, piece_id);
  if (assignment != nullptr && (instance.maintain_bed_assignment ||
                                assignment->pinned_bin_id.has_value())) {
    moves.push_back(MultiBinMoveClass::assignment_preserving_repair);
  }

  for (const auto &bin : instance.bins) {
    if (bin.instance.expanded_bin_id == current_bin_id) {
      continue;
    }
    if (classify_bin_legality(instance, state, piece_id,
                              bin.instance.expanded_bin_id)
            .allowed) {
      moves.push_back(MultiBinMoveClass::cross_bin_reassign);
      break;
    }
  }

  if (detail::find_overflow_lineage(state, current_bin_id) != nullptr) {
    moves.push_back(MultiBinMoveClass::consolidate_overflow);
  }

  if (detail::contains_bin_id(state.active_bin_ids, current_bin_id) &&
      state.active_bin_ids.size() > 1U) {
    moves.push_back(MultiBinMoveClass::empty_bin);
  }

  return moves;
}

auto legal_knapsack_move_classes(const PortInstance &instance,
                                 const PortMultiBinState &state,
                                 const std::uint32_t piece_id,
                                 const std::uint32_t current_bin_id)
    -> std::vector<KnapsackMoveClass> {
  std::vector<KnapsackMoveClass> moves;
  if (instance.objective_mode != ObjectiveMode::maximize_value) {
    return moves;
  }

  const auto *piece = detail::find_piece(instance, piece_id);
  const auto *assignment = detail::find_assignment(instance, piece_id);
  if (piece == nullptr || assignment == nullptr) {
    return moves;
  }

  moves.push_back(KnapsackMoveClass::value_swap);

  const bool pinned =
      instance.maintain_bed_assignment || assignment->pinned_bin_id.has_value();
  bool higher_value_unplaced_exists = false;
  bool injectable_unplaced_exists = false;
  for (const auto unplaced_piece_id : state.unplaced_piece_ids) {
    const auto *unplaced_piece =
        detail::find_piece(instance, unplaced_piece_id);
    if (unplaced_piece == nullptr ||
        unplaced_piece->value <= piece->value + detail::kValueTolerance) {
      continue;
    }
    higher_value_unplaced_exists = true;
    if (classify_bin_legality(instance, state, unplaced_piece_id,
                              current_bin_id)
            .allowed) {
      injectable_unplaced_exists = true;
      break;
    }
  }

  if (!pinned && injectable_unplaced_exists) {
    moves.push_back(KnapsackMoveClass::inject_unplaced);
  }
  if (!pinned && higher_value_unplaced_exists) {
    moves.push_back(KnapsackMoveClass::eject_low_value);
  }
  return moves;
}

auto weak_bin_candidates(const pack::Layout &layout)
    -> std::vector<std::uint32_t> {
  std::vector<const pack::LayoutBin *> ranked;
  ranked.reserve(layout.bins.size());
  for (const auto &bin : layout.bins) {
    if (!bin.placements.empty()) {
      ranked.push_back(&bin);
    }
  }

  std::stable_sort(
      ranked.begin(), ranked.end(),
      [](const pack::LayoutBin *lhs, const pack::LayoutBin *rhs) {
        if (lhs->utilization.utilization != rhs->utilization.utilization) {
          return lhs->utilization.utilization < rhs->utilization.utilization;
        }
        if (lhs->utilization.placement_count !=
            rhs->utilization.placement_count) {
          return lhs->utilization.placement_count <
                 rhs->utilization.placement_count;
        }
        return lhs->bin_id < rhs->bin_id;
      });

  std::vector<std::uint32_t> weak_bin_ids;
  weak_bin_ids.reserve(ranked.size());
  for (const auto *bin : ranked) {
    weak_bin_ids.push_back(bin->bin_id);
  }
  return weak_bin_ids;
}

auto bin_count_tradeoff_better(const optimize::ObjectiveValue &candidate,
                               const optimize::ObjectiveValue &reference)
    -> bool {
  return optimize::compare_objective(candidate, reference) ==
         optimize::ObjectiveOrdering::better;
}

} // namespace shiny::nesting::pack::sparrow