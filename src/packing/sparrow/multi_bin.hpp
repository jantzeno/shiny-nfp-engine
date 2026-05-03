#pragma once

#include <cstdint>
#include <vector>

#include "packing/layout.hpp"
#include "packing/sparrow/instance.hpp"
#include "packing/sparrow/optimize/objective.hpp"
#include "packing/sparrow/solution.hpp"

namespace shiny::nesting::pack::sparrow {

enum class MultiBinMoveClass : std::uint8_t {
  intra_bin_translate = 0,
  intra_bin_rotate = 1,
  cross_bin_reassign = 2,
  empty_bin = 3,
  consolidate_overflow = 4,
  assignment_preserving_repair = 5,
};

enum class KnapsackMoveClass : std::uint8_t {
  value_swap = 0,
  inject_unplaced = 1,
  eject_low_value = 2,
};

struct BinLegalityDecision {
  bool allowed{false};
  bool selected_bin{false};
  bool assignment_allowed{false};
  bool overflow_lineage_allowed{false};
};

[[nodiscard]] auto inter_bin_collision_relevant(std::uint32_t lhs_bin_id,
                                                std::uint32_t rhs_bin_id)
    -> bool;

[[nodiscard]] auto classify_bin_legality(const PortInstance &instance,
                                         const PortMultiBinState &state,
                                         std::uint32_t piece_id,
                                         std::uint32_t bin_id)
    -> BinLegalityDecision;

[[nodiscard]] auto
legal_move_classes(const PortInstance &instance, const PortMultiBinState &state,
                   std::uint32_t piece_id, std::uint32_t current_bin_id)
    -> std::vector<MultiBinMoveClass>;

[[nodiscard]] auto legal_knapsack_move_classes(const PortInstance &instance,
                                               const PortMultiBinState &state,
                                               std::uint32_t piece_id,
                                               std::uint32_t current_bin_id)
    -> std::vector<KnapsackMoveClass>;

[[nodiscard]] auto weak_bin_candidates(const pack::Layout &layout)
    -> std::vector<std::uint32_t>;

[[nodiscard]] auto
bin_count_tradeoff_better(const optimize::ObjectiveValue &candidate,
                          const optimize::ObjectiveValue &reference) -> bool;

} // namespace shiny::nesting::pack::sparrow