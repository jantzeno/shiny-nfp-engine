#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "internal/request_normalization.hpp"
#include "packing/layout.hpp"

namespace shiny::nesting::pack::constructive {

struct FillFirstOrderedPiece {
  std::uint32_t source_piece_id{0};
  std::uint32_t expanded_piece_id{0};
  std::size_t sequence{0};
  ConstructivePlacementPhase phase{ConstructivePlacementPhase::primary_order};
};

[[nodiscard]] auto
build_fill_first_primary_order(const NormalizedRequest &request)
    -> std::vector<FillFirstOrderedPiece>;

[[nodiscard]] auto build_fill_first_gap_fill_order(
    const NormalizedRequest &request,
    std::span<const std::uint32_t> unplaced_piece_ids)
    -> std::vector<FillFirstOrderedPiece>;

} // namespace shiny::nesting::pack::constructive