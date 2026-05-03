#include "packing/sparrow/adapters/layout_adapter.hpp"

#include <algorithm>

namespace shiny::nesting::pack::sparrow::adapters {

namespace {

auto append_unique_bin_id(std::vector<std::uint32_t> &active_bin_ids,
                          const std::uint32_t bin_id) -> void {
  if (std::find(active_bin_ids.begin(), active_bin_ids.end(), bin_id) ==
      active_bin_ids.end()) {
    active_bin_ids.push_back(bin_id);
  }
}

[[nodiscard]] auto to_multi_bin_state(const pack::Layout &layout,
                                      const ConstructiveReplay &constructive)
    -> PortMultiBinState {
  PortMultiBinState multi_bin;
  multi_bin.unplaced_piece_ids = layout.unplaced_piece_ids;
  multi_bin.overflow_lineage.reserve(constructive.overflow_events.size());

  for (const auto &entry : layout.placement_trace) {
    append_unique_bin_id(multi_bin.active_bin_ids, entry.bin_id);
  }
  if (multi_bin.active_bin_ids.empty()) {
    for (const auto &bin : layout.bins) {
      if (!bin.placements.empty()) {
        append_unique_bin_id(multi_bin.active_bin_ids, bin.bin_id);
      }
    }
  }

  for (const auto &overflow : constructive.overflow_events) {
    multi_bin.overflow_lineage.push_back({
        .template_bin_id = overflow.template_bin_id,
        .overflow_bin_id = overflow.overflow_bin_id,
        .source_request_bin_id = overflow.source_request_bin_id,
    });
  }

  return multi_bin;
}

} // namespace

auto to_seed_solution(const pack::Layout &layout,
                      const ConstructiveReplay &constructive) -> SeedSolution {
  SeedSolution seed{
      .layout = layout,
      .constructive = constructive,
      .multi_bin = to_multi_bin_state(layout, constructive),
      .unplaced_piece_ids = layout.unplaced_piece_ids,
  };
  seed.placements.reserve(layout.placement_trace.size());
  for (const auto &entry : layout.placement_trace) {
    seed.placements.push_back({
        .piece_id = entry.piece_id,
        .bin_id = entry.bin_id,
        .translation = entry.translation,
        .resolved_rotation = entry.resolved_rotation,
        .mirrored = entry.mirrored,
        .phase = entry.phase,
    });
  }
  return seed;
}

auto to_layout(const SeedSolution &seed) -> pack::Layout {
  pack::Layout layout = seed.layout;
  layout.placement_trace.clear();
  layout.placement_trace.reserve(seed.placements.size());
  for (const auto &placement : seed.placements) {
    pack::PlacementTraceEntry entry;
    entry.piece_id = placement.piece_id;
    entry.bin_id = placement.bin_id;
    entry.resolved_rotation = placement.resolved_rotation;
    entry.translation = placement.translation;
    entry.mirrored = placement.mirrored;
    entry.phase = placement.phase;
    layout.placement_trace.push_back(entry);
  }
  layout.unplaced_piece_ids = seed.multi_bin.unplaced_piece_ids.empty()
                                  ? seed.unplaced_piece_ids
                                  : seed.multi_bin.unplaced_piece_ids;
  return layout;
}

auto to_layout(const PortSolution &solution) -> pack::Layout {
  return solution.layout;
}

auto to_nesting_result(const PortSolution &solution,
                       const StrategyKind strategy,
                       const std::size_t total_parts) -> NestingResult {
  return {
      .strategy = strategy,
      .layout = solution.layout,
      .total_parts = total_parts,
      .effective_seed = solution.effective_seed,
      .stop_reason = solution.stop_reason,
      .constructive = solution.constructive,
  };
}

} // namespace shiny::nesting::pack::sparrow::adapters