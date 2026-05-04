#include "packing/irregular/core.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/queries/normalize.hpp"

namespace shiny::nesting::pack::detail {
namespace {

auto subtract_region(std::vector<geom::PolygonWithHoles> &regions,
                     const geom::PolygonWithHoles &obstacle) -> void {
  std::vector<geom::PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    auto difference = geom::difference_polygons(region, obstacle);
    next_regions.insert(next_regions.end(), difference.begin(),
                        difference.end());
  }
  regions = std::move(next_regions);
}

auto merge_touching_regions(std::vector<geom::PolygonWithHoles> &regions)
    -> void {
  geom::MergedRegion merged;
  for (const auto &region : regions) {
    merged = geom::merge_polygon_into_region(merged, region);
  }
  regions = std::move(merged.regions);
}

auto subtract_obstacle(std::vector<geom::PolygonWithHoles> &regions,
                       const geom::PolygonWithHoles &obstacle,
                       const double part_spacing) -> void {
  if (part_spacing > 0.0) {
    const auto inflated = geom::buffer_polygon(obstacle, part_spacing);
    for (const auto &buffered : inflated) {
      subtract_region(regions, buffered);
    }
    return;
  }
  subtract_region(regions, obstacle);
}

[[nodiscard]] auto compute_hole_regions(const WorkingBin &bin,
                                        const ExecutionPolicy &execution)
    -> std::vector<geom::PolygonWithHoles> {
  if (!execution.enable_part_in_part_placement) {
    return {};
  }

  std::vector<geom::PolygonWithHoles> holes;
  for (std::size_t placement_index = 0;
       placement_index < bin.state.placements.size(); ++placement_index) {
    const auto &placement = bin.state.placements[placement_index];
    for (const auto &hole_ring : placement.polygon.holes()) {
      if (hole_ring.size() < 3U) {
        continue;
      }

      std::vector<geom::PolygonWithHoles> regions{
          geom::normalize_polygon(geom::PolygonWithHoles(hole_ring))};
      for (const auto &zone : bin.exclusion_regions) {
        subtract_region(regions, zone);
      }
      for (std::size_t other_index = 0;
           other_index < bin.state.placements.size(); ++other_index) {
        if (other_index == placement_index) {
          continue;
        }
        subtract_region(
            regions,
            fill_polygon_holes(bin.state.placements[other_index].polygon));
      }
      holes.insert(holes.end(), regions.begin(), regions.end());
    }
  }
  return holes;
}

} // namespace

auto fill_polygon_holes(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  return geom::PolygonWithHoles(polygon.outer());
}

auto make_working_bin(const BinInstance &instance) -> WorkingBin {
  WorkingBin bin;
  bin.state.bin_id = instance.expanded.expanded_bin_id;
  bin.state.identity = instance.expanded.identity;
  bin.state.container = geom::normalize_polygon(instance.source->polygon);
  bin.state.container_geometry_revision = instance.source->geometry_revision;
  bin.state.start_corner = instance.source->start_corner;
  bin.state.utilization = summarize_bin(bin.state);

  for (const auto &zone : instance.source->exclusion_zones) {
    bin.exclusion_regions.push_back(
        geom::normalize_polygon(geom::PolygonWithHoles(zone.region.outer())));
  }
  return bin;
}

auto ensure_free_regions_cached(WorkingBin &bin,
                                const ExecutionPolicy &execution) -> void {
  if (bin.free_regions_valid &&
      bin.placements_at_cache == bin.state.placements.size()) {
    return;
  }

  // Incremental path: if we have a previously-computed cache and placements
  // have only been appended (count grew), subtract only the newly-added
  // placements rather than recomputing from scratch.  This reduces
  // per-placement cost from O(N) polygon-difference operations to O(1) for
  // forward-only passes.  Note: rebuild_working_bin() clears placements, so
  // placements_at_cache >= placements.size() after a rebuild — the incremental
  // path is never taken after a rebuild.
  const bool can_update_incrementally =
      !bin.cached_free_regions.empty() &&
      bin.placements_at_cache < bin.state.placements.size();

  if (can_update_incrementally) {
    auto &regions = bin.cached_free_regions;
    for (std::size_t i = bin.placements_at_cache;
         i < bin.state.placements.size(); ++i) {
      subtract_obstacle(
          regions,
          execution.enable_part_in_part_placement
              ? bin.state.placements[i].polygon
              : fill_polygon_holes(bin.state.placements[i].polygon),
          execution.part_spacing);
    }
    if (execution.irregular.merge_free_regions) {
      merge_touching_regions(regions);
    }
    bin.cached_region_bboxes.clear();
    bin.cached_region_bboxes.reserve(regions.size());
    for (const auto &region : regions) {
      bin.cached_region_bboxes.push_back(geom::compute_bounds(region));
    }
    bin.placements_at_cache = bin.state.placements.size();
    return;
  }

  std::vector<geom::PolygonWithHoles> regions{bin.state.container};
  for (const auto &zone : bin.exclusion_regions) {
    subtract_obstacle(regions, zone, execution.part_spacing);
  }
  for (const auto &placement : bin.state.placements) {
    subtract_obstacle(regions,
                      execution.enable_part_in_part_placement
                          ? placement.polygon
                          : fill_polygon_holes(placement.polygon),
                      execution.part_spacing);
  }
  if (execution.irregular.merge_free_regions) {
    merge_touching_regions(regions);
  }

  bin.cached_region_bboxes.clear();
  bin.cached_region_bboxes.reserve(regions.size());
  for (const auto &region : regions) {
    bin.cached_region_bboxes.push_back(geom::compute_bounds(region));
  }
  bin.cached_free_regions = std::move(regions);
  bin.placements_at_cache = bin.state.placements.size();
  bin.free_regions_valid = true;
}

auto refresh_bin_state(WorkingBin &bin, const ExecutionPolicy &execution)
    -> void {
  bin.state.holes = compute_hole_regions(bin, execution);
  ++bin.state.hole_set_revision;
  bin.state.utilization = summarize_bin(bin.state);
}

auto rebuild_working_bin(WorkingBin &bin, const ExecutionPolicy &execution)
    -> void {
  bin.state.occupied = {};
  bin.placement_bounds.clear();
  for (const auto &placement : bin.state.placements) {
    bin.placement_bounds.push_back(geom::compute_bounds(placement.polygon));
    if (bin.state.occupied.regions.empty()) {
      bin.state.occupied = geom::make_merged_region(placement.polygon);
    } else {
      bin.state.occupied = geom::merge_polygon_into_region(bin.state.occupied,
                                                           placement.polygon);
    }
  }
  ++bin.state.occupied_region_revision;
  refresh_bin_state(bin, execution);
  // Invalidate the free-region cache completely: placements may have been
  // removed or reordered, so the incremental path cannot be used.
  bin.free_regions_valid = false;
  bin.cached_free_regions.clear();
  bin.placements_at_cache = 0;
}

auto remove_trace_entries_for_piece(std::vector<PlacementTraceEntry> &trace,
                                    const std::uint32_t piece_id,
                                    const TrialStateRecorder *trial_recorder)
    -> void {
  for (std::size_t index = 0; index < trace.size();) {
    if (trace[index].piece_id != piece_id) {
      ++index;
      continue;
    }
    if (trial_recorder != nullptr &&
        trial_recorder->record_removed_trace_entry) {
      trial_recorder->record_removed_trace_entry(index, trace[index]);
    }
    trace.erase(trace.begin() + static_cast<std::ptrdiff_t>(index));
  }
}

auto remove_piece_from_bins(std::vector<WorkingBin> &bins,
                            const std::uint32_t piece_id,
                            const ExecutionPolicy &execution,
                            const TrialStateRecorder *trial_recorder) -> bool {
  for (std::size_t bin_index = 0; bin_index < bins.size(); ++bin_index) {
    auto &bin = bins[bin_index];
    const auto placement_it =
        std::find_if(bin.state.placements.begin(), bin.state.placements.end(),
                     [&](const PlacedPiece &placement) {
                       return placement.placement.piece_id == piece_id;
                     });
    if (placement_it == bin.state.placements.end()) {
      continue;
    }

    if (trial_recorder != nullptr && trial_recorder->snapshot_bin) {
      trial_recorder->snapshot_bin(bin_index);
    }

    const auto placement_index = static_cast<std::size_t>(
        std::distance(bin.state.placements.begin(), placement_it));
    bin.state.placements.erase(placement_it);
    if (placement_index < bin.placement_bounds.size()) {
      bin.placement_bounds.erase(bin.placement_bounds.begin() +
                                 static_cast<std::ptrdiff_t>(placement_index));
    }
    rebuild_working_bin(bin, execution);
    return true;
  }
  return false;
}

auto collect_recent_piece_ids(const std::vector<PlacementTraceEntry> &trace,
                              const std::size_t max_count)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> piece_ids;
  piece_ids.reserve(max_count);
  for (auto it = trace.rbegin();
       it != trace.rend() && piece_ids.size() < max_count; ++it) {
    if (std::find(piece_ids.begin(), piece_ids.end(), it->piece_id) ==
        piece_ids.end()) {
      piece_ids.push_back(it->piece_id);
    }
  }
  return piece_ids;
}

auto apply_candidate(WorkingBin &bin, const CandidatePlacement &candidate,
                     std::vector<PlacementTraceEntry> &trace,
                     const bool opened_new_bin,
                     const ExecutionPolicy &execution,
                     const ConstructivePlacementPhase phase,
                     const TrialStateRecorder *trial_recorder) -> void {
  bin.state.placements.push_back({
      .placement = candidate.placement,
      .piece_geometry_revision = candidate.piece_geometry_revision,
      .resolved_rotation = candidate.resolved_rotation,
      .polygon = candidate.polygon,
      .source = candidate.source,
      .nfp_accuracy = candidate.nfp_accuracy,
      .inside_hole = candidate.inside_hole,
      .hole_index = candidate.hole_index,
      .phase = phase,
      .score = candidate.score,
  });
  bin.placement_bounds.push_back(candidate.bounds);
  if (bin.state.occupied.regions.empty()) {
    bin.state.occupied = geom::make_merged_region(candidate.polygon);
  } else {
    bin.state.occupied =
        geom::merge_polygon_into_region(bin.state.occupied, candidate.polygon);
  }
  ++bin.state.occupied_region_revision;
  refresh_bin_state(bin, execution);
  bin.free_regions_valid = false;

  trace.push_back({
      .piece_id = candidate.placement.piece_id,
      .bin_id = candidate.placement.bin_id,
      .piece_geometry_revision = candidate.piece_geometry_revision,
      .rotation_index = candidate.placement.rotation_index,
      .resolved_rotation = candidate.resolved_rotation,
      .translation = candidate.placement.translation,
      .mirrored = candidate.placement.mirrored,
      .source = candidate.source,
      .nfp_accuracy = candidate.nfp_accuracy,
      .opened_new_bin = opened_new_bin,
      .inside_hole = candidate.inside_hole,
      .hole_index = candidate.hole_index,
      .phase = phase,
      .score = candidate.score,
  });
  if (trial_recorder != nullptr &&
      trial_recorder->record_appended_trace_entry) {
    trial_recorder->record_appended_trace_entry();
  }
}

auto build_layout(std::span<const WorkingBin> bins,
                  const std::vector<PlacementTraceEntry> &trace,
                  const std::vector<std::uint32_t> &unplaced) -> Layout {
  Layout layout;
  layout.placement_trace = trace;
  layout.unplaced_piece_ids = unplaced;
  layout.bins.reserve(bins.size());
  for (const auto &bin : bins) {
    layout.bins.push_back({
        .bin_id = bin.state.bin_id,
        .identity = bin.state.identity,
        .container = bin.state.container,
        .occupied = bin.state.occupied,
        .placements = bin.state.placements,
        .utilization = bin.state.utilization,
    });
  }
  return layout;
}

auto build_lightweight_layout(std::span<const WorkingBin> bins,
                              const std::vector<PlacementTraceEntry> &trace,
                              const std::vector<std::uint32_t> &unplaced)
    -> Layout {
  Layout layout;
  layout.placement_trace = trace;
  layout.unplaced_piece_ids = unplaced;
  layout.bins.reserve(bins.size());
  for (const auto &bin : bins) {
    layout.bins.push_back({
        .bin_id = bin.state.bin_id,
        .identity = bin.state.identity,
        .container = {},
        .occupied = {},
        .placements = bin.state.placements,
        .utilization = bin.state.utilization,
    });
  }
  return layout;
}

auto compute_utilization_percent(std::span<const WorkingBin> bins) -> double {
  double total_occupied = 0.0;
  double total_container = 0.0;
  for (const auto &bin : bins) {
    total_occupied += bin.state.utilization.occupied_area;
    total_container += bin.state.utilization.container_area;
  }
  return total_container > 0.0 ? (total_occupied / total_container) * 100.0
                               : 0.0;
}

} // namespace shiny::nesting::pack::detail
