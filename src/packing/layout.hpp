#pragma once

#include <cstdint>
#include <vector>

#include "geometry/transform.hpp"
#include "geometry/types.hpp"
#include "packing/config.hpp"
#include "placement/types.hpp"
#include "polygon_ops/merge_region.hpp"

namespace shiny::nesting::pack {

/**
 * @brief Fully realized placed piece within one bin.
 *
 * Stores the chosen pose, resolved geometry, candidate provenance, and ranking
 * score for one placed part.
 *
 * @par Invariants
 * - `polygon` is the transformed geometry associated with `placement`.
 *
 * @par Performance Notes
 * - Used directly in layout export and bin state snapshots.
 */
struct PlacedPiece {
  place::Placement placement{};
  geom::ResolvedRotation resolved_rotation{};
  geom::PolygonWithHoles polygon{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::constructive_boundary};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
  double score{0.0};
};

/**
 * @brief One step in the constructive placement trace.
 *
 * Records the order and provenance of placements so decodes can be inspected,
 * replayed, or exported without reconstructing intermediate decisions.
 *
 * @par Invariants
 * - `opened_new_bin` identifies whether this placement advanced the bin count.
 *
 * @par Performance Notes
 * - Trace entries are append-only during one decode run.
 */
struct PlacementTraceEntry {
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  geom::RotationIndex rotation_index{};
  geom::ResolvedRotation resolved_rotation{};
  geom::Point2 translation{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::constructive_boundary};
  bool opened_new_bin{false};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
  double score{0.0};
};

/**
 * @brief Utilization summary for one populated bin.
 *
 * @par Invariants
 * - `utilization` is interpreted as `occupied_area / container_area` when the
 *   container area is positive.
 *
 * @par Performance Notes
 * - Stored separately so callers can inspect utilization without recomputing
 *   merged occupied area metrics.
 */
struct BinUtilizationSummary {
  std::uint32_t bin_id{0};
  std::size_t placement_count{0};
  double occupied_area{0.0};
  double container_area{0.0};
  double utilization{0.0};
};

/**
 * @brief One container together with all placements assigned to it.
 *
 * @par Invariants
 * - `occupied` should match the union of all `placements` when the bin state is
 *   current.
 *
 * @par Performance Notes
 * - Keeps merged occupancy cached for later placement and reporting steps.
 */
struct LayoutBin {
  std::uint32_t bin_id{0};
  geom::PolygonWithHoles container{};
  poly::MergedRegion occupied{};
  std::vector<PlacedPiece> placements{};
  BinUtilizationSummary utilization{};
};

/**
 * @brief One cuttable boundary segment in the emitted cut plan.
 */
struct CutSegment {
  std::uint32_t bin_id{0};
  std::uint32_t piece_id{0};
  geom::Segment2 segment{};
  bool from_hole{false};
};

/**
 * @brief Post-processed cut plan derived from a layout.
 *
 * @par Invariants
 * - `removed_cut_length` tracks how much raw length was eliminated by
 *   simplification.
 *
 * @par Performance Notes
 * - Summaries avoid recomputing path lengths in exporters and benchmarks.
 */
struct CutPlan {
  std::vector<CutSegment> segments{};
  double raw_cut_length{0.0};
  double total_cut_length{0.0};
  double removed_cut_length{0.0};
};

/**
 * @brief Final constructive layout across all bins.
 *
 * @par Invariants
 * - `unplaced_piece_ids` lists the input pieces that never received a legal
 *   placement.
 *
 * @par Performance Notes
 * - The trace and bin summaries are kept alongside the geometric state for
 *   direct export.
 */
struct Layout {
  std::vector<LayoutBin> bins{};
  std::vector<PlacementTraceEntry> placement_trace{};
  std::vector<std::uint32_t> unplaced_piece_ids{};
};

/**
 * @brief Builds a cut plan from a finished layout.
 *
 * @par Algorithm Detail
 * - **Strategy**: Segment extraction plus optional coincident-cut reduction.
 * - **Steps**:
 *   1. Extract visible outer and hole segments from all placed polygons.
 *   2. Group segments by bin and owning piece.
 *   3. Apply the configured shared-cut simplification policy.
 *
 * @par Complexity
 * - **Time**: O(s log s) where `s` is the emitted segment count.
 * - **Space**: O(s).
 *
 * @param layout Finished layout to analyze.
 * @param config Cut simplification policy.
 * @return Cut plan and aggregate length statistics.
 */
[[nodiscard]] auto build_cut_plan(const Layout &layout,
                                  const LaserCutOptimizationConfig &config)
    -> CutPlan;

} // namespace shiny::nesting::pack
