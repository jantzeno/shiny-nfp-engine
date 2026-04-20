#include "packing/irregular_constructive_packer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry/normalize.hpp"
#include "geometry/polygon.hpp"
#include "geometry/transform.hpp"
#include "logging/shiny_log.hpp"
#include "packing/bin_state.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "predicates/point_location.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kAreaEpsilon = 1e-8;
constexpr double kDistanceEpsilon = 1e-8;
constexpr std::size_t kInterruptCheckInterval = 256;
constexpr std::uint64_t kDefaultPerPieceBudgetMs = 30'000;
constexpr std::size_t kTopKCandidates = 3;

struct PieceInstance {
  ExpandedPieceInstance expanded{};
  const PieceRequest *source{nullptr};
  std::vector<std::uint32_t> allowed_expanded_bin_ids{};
};

struct BinInstance {
  ExpandedBinInstance expanded{};
  const BinRequest *source{nullptr};
};

struct AnchorPoint {
  geom::Point2 point{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::constructive_boundary};
};

struct CandidatePlacement {
  place::Placement placement{};
  geom::ResolvedRotation resolved_rotation{};
  geom::PolygonWithHoles polygon{};
  geom::Box2 bounds{};
  place::PlacementCandidateSource source{
      place::PlacementCandidateSource::constructive_boundary};
  bool inside_hole{false};
  std::int32_t hole_index{-1};
  double score{0.0};
};

struct WorkingBin {
  BinState state{};
  std::vector<geom::PolygonWithHoles> exclusion_regions{};
  std::vector<geom::Box2> placement_bounds{};
  std::vector<geom::PolygonWithHoles> cached_free_regions{};
  std::vector<geom::Box2> cached_region_bboxes{};
  std::size_t placements_at_cache{0};
  bool free_regions_valid{false};
};

[[nodiscard]] auto polygon_area_sum(
    std::span<const geom::PolygonWithHoles> polygons) -> double {
  double total = 0.0;
  for (const auto &polygon : polygons) {
    total += geom::polygon_area(polygon);
  }
  return total;
}

[[nodiscard]] auto almost_equal(const double lhs, const double rhs) -> bool {
  return std::fabs(lhs - rhs) <= kDistanceEpsilon;
}

auto subtract_region(std::vector<geom::PolygonWithHoles> &regions,
                     const geom::PolygonWithHoles &obstacle) -> void {
  std::vector<geom::PolygonWithHoles> next_regions;
  for (const auto &region : regions) {
    auto difference = poly::difference_polygons(region, obstacle);
    next_regions.insert(next_regions.end(), difference.begin(), difference.end());
  }
  regions = std::move(next_regions);
}

[[nodiscard]] auto fill_polygon_holes(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  return {.outer = polygon.outer};
}

[[nodiscard]] auto make_working_bin(const BinInstance &instance) -> WorkingBin {
  WorkingBin bin;
  bin.state.bin_id = instance.expanded.expanded_bin_id;
  bin.state.container = geom::normalize_polygon(instance.source->polygon);
  bin.state.container_geometry_revision = instance.source->geometry_revision;
  bin.state.start_corner = instance.source->start_corner;
  bin.state.utilization = summarize_bin(bin.state);

  for (const auto &zone : instance.source->exclusion_zones) {
    bin.exclusion_regions.push_back(
        geom::normalize_polygon(geom::PolygonWithHoles{.outer = zone.region.outer}));
  }
  return bin;
}

auto subtract_obstacle(std::vector<geom::PolygonWithHoles> &regions,
                       const geom::PolygonWithHoles &obstacle,
                       double part_spacing) -> void {
  if (part_spacing > 0.0) {
    // Inflate the obstacle by part_spacing so the resulting free regions
    // already incorporate the required gap between parts.
    const auto inflated = poly::buffer_polygon(obstacle, part_spacing);
    for (const auto &buffered : inflated) {
      subtract_region(regions, buffered);
    }
  } else {
    subtract_region(regions, obstacle);
  }
}

auto ensure_free_regions_cached(WorkingBin &bin,
                                const ExecutionPolicy &execution) -> void {
  if (bin.free_regions_valid &&
      bin.placements_at_cache == bin.state.placements.size()) {
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
  bin.cached_region_bboxes.clear();
  bin.cached_region_bboxes.reserve(regions.size());
  for (const auto &region : regions) {
    bin.cached_region_bboxes.push_back(geom::compute_bounds(region));
  }
  bin.cached_free_regions = std::move(regions);
  bin.placements_at_cache = bin.state.placements.size();
  bin.free_regions_valid = true;
}

[[nodiscard]] auto compute_hole_regions(const WorkingBin &bin,
                                        const ExecutionPolicy &execution)
    -> std::vector<geom::PolygonWithHoles> {
  if (!execution.enable_part_in_part_placement) {
    return {};
  }

  std::vector<geom::PolygonWithHoles> holes;
  for (std::size_t placement_index = 0; placement_index < bin.state.placements.size();
       ++placement_index) {
    const auto &placement = bin.state.placements[placement_index];
    for (const auto &hole_ring : placement.polygon.holes) {
      if (hole_ring.size() < 3U) {
        continue;
      }

      std::vector<geom::PolygonWithHoles> regions{
          geom::normalize_polygon(geom::PolygonWithHoles{.outer = hole_ring})};
      for (const auto &zone : bin.exclusion_regions) {
        subtract_region(regions, zone);
      }
      for (std::size_t other_index = 0; other_index < bin.state.placements.size();
           ++other_index) {
        if (other_index == placement_index) {
          continue;
        }
        subtract_region(regions,
                        fill_polygon_holes(bin.state.placements[other_index].polygon));
      }
      holes.insert(holes.end(), regions.begin(), regions.end());
    }
  }
  return holes;
}

auto refresh_bin_state(WorkingBin &bin, const ExecutionPolicy &execution) -> void {
  bin.state.holes = compute_hole_regions(bin, execution);
  ++bin.state.hole_set_revision;
  bin.state.utilization = summarize_bin(bin.state);
}

[[nodiscard]] auto piece_allows_bin(const PieceInstance &piece,
                                    const std::uint32_t bin_id) -> bool {
  return piece.allowed_expanded_bin_ids.empty() ||
         std::find(piece.allowed_expanded_bin_ids.begin(),
                   piece.allowed_expanded_bin_ids.end(),
                   bin_id) != piece.allowed_expanded_bin_ids.end();
}

[[nodiscard]] auto allowed_rotations_for(
    const PieceRequest &piece, const ExecutionPolicy &execution)
    -> const geom::DiscreteRotationSet & {
  return piece.allowed_rotations.has_value() ? *piece.allowed_rotations
                                             : execution.default_rotations;
}

[[nodiscard]] auto bounds_corner_for_start_corner(
    const geom::Box2 &bounds, const place::PlacementStartCorner start_corner)
    -> geom::Point2 {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
    return bounds.min;
  case place::PlacementStartCorner::bottom_right:
    return {.x = bounds.max.x, .y = bounds.min.y};
  case place::PlacementStartCorner::top_left:
    return {.x = bounds.min.x, .y = bounds.max.y};
  case place::PlacementStartCorner::top_right:
    return bounds.max;
  }
  return bounds.min;
}

[[nodiscard]] auto primary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
  case place::PlacementStartCorner::bottom_right:
    return piece_bounds.min.y - container_bounds.min.y;
  case place::PlacementStartCorner::top_left:
  case place::PlacementStartCorner::top_right:
    return container_bounds.max.y - piece_bounds.max.y;
  }
  return piece_bounds.min.y - container_bounds.min.y;
}

[[nodiscard]] auto secondary_edge_distance(
    const geom::Box2 &container_bounds, const geom::Box2 &piece_bounds,
    const place::PlacementStartCorner start_corner) -> double {
  switch (start_corner) {
  case place::PlacementStartCorner::bottom_left:
  case place::PlacementStartCorner::top_left:
    return piece_bounds.min.x - container_bounds.min.x;
  case place::PlacementStartCorner::bottom_right:
  case place::PlacementStartCorner::top_right:
    return container_bounds.max.x - piece_bounds.max.x;
  }
  return piece_bounds.min.x - container_bounds.min.x;
}

[[nodiscard]] auto envelope_bounds_with_candidate(const WorkingBin &bin,
                                                  const geom::Box2 &candidate_bounds)
    -> geom::Box2 {
  geom::Box2 envelope = candidate_bounds;
  for (const auto &bounds : bin.placement_bounds) {
    envelope.min.x = std::min(envelope.min.x, bounds.min.x);
    envelope.min.y = std::min(envelope.min.y, bounds.min.y);
    envelope.max.x = std::max(envelope.max.x, bounds.max.x);
    envelope.max.y = std::max(envelope.max.y, bounds.max.y);
  }
  return envelope;
}

[[nodiscard]] auto candidate_utilization(const WorkingBin &bin,
                                         const geom::PolygonWithHoles &piece_polygon,
                                         const geom::Box2 &candidate_bounds)
    -> double {
  const auto envelope = envelope_bounds_with_candidate(bin, candidate_bounds);
  const auto envelope_area =
      std::max(0.0, geom::box_width(envelope)) *
      std::max(0.0, geom::box_height(envelope));
  const auto occupied_area = bin.state.utilization.occupied_area +
                             geom::polygon_area(piece_polygon);
  return envelope_area > kAreaEpsilon ? occupied_area / envelope_area : 0.0;
}

[[nodiscard]] auto candidate_strip_length(const WorkingBin &bin,
                                          const geom::Box2 &candidate_bounds)
    -> double {
  return geom::box_width(envelope_bounds_with_candidate(bin, candidate_bounds));
}

[[nodiscard]] auto better_candidate(const WorkingBin &bin,
                                    const place::PlacementPolicy policy,
                                    const CandidatePlacement &lhs,
                                    const CandidatePlacement &rhs) -> bool {
  const auto container_bounds = geom::compute_bounds(bin.state.container);
  const auto lhs_primary =
      primary_edge_distance(container_bounds, lhs.bounds, bin.state.start_corner);
  const auto rhs_primary =
      primary_edge_distance(container_bounds, rhs.bounds, bin.state.start_corner);
  const auto lhs_secondary = secondary_edge_distance(container_bounds, lhs.bounds,
                                                     bin.state.start_corner);
  const auto rhs_secondary = secondary_edge_distance(container_bounds, rhs.bounds,
                                                     bin.state.start_corner);

  switch (policy) {
  case place::PlacementPolicy::bottom_left:
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::minimum_length:
    if (!almost_equal(lhs.score, rhs.score)) {
      return lhs.score < rhs.score;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::maximum_utilization:
    if (!almost_equal(lhs.score, rhs.score)) {
      return lhs.score > rhs.score;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  }

  if (lhs.inside_hole != rhs.inside_hole) {
    return lhs.inside_hole && !rhs.inside_hole;
  }
  if (lhs.placement.rotation_index.value != rhs.placement.rotation_index.value) {
    return lhs.placement.rotation_index.value < rhs.placement.rotation_index.value;
  }
  if (!almost_equal(lhs.placement.translation.y, rhs.placement.translation.y)) {
    return lhs.placement.translation.y < rhs.placement.translation.y;
  }
  return lhs.placement.translation.x < rhs.placement.translation.x;
}

[[nodiscard]] auto fits_region(const geom::PolygonWithHoles &piece,
                               const geom::Box2 &piece_bbox,
                               const geom::PolygonWithHoles &region,
                               const geom::Box2 &region_bbox) -> bool {
  if (!geom::box_contains(region_bbox, piece_bbox)) {
    return false;
  }
  bool all_interior = true;
  for (const auto &vertex : piece.outer) {
    const auto loc = pred::locate_point_in_polygon(vertex, region);
    if (loc.location == pred::PointLocation::exterior) {
      return false;
    }
    if (loc.location != pred::PointLocation::interior) {
      all_interior = false;
    }
  }
  if (all_interior && piece.holes.empty()) {
    return true;
  }
  const auto remainder = poly::difference_polygons(piece, region);
  return polygon_area_sum(remainder) <= kAreaEpsilon;
}

[[nodiscard]] auto fits_any_region(
    const geom::PolygonWithHoles &piece,
    const geom::Box2 &piece_bbox,
    std::span<const geom::PolygonWithHoles> regions,
    std::span<const geom::Box2> region_bboxes) -> bool {
  for (std::size_t i = 0; i < regions.size(); ++i) {
    if (fits_region(piece, piece_bbox, regions[i], region_bboxes[i])) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] auto hole_index_for_candidate(
    const geom::PolygonWithHoles &piece,
    const geom::Box2 &piece_bbox,
    std::span<const geom::PolygonWithHoles> holes) -> std::int32_t {
  for (std::size_t index = 0; index < holes.size(); ++index) {
    const auto hole_bbox = geom::compute_bounds(holes[index]);
    if (fits_region(piece, piece_bbox, holes[index], hole_bbox)) {
      return static_cast<std::int32_t>(index);
    }
  }
  return -1;
}

[[nodiscard]] auto respects_spacing(
    const geom::PolygonWithHoles &piece, const WorkingBin &bin,
    const ExecutionPolicy &execution) -> bool {
  if (execution.part_spacing <= 0.0) {
    return true;
  }

  for (const auto &zone : bin.exclusion_regions) {
    if (poly::polygon_distance(piece, zone) + kDistanceEpsilon <
        execution.part_spacing) {
      return false;
    }
  }
  for (const auto &placement : bin.state.placements) {
    if (poly::polygon_distance(piece, placement.polygon) + kDistanceEpsilon <
        execution.part_spacing) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] auto compute_candidate_score(const WorkingBin &bin,
                                           const ExecutionPolicy &execution,
                                           const geom::PolygonWithHoles &piece_polygon,
                                           const geom::Box2 &candidate_bounds)
    -> double {
  switch (execution.placement_policy) {
  case place::PlacementPolicy::bottom_left:
    return 0.0;
  case place::PlacementPolicy::minimum_length:
    return candidate_strip_length(bin, candidate_bounds);
  case place::PlacementPolicy::maximum_utilization:
    return candidate_utilization(bin, piece_polygon, candidate_bounds);
  }
  return 0.0;
}

auto append_unique_anchor(std::vector<AnchorPoint> &anchors,
                          const AnchorPoint &candidate) -> void {
  const auto duplicate = std::find_if(
      anchors.begin(), anchors.end(), [&](const AnchorPoint &existing) {
        return almost_equal(existing.point.x, candidate.point.x) &&
               almost_equal(existing.point.y, candidate.point.y) &&
               existing.source == candidate.source;
      });
  if (duplicate == anchors.end()) {
    anchors.push_back(candidate);
  }
}

auto append_ring_anchors(std::vector<AnchorPoint> &anchors,
                         std::span<const geom::Point2> ring,
                         const place::PlacementCandidateSource source) -> void {
  for (const auto &point : ring) {
    append_unique_anchor(anchors, {.point = point, .source = source});
  }
}

[[nodiscard]] auto collect_anchors(const WorkingBin &bin,
                                  double part_spacing) -> std::vector<AnchorPoint> {
  std::vector<AnchorPoint> anchors;
  append_ring_anchors(anchors, bin.state.container.outer,
                      place::PlacementCandidateSource::bin_boundary);
  for (const auto &hole : bin.state.container.holes) {
    append_ring_anchors(anchors, hole,
                        place::PlacementCandidateSource::constructive_boundary);
  }

  if (part_spacing > 0.0) {
    // When spacing is active, placed-piece and exclusion-zone vertices are
    // right on the obstacle boundary (distance 0).  Candidates aligned to
    // them always fail the spacing check.  Instead, use the free-region
    // boundary vertices — these already incorporate the spacing inflation
    // performed in ensure_free_regions_cached.
    for (const auto &region : bin.cached_free_regions) {
      append_ring_anchors(anchors, region.outer,
                          place::PlacementCandidateSource::constructive_boundary);
      for (const auto &hole : region.holes) {
        append_ring_anchors(anchors, hole,
                            place::PlacementCandidateSource::constructive_boundary);
      }
    }
  } else {
    for (const auto &zone : bin.exclusion_regions) {
      append_ring_anchors(anchors, zone.outer,
                          place::PlacementCandidateSource::constructive_boundary);
    }
    for (const auto &placement : bin.state.placements) {
      append_ring_anchors(anchors, placement.polygon.outer,
                          place::PlacementCandidateSource::constructive_boundary);
    }
  }

  for (const auto &hole : bin.state.holes) {
    append_ring_anchors(anchors, hole.outer,
                        place::PlacementCandidateSource::hole_boundary);
  }
  return anchors;
}

[[nodiscard]] auto candidate_reference_points(
    const geom::PolygonWithHoles &polygon, const geom::Box2 &bounds,
    const place::PlacementStartCorner start_corner) -> std::vector<geom::Point2> {
  std::vector<geom::Point2> points = polygon.outer;
  points.push_back(bounds_corner_for_start_corner(bounds, start_corner));
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  return points;
}

[[nodiscard]] auto build_piece_instances(const NormalizedRequest &request)
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
  for (const auto &expanded : request.expanded_pieces) {
    const auto it = pieces_by_id.find(expanded.source_piece_id);
    if (it != pieces_by_id.end()) {
      PieceInstance instance{.expanded = expanded, .source = it->second};
      for (const auto source_bin_id : it->second->allowed_bin_ids) {
        const auto bins_it = expanded_bin_ids_by_source.find(source_bin_id);
        if (bins_it == expanded_bin_ids_by_source.end()) {
          continue;
        }
        instance.allowed_expanded_bin_ids.insert(
            instance.allowed_expanded_bin_ids.end(), bins_it->second.begin(),
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
  }
  return instances;
}

[[nodiscard]] auto build_bin_instances(const NormalizedRequest &request)
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

[[nodiscard]] auto make_budget(const SolveControl &control,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch,
                               const std::size_t iterations_completed)
    -> BudgetState {
  return {
      .iteration_limit_enabled = control.iteration_limit > 0U,
      .iteration_limit = control.iteration_limit,
      .iterations_completed = iterations_completed,
      .time_limit_enabled = time_budget.enabled(),
      .time_limit_milliseconds = time_budget.limit_milliseconds(),
      .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
      .cancellation_requested = control.cancellation.stop_requested(),
  };
}

[[nodiscard]] auto build_layout(std::span<const WorkingBin> bins,
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
        .container = bin.state.container,
        .occupied = bin.state.occupied,
        .placements = bin.state.placements,
        .utilization = bin.state.utilization,
    });
  }
  return layout;
}

/// Lightweight variant that omits heavy container/occupied polygon data.
/// Used for intermediate progress snapshots where only placements and
/// utilization are needed for the live overlay.
[[nodiscard]] auto build_lightweight_layout(
    std::span<const WorkingBin> bins,
    const std::vector<PlacementTraceEntry> &trace,
    const std::vector<std::uint32_t> &unplaced) -> Layout {
  Layout layout;
  layout.placement_trace = trace;
  layout.unplaced_piece_ids = unplaced;
  layout.bins.reserve(bins.size());
  for (const auto &bin : bins) {
    layout.bins.push_back({
        .bin_id = bin.state.bin_id,
        .container = {},
        .occupied = {},
        .placements = bin.state.placements,
        .utilization = bin.state.utilization,
    });
  }
  return layout;
}

[[nodiscard]] auto compute_utilization_percent(std::span<const WorkingBin> bins)
    -> double {
  double total_occupied = 0.0;
  double total_container = 0.0;
  for (const auto &bin : bins) {
    total_occupied += bin.state.utilization.occupied_area;
    total_container += bin.state.utilization.container_area;
  }
  return total_container > 0.0 ? (total_occupied / total_container) * 100.0
                                : 0.0;
}

auto emit_progress(const SolveControl &control, const std::size_t sequence,
                   const std::size_t placed_parts, const std::size_t total_parts,
                   std::span<const WorkingBin> bins,
                   const std::vector<PlacementTraceEntry> &trace,
                   const std::vector<std::uint32_t> &unplaced,
                   const BudgetState &budget, const StopReason stop_reason,
                   const std::string &phase_detail,
                   const bool lightweight)
    -> void {
  if (!control.on_progress) {
    return;
  }

  SHINY_DEBUG("constructive emit_progress: seq={} placed={}/{} bins={} "
              "lightweight={} elapsed_ms={}",
              sequence, placed_parts, total_parts, bins.size(), lightweight,
              budget.elapsed_milliseconds);

  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placed_parts = placed_parts,
      .total_parts = total_parts,
      .layout = lightweight
                    ? build_lightweight_layout(bins, trace, unplaced)
                    : build_layout(bins, trace, unplaced),
      .budget = budget,
      .stop_reason = stop_reason,
      .phase = ProgressPhase::constructive,
      .phase_detail = phase_detail,
      .utilization_percent = compute_utilization_percent(bins),
      .improved = !lightweight,
  });
}

auto emit_search_progress(const SolveControl &control,
                          const std::size_t placed_parts,
                          const std::size_t total_parts,
                          const BudgetState &budget,
                          const std::string &phase_detail) -> void {
  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placed_parts = placed_parts,
      .total_parts = total_parts,
      .layout = {},
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::constructive,
      .phase_detail = phase_detail,
      .utilization_percent = 0.0,
      .improved = false,
  });
}

auto apply_candidate(WorkingBin &bin, const CandidatePlacement &candidate,
                     std::vector<PlacementTraceEntry> &trace,
                     const bool opened_new_bin,
                     const ExecutionPolicy &execution) -> void {
  bin.state.placements.push_back({
      .placement = candidate.placement,
      .resolved_rotation = candidate.resolved_rotation,
      .polygon = candidate.polygon,
      .source = candidate.source,
      .inside_hole = candidate.inside_hole,
      .hole_index = candidate.hole_index,
      .score = candidate.score,
  });
  bin.placement_bounds.push_back(candidate.bounds);
  if (bin.state.occupied.regions.empty()) {
    bin.state.occupied = poly::make_merged_region(candidate.polygon);
  } else {
    bin.state.occupied =
        poly::merge_polygon_into_region(bin.state.occupied, candidate.polygon);
  }
  ++bin.state.occupied_region_revision;
  refresh_bin_state(bin, execution);
  bin.free_regions_valid = false;
  trace.push_back({
      .piece_id = candidate.placement.piece_id,
      .bin_id = candidate.placement.bin_id,
      .rotation_index = candidate.placement.rotation_index,
      .resolved_rotation = candidate.resolved_rotation,
      .translation = candidate.placement.translation,
      .source = candidate.source,
      .opened_new_bin = opened_new_bin,
      .inside_hole = candidate.inside_hole,
      .hole_index = candidate.hole_index,
      .score = candidate.score,
  });
}

[[nodiscard]] auto interrupted(const SolveControl &control,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch) -> bool {
  return control.cancellation.stop_requested() || time_budget.expired(stopwatch);
}

[[nodiscard]] auto find_best_for_bin(
    WorkingBin &bin, const PieceInstance &piece,
    const NormalizedRequest &request, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch, const SolveControl &control,
    ProgressThrottle &search_throttle,
    const std::size_t placed_parts, const std::size_t total_parts,
    const std::uint64_t per_piece_budget_ms,
    runtime::DeterministicRng *rng)
    -> std::optional<CandidatePlacement> {
  if (!piece_allows_bin(piece, bin.state.bin_id)) {
    return std::nullopt;
  }

  ensure_free_regions_cached(bin, request.request.execution);
  const auto &free_regions = bin.cached_free_regions;
  if (free_regions.empty()) {
    return std::nullopt;
  }
  const auto &region_bboxes = bin.cached_region_bboxes;

  const auto anchors = collect_anchors(bin, request.request.execution.part_spacing);
  const auto &rotations = allowed_rotations_for(*piece.source, request.request.execution);
  std::optional<CandidatePlacement> best;
  std::vector<CandidatePlacement> top_k;
  const auto piece_start_ms = stopwatch.elapsed_milliseconds();
  std::size_t eval_count = 0;
  bool search_done = false;

  for (std::size_t rotation_value = 0;
       rotation_value < rotations.angles_degrees.size() && !search_done;
       ++rotation_value) {
    if (interrupted(control, time_budget, stopwatch)) {
      break;
    }

    const geom::RotationIndex rotation_index{
        static_cast<std::uint16_t>(rotation_value)};
    const auto resolved_rotation =
        geom::resolve_rotation(rotation_index, rotations);
    if (!resolved_rotation.has_value()) {
      continue;
    }

    const auto rotated_piece =
        geom::rotate_polygon(piece.source->polygon, *resolved_rotation);
    if (rotated_piece.outer.empty()) {
      continue;
    }

    const auto rotated_bounds = geom::compute_bounds(rotated_piece);
    const auto reference_points = candidate_reference_points(
        rotated_piece, rotated_bounds, bin.state.start_corner);

    for (std::size_t anchor_idx = 0;
         anchor_idx < anchors.size() && !search_done; ++anchor_idx) {
      const auto &anchor = anchors[anchor_idx];
      for (const auto &reference_point : reference_points) {
        if (++eval_count % kInterruptCheckInterval == 0) {
          if (interrupted(control, time_budget, stopwatch)) {
            search_done = true;
            break;
          }
          if (per_piece_budget_ms > 0 &&
              (stopwatch.elapsed_milliseconds() - piece_start_ms) >
                  per_piece_budget_ms) {
            SHINY_DEBUG("find_best_for_bin: per-piece budget exceeded "
                        "({}ms > {}ms limit)",
                        stopwatch.elapsed_milliseconds() - piece_start_ms,
                        per_piece_budget_ms);
            search_done = true;
            break;
          }
          if (search_throttle.should_emit()) {
            emit_search_progress(
                control, placed_parts, total_parts,
                make_budget(control, time_budget, stopwatch, 0),
                std::format("Searching rotation {}/{}, candidates evaluated: {}",
                            rotation_value + 1, rotations.angles_degrees.size(),
                            eval_count));
          }
        }

        const geom::Vector2 translation{
            .x = anchor.point.x - reference_point.x,
            .y = anchor.point.y - reference_point.y,
        };
        const geom::Box2 candidate_bbox{
            .min = {.x = rotated_bounds.min.x + translation.x,
                    .y = rotated_bounds.min.y + translation.y},
            .max = {.x = rotated_bounds.max.x + translation.x,
                    .y = rotated_bounds.max.y + translation.y},
        };
        bool any_bbox_fit = false;
        for (const auto &rbbox : region_bboxes) {
          if (geom::box_contains(rbbox, candidate_bbox)) {
            any_bbox_fit = true;
            break;
          }
        }
        if (!any_bbox_fit) {
          continue;
        }

        const auto transformed_piece =
            geom::translate_polygon(rotated_piece, translation);
        if (!fits_any_region(transformed_piece, candidate_bbox,
                             free_regions, region_bboxes) ||
            !respects_spacing(transformed_piece, bin,
                              request.request.execution)) {
          continue;
        }

        const auto hole_index =
            hole_index_for_candidate(transformed_piece, candidate_bbox,
                                     bin.state.holes);
        CandidatePlacement candidate{
            .placement =
                {
                    .piece_id = piece.expanded.expanded_piece_id,
                    .bin_id = bin.state.bin_id,
                    .rotation_index = rotation_index,
                    .translation = {.x = translation.x, .y = translation.y},
                },
            .resolved_rotation = *resolved_rotation,
            .polygon = transformed_piece,
            .bounds = candidate_bbox,
            .source = hole_index >= 0
                          ? place::PlacementCandidateSource::hole_boundary
                          : anchor.source,
            .inside_hole = hole_index >= 0,
            .hole_index = hole_index,
            .score = compute_candidate_score(bin, request.request.execution,
                                             transformed_piece, candidate_bbox),
        };

        if (rng != nullptr) {
          // Stochastic mode: maintain a top-K list of near-best candidates
          if (top_k.size() < kTopKCandidates) {
            top_k.push_back(std::move(candidate));
          } else {
            // Find the worst in top_k and replace if candidate is better
            std::size_t worst_idx = 0;
            for (std::size_t k = 1; k < top_k.size(); ++k) {
              if (better_candidate(bin, request.request.execution.placement_policy,
                                   top_k[worst_idx], top_k[k])) {
                worst_idx = k;
              }
            }
            if (better_candidate(bin, request.request.execution.placement_policy,
                                 candidate, top_k[worst_idx])) {
              top_k[worst_idx] = std::move(candidate);
            }
          }
        } else {
          // Deterministic mode: keep only the single best
          if (!best.has_value() ||
              better_candidate(bin, request.request.execution.placement_policy,
                               candidate, *best)) {
            best = std::move(candidate);
          }
        }
      }
    }
  }

  if (rng != nullptr && !top_k.empty()) {
    // Randomly select from the top-K candidates
    const std::size_t selected = rng->uniform_index(top_k.size());
    return std::move(top_k[selected]);
  }
  return best;
}

} // namespace

auto IrregularConstructivePacker::solve(const NormalizedRequest &request,
                                        const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  const auto piece_instances = build_piece_instances(request);
  const auto bin_instances = build_bin_instances(request);
  if (piece_instances.size() != request.expanded_pieces.size() ||
      bin_instances.size() != request.expanded_bins.size()) {
    return util::Status::invalid_input;
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);

  // Create RNG when seed is non-zero (stochastic mode).
  // When seed == 0, rng_ptr remains null and behavior is fully deterministic.
  std::optional<runtime::DeterministicRng> rng_storage;
  runtime::DeterministicRng *rng_ptr = nullptr;
  if (control.random_seed != 0) {
    rng_storage.emplace(control.random_seed);
    rng_ptr = &*rng_storage;
  }

  ProgressThrottle throttle;
  ProgressThrottle search_throttle;
  std::vector<WorkingBin> opened_bins;
  std::vector<bool> opened_flags(bin_instances.size(), false);
  std::vector<PlacementTraceEntry> trace;
  std::vector<std::uint32_t> unplaced_piece_ids;
  std::size_t placements_completed = 0;
  std::size_t processed_pieces = 0;
  std::size_t sequence = 0;
  StopReason stop_reason = StopReason::completed;

  for (std::size_t piece_index = 0; piece_index < piece_instances.size(); ++piece_index) {
    if (control.cancellation.stop_requested()) {
      stop_reason = StopReason::cancelled;
      break;
    }
    if (time_budget.expired(stopwatch)) {
      stop_reason = StopReason::time_limit_reached;
      break;
    }
    if (control.iteration_limit > 0U &&
        processed_pieces >= control.iteration_limit) {
      stop_reason = StopReason::iteration_limit_reached;
      break;
    }

    const auto &piece = piece_instances[piece_index];
    ++processed_pieces;

    const auto remaining_pieces = piece_instances.size() - piece_index;
    std::uint64_t per_piece_budget_ms = kDefaultPerPieceBudgetMs;
    if (time_budget.enabled() && remaining_pieces > 0) {
      const auto elapsed = stopwatch.elapsed_milliseconds();
      const auto limit = time_budget.limit_milliseconds();
      if (elapsed < limit) {
        per_piece_budget_ms =
            std::min(per_piece_budget_ms, (limit - elapsed) / remaining_pieces);
      }
    }

    std::optional<CandidatePlacement> best_existing;
    std::size_t best_existing_index = 0;
    for (std::size_t bin_index = 0; bin_index < opened_bins.size(); ++bin_index) {
      const auto candidate = find_best_for_bin(
          opened_bins[bin_index], piece, request, time_budget, stopwatch, control,
          search_throttle, placements_completed, piece_instances.size(),
          per_piece_budget_ms, rng_ptr);
      if (!candidate.has_value()) {
        continue;
      }
      if (!best_existing.has_value() ||
          better_candidate(opened_bins[best_existing_index],
                           request.request.execution.placement_policy, *candidate,
                           *best_existing)) {
        best_existing = candidate;
        best_existing_index = bin_index;
      }
    }

    if (interrupted(control, time_budget, stopwatch)) {
      stop_reason = control.cancellation.stop_requested()
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    if (best_existing.has_value()) {
      apply_candidate(opened_bins[best_existing_index], *best_existing, trace, false,
                      request.request.execution);
      ++placements_completed;
      ++sequence;
      if (throttle.should_emit()) {
        emit_progress(control, sequence, placements_completed, piece_instances.size(),
                      opened_bins, trace, unplaced_piece_ids,
                      make_budget(control, time_budget, stopwatch, processed_pieces),
                      StopReason::none,
                      std::format("Placing part {}/{}", placements_completed,
                                  piece_instances.size()),
                      true);
      }
      continue;
    }

    bool placed = false;
    for (std::size_t bin_index = 0; bin_index < bin_instances.size(); ++bin_index) {
      if (opened_flags[bin_index]) {
        continue;
      }
      auto working_bin = make_working_bin(bin_instances[bin_index]);
      refresh_bin_state(working_bin, request.request.execution);
      const auto candidate = find_best_for_bin(
          working_bin, piece, request, time_budget, stopwatch, control,
          search_throttle, placements_completed, piece_instances.size(),
          per_piece_budget_ms, rng_ptr);
      if (!candidate.has_value()) {
        continue;
      }

      apply_candidate(working_bin, *candidate, trace, true, request.request.execution);
      opened_flags[bin_index] = true;
      opened_bins.push_back(std::move(working_bin));
      placed = true;
      ++placements_completed;
      ++sequence;
      if (throttle.should_emit()) {
        emit_progress(control, sequence, placements_completed, piece_instances.size(),
                      opened_bins, trace, unplaced_piece_ids,
                      make_budget(control, time_budget, stopwatch, processed_pieces),
                      StopReason::none,
                      std::format("Placing part {}/{}", placements_completed,
                                  piece_instances.size()),
                      true);
      }
      break;
    }

    if (interrupted(control, time_budget, stopwatch)) {
      stop_reason = control.cancellation.stop_requested()
                        ? StopReason::cancelled
                        : StopReason::time_limit_reached;
      break;
    }

    if (!placed) {
      unplaced_piece_ids.push_back(piece.expanded.expanded_piece_id);
    }
  }

  if (stop_reason != StopReason::completed) {
    for (std::size_t piece_index = processed_pieces; piece_index < piece_instances.size();
         ++piece_index) {
      unplaced_piece_ids.push_back(piece_instances[piece_index].expanded.expanded_piece_id);
    }
  }

  NestingResult result{
      .strategy = StrategyKind::irregular_constructive,
      .layout = build_layout(opened_bins, trace, unplaced_piece_ids),
      .total_parts = piece_instances.size(),
      .budget = make_budget(control, time_budget, stopwatch, processed_pieces),
      .stop_reason = stop_reason,
  };

  ++sequence;
  emit_progress(control, sequence, placements_completed, piece_instances.size(),
                opened_bins, trace, unplaced_piece_ids, result.budget, stop_reason,
                std::format("Placed {}/{}", placements_completed,
                            piece_instances.size()),
                false);
  return result;
}

} // namespace shiny::nesting::pack
