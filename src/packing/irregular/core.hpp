#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "cache/nfp_cache.hpp"
#include "geometry/rotation_refinement.hpp"
#include "observer.hpp"
#include "packing/bin_state.hpp"
#include "packing/layout.hpp"
#include "request.hpp"
#include "result.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/hash.hpp"
#include "runtime/timing.hpp"
#include "solve.hpp"
#include "util/status.hpp"

namespace shiny::nesting::pack::detail {

constexpr double kAreaEpsilon = 1e-6;
constexpr double kDistanceEpsilon = 1e-8;
constexpr std::size_t kInterruptCheckInterval = 256;
constexpr std::uint64_t kDefaultPerPieceBudgetMs = 30'000;
constexpr std::size_t kTopKCandidates = 3;

struct PieceInstance {
  ExpandedPieceInstance expanded{};
  const PieceRequest *source{nullptr};
  std::vector<std::uint32_t> allowed_expanded_bin_ids{};
  bool restricted_to_allowed_bins{false};
  std::optional<geom::RotationIndex> forced_rotation_index{};
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
  std::uint64_t piece_geometry_revision{0};
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

struct TrialStateRecorder {
  std::function<void(std::size_t)> snapshot_bin{};
  std::function<void(std::size_t)> record_opened_bin{};
  std::function<void(std::size_t, const PlacementTraceEntry &)>
      record_removed_trace_entry{};
  std::function<void()> record_appended_trace_entry{};
};

[[nodiscard]] inline auto almost_equal(const double lhs, const double rhs)
    -> bool {
  return std::fabs(lhs - rhs) <= kDistanceEpsilon;
}

[[nodiscard]] inline auto
effective_geometry_revision(const std::uint64_t revision, const bool mirrored)
    -> std::uint64_t {
  return mirrored ? (revision ^ runtime::hash::kGoldenRatio64) : revision;
}

[[nodiscard]] inline auto piece_allows_bin(const PieceInstance &piece,
                                           const std::uint32_t bin_id) -> bool {
  return (!piece.restricted_to_allowed_bins &&
          piece.allowed_expanded_bin_ids.empty()) ||
         std::find(piece.allowed_expanded_bin_ids.begin(),
                   piece.allowed_expanded_bin_ids.end(),
                   bin_id) != piece.allowed_expanded_bin_ids.end();
}

[[nodiscard]] inline auto
allowed_rotations_for(const PieceRequest &piece,
                      const ExecutionPolicy &execution)
    -> const geom::DiscreteRotationSet & {
  return piece.allowed_rotations.has_value() ? *piece.allowed_rotations
                                             : execution.default_rotations;
}

[[nodiscard]] auto fill_polygon_holes(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles;
[[nodiscard]] auto make_working_bin(const BinInstance &instance) -> WorkingBin;
auto ensure_free_regions_cached(WorkingBin &bin,
                                const ExecutionPolicy &execution) -> void;
auto refresh_bin_state(WorkingBin &bin, const ExecutionPolicy &execution)
    -> void;
auto rebuild_working_bin(WorkingBin &bin, const ExecutionPolicy &execution)
    -> void;
auto remove_trace_entries_for_piece(
    std::vector<PlacementTraceEntry> &trace, std::uint32_t piece_id,
    const TrialStateRecorder *trial_recorder = nullptr) -> void;
auto apply_candidate(WorkingBin &bin, const CandidatePlacement &candidate,
                     std::vector<PlacementTraceEntry> &trace,
                     bool opened_new_bin, const ExecutionPolicy &execution,
                     const TrialStateRecorder *trial_recorder = nullptr)
    -> void;
auto remove_piece_from_bins(std::vector<WorkingBin> &bins,
                            std::uint32_t piece_id,
                            const ExecutionPolicy &execution,
                            const TrialStateRecorder *trial_recorder = nullptr)
    -> bool;
[[nodiscard]] auto
collect_recent_piece_ids(const std::vector<PlacementTraceEntry> &trace,
                         std::size_t max_count) -> std::vector<std::uint32_t>;

[[nodiscard]] auto better_candidate(const WorkingBin &bin,
                                    place::PlacementPolicy policy,
                                    const CandidatePlacement &lhs,
                                    const CandidatePlacement &rhs) -> bool;
[[nodiscard]] auto
fits_any_region(const geom::PolygonWithHoles &piece,
                const geom::Box2 &piece_bbox,
                std::span<const geom::PolygonWithHoles> regions,
                std::span<const geom::Box2> region_bboxes) -> bool;
[[nodiscard]] auto fits_bin_direct(const geom::PolygonWithHoles &piece,
                                   const geom::Box2 &piece_bbox,
                                   const WorkingBin &bin,
                                   const ExecutionPolicy &execution) -> bool;
[[nodiscard]] auto hole_index_for_candidate(
    const geom::PolygonWithHoles &piece, const geom::Box2 &piece_bbox,
    std::span<const geom::PolygonWithHoles> holes) -> std::int32_t;
[[nodiscard]] auto respects_spacing(const geom::PolygonWithHoles &piece,
                                    const WorkingBin &bin,
                                    const ExecutionPolicy &execution) -> bool;
[[nodiscard]] auto
compute_candidate_score(const WorkingBin &bin, const ExecutionPolicy &execution,
                        const geom::PolygonWithHoles &piece_polygon,
                        const geom::Box2 &candidate_bounds) -> double;

enum class PlacementSearchStatus : std::uint8_t {
  found = 0,
  no_candidate = 1,
  placement_budget_exhausted = 2,
  interrupted = 3,
};

struct PlacementAttemptContext {
  std::uint64_t started_milliseconds{0};
  std::uint64_t budget_milliseconds{0};
  std::size_t candidate_evaluations_completed{0};
};

struct PlacementSearchResult {
  PlacementSearchStatus status{PlacementSearchStatus::no_candidate};
  std::optional<CandidatePlacement> candidate{};
};

[[nodiscard]] auto find_best_for_bin(
    WorkingBin &bin, const PieceInstance &piece,
    const NormalizedRequest &request, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch, const SolveControl &control,
    ProgressThrottle &search_throttle, std::size_t placed_parts,
    std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, runtime::DeterministicRng *rng)
    -> PlacementSearchResult;

[[nodiscard]] auto try_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    std::span<const BinInstance> bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace,
    const runtime::TimeBudget &time_budget, const runtime::Stopwatch &stopwatch,
    const SolveControl &control, ProgressThrottle &search_throttle,
    std::size_t placed_parts, std::size_t total_parts,
    PlacementAttemptContext &attempt_context, cache::NfpCache *nfp_cache,
    runtime::DeterministicRng *rng_ptr,
    const TrialStateRecorder *trial_recorder = nullptr)
    -> PlacementSearchStatus;

[[nodiscard]] auto try_backtrack_place_piece(
    const PieceInstance &piece, const NormalizedRequest &request,
    std::span<const BinInstance> bin_instances,
    std::vector<WorkingBin> &opened_bins, std::vector<bool> &opened_flags,
    std::vector<PlacementTraceEntry> &trace,
    const std::unordered_map<std::uint32_t, const PieceInstance *> &piece_by_id,
    const runtime::TimeBudget &time_budget, const runtime::Stopwatch &stopwatch,
    const SolveControl &control, ProgressThrottle &search_throttle,
    std::size_t total_parts, PlacementAttemptContext &attempt_context,
    cache::NfpCache *nfp_cache, runtime::DeterministicRng *rng_ptr,
    std::uint32_t max_backtrack_pieces) -> PlacementSearchStatus;

[[nodiscard]] auto build_layout(std::span<const WorkingBin> bins,
                                const std::vector<PlacementTraceEntry> &trace,
                                const std::vector<std::uint32_t> &unplaced)
    -> Layout;
[[nodiscard]] auto
build_lightweight_layout(std::span<const WorkingBin> bins,
                         const std::vector<PlacementTraceEntry> &trace,
                         const std::vector<std::uint32_t> &unplaced) -> Layout;
[[nodiscard]] auto compute_utilization_percent(std::span<const WorkingBin> bins)
    -> double;

[[nodiscard]] auto make_budget(const SolveControl &control,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch,
                               std::size_t operations_completed) -> BudgetState;
auto emit_progress(const SolveControl &control, std::size_t sequence,
                   std::size_t placement_attempts_completed,
                   std::size_t placements_successful,
                   std::size_t total_requested_parts,
                   std::size_t candidate_evaluations_completed,
                   std::span<const WorkingBin> bins,
                   const std::vector<PlacementTraceEntry> &trace,
                   const std::vector<std::uint32_t> &unplaced,
                   const BudgetState &budget, StopReason stop_reason,
                   const std::string &phase_detail, bool lightweight) -> void;
auto emit_search_progress(const SolveControl &control,
                          std::size_t placements_successful,
                          std::size_t total_requested_parts,
                          std::size_t candidate_evaluations_completed,
                          const BudgetState &budget,
                          const std::string &phase_detail) -> void;

[[nodiscard]] auto interrupted(const SolveControl &control,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch) -> bool;

[[nodiscard]] auto build_piece_instances(const NormalizedRequest &request)
    -> std::vector<PieceInstance>;
[[nodiscard]] auto build_bin_instances(const NormalizedRequest &request)
    -> std::vector<BinInstance>;
[[nodiscard]] auto order_piece_instances(std::vector<PieceInstance> pieces,
                                         const ExecutionPolicy &execution)
    -> std::vector<PieceInstance>;

} // namespace shiny::nesting::pack::detail
