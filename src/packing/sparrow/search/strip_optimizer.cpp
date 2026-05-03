#include "packing/sparrow/search/strip_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry/operations/merge_region.hpp"
#include "geometry/polygon.hpp"
#include "packing/irregular/constructive_packer.hpp"
#include "packing/irregular/workspace.hpp"
#include "packing/separator.hpp"
#include "runtime/hash.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"
#include "packing/sparrow/search/disruption.hpp"
#include "validation/layout_validation.hpp"

namespace shiny::nesting::search {
namespace {

constexpr double kStripLengthTolerance = 1e-9;
constexpr double kSeparatorWidthTolerance = 1e-6;
constexpr std::uint64_t kStripBudgetSafetyDivisor = 10U;

[[nodiscard]] auto
strip_budget_exhausted(const SolveControl &control,
                       const runtime::TimeBudget &time_budget,
                       const runtime::Stopwatch &stopwatch) -> bool {
  if (control.cancellation.stop_requested()) {
    return true;
  }
  if (!time_budget.enabled()) {
    return false;
  }
  if (time_budget.expired(stopwatch)) {
    return true;
  }

  const auto limit = time_budget.limit_milliseconds();
  if (limit <= 10U) {
    return false;
  }

  const auto safety_margin =
      std::max<std::uint64_t>(1U, limit / kStripBudgetSafetyDivisor);
  return stopwatch.elapsed_milliseconds() + safety_margin >= limit;
}

} // namespace

namespace detail {

auto derive_iteration_budget(const ProductionSearchConfig &config,
                             const std::size_t piece_count) -> std::size_t {
  if (config.polishing_passes == 0U || piece_count < 2U) {
    return 0U;
  }

  const auto base_budget =
      config.polishing_passes * 8U +
      std::max<std::size_t>(2U, config.diversification_swaps);
  return base_budget + piece_count;
}

auto schedule_ratio(const std::size_t iteration,
                    const std::size_t total_iterations, const double max_ratio,
                    const double min_ratio) -> double {
  const double upper = std::max(max_ratio, min_ratio);
  const double lower = std::min(max_ratio, min_ratio);
  if (total_iterations <= 1U) {
    return upper;
  }

  const auto clamped_iteration = std::min(iteration, total_iterations - 1U);
  const double progress = static_cast<double>(clamped_iteration) /
                          static_cast<double>(total_iterations - 1U);
  return upper - ((upper - lower) * progress);
}

} // namespace detail

namespace {

[[nodiscard]] auto seed_from_order(std::span<const std::size_t> order,
                                   const std::uint64_t base_seed)
    -> std::uint64_t {
  return runtime::hash::fnv1a_of<std::size_t>(order) ^ base_seed;
}

[[nodiscard]] auto reordered_request_for(std::span<const std::size_t> order,
                                         const NormalizedRequest &request)
    -> NormalizedRequest {
  NormalizedRequest reordered = request;
  reordered.expanded_pieces.clear();
  reordered.forced_rotations.clear();
  reordered.expanded_pieces.reserve(order.size());
  reordered.forced_rotations.reserve(order.size());
  for (const auto index : order) {
    reordered.expanded_pieces.push_back(request.expanded_pieces[index]);
    reordered.forced_rotations.push_back(index < request.forced_rotations.size()
                                             ? request.forced_rotations[index]
                                             : std::nullopt);
  }
  return reordered;
}

[[nodiscard]] auto evaluate_order(std::span<const std::size_t> order,
                                  const NormalizedRequest &request,
                                  const SolveControl &control,
                                  pack::PackerWorkspace &workspace,
                                  const runtime::TimeBudget &time_budget,
                                  const runtime::Stopwatch &stopwatch)
    -> SolutionPoolEntry {
  SolutionPoolEntry evaluated;
  evaluated.order.assign(order.begin(), order.end());
  if (request.forced_rotations.size() == request.expanded_pieces.size()) {
    evaluated.piece_indexed_forced_rotations = request.forced_rotations;
  } else {
    evaluated.piece_indexed_forced_rotations.assign(order.size(), std::nullopt);
  }

  pack::IrregularConstructivePacker packer;
  SolveControl decode_control{};
  decode_control.cancellation = control.cancellation;
  decode_control.random_seed = seed_from_order(order, control.random_seed);
  decode_control.workspace = &workspace;
  if (time_budget.enabled()) {
    const auto elapsed = stopwatch.elapsed_milliseconds();
    decode_control.time_limit_milliseconds =
        elapsed >= time_budget.limit_milliseconds()
            ? 1U
            : time_budget.limit_milliseconds() - elapsed;
  }

  const auto result_or =
      packer.solve(reordered_request_for(order, request), decode_control);
  if (result_or.has_value() &&
      !validation::layout_has_geometry_violation(request, result_or.value())) {
    evaluated.result = result_or.value();
    validation::finalize_result(request, evaluated.result);
    evaluated.metrics = metrics_for_layout(evaluated.result.layout);
    return evaluated;
  }

  evaluated.result.strategy = StrategyKind::metaheuristic_search;
  evaluated.result.total_parts = request.expanded_pieces.size();
  evaluated.result.stop_reason = StopReason::invalid_request;
  return evaluated;
}

[[nodiscard]] auto pool_capacity_for(const ProductionSearchConfig &config)
    -> std::size_t {
  return std::clamp<std::size_t>(config.elite_count + config.mutant_count + 2U,
                                 4U, 16U);
}

[[nodiscard]] auto primary_metrics_preserved(const LayoutMetrics &candidate,
                                             const LayoutMetrics &reference)
    -> bool {
  if (candidate.placed_parts != reference.placed_parts) {
    return candidate.placed_parts > reference.placed_parts;
  }
  if (candidate.bin_count != reference.bin_count) {
    return candidate.bin_count < reference.bin_count;
  }
  return true;
}

[[nodiscard]] auto shrink_target(const double strip_length, const double ratio)
    -> double {
  return strip_length > 0.0 ? strip_length * (1.0 - ratio) : 0.0;
}

[[nodiscard]] auto expand_target(const double target_strip_length,
                                 const double ceiling_strip_length,
                                 const double ratio) -> double {
  if (ceiling_strip_length <= 0.0) {
    return 0.0;
  }
  if (target_strip_length <= 0.0) {
    return ceiling_strip_length;
  }
  return std::min(ceiling_strip_length, target_strip_length * (1.0 + ratio));
}

[[nodiscard]] auto accepted_for_target(const LayoutMetrics &candidate,
                                       const LayoutMetrics &incumbent,
                                       const double target_strip_length)
    -> bool {
  if (!primary_metrics_preserved(candidate, incumbent)) {
    return false;
  }
  if (better_metrics(candidate, incumbent)) {
    return true;
  }
  return candidate.strip_length <= target_strip_length;
}

[[nodiscard]] auto make_strip_container(const geom::Box2 &bounds,
                                        const double target_width)
    -> geom::PolygonWithHoles {
  return geom::box_to_polygon_clamped(bounds, target_width);
}

[[nodiscard]] auto translation_delta(const geom::PolygonWithHoles &from,
                                     const geom::PolygonWithHoles &to)
    -> geom::Vector2 {
  if (from.outer().empty() || to.outer().empty()) {
    return {};
  }
  return {to.outer().front().x() - from.outer().front().x(),
          to.outer().front().y() - from.outer().front().y()};
}

auto refresh_layout_bin(pack::LayoutBin &bin) -> void {
  bin.occupied = {};
  bin.utilization.occupied_area = 0.0;
  bin.utilization.container_area = geom::polygon_area(bin.container);
  for (const auto &placement : bin.placements) {
    if (bin.occupied.regions.empty()) {
      bin.occupied = geom::make_merged_region(placement.polygon);
    } else {
      bin.occupied =
          geom::merge_polygon_into_region(bin.occupied, placement.polygon);
    }
    bin.utilization.occupied_area += geom::polygon_area(placement.polygon);
  }
  bin.utilization.bin_id = bin.bin_id;
  bin.utilization.placement_count = bin.placements.size();
  bin.utilization.utilization =
      bin.utilization.container_area > 0.0
          ? bin.utilization.occupied_area / bin.utilization.container_area
          : 0.0;
}

[[nodiscard]] auto try_separator_compaction(
    const SolutionPoolEntry &entry, const NormalizedRequest &request,
    const double target_strip_length, const std::uint64_t base_seed,
    const ProductionSearchConfig &config, SeparatorReplayMetrics *metrics)
    -> std::optional<SolutionPoolEntry> {
  if (entry.result.layout.bins.empty() || entry.metrics.strip_length <= 0.0 ||
      target_strip_length <= 0.0 ||
      target_strip_length >=
          entry.metrics.strip_length - kStripLengthTolerance) {
    return std::nullopt;
  }

  SolutionPoolEntry compacted = entry;
  auto &layout = compacted.result.layout;
  const auto target_width_scale =
      std::clamp(target_strip_length / entry.metrics.strip_length, 0.0, 1.0);
  bool moved_any = false;

  std::unordered_map<std::uint32_t, bool> rotation_locked_by_piece;
  if (entry.piece_indexed_forced_rotations.size() ==
      request.expanded_pieces.size()) {
    for (std::size_t piece_index = 0;
         piece_index < request.expanded_pieces.size(); ++piece_index) {
      rotation_locked_by_piece.emplace(
          request.expanded_pieces[piece_index].expanded_piece_id,
          entry.piece_indexed_forced_rotations[piece_index].has_value());
    }
  }

  std::unordered_map<std::uint32_t, geom::Point2> translations_by_piece;
  for (auto &bin : layout.bins) {
    if (bin.placements.size() < 2U) {
      for (const auto &placement : bin.placements) {
        translations_by_piece.emplace(placement.placement.piece_id,
                                      placement.placement.translation);
      }
      continue;
    }

    geom::Box2 envelope{};
    bool initialized = false;
    double occupied_area = 0.0;
    std::vector<pack::CollisionTrackerItem> items;
    items.reserve(bin.placements.size());
    for (const auto &placement : bin.placements) {
      const auto bounds = geom::compute_bounds(placement.polygon);
      if (!initialized) {
        envelope = bounds;
        initialized = true;
      } else {
        envelope.min.set_x(std::min(envelope.min.x(), bounds.min.x()));
        envelope.min.set_y(std::min(envelope.min.y(), bounds.min.y()));
        envelope.max.set_x(std::max(envelope.max.x(), bounds.max.x()));
        envelope.max.set_y(std::max(envelope.max.y(), bounds.max.y()));
      }
      occupied_area += geom::polygon_area(placement.polygon);
      items.push_back({
          .item_id = placement.placement.piece_id,
          .geometry_revision = placement.piece_geometry_revision,
          .polygon = placement.polygon,
          .rotation_locked =
              rotation_locked_by_piece.contains(placement.placement.piece_id) &&
              rotation_locked_by_piece.at(placement.placement.piece_id),
      });
    }

    const auto current_width = geom::box_width(envelope);
    const auto target_width = current_width * target_width_scale;
    const auto strip_height =
        std::max(geom::box_height(envelope), kStripLengthTolerance);
    const auto min_feasible_width = occupied_area / strip_height;
    if (current_width - target_width <= kSeparatorWidthTolerance ||
        target_width + kSeparatorWidthTolerance < min_feasible_width) {
      for (const auto &placement : bin.placements) {
        translations_by_piece.emplace(placement.placement.piece_id,
                                      placement.placement.translation);
      }
      continue;
    }

    pack::SeparatorConfig separator_config;
    separator_config.max_iterations = config.separator_max_iterations;
    separator_config.iter_no_improvement_limit =
        config.separator_iter_no_improvement_limit;
    separator_config.strike_limit = config.separator_strike_limit;
    separator_config.worker_count = config.separator_worker_count;
    separator_config.gls_weight_cap = config.gls_weight_cap;
    separator_config.mover.global_samples = config.separator_global_samples;
    separator_config.mover.focused_samples = config.separator_focused_samples;
    separator_config.mover.coordinate_descent_iterations =
        config.separator_coordinate_descent_iterations;
    // Strip compaction only updates placement translations today. Keep
    // continuous separator rotation gated off here until the layout/IO
    // surfaces carry post-compaction orientations end-to-end.
    separator_config.mover.enable_rotation_axis = false;
    // Mix the caller-provided base_seed (typically the search RNG seed)
    // with bin_id and a golden-ratio salt so different runs and
    // different bins explore different separator-RNG trajectories.
    // Without the base_seed, every solve() of the same input would
    // resolve overlaps along an identical descent path (finding #40).
    const auto separator_seed = runtime::hash::combine(
        runtime::hash::combine(static_cast<std::size_t>(base_seed),
                               static_cast<std::size_t>(bin.bin_id)),
        static_cast<std::size_t>(runtime::hash::kGoldenRatio64));
    const auto separated = pack::run_separator(
        make_strip_container(envelope, target_width), items, separator_config,
        static_cast<std::uint64_t>(separator_seed));
    if (metrics != nullptr) {
      ++metrics->attempts;
      metrics->total_iterations += separated.iterations;
      metrics->worker_count = config.separator_worker_count;
      metrics->max_iterations = config.separator_max_iterations;
      metrics->iter_no_improvement_limit =
          config.separator_iter_no_improvement_limit;
      metrics->strike_limit = config.separator_strike_limit;
      metrics->global_samples = config.separator_global_samples;
      metrics->focused_samples = config.separator_focused_samples;
      metrics->coordinate_descent_iterations =
          config.separator_coordinate_descent_iterations;
      if (separated.converged) {
        ++metrics->converged_runs;
      }
    }
    if (!separated.converged ||
        separated.polygons.size() != bin.placements.size()) {
      for (const auto &placement : bin.placements) {
        translations_by_piece.emplace(placement.placement.piece_id,
                                      placement.placement.translation);
      }
      continue;
    }

    for (std::size_t index = 0; index < bin.placements.size(); ++index) {
      const auto delta = translation_delta(bin.placements[index].polygon,
                                           separated.polygons[index]);
      moved_any |= std::fabs(delta.x()) > kStripLengthTolerance ||
                   std::fabs(delta.y()) > kStripLengthTolerance;
      bin.placements[index].polygon = separated.polygons[index];
      bin.placements[index].placement.translation.set_x(
          bin.placements[index].placement.translation.x() + delta.x());
      bin.placements[index].placement.translation.set_y(
          bin.placements[index].placement.translation.y() + delta.y());
      translations_by_piece[bin.placements[index].placement.piece_id] =
          bin.placements[index].placement.translation;
    }
    refresh_layout_bin(bin);
  }

  if (!moved_any) {
    return std::nullopt;
  }

  if (metrics != nullptr) {
    ++metrics->accepted_compactions;
  }

  for (auto &trace : layout.placement_trace) {
    const auto it = translations_by_piece.find(trace.piece_id);
    if (it != translations_by_piece.end()) {
      trace.translation = it->second;
    }
  }

  compacted.metrics = metrics_for_layout(layout);
  if (better_metrics(compacted.metrics, entry.metrics) ||
      (primary_metrics_preserved(compacted.metrics, entry.metrics) &&
       compacted.metrics.strip_length + kStripLengthTolerance <
           entry.metrics.strip_length)) {
    return compacted;
  }
  return std::nullopt;
}

[[nodiscard]] auto compression_neighbor(std::span<const std::size_t> order,
                                        std::span<const double> piece_areas,
                                        const std::size_t step,
                                        runtime::DeterministicRng &rng)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> neighbor(order.begin(), order.end());
  const std::vector<std::size_t> original(order.begin(), order.end());
  if (neighbor.size() < 2U || piece_areas.size() < neighbor.size()) {
    return neighbor;
  }

  std::vector<std::size_t> positions(neighbor.size());
  std::iota(positions.begin(), positions.end(), 0U);
  std::stable_sort(
      positions.begin(), positions.end(),
      [&](const std::size_t lhs, const std::size_t rhs) {
        if (piece_areas[neighbor[lhs]] != piece_areas[neighbor[rhs]]) {
          return piece_areas[neighbor[lhs]] > piece_areas[neighbor[rhs]];
        }
        return lhs < rhs;
      });

  const auto pivot = positions[step % positions.size()];
  if (pivot > 0U) {
    std::rotate(neighbor.begin() + pivot - 1U, neighbor.begin() + pivot,
                neighbor.begin() + pivot + 1U);
  } else {
    std::swap(neighbor.front(), neighbor[1]);
  }

  if (neighbor == original) {
    auto lhs = rng.uniform_index(neighbor.size());
    auto rhs = rng.uniform_index(neighbor.size());
    if (lhs == rhs) {
      rhs = (rhs + 1U) % neighbor.size();
    }
    std::swap(neighbor[lhs], neighbor[rhs]);
  }

  return neighbor;
}

} // namespace

// Sparrow §7 strip optimizer — exploration + compression two-phase
// search.
//
// Phase 1 (exploration, 80% of budget):
//   * Pull a base order from the SolutionPool (gaussian-biased).
//   * Disrupt it (large-item swap), decode via constructive packer.
//   * Optionally compact each bin via run_separator with a shrunk
//     target width (try_separator_compaction).
//   * Adaptive target length: shrink on accept, expand on reject.
//   * shrink_ratio interpolates from 25% → 2% across the phase.
//
// Phase 2 (compression, 20% of budget):
//   * Local neighbourhood (rotate adjacent piece in area-sorted
//     order); decode + compact; accept iff ≤ shrink_target.
//   * shrink_ratio interpolates from 1% → 0.1% across the phase.
//   * `derive_iteration_budget()` scales with catalogue size; wall-clock
//     budget and plateau detection can stop earlier.
auto StripOptimizer::optimize(const NormalizedRequest &request,
                              const SolveControl &control,
                              const runtime::TimeBudget &time_budget,
                              const runtime::Stopwatch &stopwatch,
                              const SolutionPoolEntry &seed,
                              const ProductionSearchConfig &config) const
    -> StripOptimizerResult {
  StripOptimizerResult result{.best_solution = seed};
  result.separator_metrics.worker_count = config.separator_worker_count;
  result.separator_metrics.max_iterations = config.separator_max_iterations;
  result.separator_metrics.iter_no_improvement_limit =
      config.separator_iter_no_improvement_limit;
  result.separator_metrics.strike_limit = config.separator_strike_limit;
  result.separator_metrics.global_samples = config.separator_global_samples;
  result.separator_metrics.focused_samples = config.separator_focused_samples;
  result.separator_metrics.coordinate_descent_iterations =
      config.separator_coordinate_descent_iterations;
  if (request.expanded_pieces.size() < 2U) {
    return result;
  }

  const auto total_iterations = std::min(
      detail::derive_iteration_budget(config, request.expanded_pieces.size()),
      std::max<std::size_t>(1U, config.max_iterations) *
          std::max<std::size_t>(1U, config.polishing_passes));
  if (total_iterations == 0U) {
    return result;
  }

  const auto piece_areas = detail::piece_areas_for(request);
  const auto plateau_window = request.request.execution.production.plateau_window;
  runtime::DeterministicRng rng(control.random_seed ^
                                runtime::hash::kGoldenRatio64);
  pack::PackerWorkspace local_workspace;
  pack::PackerWorkspace &workspace =
      control.workspace != nullptr ? *control.workspace : local_workspace;

  SolutionPool pool(pool_capacity_for(config), &request);
  pool.insert(seed);

  auto best = seed;
  auto incumbent = seed;
  result.phase_metrics.exploration_iteration_budget =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::llround(
                                    static_cast<double>(total_iterations) *
                                    config.strip_exploration_ratio)));
  result.phase_metrics.compression_iteration_budget =
      total_iterations - result.phase_metrics.exploration_iteration_budget;
  if (result.phase_metrics.compression_iteration_budget == 0U &&
      total_iterations > 1U) {
    --result.phase_metrics.exploration_iteration_budget;
    result.phase_metrics.compression_iteration_budget = 1U;
  }

  const auto exploration_limit =
      result.phase_metrics.exploration_iteration_budget;
  double target_strip_length = shrink_target(
      seed.metrics.strip_length,
      detail::schedule_ratio(0U, exploration_limit,
                             config.strip_exploration_shrink_max_ratio,
                             config.strip_exploration_shrink_min_ratio));
  SolutionPool infeasible_pool(
      std::max<std::size_t>(1U, config.infeasible_pool_capacity), &request);
  std::size_t rejected_since_rollback = 0U;

  // Single phase loop. `generate_order` produces the candidate sequence
  // for this iteration; `compaction_target` returns the strip length
  // passed to try_separator_compaction; `handle_candidate` performs the
  // phase-specific accept/reject + best/incumbent bookkeeping and
  // returns true iff the iteration counts as an improvement (resets
  // the plateau counter). The loop owns budget exhaustion, schedule
  // ratios, evaluation, compaction, the iteration counter, and plateau
  // detection.
  const auto run_phase = [&](const std::size_t operation_limit,
                             const double max_ratio, const double min_ratio,
                             std::size_t &phase_iteration_counter,
                             auto generate_order, auto compaction_target,
                             auto handle_candidate) {
    std::size_t no_improvement = 0U;
    for (std::size_t iteration = 0; iteration < operation_limit; ++iteration) {
      if (strip_budget_exhausted(control, time_budget, stopwatch)) {
        return;
      }
      const double shrink_ratio = detail::schedule_ratio(
          iteration, operation_limit, max_ratio, min_ratio);
      auto order = generate_order(iteration, shrink_ratio);
      auto candidate = evaluate_order(order, request, control, workspace,
                                      time_budget, stopwatch);
      if (const auto compacted = try_separator_compaction(
              candidate, request, compaction_target(shrink_ratio),
              control.random_seed, config, &result.separator_metrics);
          compacted.has_value()) {
        candidate = std::move(*compacted);
      }
      ++phase_iteration_counter;
      const bool improved =
          handle_candidate(std::move(candidate), shrink_ratio);
      if (improved) {
        no_improvement = 0U;
      } else {
        ++no_improvement;
        if (plateau_window > 0U && no_improvement >= plateau_window) {
          return;
        }
      }
    }
  };

  run_phase(
      exploration_limit, config.strip_exploration_shrink_max_ratio,
      config.strip_exploration_shrink_min_ratio, result.exploration_iterations,
      [&](std::size_t /*iteration*/, double /*shrink_ratio*/) {
        const auto *base = pool.select(rng);
        if (base == nullptr) {
          base = &incumbent;
        }
        return disrupt_large_items(base->order, request, piece_areas, rng)
            .order;
      },
      [&](double /*shrink_ratio*/) { return target_strip_length; },
      [&](SolutionPoolEntry candidate, double shrink_ratio) -> bool {
        pool.insert(candidate);
        result.phase_metrics.exploration_iterations =
            result.exploration_iterations;
        if (better_metrics(candidate.metrics, best.metrics)) {
          best = candidate;
          incumbent = std::move(candidate);
          ++result.accepted_moves;
          ++result.phase_metrics.accepted_moves;
          rejected_since_rollback = 0U;
          target_strip_length =
              shrink_target(best.metrics.strip_length, shrink_ratio);
          return true;
        }
        if (accepted_for_target(candidate.metrics, incumbent.metrics,
                                target_strip_length)) {
          incumbent = std::move(candidate);
          ++result.accepted_moves;
          ++result.phase_metrics.accepted_moves;
          rejected_since_rollback = 0U;
          target_strip_length =
              shrink_target(incumbent.metrics.strip_length, shrink_ratio);
          return false;
        }
        ++result.phase_metrics.infeasible_candidates;
        ++rejected_since_rollback;
        if (config.infeasible_pool_capacity > 0U) {
          infeasible_pool.insert(candidate);
        }
        if (config.infeasible_pool_capacity > 0U &&
            config.infeasible_rollback_after > 0U &&
            rejected_since_rollback >= config.infeasible_rollback_after) {
          if (const auto *rollback = infeasible_pool.select(rng);
              rollback != nullptr) {
            incumbent = *rollback;
            ++result.phase_metrics.infeasible_rollbacks;
            rejected_since_rollback = 0U;
          }
        }
        target_strip_length = expand_target(
            target_strip_length, best.metrics.strip_length, shrink_ratio * 0.5);
        return false;
      });

  incumbent = best;
  const auto compression_limit =
      result.phase_metrics.compression_iteration_budget;
  run_phase(
      compression_limit, config.strip_compression_shrink_max_ratio,
      config.strip_compression_shrink_min_ratio, result.compression_iterations,
      [&](std::size_t iteration, double /*shrink_ratio*/) {
        return compression_neighbor(incumbent.order, piece_areas, iteration,
                                    rng);
      },
      [&](double shrink_ratio) {
        return shrink_target(incumbent.metrics.strip_length, shrink_ratio);
      },
      [&](SolutionPoolEntry candidate, double shrink_ratio) -> bool {
        result.phase_metrics.compression_iterations =
            result.compression_iterations;
        if (!accepted_for_target(
                candidate.metrics, incumbent.metrics,
                shrink_target(incumbent.metrics.strip_length, shrink_ratio))) {
          ++result.phase_metrics.infeasible_candidates;
          return false;
        }
        const bool improved_best =
            better_metrics(candidate.metrics, best.metrics);
        incumbent = candidate;
        pool.insert(candidate);
        ++result.accepted_moves;
        ++result.phase_metrics.accepted_moves;
        if (improved_best) {
          best = std::move(candidate);
        }
        return improved_best;
      });

  result.best_solution = best;
  result.phase_metrics.exploration_iterations = result.exploration_iterations;
  result.phase_metrics.compression_iterations = result.compression_iterations;
  return result;
}

} // namespace shiny::nesting::search
