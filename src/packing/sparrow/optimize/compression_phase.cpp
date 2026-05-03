#include "packing/sparrow/optimize/compression_phase.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <unordered_map>

#include "geometry/operations/merge_region.hpp"
#include "geometry/polygon.hpp"
#include "packing/separator.hpp"
#include "runtime/hash.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

namespace {

constexpr double kStripLengthTolerance = 1e-9;
constexpr double kSeparatorWidthTolerance = 1e-6;

[[nodiscard]] auto
phase_interrupted(const SolveControl &control,
                  const ::shiny::nesting::runtime::TimeBudget &time_budget,
                  const ::shiny::nesting::runtime::Stopwatch &stopwatch)
    -> bool {
  return control.cancellation.stop_requested() ||
         time_budget.expired(stopwatch);
}

[[nodiscard]] auto schedule_ratio(const std::size_t iteration,
                                  const std::size_t total_iterations,
                                  const double max_ratio,
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

[[nodiscard]] auto shrink_target(const double strip_length, const double ratio)
    -> double {
  return strip_length > 0.0 ? strip_length * (1.0 - ratio) : 0.0;
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

[[nodiscard]] auto accepted_for_target(const search::LayoutMetrics &candidate,
                                       const search::LayoutMetrics &incumbent,
                                       const double target_strip_length)
    -> bool {
  if (!search::detail::primary_metrics_preserved(candidate, incumbent)) {
    return false;
  }
  if (better_metrics(candidate, incumbent)) {
    return true;
  }
  return candidate.strip_length <= target_strip_length;
}

[[nodiscard]] auto compression_neighbor(
    std::span<const std::size_t> order, std::span<const double> piece_areas,
    const std::size_t step, ::shiny::nesting::runtime::DeterministicRng &rng)
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

[[nodiscard]] auto try_separator_compaction(
    const search::SolutionPoolEntry &entry, const NormalizedRequest &request,
    const double target_strip_length, const std::uint64_t base_seed,
    const CompressionPhaseConfig &config, SeparatorReplayMetrics *metrics)
    -> std::optional<search::SolutionPoolEntry> {
  if (entry.result.layout.bins.empty() || entry.metrics.strip_length <= 0.0 ||
      target_strip_length <= 0.0 ||
      target_strip_length >=
          entry.metrics.strip_length - kStripLengthTolerance) {
    return std::nullopt;
  }

  search::SolutionPoolEntry compacted = entry;
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
    separator_config.mover.enable_rotation_axis = false;
    const auto separator_seed = ::shiny::nesting::runtime::hash::combine(
        ::shiny::nesting::runtime::hash::combine(
            static_cast<std::size_t>(base_seed),
            static_cast<std::size_t>(bin.bin_id)),
        static_cast<std::size_t>(
            ::shiny::nesting::runtime::hash::kGoldenRatio64));
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

  for (auto &placement_trace : layout.placement_trace) {
    const auto trace_it = translations_by_piece.find(placement_trace.piece_id);
    if (trace_it != translations_by_piece.end()) {
      placement_trace.translation = trace_it->second;
    }
  }

  compacted.metrics = search::metrics_for_layout(request, layout);
  if (better_metrics(compacted.metrics, entry.metrics) ||
      (search::detail::primary_metrics_preserved(compacted.metrics,
                                                 entry.metrics) &&
       compacted.metrics.strip_length + kStripLengthTolerance <
           entry.metrics.strip_length)) {
    return compacted;
  }
  return std::nullopt;
}

} // namespace

auto run_compression_phase(
    const NormalizedRequest &request, const SolveControl &control,
    const ::shiny::nesting::runtime::TimeBudget &time_budget,
    const ::shiny::nesting::runtime::Stopwatch &stopwatch,
    const search::SolutionPoolEntry &seed, const CompressionPhaseConfig &config,
    runtime::TraceCapture *trace) -> CompressionPhaseResult {
  CompressionPhaseResult result{
      .best_solution = seed,
      .incumbent_solution = seed,
  };
  result.phase_metrics.compression_iteration_budget = config.iteration_budget;

  if (config.iteration_budget == 0U || request.expanded_pieces.size() < 2U ||
      seed.order.size() < 2U) {
    return result;
  }

  search::detail::OrderEvaluator evaluator(request, control, time_budget,
                                           stopwatch);
  ::shiny::nesting::runtime::DeterministicRng rng(
      control.random_seed ^ ::shiny::nesting::runtime::hash::kGoldenRatio64);
  std::size_t no_improvement = 0U;

  for (std::size_t iteration = 0; iteration < config.iteration_budget;
       ++iteration) {
    if (phase_interrupted(control, time_budget, stopwatch) ||
        evaluator.interrupted()) {
      break;
    }

    const double shrink_ratio =
        schedule_ratio(iteration, config.iteration_budget,
                       config.shrink_max_ratio, config.shrink_min_ratio);
    const auto order =
        compression_neighbor(result.incumbent_solution.order,
                             evaluator.piece_areas(), iteration, rng);
    auto candidate = evaluator.evaluate(
        order, result.incumbent_solution.piece_indexed_forced_rotations,
        iteration + 1U);
    if (const auto compacted = try_separator_compaction(
            candidate, request,
            shrink_target(result.incumbent_solution.metrics.strip_length,
                          shrink_ratio),
            control.random_seed, config, &result.separator_metrics);
        compacted.has_value()) {
      candidate = std::move(*compacted);
    }
    const bool accepted = accepted_for_target(
        candidate.metrics, result.incumbent_solution.metrics,
        shrink_target(result.incumbent_solution.metrics.strip_length,
                      shrink_ratio));
    const bool improved_best =
        accepted &&
        better_metrics(candidate.metrics, result.best_solution.metrics);

    result.steps.push_back({
        .iteration = iteration + 1U,
        .shrink_ratio = shrink_ratio,
        .accepted = accepted,
        .improved_best = improved_best,
    });
    if (trace != nullptr) {
      trace->compression_attempts.push_back({
          .seed = control.random_seed,
          .iteration = iteration + 1U,
          .shrink_ratio = shrink_ratio,
          .accepted = accepted,
      });
    }
    ++result.iterations;

    if (accepted) {
      result.incumbent_solution = candidate;
      ++result.accepted_moves;
      ++result.phase_metrics.accepted_moves;
      if (improved_best) {
        result.best_solution = candidate;
      }
      no_improvement = 0U;
    } else {
      ++result.phase_metrics.infeasible_candidates;
      ++no_improvement;
      if (config.plateau_limit > 0U && no_improvement >= config.plateau_limit) {
        break;
      }
    }
  }

  result.phase_metrics.compression_iterations = result.iterations;
  return result;
}

} // namespace shiny::nesting::pack::sparrow::optimize