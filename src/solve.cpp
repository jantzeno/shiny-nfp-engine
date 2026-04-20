#include "solve.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <optional>
#include <utility>

#include "packing/bounding_box_packer.hpp"
#include "packing/irregular_constructive_packer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/brkga_search.hpp"

namespace shiny::nesting {
namespace {

[[nodiscard]] auto compute_efficiency_percent(const pack::Layout &layout) -> double {
  double total_occupied_area = 0.0;
  double total_container_area = 0.0;
  for (const auto &bin : layout.bins) {
    total_occupied_area += bin.utilization.occupied_area;
    total_container_area += bin.utilization.container_area;
  }
  if (total_container_area <= 0.0) {
    return 0.0;
  }
  return (total_occupied_area / total_container_area) * 100.0;
}

[[nodiscard]] auto bounding_box_result_better(const pack::DecoderResult &lhs,
                                              const pack::DecoderResult &rhs)
    -> bool {
  const std::size_t lhs_placed = lhs.layout.placement_trace.size();
  const std::size_t rhs_placed = rhs.layout.placement_trace.size();
  if (lhs_placed != rhs_placed) {
    return lhs_placed > rhs_placed;
  }

  const double lhs_efficiency = compute_efficiency_percent(lhs.layout);
  const double rhs_efficiency = compute_efficiency_percent(rhs.layout);
  if (std::abs(lhs_efficiency - rhs_efficiency) > 1e-9) {
    return lhs_efficiency > rhs_efficiency;
  }

  const bool lhs_completed = !lhs.interrupted;
  const bool rhs_completed = !rhs.interrupted;
  if (lhs_completed != rhs_completed) {
    return lhs_completed;
  }

  return lhs.layout.bins.size() < rhs.layout.bins.size();
}

[[nodiscard]] auto
best_bounding_box_attempt_index(const std::vector<pack::DecoderResult> &results)
    -> std::optional<std::size_t> {
  if (results.empty()) {
    return std::nullopt;
  }

  std::size_t best_index = 0;
  for (std::size_t index = 1; index < results.size(); ++index) {
    if (bounding_box_result_better(results[index], results[best_index])) {
      best_index = index;
    }
  }
  return best_index;
}

auto emit_snapshot(const SolveControl &control, const ProgressSnapshot &snapshot)
    -> void {
  if (!control.on_progress) {
    return;
  }

  control.on_progress(snapshot);
}

[[nodiscard]] auto stop_reason_for_bounding_box(
    const SolveControl &control, const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch, const bool interrupted,
    const bool hit_iteration_limit) -> StopReason {
  if (control.cancellation.stop_requested()) {
    return StopReason::cancelled;
  }
  if (time_budget.expired(stopwatch)) {
    return StopReason::time_limit_reached;
  }
  if (hit_iteration_limit) {
    return StopReason::iteration_limit_reached;
  }
  if (!interrupted) {
    return StopReason::completed;
  }
  return StopReason::cancelled;
}

[[nodiscard]] auto constructive_result_better(const NestingResult &lhs,
                                              const NestingResult &rhs) -> bool {
  const std::size_t lhs_placed = lhs.layout.placement_trace.size();
  const std::size_t rhs_placed = rhs.layout.placement_trace.size();
  if (lhs_placed != rhs_placed) {
    return lhs_placed > rhs_placed;
  }

  const double lhs_efficiency = compute_efficiency_percent(lhs.layout);
  const double rhs_efficiency = compute_efficiency_percent(rhs.layout);
  if (std::abs(lhs_efficiency - rhs_efficiency) > 1e-9) {
    return lhs_efficiency > rhs_efficiency;
  }

  return lhs.layout.bins.size() < rhs.layout.bins.size();
}

[[nodiscard]] auto derive_seed(const std::uint64_t base_seed,
                               const std::size_t iteration,
                               const SeedProgressionMode mode,
                               runtime::DeterministicRng &meta_rng)
    -> std::uint64_t {
  switch (mode) {
  case SeedProgressionMode::increment:
    return base_seed + iteration;
  case SeedProgressionMode::decrement:
    return base_seed - iteration;
  case SeedProgressionMode::random:
    return meta_rng.next_u64();
  }
  return base_seed + iteration;
}

} // namespace

auto solve(const NestingRequest &request, const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  auto normalized_request = normalize_request(request);
  if (!normalized_request.ok()) {
    return normalized_request.status();
  }

  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);

  switch (request.execution.strategy) {
  case StrategyKind::bounding_box: {
    auto decoder_request =
        to_bounding_box_decoder_request(normalized_request.value());
    if (!decoder_request.ok()) {
      return decoder_request.status();
    }

    const std::uint32_t configured_attempts =
        decoder_request.value().config.deterministic_attempts.max_attempts;
    if (control.iteration_limit > 0U) {
      decoder_request.value().config.deterministic_attempts.max_attempts =
          static_cast<std::uint32_t>(
              std::min<std::size_t>(control.iteration_limit, configured_attempts));
    }

    pack::BoundingBoxPacker packer;
    const auto interruption_requested = [&]() {
      return control.cancellation.stop_requested() ||
             time_budget.expired(stopwatch);
    };
    std::vector<pack::DecoderResult> results = packer.decode_attempts(
        decoder_request.value(), interruption_requested,
        [&](const std::size_t attempt_index, const pack::DecoderResult &result) {
          const BudgetState budget{
              .iteration_limit_enabled = control.iteration_limit > 0U,
              .iteration_limit = control.iteration_limit,
              .iterations_completed = attempt_index + 1U,
              .time_limit_enabled = time_budget.enabled(),
              .time_limit_milliseconds = time_budget.limit_milliseconds(),
              .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
              .cancellation_requested = control.cancellation.stop_requested(),
          };
          emit_snapshot(control, ProgressSnapshot{
                                    .sequence = attempt_index + 1U,
                                    .placed_parts =
                                        result.layout.placement_trace.size(),
                                    .total_parts =
                                        normalized_request.value()
                                            .expanded_pieces.size(),
                                    .layout = result.layout,
                                    .budget = budget,
                                    .stop_reason = StopReason::none,
                                });
        });
    const auto best_attempt_index = best_bounding_box_attempt_index(results);
    if (!best_attempt_index.has_value()) {
      return util::Status::invalid_input;
    }

    const bool hit_iteration_limit =
        control.iteration_limit > 0U &&
        decoder_request.value().config.deterministic_attempts.max_attempts <
            configured_attempts &&
        results.size() >=
            decoder_request.value().config.deterministic_attempts.max_attempts &&
        !results.back().interrupted && !control.cancellation.stop_requested() &&
        !time_budget.expired(stopwatch);
    const auto &best_result = results[*best_attempt_index];

    NestingResult result{
        .strategy = StrategyKind::bounding_box,
        .layout = best_result.layout,
        .total_parts = normalized_request.value().expanded_pieces.size(),
        .budget =
            {
                .iteration_limit_enabled = control.iteration_limit > 0U,
                .iteration_limit = control.iteration_limit,
                .iterations_completed = results.size(),
                .time_limit_enabled = time_budget.enabled(),
                .time_limit_milliseconds = time_budget.limit_milliseconds(),
                .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
                .cancellation_requested = control.cancellation.stop_requested(),
            },
        .stop_reason = stop_reason_for_bounding_box(
            control, time_budget, stopwatch, best_result.interrupted,
            hit_iteration_limit),
    };
    return result;
  }
  case StrategyKind::irregular_constructive: {
    // Multi-start constructive: run the packer multiple times with different
    // seeds, keeping the best result.  Activates when the caller provides a
    // non-zero seed and doesn't explicitly cap iterations to 1.  The loop
    // checks cancellation and time budget on every iteration, so the caller
    // is responsible for providing at least one stopping mechanism.
    const bool multi_start =
        control.random_seed != 0 &&
        (control.iteration_limit == 0 || control.iteration_limit > 1);

    if (!multi_start) {
      pack::IrregularConstructivePacker packer;
      return packer.solve(normalized_request.value(), control);
    }

    runtime::DeterministicRng meta_rng(control.random_seed);
    const std::size_t max_iterations =
        control.iteration_limit > 0 ? control.iteration_limit
                                    : std::numeric_limits<std::size_t>::max();
    std::optional<NestingResult> best_result;
    std::size_t iterations_completed = 0;
    StopReason final_stop_reason = StopReason::completed;

    for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
      if (control.cancellation.stop_requested()) {
        final_stop_reason = StopReason::cancelled;
        break;
      }
      if (time_budget.expired(stopwatch)) {
        final_stop_reason = StopReason::time_limit_reached;
        break;
      }

      const std::uint64_t iter_seed =
          derive_seed(control.random_seed, iteration, control.seed_mode, meta_rng);

      // Build per-iteration control: no iteration_limit on pieces, but
      // respect time budget (remaining time only).
      SolveControl iter_control{};
      iter_control.cancellation = control.cancellation;
      iter_control.random_seed = iter_seed;
      if (time_budget.enabled()) {
        const auto elapsed = stopwatch.elapsed_milliseconds();
        iter_control.time_limit_milliseconds =
            elapsed >= time_budget.limit_milliseconds()
                ? 1U
                : time_budget.limit_milliseconds() - elapsed;
      }

      // Forward progress but tag it with the multi-start sequence
      iter_control.on_progress =
          [&control, iteration](const ProgressSnapshot &inner) {
            if (!control.on_progress) {
              return;
            }
            control.on_progress(ProgressSnapshot{
                .sequence = iteration + 1U,
                .placed_parts = inner.placed_parts,
                .total_parts = inner.total_parts,
                .layout = inner.layout,
                .budget = inner.budget,
                .stop_reason = inner.stop_reason,
                .phase = ProgressPhase::constructive,
                .phase_detail =
                    std::format("Multi-start iteration {}: placing {}/{}",
                                iteration + 1U, inner.placed_parts,
                                inner.total_parts),
                .utilization_percent = inner.utilization_percent,
                .improved = inner.improved,
            });
          };

      pack::IrregularConstructivePacker packer;
      auto iter_result = packer.solve(normalized_request.value(), iter_control);
      ++iterations_completed;

      if (!iter_result.ok()) {
        continue;
      }

      const bool improved = !best_result.has_value() ||
                            constructive_result_better(iter_result.value(),
                                                      *best_result);
      if (improved) {
        best_result = std::move(iter_result.value());

        // Emit improved snapshot
        if (control.on_progress) {
          const BudgetState budget{
              .iteration_limit_enabled = control.iteration_limit > 0U,
              .iteration_limit = control.iteration_limit,
              .iterations_completed = iterations_completed,
              .time_limit_enabled = time_budget.enabled(),
              .time_limit_milliseconds = time_budget.limit_milliseconds(),
              .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
              .cancellation_requested = control.cancellation.stop_requested(),
          };
          control.on_progress(ProgressSnapshot{
              .sequence = iterations_completed,
              .placed_parts = best_result->layout.placement_trace.size(),
              .total_parts = best_result->total_parts,
              .layout = best_result->layout,
              .budget = budget,
              .stop_reason = StopReason::none,
              .phase = ProgressPhase::constructive,
              .phase_detail =
                  std::format("Multi-start iteration {} (improved)",
                              iterations_completed),
              .utilization_percent =
                  compute_efficiency_percent(best_result->layout),
              .improved = true,
          });
        }
      }
    }

    if (control.iteration_limit > 0 &&
        iterations_completed >= control.iteration_limit &&
        final_stop_reason == StopReason::completed) {
      final_stop_reason = StopReason::iteration_limit_reached;
    }

    if (!best_result.has_value()) {
      return util::Status::invalid_input;
    }

    best_result->budget = {
        .iteration_limit_enabled = control.iteration_limit > 0U,
        .iteration_limit = control.iteration_limit,
        .iterations_completed = iterations_completed,
        .time_limit_enabled = time_budget.enabled(),
        .time_limit_milliseconds = time_budget.limit_milliseconds(),
        .elapsed_milliseconds = stopwatch.elapsed_milliseconds(),
        .cancellation_requested = control.cancellation.stop_requested(),
    };
    best_result->stop_reason = final_stop_reason;
    return *best_result;
  }
  case StrategyKind::irregular_production: {
    search::BrkgaProductionSearch search;
    return search.solve(normalized_request.value(), control);
  }
  }

  return util::Status::invalid_input;
}

} // namespace shiny::nesting
