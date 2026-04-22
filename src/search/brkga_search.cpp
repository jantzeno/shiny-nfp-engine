#include "search/brkga_search.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry/polygon.hpp"
#include "logging/shiny_log.hpp"
#include "packing/sequential_backtrack_packer.hpp"
#include "packing/packer_workspace.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/hash.hpp"
#include "runtime/timing.hpp"
#include "search/detail/driver_scaffolding.hpp"
#include "search/detail/neighborhood_ops.hpp"
#include "search/strategy_registry.hpp"
#include "search/solution_pool.hpp"
#include "search/strip_optimizer.hpp"

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto run_brkga_strategy(const NormalizedRequest &request,
                                      const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  BrkgaProductionSearch search;
  return search.solve(request, control);
}

} // namespace

void register_brkga_strategy() {
  StrategyRegistry::instance().register_production_strategy({
      .name = "brkga",
      .kind = ProductionOptimizerKind::brkga,
      .run = &run_brkga_strategy,
  });
}

namespace {

struct EvaluatedChromosome {
  std::vector<double> genes{};
  LayoutMetrics metrics{};
  NestingResult result{};
};

[[nodiscard]] auto order_from_genes(std::span<const double> genes)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> order(genes.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (genes[lhs] != genes[rhs]) {
                       return genes[lhs] < genes[rhs];
                     }
                     return lhs < rhs;
                   });
  return order;
}

// Hash gene ordering into a unique seed for the constructive packer.
[[nodiscard]] auto seed_from_genes(std::span<const double> genes,
                                   const std::uint64_t base_seed)
    -> std::uint64_t {
  const auto order = order_from_genes(genes);
  return runtime::hash::fnv1a_of<std::size_t>(order) ^ base_seed;
}

// Signature for detecting equivalent orderings of identical pieces.
struct OrderSignature {
  std::size_t hash{0};
  bool operator==(const OrderSignature &other) const {
    return hash == other.hash;
  }
};

struct OrderSignatureHash {
  std::size_t operator()(const OrderSignature &sig) const { return sig.hash; }
};

[[nodiscard]] auto compute_order_signature(
    std::span<const double> genes, const NormalizedRequest &request)
    -> OrderSignature {
  const auto order = order_from_genes(genes);
  std::vector<std::uint32_t> canonical;
  canonical.reserve(order.size());
  for (const auto idx : order) {
    canonical.push_back(request.expanded_pieces[idx].source_piece_id);
  }
  return {.hash = runtime::hash::fnv1a_of<std::uint32_t>(canonical)};
}

[[nodiscard]] auto genes_from_order(std::span<const std::size_t> order)
    -> std::vector<double> {
  std::vector<double> genes(order.size(), 0.0);
  for (std::size_t position = 0; position < order.size(); ++position) {
    genes[order[position]] = static_cast<double>(position);
  }
  return genes;
}

[[nodiscard]] auto random_genes(const std::size_t count,
                                runtime::DeterministicRng &rng)
    -> std::vector<double> {
  std::vector<double> genes;
  genes.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    genes.push_back(rng.uniform_real());
  }
  return genes;
}

[[nodiscard]] auto reordered_request_for(std::span<const double> genes,
                                         const NormalizedRequest &request)
    -> NormalizedRequest {
  NormalizedRequest reordered = request;
  const auto order = order_from_genes(genes);
  reordered.expanded_pieces.clear();
  reordered.expanded_pieces.reserve(order.size());
  for (const auto index : order) {
    reordered.expanded_pieces.push_back(request.expanded_pieces[index]);
  }
  return reordered;
}

[[nodiscard]] auto evaluate_genes(std::span<const double> genes,
                                  const NormalizedRequest &request,
                                  const SolveControl &control,
                                  pack::PackerWorkspace &workspace,
                                  const runtime::TimeBudget &time_budget,
                                  const runtime::Stopwatch &stopwatch,
                                  ProgressThrottle &outer_throttle,
                                  const std::size_t generation,
                                  const std::size_t generation_limit,
                                  const std::size_t chromo_idx,
                                  const std::size_t pop_size)
    -> EvaluatedChromosome {
  const auto reordered = reordered_request_for(genes, request);

  pack::SequentialBacktrackPacker packer;
  SolveControl decode_control{};
  decode_control.cancellation = control.cancellation;
  decode_control.workspace = &workspace;
  // Derive a unique seed from the chromosome genes so each evaluation
  // produces a genuinely different layout via stochastic tie-breaking.
  decode_control.random_seed = seed_from_genes(genes, control.random_seed);
  if (time_budget.enabled()) {
    const auto elapsed = stopwatch.elapsed_milliseconds();
    decode_control.time_limit_milliseconds =
        elapsed >= time_budget.limit_milliseconds()
            ? 1U
            : time_budget.limit_milliseconds() - elapsed;
  }

  // Forward lightweight progress from inner constructive packer so the
  // UI sees per-part placement detail during BRKGA chromosome evaluation.
  if (control.on_progress) {
    decode_control.on_progress =
        [&control, &outer_throttle, generation, generation_limit,
         chromo_idx, pop_size](const ProgressSnapshot &inner) {
          if (!outer_throttle.should_emit()) {
            return;
          }
          control.on_progress(ProgressSnapshot{
              .sequence = 0,
              .placed_parts = inner.placed_parts,
              .total_parts = inner.total_parts,
              .layout = inner.layout,
              .budget = inner.budget,
              .stop_reason = StopReason::none,
              .phase = ProgressPhase::part_placement,
              .phase_detail = std::format(
                  "Gen {}/{} chromo {}/{}: placing {}/{}",
                  generation, generation_limit,
                  chromo_idx + 1U, pop_size,
                  inner.placed_parts, inner.total_parts),
              .utilization_percent = inner.utilization_percent,
              .improved = false,
          });
        };
  }

  const auto result_or = packer.solve(reordered, decode_control);

  EvaluatedChromosome evaluated;
  evaluated.genes.assign(genes.begin(), genes.end());
  if (result_or.ok()) {
    evaluated.result = result_or.value();
    evaluated.metrics = metrics_for_layout(evaluated.result.layout);
  } else {
    evaluated.result.strategy = StrategyKind::sequential_backtrack;
    evaluated.result.stop_reason = StopReason::invalid_request;
  }
  return evaluated;
}

[[nodiscard]] auto better_candidate(const EvaluatedChromosome &lhs,
                                    const EvaluatedChromosome &rhs) -> bool {
  return better_metrics(lhs.metrics, rhs.metrics);
}

auto emit_best_progress(const SolveControl &control, const std::size_t sequence,
                        const std::size_t iteration,
                        const EvaluatedChromosome &best,
                        const BudgetState &budget,
                        SearchReplay &search_replay,
                        const ProgressPhase phase,
                        const std::string &phase_detail) -> void {
  SHINY_DEBUG("brkga emit_best: seq={} gen={} placed={}/{} util={:.1f}%",
              sequence, iteration, best.metrics.placed_parts,
              best.result.total_parts, best.metrics.utilization * 100.0);

  search_replay.progress.push_back({
      .iteration = iteration,
      .improved = true,
      .layout = best.result.layout,
      .budget = budget,
  });

  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placed_parts = best.metrics.placed_parts,
      .total_parts = best.result.total_parts,
      .layout = best.result.layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = phase,
      .phase_detail = phase_detail,
      .utilization_percent = best.metrics.utilization * 100.0,
      .improved = true,
  });
}

auto emit_generation_progress(const SolveControl &control,
                              ProgressThrottle &throttle,
                              const std::size_t generation,
                              const std::size_t generation_limit,
                              const std::size_t best_placed_parts,
                              const std::size_t total_parts,
                              const double best_utilization,
                              const BudgetState &budget,
                              const pack::Layout &best_layout) -> void {
  if (!control.on_progress || !throttle.should_emit()) {
    return;
  }
  SHINY_DEBUG("brkga emit_gen: gen={}/{} placed={}/{} util={:.1f}%",
              generation, generation_limit, best_placed_parts, total_parts,
              best_utilization * 100.0);

  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placed_parts = best_placed_parts,
      .total_parts = total_parts,
      .layout = best_layout,
      .budget = budget,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::part_placement,
      .phase_detail = std::format("Generation {}/{}", generation, generation_limit),
      .utilization_percent = best_utilization * 100.0,
      .improved = false,
  });
}

[[nodiscard]] auto make_offspring(const std::vector<double> &elite,
                                  const std::vector<double> &other,
                                  const double elite_bias,
                                  runtime::DeterministicRng &rng)
    -> std::vector<double> {
  std::vector<double> child;
  child.reserve(elite.size());
  for (std::size_t index = 0; index < elite.size(); ++index) {
    child.push_back(rng.bernoulli(elite_bias) ? elite[index] : other[index]);
  }
  return child;
}

auto perturb_genes(std::vector<double> &genes, const std::size_t swaps,
                   runtime::DeterministicRng &rng) -> void {
  if (genes.size() < 2U) {
    return;
  }

  for (std::size_t swap_index = 0; swap_index < swaps; ++swap_index) {
    const auto lhs = rng.uniform_index(genes.size());
    auto rhs = rng.uniform_index(genes.size());
    if (lhs == rhs) {
      rhs = (rhs + 1U) % genes.size();
    }
    std::swap(genes[lhs], genes[rhs]);
  }
}

[[nodiscard]] auto polish_best(const EvaluatedChromosome &seed,
                               const NormalizedRequest &request,
                               const SolveControl &control,
                               pack::PackerWorkspace &workspace,
                               const runtime::TimeBudget &time_budget,
                               const runtime::Stopwatch &stopwatch,
                               const ProductionSearchConfig &config,
                               const std::vector<double> &piece_areas,
                               ProgressThrottle &throttle)
    -> EvaluatedChromosome {
  EvaluatedChromosome best = seed;
  auto best_order = order_from_genes(best.genes);

  for (std::size_t pass = 0; pass < config.polishing_passes; ++pass) {
    bool improved = false;
    for (std::size_t index = 0; index < best_order.size(); ++index) {
      if (detail::driver_interrupted(control, time_budget, stopwatch)) {
        return best;
      }

      std::size_t largest_index = index;
      for (std::size_t probe = index + 1U; probe < best_order.size(); ++probe) {
        if (piece_areas[best_order[probe]] > piece_areas[best_order[largest_index]]) {
          largest_index = probe;
        }
      }
      if (largest_index == index) {
        continue;
      }

      auto neighbor = best_order;
      std::rotate(neighbor.begin() + index, neighbor.begin() + largest_index,
                  neighbor.begin() + largest_index + 1U);
      auto evaluated = evaluate_genes(genes_from_order(neighbor), request, control,
                                      workspace,
                                      time_budget, stopwatch, throttle, 0, 0, 0, 0);
      if (better_candidate(evaluated, best)) {
        best = std::move(evaluated);
        best_order = order_from_genes(best.genes);
        improved = true;
      }
    }

    for (std::size_t index = 1; index < best_order.size(); ++index) {
      if (detail::driver_interrupted(control, time_budget, stopwatch)) {
        return best;
      }

      auto neighbor = best_order;
      std::swap(neighbor[index - 1U], neighbor[index]);
      auto evaluated = evaluate_genes(genes_from_order(neighbor), request, control,
                                      workspace,
                                      time_budget, stopwatch, throttle, 0, 0, 0, 0);
      if (better_candidate(evaluated, best)) {
        best = std::move(evaluated);
        best_order = order_from_genes(best.genes);
        improved = true;
      }
    }

    if (!improved) {
      break;
    }
  }

  return best;
}

} // namespace

auto BrkgaProductionSearch::solve(const NormalizedRequest &request,
                                  const SolveControl &control)
    -> util::StatusOr<NestingResult> {
  if (!request.request.is_valid()) {
    return util::Status::invalid_input;
  }

  runtime::DeterministicRng rng(control.random_seed);
  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_production_strategy_config(
      request.request.execution, ProductionOptimizerKind::brkga,
      request.request.execution.production);
  const auto piece_areas = detail::piece_areas_for(request);
  const auto generation_limit =
      control.iteration_limit > 0U ? control.iteration_limit : config.max_iterations;

  SearchReplay replay{
      .optimizer = OptimizerKind::brkga,
      .sparrow_polished = config.polishing_passes > 0U,
  };

  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::metaheuristic_search,
                                       std::move(replay), control, time_budget,
                                       stopwatch);
  }

  std::vector<std::vector<double>> population;
  population.reserve(config.population_size);

  std::vector<std::size_t> original_order(request.expanded_pieces.size());
  std::iota(original_order.begin(), original_order.end(), 0U);
  population.push_back(genes_from_order(original_order));

  auto descending_area = original_order;
  std::stable_sort(descending_area.begin(), descending_area.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (piece_areas[lhs] != piece_areas[rhs]) {
                       return piece_areas[lhs] > piece_areas[rhs];
                     }
                     return lhs < rhs;
                   });
  population.push_back(genes_from_order(descending_area));

  auto reverse_order = original_order;
  std::reverse(reverse_order.begin(), reverse_order.end());
  population.push_back(genes_from_order(reverse_order));

  while (population.size() < config.population_size) {
    population.push_back(random_genes(request.expanded_pieces.size(), rng));
  }

  std::optional<EvaluatedChromosome> best;
  std::size_t plateau_generations = 0;
  std::size_t sequence = 0;
  std::size_t iterations_completed = 0;
  bool hit_iteration_limit = false;
  ProgressThrottle throttle;
  std::unordered_set<OrderSignature, OrderSignatureHash> seen_signatures;
  pack::PackerWorkspace local_workspace;
  pack::PackerWorkspace &workspace =
      control.workspace != nullptr ? *control.workspace : local_workspace;

  for (std::size_t generation = 0; generation < generation_limit; ++generation) {
    if (detail::driver_interrupted(control, time_budget, stopwatch)) {
      break;
    }

    ++iterations_completed;
    std::vector<EvaluatedChromosome> evaluated_population;
    evaluated_population.reserve(population.size());
    for (std::size_t chromo_idx = 0; chromo_idx < population.size();
         ++chromo_idx) {
      if (detail::driver_interrupted(control, time_budget, stopwatch)) {
        break;
      }

      // Skip chromosomes that produce an ordering identical to one
      // already evaluated (identical-piece normalization).
      const auto sig = compute_order_signature(population[chromo_idx], request);
      if (seen_signatures.contains(sig)) {
        SHINY_DEBUG("brkga: skipping duplicate ordering gen={} chromo={}",
                    generation + 1U, chromo_idx);
        continue;
      }
      seen_signatures.insert(sig);

      evaluated_population.push_back(evaluate_genes(
          population[chromo_idx], request, control, workspace, time_budget, stopwatch,
          throttle, generation + 1U, generation_limit, chromo_idx,
          population.size()));

      // Emit throttled progress during evaluation so the UI shows activity
      if (control.on_progress && throttle.should_emit()) {
        const std::size_t best_placed =
            best.has_value() ? best->metrics.placed_parts : 0U;
        const std::size_t total =
            best.has_value() ? best->result.total_parts
                             : request.expanded_pieces.size();
        const double best_util =
            best.has_value() ? best->metrics.utilization : 0.0;
        control.on_progress(ProgressSnapshot{
            .sequence = 0,
            .placed_parts = best_placed,
            .total_parts = total,
            .layout = best.has_value() ? best->result.layout : pack::Layout{},
            .budget = detail::driver_make_budget(control, time_budget, stopwatch,
                                  iterations_completed),
            .stop_reason = StopReason::none,
            .phase = ProgressPhase::part_placement,
            .phase_detail =
                std::format("Evaluating {}/{} in generation {}/{}",
                            chromo_idx + 1U, population.size(),
                            generation + 1U, generation_limit),
            .utilization_percent = best_util * 100.0,
            .improved = false,
        });
      }
    }
    if (evaluated_population.empty()) {
      break;
    }

    std::sort(evaluated_population.begin(), evaluated_population.end(),
              better_candidate);

    if (!best.has_value() || better_candidate(evaluated_population.front(), *best)) {
      best = evaluated_population.front();
      plateau_generations = 0;
      ++sequence;
      emit_best_progress(control, sequence, generation + 1U, *best,
                         detail::driver_make_budget(control, time_budget, stopwatch,
                                     iterations_completed),
                         replay, ProgressPhase::part_placement,
                         std::format("Generation {}/{} (improved)",
                                     generation + 1U, generation_limit));
    } else {
      ++plateau_generations;
      emit_generation_progress(
          control, throttle, generation + 1U, generation_limit,
          best->metrics.placed_parts, best->result.total_parts,
          best->metrics.utilization,
          detail::driver_make_budget(control, time_budget, stopwatch, iterations_completed),
          best->result.layout);
    }

    if (generation + 1U >= generation_limit) {
      hit_iteration_limit = control.iteration_limit > 0U;
      break;
    }
    if (detail::driver_interrupted(control, time_budget, stopwatch)) {
      break;
    }

    std::vector<std::vector<double>> next_population;
    next_population.reserve(config.population_size);
    const auto elite_count =
        std::min(config.elite_count, evaluated_population.size());
    for (std::size_t elite_index = 0; elite_index < elite_count; ++elite_index) {
      next_population.push_back(evaluated_population[elite_index].genes);
    }

    for (std::size_t mutant_index = 0;
         mutant_index < config.mutant_count &&
         next_population.size() < config.population_size;
         ++mutant_index) {
      next_population.push_back(random_genes(request.expanded_pieces.size(), rng));
    }

    while (next_population.size() < config.population_size) {
      const auto elite_parent = evaluated_population[rng.uniform_index(elite_count)];
      const auto other_parent = evaluated_population[rng.uniform_index(
          evaluated_population.size() > elite_count
              ? evaluated_population.size() - elite_count
              : elite_count)];

      std::vector<double> child;
      if (evaluated_population.size() > elite_count) {
        child = make_offspring(
            elite_parent.genes,
            evaluated_population[elite_count +
                                 rng.uniform_index(evaluated_population.size() -
                                                   elite_count)]
                .genes,
            config.elite_bias, rng);
      } else {
        child = make_offspring(elite_parent.genes, other_parent.genes,
                               config.elite_bias, rng);
      }

      if (plateau_generations >= 2U && config.diversification_swaps > 0U) {
        perturb_genes(child, config.diversification_swaps, rng);
      }
      next_population.push_back(std::move(child));
    }

    population = std::move(next_population);
  }

  if (!best.has_value()) {
    NestingResult result{
        .strategy = StrategyKind::metaheuristic_search,
        .total_parts = request.expanded_pieces.size(),
        .budget = detail::driver_make_budget(control, time_budget, stopwatch,
                              iterations_completed),
        .stop_reason =
            detail::driver_stop_reason(control, time_budget, stopwatch, hit_iteration_limit),
        .search = std::move(replay),
    };
    return result;
  }

  if (!detail::driver_interrupted(control, time_budget, stopwatch) && !hit_iteration_limit &&
      config.polishing_passes > 0U) {
    if (control.on_progress) {
      control.on_progress(ProgressSnapshot{
          .sequence = 0,
          .placed_parts = best->metrics.placed_parts,
          .total_parts = best->result.total_parts,
          .layout = {},
          .budget = detail::driver_make_budget(control, time_budget, stopwatch, iterations_completed),
          .stop_reason = StopReason::none,
          .phase = ProgressPhase::part_refinement,
          .phase_detail = "Polishing best solution",
          .utilization_percent = best->metrics.utilization * 100.0,
          .improved = false,
      });
    }
      auto polished = polish_best(*best, request, control, workspace, time_budget,
                                  stopwatch, config, piece_areas, throttle);
    if (better_candidate(polished, *best)) {
      best = std::move(polished);
      ++sequence;
      emit_best_progress(control, sequence, iterations_completed, *best,
                         detail::driver_make_budget(control, time_budget, stopwatch,
                                     iterations_completed),
                         replay, ProgressPhase::part_refinement,
                         "Polishing improved solution");
    }

    if (!detail::driver_interrupted(control, time_budget, stopwatch)) {
      StripOptimizer strip_optimizer;
      const auto optimized = strip_optimizer.optimize(
          request, control, time_budget, stopwatch,
          SolutionPoolEntry{
              .order = order_from_genes(best->genes),
              .metrics = best->metrics,
              .result = best->result,
          },
          config);
      if (better_metrics(optimized.best_solution.metrics, best->metrics)) {
        best->genes = genes_from_order(optimized.best_solution.order);
        best->metrics = optimized.best_solution.metrics;
        best->result = optimized.best_solution.result;
        ++sequence;
        emit_best_progress(control, sequence, iterations_completed, *best,
                           detail::driver_make_budget(control, time_budget, stopwatch,
                                       iterations_completed),
                           replay, ProgressPhase::part_refinement,
                           "Strip optimization improved solution");
      }
    }
  }

  NestingResult result = best->result;
  result.strategy = StrategyKind::metaheuristic_search;
  result.budget = detail::driver_make_budget(control, time_budget, stopwatch, iterations_completed);
  result.stop_reason =
      detail::driver_stop_reason(control, time_budget, stopwatch, hit_iteration_limit);
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
