#include "packing/sparrow/search/brkga_search.hpp"

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

#include "logging/shiny_log.hpp"
#include "packing/irregular/constructive_packer.hpp"
#include "packing/irregular/workspace.hpp"
#include "packing/sparrow/search/detail/driver_scaffolding.hpp"
#include "packing/sparrow/search/detail/neighborhood_search.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "packing/sparrow/search/strip_optimizer.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/hash.hpp"
#include "runtime/timing.hpp"
#include "validation/layout_validation.hpp"

namespace shiny::nesting::search {
namespace {

struct EvaluatedChromosome {
  std::vector<double> genes{};
  LayoutMetrics metrics{};
  NestingResult result{};
  std::vector<std::optional<geom::RotationIndex>> forced_rotations{};
  bool valid{false};
};

constexpr std::size_t kGenesPerPiece = 2U;

[[nodiscard]] auto order_gene(const std::span<const double> genes,
                              const std::size_t piece_index) -> double {
  return genes[piece_index * kGenesPerPiece];
}

[[nodiscard]] auto rotation_gene(const std::span<const double> genes,
                                 const std::size_t piece_index) -> double {
  return genes[piece_index * kGenesPerPiece + 1U];
}

[[nodiscard]] auto piece_rotation_counts(const NormalizedRequest &request)
    -> std::vector<std::size_t> {
  std::unordered_map<std::uint32_t, const PieceRequest *> source_piece_by_id;
  source_piece_by_id.reserve(request.request.pieces.size());
  for (const auto &piece : request.request.pieces) {
    source_piece_by_id.emplace(piece.piece_id, &piece);
  }

  std::vector<std::size_t> counts;
  counts.reserve(request.expanded_pieces.size());
  for (const auto &expanded_piece : request.expanded_pieces) {
    const auto piece_it =
        source_piece_by_id.find(expanded_piece.source_piece_id);
    if (piece_it == source_piece_by_id.end()) {
      counts.push_back(1U);
      continue;
    }
    const auto &rotations = piece_it->second->allowed_rotations.has_value()
                                ? *piece_it->second->allowed_rotations
                                : request.request.execution.default_rotations;
    counts.push_back(
        std::max<std::size_t>(1U, geom::rotation_count(rotations)));
  }
  return counts;
}

[[nodiscard]] auto order_from_genes(std::span<const double> genes)
    -> std::vector<std::size_t> {
  const auto piece_count = genes.size() / kGenesPerPiece;
  std::vector<std::size_t> order(piece_count);
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(order.begin(), order.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (order_gene(genes, lhs) != order_gene(genes, rhs)) {
                       return order_gene(genes, lhs) < order_gene(genes, rhs);
                     }
                     return lhs < rhs;
                   });
  return order;
}

[[nodiscard]] auto
forced_rotations_from_genes(std::span<const double> genes,
                            const NormalizedRequest &request,
                            std::span<const std::size_t> rotation_counts)
    -> std::vector<std::optional<geom::RotationIndex>> {
  std::vector<std::optional<geom::RotationIndex>> forced_rotations;
  forced_rotations.reserve(request.expanded_pieces.size());
  for (std::size_t piece_index = 0;
       piece_index < request.expanded_pieces.size(); ++piece_index) {
    if (piece_index < request.forced_rotations.size() &&
        request.forced_rotations[piece_index].has_value()) {
      forced_rotations.push_back(request.forced_rotations[piece_index]);
      continue;
    }
    const auto rotation_count = piece_index < rotation_counts.size()
                                    ? rotation_counts[piece_index]
                                    : 1U;
    if (rotation_count <= 1U) {
      forced_rotations.push_back(std::nullopt);
      continue;
    }
    if (rotation_gene(genes, piece_index) <= 0.0) {
      forced_rotations.push_back(std::nullopt);
      continue;
    }
    const auto clamped_gene =
        std::clamp(rotation_gene(genes, piece_index), 0.0, 0.999999999999);
    const auto rotation_slot = std::min<std::size_t>(
        rotation_count - 1U,
        static_cast<std::size_t>(clamped_gene * rotation_count));
    forced_rotations.push_back(static_cast<geom::RotationIndex>(rotation_slot));
  }
  return forced_rotations;
}

// Hash gene ordering into a unique seed for the constructive packer.
[[nodiscard]] auto seed_from_genes(std::span<const double> genes,
                                   std::span<const std::size_t> rotation_counts,
                                   const NormalizedRequest &request,
                                   const std::uint64_t base_seed)
    -> std::uint64_t {
  const auto order = order_from_genes(genes);
  const auto forced_rotations =
      forced_rotations_from_genes(genes, request, rotation_counts);
  std::vector<std::size_t> signature;
  signature.reserve(order.size() * 2U);
  for (std::size_t position = 0; position < order.size(); ++position) {
    const auto piece_index = order[position];
    signature.push_back(piece_index);
    signature.push_back(piece_index < forced_rotations.size() &&
                                forced_rotations[piece_index].has_value()
                            ? static_cast<std::size_t>(
                                  forced_rotations[piece_index]->value + 1U)
                            : 0U);
  }
  return runtime::hash::fnv1a_of<std::size_t>(signature) ^ base_seed;
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
    std::span<const double> genes, const NormalizedRequest &request,
    std::span<const std::size_t> rotation_counts) -> OrderSignature {
  const auto order = order_from_genes(genes);
  const auto forced_rotations =
      forced_rotations_from_genes(genes, request, rotation_counts);
  std::vector<std::uint32_t> canonical;
  canonical.reserve(order.size() * 2U);
  for (std::size_t position = 0; position < order.size(); ++position) {
    const auto idx = order[position];
    canonical.push_back(request.expanded_pieces[idx].source_piece_id);
    canonical.push_back(
        idx < forced_rotations.size() && forced_rotations[idx].has_value()
            ? static_cast<std::uint32_t>(forced_rotations[idx]->value + 1U)
            : 0U);
  }
  return {.hash = runtime::hash::fnv1a_of<std::uint32_t>(canonical)};
}

[[nodiscard]] auto genes_from_solution(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    std::span<const std::size_t> rotation_counts) -> std::vector<double> {
  std::vector<double> genes(order.size() * kGenesPerPiece, 0.0);
  for (std::size_t position = 0; position < order.size(); ++position) {
    const auto piece_index = order[position];
    genes[piece_index * kGenesPerPiece] = static_cast<double>(position);
    const auto rotation_count = piece_index < rotation_counts.size()
                                    ? rotation_counts[piece_index]
                                    : 1U;
    if (piece_index < forced_rotations.size() &&
        forced_rotations[piece_index] && rotation_count > 1U) {
      genes[piece_index * kGenesPerPiece + 1U] =
          (static_cast<double>(forced_rotations[piece_index]->value) + 0.5) /
          static_cast<double>(rotation_count);
    }
  }
  return genes;
}

[[nodiscard]] auto random_genes(const std::size_t piece_count,
                                runtime::DeterministicRng &rng)
    -> std::vector<double> {
  std::vector<double> genes;
  genes.reserve(piece_count * kGenesPerPiece);
  for (std::size_t index = 0; index < piece_count * kGenesPerPiece; ++index) {
    genes.push_back(rng.uniform_real());
  }
  return genes;
}

[[nodiscard]] auto reordered_request_for(
    std::span<const double> genes, const NormalizedRequest &request,
    std::span<const std::size_t> rotation_counts) -> NormalizedRequest {
  NormalizedRequest reordered = request;
  const auto order = order_from_genes(genes);
  const auto forced_rotations =
      forced_rotations_from_genes(genes, request, rotation_counts);
  // BRKGA encodes the constructive decode order in the reordered expanded-piece
  // list. Use largest_area_first so the packer always tries large pieces before
  // small ones within the gene-decoded order, matching the fill-first strategy.
  reordered.request.execution.irregular.piece_ordering =
      PieceOrdering::largest_area_first;
  reordered.expanded_pieces.clear();
  reordered.forced_rotations.clear();
  reordered.expanded_pieces.reserve(order.size());
  reordered.forced_rotations.reserve(order.size());
  for (const auto index : order) {
    reordered.expanded_pieces.push_back(request.expanded_pieces[index]);
    reordered.forced_rotations.push_back(index < forced_rotations.size()
                                             ? forced_rotations[index]
                                             : std::nullopt);
  }
  return reordered;
}

[[nodiscard]] auto
evaluate_genes(std::span<const double> genes, const NormalizedRequest &request,
               const SolveControl &control, pack::PackerWorkspace &workspace,
               std::span<const std::size_t> rotation_counts,
               const runtime::TimeBudget &time_budget,
               const runtime::Stopwatch &stopwatch,
               ProgressThrottle &outer_throttle, const std::size_t generation,
               const std::size_t generation_limit, const std::size_t chromo_idx,
               const std::size_t pop_size) -> EvaluatedChromosome {
  const auto reordered = reordered_request_for(genes, request, rotation_counts);
  const auto forced_rotations =
      forced_rotations_from_genes(genes, request, rotation_counts);

  pack::IrregularConstructivePacker packer;
  SolveControl decode_control{};
  decode_control.cancellation = control.cancellation;
  decode_control.workspace = &workspace;
  // Derive a unique seed from the chromosome genes so each evaluation
  // produces a genuinely different layout via stochastic tie-breaking.
  decode_control.random_seed =
      seed_from_genes(genes, rotation_counts, request, control.random_seed);
  if (time_budget.enabled()) {
    const auto remaining = time_budget.remaining_milliseconds(stopwatch);
    if (remaining == 0U) {
      EvaluatedChromosome evaluated;
      evaluated.genes.assign(genes.begin(), genes.end());
      evaluated.forced_rotations = forced_rotations;
      evaluated.result.strategy = StrategyKind::metaheuristic_search;
      evaluated.result.total_parts = request.expanded_pieces.size();
      evaluated.result.stop_reason = StopReason::time_limit_reached;
      return evaluated;
    }
    decode_control.time_limit_milliseconds = remaining;
  }

  // Forward lightweight progress from inner constructive packer so the
  // UI sees per-part placement detail during BRKGA chromosome evaluation.
  if (control.on_progress) {
    decode_control.on_progress = [&control, &outer_throttle, generation,
                                  generation_limit, chromo_idx,
                                  pop_size](const ProgressSnapshot &inner) {
      if (!outer_throttle.should_emit()) {
        return;
      }
      control.on_progress(ProgressSnapshot{
          .sequence = 0,
          .placements_successful = inner.placements_successful,
          .total_requested_parts = inner.total_requested_parts,
          .layout = inner.layout,
          .stop_reason = StopReason::none,
          .phase = ProgressPhase::placement,
          .phase_detail = std::format(
              "Gen {}/{} chromo {}/{}: placing {}/{}", generation,
              generation_limit, chromo_idx + 1U, pop_size,
              inner.placements_successful, inner.total_requested_parts),
          .utilization_percent = inner.utilization_percent,
          .improved = false,
      });
    };
  }

  const auto result_or = packer.solve(reordered, decode_control);

  EvaluatedChromosome evaluated;
  evaluated.genes.assign(genes.begin(), genes.end());
  evaluated.forced_rotations = std::move(forced_rotations);
  if (result_or.has_value() &&
      !validation::layout_has_geometry_violation(request, result_or.value())) {
    evaluated.result = result_or.value();
    validation::finalize_result(request, evaluated.result);
    evaluated.metrics = metrics_for_layout(evaluated.result.layout);
    evaluated.valid = true;
  } else {
    evaluated.result.strategy = StrategyKind::metaheuristic_search;
    evaluated.result.total_parts = request.expanded_pieces.size();
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
                        SearchReplay &search_replay, const ProgressPhase phase,
                        const std::string &phase_detail) -> void {
  SHINY_DEBUG("brkga emit_best: seq={} gen={} placed={}/{} util={:.1f}%",
              sequence, iteration, best.metrics.placed_parts,
              best.result.total_parts, best.metrics.utilization * 100.0);

  search_replay.progress.push_back({
      .iteration = iteration,
      .improved = true,
      .layout = best.result.layout,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
  });

  if (!control.on_progress) {
    return;
  }
  control.on_progress(ProgressSnapshot{
      .sequence = sequence,
      .placements_successful = best.metrics.placed_parts,
      .total_requested_parts = best.result.total_parts,
      .layout = best.result.layout,
      .stop_reason = StopReason::none,
      .phase = phase,
      .phase_detail = phase_detail,
      .utilization_percent = best.metrics.utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
      .improved = true,
  });
}

auto emit_generation_progress(const SolveControl &control,
                              const std::size_t generation,
                              const std::size_t generation_limit,
                              const std::size_t best_placed_parts,
                              const std::size_t total_parts,
                              const double best_utilization,
                              const EvaluatedChromosome &best) -> void {
  if (!control.on_progress) {
    return;
  }
  SHINY_DEBUG("brkga emit_gen: gen={}/{} placed={}/{} util={:.1f}%", generation,
              generation_limit, best_placed_parts, total_parts,
              best_utilization * 100.0);

  control.on_progress(ProgressSnapshot{
      .sequence = 0,
      .placements_successful = best_placed_parts,
      .total_requested_parts = total_parts,
      .layout = best.result.layout,
      .stop_reason = StopReason::none,
      .phase = ProgressPhase::placement,
      .phase_detail =
          std::format("Generation {}/{}", generation, generation_limit),
      .utilization_percent = best_utilization * 100.0,
      .layout_valid = best.result.layout_valid(),
      .validation_issue_count = best.result.validation.issues.size(),
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

[[nodiscard]] auto
polish_best(const EvaluatedChromosome &seed, const NormalizedRequest &request,
            const SolveControl &control, pack::PackerWorkspace &workspace,
            std::span<const std::size_t> rotation_counts,
            const runtime::TimeBudget &time_budget,
            const runtime::Stopwatch &stopwatch,
            const ProductionSearchConfig &config,
            const std::vector<double> &piece_areas, ProgressThrottle &throttle)
    -> EvaluatedChromosome {
  EvaluatedChromosome best = seed;
  auto best_order = order_from_genes(best.genes);
  const std::size_t refinement_limit =
      std::max<std::size_t>(1U, config.max_iterations) *
      std::max<std::size_t>(1U, config.polishing_passes);
  std::size_t refinements_completed = 0U;

  for (std::size_t pass = 0; pass < config.polishing_passes; ++pass) {
    bool improved = false;
    for (std::size_t index = 0; index < best_order.size(); ++index) {
      if (refinements_completed >= refinement_limit ||
          detail::driver_interrupted(control, time_budget, stopwatch)) {
        return best;
      }

      std::size_t largest_index = index;
      for (std::size_t probe = index + 1U; probe < best_order.size(); ++probe) {
        if (piece_areas[best_order[probe]] >
            piece_areas[best_order[largest_index]]) {
          largest_index = probe;
        }
      }
      if (largest_index == index) {
        continue;
      }

      auto neighbor = best_order;
      std::rotate(neighbor.begin() + index, neighbor.begin() + largest_index,
                  neighbor.begin() + largest_index + 1U);
      auto evaluated = evaluate_genes(
          genes_from_solution(neighbor, best.forced_rotations, rotation_counts),
          request, control, workspace, rotation_counts, time_budget, stopwatch,
          throttle, 0, 0, 0, 0);
      ++refinements_completed;
      if (better_candidate(evaluated, best)) {
        best = std::move(evaluated);
        best_order = order_from_genes(best.genes);
        improved = true;
      }
    }

    for (std::size_t index = 1; index < best_order.size(); ++index) {
      if (refinements_completed >= refinement_limit ||
          detail::driver_interrupted(control, time_budget, stopwatch)) {
        return best;
      }

      auto neighbor = best_order;
      std::swap(neighbor[index - 1U], neighbor[index]);
      auto evaluated = evaluate_genes(
          genes_from_solution(neighbor, best.forced_rotations, rotation_counts),
          request, control, workspace, rotation_counts, time_budget, stopwatch,
          throttle, 0, 0, 0, 0);
      ++refinements_completed;
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
    -> std::expected<NestingResult, util::Status> {
  if (!request.request.is_valid()) {
    return std::unexpected(util::Status::invalid_input);
  }

  runtime::DeterministicRng rng(control.random_seed);
  runtime::Stopwatch stopwatch;
  const runtime::TimeBudget time_budget(control.time_limit_milliseconds);
  const auto &config = resolve_production_strategy_config(
      request.request.execution, ProductionOptimizerKind::brkga,
      request.request.execution.production);
  const auto piece_areas = detail::piece_areas_for(request);
  const auto rotation_counts = piece_rotation_counts(request);
  const auto operation_budget =
      detail::make_operation_budget(control, config.max_iterations);
  const auto generation_limit = operation_budget.iteration_limit();

  SearchReplay replay{
      .optimizer = OptimizerKind::brkga,
      .sparrow_polished = config.polishing_passes > 0U,
  };

  if (request.expanded_pieces.empty()) {
    return detail::driver_empty_result(StrategyKind::metaheuristic_search,
                                       std::move(replay));
  }

  std::vector<std::vector<double>> population;
  population.reserve(config.population_size);

  std::vector<std::size_t> original_order(request.expanded_pieces.size());
  std::iota(original_order.begin(), original_order.end(), 0U);
  population.push_back(genes_from_solution(
      original_order, request.forced_rotations, rotation_counts));

  auto descending_area = original_order;
  std::stable_sort(descending_area.begin(), descending_area.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     if (piece_areas[lhs] != piece_areas[rhs]) {
                       return piece_areas[lhs] > piece_areas[rhs];
                     }
                     return lhs < rhs;
                   });
  population.push_back(genes_from_solution(
      descending_area, request.forced_rotations, rotation_counts));

  auto reverse_order = original_order;
  std::reverse(reverse_order.begin(), reverse_order.end());
  population.push_back(genes_from_solution(
      reverse_order, request.forced_rotations, rotation_counts));

  while (population.size() < config.population_size) {
    population.push_back(random_genes(request.expanded_pieces.size(), rng));
  }

  std::optional<EvaluatedChromosome> best;
  std::size_t plateau_generations = 0;
  std::size_t sequence = 0;
  std::size_t operations_completed = 0;
  bool generation_budget_exhausted = false;
  bool hit_operation_limit = false;
  bool completed_full_solution = false;
  ProgressThrottle throttle;
  std::unordered_set<OrderSignature, OrderSignatureHash> seen_signatures;
  pack::PackerWorkspace local_workspace;
  pack::PackerWorkspace &workspace =
      control.workspace != nullptr ? *control.workspace : local_workspace;
  workspace.reset_search_metrics();

  for (std::size_t generation = 0; generation < generation_limit;
       ++generation) {
    if (detail::driver_interrupted(control, time_budget, stopwatch)) {
      break;
    }

    ++operations_completed;
    std::vector<EvaluatedChromosome> evaluated_population;
    evaluated_population.reserve(population.size());
    for (std::size_t chromo_idx = 0; chromo_idx < population.size();
         ++chromo_idx) {
      if (detail::driver_interrupted(control, time_budget, stopwatch)) {
        break;
      }

      // Skip chromosomes that produce an ordering identical to one
      // already evaluated (identical-piece normalization).
      const auto sig = compute_order_signature(population[chromo_idx], request,
                                               rotation_counts);
      if (seen_signatures.contains(sig)) {
        SHINY_DEBUG("brkga: skipping duplicate ordering gen={} chromo={}",
                    generation + 1U, chromo_idx);
        continue;
      }
      seen_signatures.insert(sig);

      auto evaluated = evaluate_genes(
          population[chromo_idx], request, control, workspace, rotation_counts,
          time_budget, stopwatch, throttle, generation + 1U, generation_limit,
          chromo_idx, population.size());
      if (!evaluated.valid) {
        SHINY_DEBUG(
            "brkga: rejecting invalid constructive decode gen={} chromo={}",
            generation + 1U, chromo_idx);
        continue;
      }
      evaluated_population.push_back(std::move(evaluated));

      // Emit throttled progress during evaluation so the UI shows activity
      if (control.on_progress && throttle.should_emit()) {
        const std::size_t best_placed =
            best.has_value() ? best->metrics.placed_parts : 0U;
        const std::size_t total = best.has_value()
                                      ? best->result.total_parts
                                      : request.expanded_pieces.size();
        const double best_util =
            best.has_value() ? best->metrics.utilization : 0.0;
        control.on_progress(ProgressSnapshot{
            .sequence = 0,
            .placements_successful = best_placed,
            .total_requested_parts = total,
            .layout = best.has_value() ? best->result.layout : pack::Layout{},
            .stop_reason = StopReason::none,
            .phase = ProgressPhase::placement,
            .phase_detail = std::format("Evaluating {}/{} in generation {}/{}",
                                        chromo_idx + 1U, population.size(),
                                        generation + 1U, generation_limit),
            .utilization_percent = best_util * 100.0,
            .improved = false,
        });
      }
    }
    if (evaluated_population.empty()) {
      generation_budget_exhausted = true;
      break;
    }

    std::sort(evaluated_population.begin(), evaluated_population.end(),
              better_candidate);

    if (!best.has_value() ||
        better_candidate(evaluated_population.front(), *best)) {
      best = evaluated_population.front();
      plateau_generations = 0;
      ++sequence;
      emit_best_progress(control, sequence, generation + 1U, *best, replay,
                         ProgressPhase::placement,
                         std::format("Generation {}/{} (improved)",
                                     generation + 1U, generation_limit));
    } else {
      ++plateau_generations;
      emit_generation_progress(control, generation + 1U, generation_limit,
                               best->metrics.placed_parts,
                               best->result.total_parts,
                               best->metrics.utilization, *best);
    }

    if (generation + 1U >= generation_limit) {
      generation_budget_exhausted = true;
      hit_operation_limit =
          operation_budget.external_limit_reached(operations_completed);
      break;
    }
    if (best.has_value() &&
        best->metrics.placed_parts == best->result.total_parts &&
        best->result.layout.unplaced_piece_ids.empty()) {
      completed_full_solution = true;
      break;
    }
    if (detail::driver_interrupted(control, time_budget, stopwatch)) {
      break;
    }

    std::vector<std::vector<double>> next_population;
    next_population.reserve(config.population_size);
    const auto elite_count =
        std::min(config.elite_count, evaluated_population.size());
    for (std::size_t elite_index = 0; elite_index < elite_count;
         ++elite_index) {
      next_population.push_back(evaluated_population[elite_index].genes);
    }

    for (std::size_t mutant_index = 0;
         mutant_index < config.mutant_count &&
         next_population.size() < config.population_size;
         ++mutant_index) {
      next_population.push_back(
          random_genes(request.expanded_pieces.size(), rng));
    }

    while (next_population.size() < config.population_size) {
      const auto elite_parent =
          evaluated_population[rng.uniform_index(elite_count)];
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
        .stop_reason = detail::driver_stop_reason(
            control, time_budget, stopwatch, hit_operation_limit),
        .search = std::move(replay),
    };
    return result;
  }

  if (!detail::driver_interrupted(control, time_budget, stopwatch) &&
      !generation_budget_exhausted && !hit_operation_limit &&
      !completed_full_solution && config.polishing_passes > 0U) {
    if (control.on_progress) {
      control.on_progress(ProgressSnapshot{
          .sequence = 0,
          .placements_successful = best->metrics.placed_parts,
          .total_requested_parts = best->result.total_parts,
          .layout = {},
          .stop_reason = StopReason::none,
          .phase = ProgressPhase::refinement,
          .phase_detail = "Polishing best solution",
          .utilization_percent = best->metrics.utilization * 100.0,
          .layout_valid = best->result.layout_valid(),
          .validation_issue_count = best->result.validation.issues.size(),
          .improved = false,
      });
    }
    auto polished =
        polish_best(*best, request, control, workspace, rotation_counts,
                    time_budget, stopwatch, config, piece_areas, throttle);
    if (better_candidate(polished, *best)) {
      best = std::move(polished);
      ++sequence;
      emit_best_progress(control, sequence, operations_completed, *best, replay,
                         ProgressPhase::refinement,
                         "Polishing improved solution");
    }

    if (!detail::driver_interrupted(control, time_budget, stopwatch)) {
      StripOptimizer strip_optimizer;
      const auto optimized = strip_optimizer.optimize(
          request, control, time_budget, stopwatch,
          SolutionPoolEntry{
              .order = order_from_genes(best->genes),
              .piece_indexed_forced_rotations = best->forced_rotations,
              .metrics = best->metrics,
              .result = best->result,
          },
          config);
      replay.phase_metrics = optimized.phase_metrics;
      replay.separator_metrics = optimized.separator_metrics;
      if (better_metrics(optimized.best_solution.metrics, best->metrics)) {
        best->genes = genes_from_solution(
            optimized.best_solution.order,
            optimized.best_solution.piece_indexed_forced_rotations,
            rotation_counts);
        best->metrics = optimized.best_solution.metrics;
        best->result = optimized.best_solution.result;
        best->forced_rotations =
            optimized.best_solution.piece_indexed_forced_rotations;
        ++sequence;
        emit_best_progress(control, sequence, operations_completed, *best,
                           replay, ProgressPhase::refinement,
                           "Strip optimization improved solution");
      }
    }
  }

  NestingResult result = best->result;
  result.strategy = StrategyKind::metaheuristic_search;
  // Give an explicit operation_limit cap priority over completed_full_solution.
  // When the caller set operation_limit > 0, they asked for a bounded run;
  // report operation_limit_reached even if the full solution was found within
  // that budget, so the caller can reliably distinguish "ran to completion"
  // (operation_limit == 0) from "ran to cap" (operation_limit > 0).
  result.stop_reason =
      completed_full_solution && control.operation_limit == 0U
          ? StopReason::completed
          : detail::driver_stop_reason(control, time_budget, stopwatch,
                                       hit_operation_limit ||
                                           completed_full_solution);
  replay.cache_metrics.exact_nfp_cache_hits =
      workspace.search_metrics.exact_nfp_cache_hits;
  replay.cache_metrics.conservative_bbox_cache_hits =
      workspace.search_metrics.conservative_bbox_cache_hits;
  replay.cache_metrics.exact_nfp_computations =
      workspace.search_metrics.exact_nfp_computations;
  replay.cache_metrics.conservative_bbox_fallbacks =
      workspace.search_metrics.conservative_bbox_fallbacks;
  replay.fallback_metrics.conservative_bbox_candidate_points =
      workspace.search_metrics.conservative_bbox_candidate_points;
  replay.fallback_metrics.selected_fallback_placements =
      workspace.search_metrics.selected_fallback_placements;
  result.search = std::move(replay);
  return result;
}

} // namespace shiny::nesting::search
