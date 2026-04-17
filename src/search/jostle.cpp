#include "search/jostle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <numeric>
#include <span>
#include <thread>
#include <utility>
#include <vector>

#include "search/detail/run_event_sink.hpp"
#include "search/detail/run_interruption_gate.hpp"
#include "util/detail/fnv_hash.hpp"

namespace shiny::nfp::search {
namespace {

constexpr double kScoreEpsilon = 1e-9;
constexpr std::uint32_t kSearchRevision = 1;
constexpr auto kAlgorithmKind = AlgorithmKind::jostle_search;
constexpr std::size_t kParallelBatchTaskCutoff = 8U;

using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

struct SearchCounters {
  std::size_t evaluated_layout_count{0};
  std::size_t reevaluation_cache_hits{0};
};

struct RunTiming {
  runtime::ExecutionControlConfig control{};
  SteadyClock::time_point started_steady{SteadyClock::now()};

  [[nodiscard]] auto elapsed_ms() const -> std::uint64_t {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - started_steady)
            .count());
  }

  [[nodiscard]] auto event_elapsed_ms() const -> std::uint64_t {
    return control.capture_timestamps ? elapsed_ms() : 0U;
  }

  [[nodiscard]] auto timestamp_unix_ms() const -> std::uint64_t {
    if (!control.capture_timestamps) {
      return 0U;
    }

    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            SystemClock::now().time_since_epoch())
            .count());
  }
};

struct NeighborChoice {
  bool found{false};
  bool improved{false};
  bool plateau{false};
  bool interrupted{false};
  LayoutEvaluation evaluation{};
  std::vector<std::size_t> order{};
};

struct WorkerContext {
  explicit WorkerContext(cache::CachePolicyConfig cache_policy)
      : reevaluation_cache(cache_policy), decoder(cache_policy) {}

  cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation>
      reevaluation_cache{};
  pack::ConstructiveDecoder decoder{};
};

struct EvaluationTask {
  std::size_t task_index{0};
  std::vector<std::size_t> order{};
  cache::LayoutEvalKey key{};
};

struct IndexedEvaluation {
  std::size_t task_index{0};
  LayoutEvaluation evaluation{};
};

struct BatchEvaluationResult {
  std::vector<LayoutEvaluation> evaluations{};
  SearchCounters counters{};
  bool interrupted{false};
};

[[nodiscard]] auto rotate_index(std::size_t count, std::size_t start_offset,
                                std::size_t relative_index) -> std::size_t {
  return count == 0U ? 0U : (start_offset + relative_index) % count;
}

[[nodiscard]] auto
compute_rotation_assignment_hash(const pack::DecoderRequest &request)
    -> std::uint64_t {
  auto seed = shiny::nfp::detail::fnv_hash_values({static_cast<std::uint64_t>(
      request.config.placement.allowed_rotations.angles_degrees.size())});

  for (const auto angle :
       request.config.placement.allowed_rotations.angles_degrees) {
    seed = shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(angle));
  }

  seed = shiny::nfp::detail::fnv_hash_mix(
      seed,
      static_cast<std::uint64_t>(request.config.placement.bed_grain_direction));

  return seed;
}

[[nodiscard]] auto compute_bin_policy_hash(const pack::DecoderRequest &request)
    -> std::uint64_t {
  auto seed = shiny::nfp::detail::fnv_hash_values(
      {static_cast<std::uint64_t>(request.bins.size()),
       static_cast<std::uint64_t>(request.policy),
       std::hash<double>{}(request.config.placement.part_clearance),
       static_cast<std::uint64_t>(request.config.enable_hole_first_placement),
       static_cast<std::uint64_t>(
           request.config.placement.enable_part_in_part_placement),
       static_cast<std::uint64_t>(
           request.config.placement.explore_concave_candidates),
       static_cast<std::uint64_t>(request.config.laser_cut_optimization.mode),
       static_cast<std::uint64_t>(
           request.config.laser_cut_optimization.require_exact_collinearity),
       static_cast<std::uint64_t>(
           request.config.laser_cut_optimization.preserve_visible_notches)});

  for (const auto &zone : request.config.placement.exclusion_zones) {
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, std::hash<std::uint32_t>{}(zone.zone_id));
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, std::hash<bool>{}(zone.bin_id.has_value()));
    if (zone.bin_id.has_value()) {
      seed = shiny::nfp::detail::fnv_hash_mix(
          seed, std::hash<std::uint32_t>{}(*zone.bin_id));
    }
    for (const auto &point : zone.region.outer) {
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.x));
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.y));
    }
  }

  for (const auto &bin : request.bins) {
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, std::hash<std::uint32_t>{}(bin.bin_id));
    seed = shiny::nfp::detail::fnv_hash_mix(seed, bin.geometry_revision);
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, static_cast<std::uint64_t>(bin.start_corner));
    for (const auto &point : bin.polygon.outer) {
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.x));
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.y));
    }
    for (const auto &hole : bin.polygon.holes) {
      for (const auto &point : hole) {
        seed = shiny::nfp::detail::fnv_hash_mix(seed,
                                                std::hash<double>{}(point.x));
        seed = shiny::nfp::detail::fnv_hash_mix(seed,
                                                std::hash<double>{}(point.y));
      }
    }
  }

  return seed;
}

[[nodiscard]] auto
compute_piece_order_hash(const std::vector<pack::PieceInput> &pieces,
                         const std::vector<std::size_t> &order)
    -> std::uint64_t {
  auto seed = shiny::nfp::detail::fnv_hash_values(
      {static_cast<std::uint64_t>(order.size())});
  for (const auto index : order) {
    const auto &piece = pieces[index];
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, static_cast<std::uint64_t>(piece.piece_id));
    seed = shiny::nfp::detail::fnv_hash_mix(seed, piece.geometry_revision);
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, static_cast<std::uint64_t>(piece.grain_compatibility));
    seed = shiny::nfp::detail::fnv_hash_mix(
        seed, static_cast<std::uint64_t>(piece.allowed_bin_ids.size()));
    for (const auto bin_id : piece.allowed_bin_ids) {
      seed = shiny::nfp::detail::fnv_hash_mix(
          seed, std::hash<std::uint32_t>{}(bin_id));
    }
  }
  return seed;
}

[[nodiscard]] auto make_layout_eval_key(const pack::DecoderRequest &request,
                                        const std::vector<std::size_t> &order)
    -> cache::LayoutEvalKey {
  return {
      .piece_order_hash = compute_piece_order_hash(request.pieces, order),
      .rotation_assignment_hash = compute_rotation_assignment_hash(request),
      .bin_policy_hash = compute_bin_policy_hash(request),
      .search_revision = kSearchRevision,
  };
}

[[nodiscard]] auto resolve_worker_count(std::uint32_t requested,
                                        std::size_t task_count) -> std::size_t {
  const auto bounded =
      std::max<std::size_t>(1U, static_cast<std::size_t>(requested));
  if (task_count == 0U) {
    return bounded;
  }
  return std::min(bounded, task_count);
}

[[nodiscard]] auto make_extra_workers(std::uint32_t requested_worker_count,
                                      cache::CachePolicyConfig cache_policy)
    -> std::vector<std::unique_ptr<WorkerContext>> {
  const auto worker_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(requested_worker_count));

  std::vector<std::unique_ptr<WorkerContext>> extra_workers;
  extra_workers.reserve(worker_count > 0U ? worker_count - 1U : 0U);
  for (std::size_t worker_index = 1U; worker_index < worker_count;
       ++worker_index) {
    extra_workers.push_back(std::make_unique<WorkerContext>(cache_policy));
  }

  return extra_workers;
}

[[nodiscard]] auto select_worker_index(std::size_t task_index,
                                       std::size_t worker_count)
    -> std::size_t {
  if (worker_count == 0U) {
    return 0U;
  }
  return task_index % worker_count;
}

[[nodiscard]] auto
extract_piece_order_ids(const std::vector<pack::PieceInput> &pieces,
                        const std::vector<std::size_t> &order)
    -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> piece_order;
  piece_order.reserve(order.size());
  for (const auto index : order) {
    piece_order.push_back(pieces[index].piece_id);
  }
  return piece_order;
}

[[nodiscard]] auto make_reordered_request(const pack::DecoderRequest &request,
                                          const std::vector<std::size_t> &order)
    -> pack::DecoderRequest {
  auto reordered = request;
  reordered.pieces.clear();
  reordered.pieces.reserve(order.size());

  for (const auto index : order) {
    reordered.pieces.push_back(request.pieces[index]);
  }

  return reordered;
}

[[nodiscard]] auto
summarize_evaluation(const std::vector<std::uint32_t> &piece_order,
                     pack::DecoderResult decode) -> LayoutEvaluation {
  LayoutEvaluation evaluation{};
  evaluation.piece_order = piece_order;
  evaluation.decode = std::move(decode);
  evaluation.bin_count = evaluation.decode.bins.size();
  evaluation.unplaced_piece_count =
      evaluation.decode.layout.unplaced_piece_ids.size();
  evaluation.placed_piece_count =
      evaluation.piece_order.size() - evaluation.unplaced_piece_count;

  for (const auto &bin : evaluation.decode.bins) {
    evaluation.total_utilization += bin.utilization.utilization;
  }

  if (evaluation.bin_count > 0U) {
    evaluation.average_utilization = evaluation.total_utilization /
                                     static_cast<double>(evaluation.bin_count);
    evaluation.last_bin_utilization =
        evaluation.decode.bins.back().utilization.utilization;
  }

  return evaluation;
}

[[nodiscard]] auto equal_score(double lhs, double rhs) -> bool {
  return std::fabs(lhs - rhs) <= kScoreEpsilon;
}

[[nodiscard]] auto strictly_better_score(const LayoutEvaluation &lhs,
                                         const LayoutEvaluation &rhs) -> bool {
  if (lhs.unplaced_piece_count != rhs.unplaced_piece_count) {
    return lhs.unplaced_piece_count < rhs.unplaced_piece_count;
  }
  if (lhs.bin_count != rhs.bin_count) {
    return lhs.bin_count < rhs.bin_count;
  }
  if (!equal_score(lhs.total_utilization, rhs.total_utilization)) {
    return lhs.total_utilization > rhs.total_utilization;
  }
  if (!equal_score(lhs.last_bin_utilization, rhs.last_bin_utilization)) {
    return lhs.last_bin_utilization > rhs.last_bin_utilization;
  }
  if (!equal_score(lhs.average_utilization, rhs.average_utilization)) {
    return lhs.average_utilization > rhs.average_utilization;
  }
  return false;
}

[[nodiscard]] auto same_objective_score(const LayoutEvaluation &lhs,
                                        const LayoutEvaluation &rhs) -> bool {
  return lhs.unplaced_piece_count == rhs.unplaced_piece_count &&
         lhs.bin_count == rhs.bin_count &&
         equal_score(lhs.total_utilization, rhs.total_utilization) &&
         equal_score(lhs.last_bin_utilization, rhs.last_bin_utilization) &&
         equal_score(lhs.average_utilization, rhs.average_utilization);
}

[[nodiscard]] auto evaluation_better(const LayoutEvaluation &lhs,
                                     const LayoutEvaluation &rhs) -> bool {
  if (strictly_better_score(lhs, rhs)) {
    return true;
  }
  if (strictly_better_score(rhs, lhs)) {
    return false;
  }
  return lhs.piece_order < rhs.piece_order;
}

[[nodiscard]] auto choice_rank(const NeighborChoice &choice) -> int {
  if (choice.improved) {
    return 2;
  }
  if (choice.plateau) {
    return 1;
  }
  return 0;
}

[[nodiscard]] auto evaluate_order(
    const SearchRequest &request, const std::vector<std::size_t> &order,
    const cache::LayoutEvalKey &key,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder, SearchCounters &counters,
    detail::RunInterruptionGate *interruption_gate = nullptr)
    -> LayoutEvaluation {
  if (const auto *cached = cache_store.get(key)) {
    ++counters.reevaluation_cache_hits;
    return *cached;
  }

  ++counters.evaluated_layout_count;

  const auto piece_order =
      extract_piece_order_ids(request.decoder_request.pieces, order);
  const auto reordered_request =
      make_reordered_request(request.decoder_request, order);
  auto decode = interruption_gate == nullptr
                    ? decoder.decode(reordered_request)
                    : decoder.decode(reordered_request, [&]() {
                        return interruption_gate->poll();
                      });
  auto evaluation = summarize_evaluation(piece_order, std::move(decode));
  if (!evaluation.decode.interrupted) {
    cache_store.put(key, evaluation);
  }
  return evaluation;
}

[[nodiscard]] auto evaluate_order(
    const SearchRequest &request, const std::vector<std::size_t> &order,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder, SearchCounters &counters,
    detail::RunInterruptionGate *interruption_gate = nullptr)
    -> LayoutEvaluation {
  return evaluate_order(request, order,
                        make_layout_eval_key(request.decoder_request, order),
                        cache_store, decoder, counters, interruption_gate);
}

[[nodiscard]] auto evaluate_task_batch(
    const SearchRequest &request, std::span<const EvaluationTask> tasks,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation>
        &primary_cache_store,
    pack::ConstructiveDecoder &primary_decoder,
    const std::vector<std::unique_ptr<WorkerContext>> &extra_workers,
    detail::RunInterruptionGate &interruption_gate) -> BatchEvaluationResult {
  BatchEvaluationResult batch{};
  if (tasks.empty()) {
    return batch;
  }

  const auto worker_count = resolve_worker_count(
      static_cast<std::uint32_t>(1U + extra_workers.size()), tasks.size());
  if (worker_count <= 1U || tasks.size() < kParallelBatchTaskCutoff) {
    batch.evaluations.reserve(tasks.size());
    for (const auto &task : tasks) {
      if (interruption_gate.poll()) {
        batch.interrupted = true;
        return batch;
      }

      batch.evaluations.push_back(
          evaluate_order(request, task.order, task.key, primary_cache_store,
                         primary_decoder, batch.counters, &interruption_gate));
      if (batch.evaluations.back().decode.interrupted) {
        batch.interrupted = true;
        return batch;
      }
    }
    return batch;
  }

  std::vector<std::vector<EvaluationTask>> worker_tasks(worker_count);
  for (const auto &task : tasks) {
    worker_tasks[select_worker_index(task.task_index, worker_count)].push_back(
        task);
  }

  std::vector<SearchCounters> worker_counters(worker_count);
  std::vector<std::vector<IndexedEvaluation>> worker_results(worker_count);

  const auto run_worker = [&](const std::size_t worker_index) {
    auto *cache_store = &primary_cache_store;
    auto *decoder = &primary_decoder;
    if (worker_index > 0U) {
      cache_store = &extra_workers[worker_index - 1U]->reevaluation_cache;
      decoder = &extra_workers[worker_index - 1U]->decoder;
    }

    auto &results = worker_results[worker_index];
    results.reserve(worker_tasks[worker_index].size());

    for (const auto &task : worker_tasks[worker_index]) {
      if (interruption_gate.poll()) {
        return;
      }

      results.push_back({
          .task_index = task.task_index,
          .evaluation = evaluate_order(
              request, task.order, task.key, *cache_store, *decoder,
              worker_counters[worker_index], &interruption_gate),
      });
      if (results.back().evaluation.decode.interrupted) {
        return;
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(worker_count > 0U ? worker_count - 1U : 0U);
  for (std::size_t worker_index = 1U; worker_index < worker_count;
       ++worker_index) {
    threads.emplace_back(run_worker, worker_index);
  }

  run_worker(0U);

  for (auto &thread : threads) {
    thread.join();
  }

  batch.interrupted = interruption_gate.interrupted();
  if (batch.interrupted) {
    return batch;
  }

  batch.evaluations.resize(tasks.size());
  for (std::size_t worker_index = 0U; worker_index < worker_count;
       ++worker_index) {
    batch.counters.evaluated_layout_count +=
        worker_counters[worker_index].evaluated_layout_count;
    batch.counters.reevaluation_cache_hits +=
        worker_counters[worker_index].reevaluation_cache_hits;

    for (auto &result : worker_results[worker_index]) {
      batch.evaluations[result.task_index] = std::move(result.evaluation);
    }
  }

  return batch;
}

auto record_progress(SearchResult &result, std::uint32_t iteration,
                     std::uint32_t iteration_budget, SearchMoveKind move_kind,
                     bool improved, const RunTiming &timing)
    -> const SearchProgressEntry & {
  result.progress.push_back({
      .algorithm_kind = kAlgorithmKind,
      .iteration = iteration,
      .iteration_budget = iteration_budget,
      .move_kind = move_kind,
      .improved = improved,
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
      .evaluated_layout_count = result.evaluated_layout_count,
      .reevaluation_cache_hits = result.reevaluation_cache_hits,
      .best_bin_count = result.best.bin_count,
      .best_placed_piece_count = result.best.placed_piece_count,
      .best_unplaced_piece_count = result.best.unplaced_piece_count,
      .best_total_utilization = result.best.total_utilization,
      .best_piece_order = result.best.piece_order,
  });

  return result.progress.back();
}

[[nodiscard]] auto make_run_summary(const SearchResult &result,
                                    std::uint32_t iteration_budget,
                                    const RunTiming &timing)
    -> SearchRunSummary {
  return {
      .algorithm_kind = kAlgorithmKind,
      .iterations_completed = result.iterations_completed,
      .iteration_budget = iteration_budget,
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
      .evaluated_layout_count = result.evaluated_layout_count,
      .reevaluation_cache_hits = result.reevaluation_cache_hits,
      .best_bin_count = result.best.bin_count,
      .best_placed_piece_count = result.best.placed_piece_count,
      .best_unplaced_piece_count = result.best.unplaced_piece_count,
      .best_total_utilization = result.best.total_utilization,
      .best_piece_order = result.best.piece_order,
  };
}

auto emit_terminal_event(detail::RunEventSink &event_sink,
                         const SearchRequest &request, SearchResult &result,
                         const RunTiming &timing) -> void {
  const auto summary =
      make_run_summary(result, request.local_search.max_iterations, timing);
  switch (result.status) {
  case SearchRunStatus::completed:
    event_sink.emit(SearchRunCompletedEvent{.summary = summary});
    break;
  case SearchRunStatus::timed_out:
    event_sink.emit(SearchTimeoutReachedEvent{.summary = summary});
    break;
  case SearchRunStatus::cancelled:
    event_sink.emit(SearchCancellationAcknowledgedEvent{.summary = summary});
    break;
  case SearchRunStatus::invalid_request:
    break;
  }
}

[[nodiscard]] auto identity_order(std::size_t count)
    -> std::vector<std::size_t> {
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0U);
  return order;
}

[[nodiscard]] auto make_jostle_neighbors(const std::vector<std::size_t> &order,
                                         std::uint32_t seed,
                                         std::uint32_t iteration)
    -> std::vector<std::vector<std::size_t>> {
  std::vector<std::vector<std::size_t>> neighbors;
  if (order.size() < 2U) {
    return neighbors;
  }

  neighbors.reserve(order.size() - 1U);

  const auto pair_count = order.size() - 1U;
  const auto forward = ((seed + iteration) % 2U) == 0U;
  const auto start = static_cast<std::size_t>(seed + iteration) % pair_count;

  for (std::size_t offset = 0; offset < pair_count; ++offset) {
    std::size_t pair_index = rotate_index(pair_count, start, offset);
    if (!forward) {
      pair_index = pair_count - 1U - pair_index;
    }

    auto candidate = order;
    std::swap(candidate[pair_index], candidate[pair_index + 1U]);
    neighbors.push_back(std::move(candidate));
  }

  return neighbors;
}

[[nodiscard]] auto move_piece_to_index(std::vector<std::size_t> order,
                                       std::size_t source_index,
                                       std::size_t target_index)
    -> std::vector<std::size_t> {
  if (source_index == target_index) {
    return order;
  }

  if (source_index < target_index) {
    std::rotate(order.begin() + static_cast<std::ptrdiff_t>(source_index),
                order.begin() + static_cast<std::ptrdiff_t>(source_index + 1U),
                order.begin() + static_cast<std::ptrdiff_t>(target_index + 1U));
  } else {
    std::rotate(order.begin() + static_cast<std::ptrdiff_t>(target_index),
                order.begin() + static_cast<std::ptrdiff_t>(source_index),
                order.begin() + static_cast<std::ptrdiff_t>(source_index + 1U));
  }

  return order;
}

[[nodiscard]] auto make_insert_neighbors(const std::vector<std::size_t> &order,
                                         std::uint32_t seed,
                                         std::uint32_t iteration)
    -> std::vector<std::vector<std::size_t>> {
  std::vector<std::vector<std::size_t>> neighbors;
  if (order.size() < 2U) {
    return neighbors;
  }

  const auto count = order.size();
  neighbors.reserve(count * (count - 1U));

  const auto source_start = static_cast<std::size_t>(seed + iteration) % count;
  const auto target_start =
      static_cast<std::size_t>(seed * 3U + iteration) % count;

  for (std::size_t source_offset = 0; source_offset < count; ++source_offset) {
    const auto source_index = rotate_index(count, source_start, source_offset);
    for (std::size_t target_offset = 0; target_offset < count;
         ++target_offset) {
      const auto target_index =
          rotate_index(count, target_start, target_offset);
      if (source_index == target_index) {
        continue;
      }
      neighbors.push_back(
          move_piece_to_index(order, source_index, target_index));
    }
  }

  return neighbors;
}

[[nodiscard]] auto best_neighbor_from(
    const SearchRequest &request, const LayoutEvaluation &current,
    std::span<const std::vector<std::size_t>> neighbors,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder,
    const std::vector<std::unique_ptr<WorkerContext>> &extra_workers,
    detail::RunInterruptionGate &interruption_gate, SearchCounters &counters)
    -> NeighborChoice {
  NeighborChoice choice{};

  std::vector<EvaluationTask> tasks;
  tasks.reserve(neighbors.size());
  for (std::size_t index = 0U; index < neighbors.size(); ++index) {
    tasks.push_back({
        .task_index = index,
        .order = neighbors[index],
        .key = make_layout_eval_key(request.decoder_request, neighbors[index]),
    });
  }

  auto batch = evaluate_task_batch(request, tasks, cache_store, decoder,
                                   extra_workers, interruption_gate);
  if (batch.interrupted) {
    choice.interrupted = true;
    return choice;
  }

  counters.evaluated_layout_count += batch.counters.evaluated_layout_count;
  counters.reevaluation_cache_hits += batch.counters.reevaluation_cache_hits;

  for (std::size_t index = 0U; index < neighbors.size(); ++index) {
    const auto &order = neighbors[index];
    const auto &candidate = batch.evaluations[index];
    const bool candidate_improved = strictly_better_score(candidate, current);
    const bool candidate_plateau =
        !candidate_improved && same_objective_score(candidate, current);
    if (!candidate_improved && !candidate_plateau) {
      continue;
    }

    const auto candidate_rank = candidate_improved ? 2 : 1;
    if (!choice.found || candidate_rank > choice_rank(choice) ||
        (candidate_rank == choice_rank(choice) &&
         evaluation_better(candidate, choice.evaluation))) {
      choice.found = true;
      choice.improved = candidate_improved;
      choice.plateau = candidate_plateau;
      choice.evaluation = candidate;
      choice.order = order;
    }
  }

  return choice;
}

} // namespace

auto JostleSearch::improve(const SearchRequest &request) -> SearchResult {
  SearchResult result{};
  result.algorithm = kAlgorithmKind;
  result.deterministic_seed = request.local_search.deterministic_seed;

  if (!request.local_search.is_valid() ||
      !request.decoder_request.config.is_valid() ||
      !request.execution.control.is_valid()) {
    return result;
  }

  result.status = SearchRunStatus::completed;
  const RunTiming timing{.control = request.execution.control};
  detail::RunEventSink event_sink{request.execution, result.events};
  detail::RunInterruptionGate interruption_gate{
      request.execution, [&timing]() { return timing.elapsed_ms(); }};

  event_sink.emit(SearchRunStartedEvent{
      .algorithm_kind = kAlgorithmKind,
      .deterministic_seed = request.local_search.deterministic_seed,
      .iteration_budget = request.local_search.max_iterations,
      .piece_count = request.decoder_request.pieces.size(),
      .timestamp_unix_ms = timing.timestamp_unix_ms(),
      .elapsed_ms = timing.event_elapsed_ms(),
  });

  SearchCounters counters{};
  const auto baseline_order =
      identity_order(request.decoder_request.pieces.size());
  result.baseline = evaluate_order(request, baseline_order, reevaluation_cache_,
                                   decoder_, counters, &interruption_gate);
  result.best = result.baseline;
  auto current = result.baseline;

  result.evaluated_layout_count = counters.evaluated_layout_count;
  result.reevaluation_cache_hits = counters.reevaluation_cache_hits;
  const auto &baseline_progress =
      record_progress(result, 0U, request.local_search.max_iterations,
                      SearchMoveKind::none, false, timing);
  event_sink.emit(SearchStepProgressEvent{.progress = baseline_progress});

  if (result.baseline.decode.interrupted) {
    result.status = interruption_gate.status();
    emit_terminal_event(event_sink, request, result, timing);
    return result;
  }

  if (interruption_gate.poll()) {
    result.status = interruption_gate.status();
    emit_terminal_event(event_sink, request, result, timing);
    return result;
  }

  std::vector<std::size_t> current_order = baseline_order;
  std::uint32_t consecutive_non_improving = 0U;
  auto extra_workers =
      make_extra_workers(request.execution.control.worker_count, cache_policy_);

  for (std::uint32_t iteration = 1U;
       iteration <= request.local_search.max_iterations; ++iteration) {
    if (interruption_gate.poll()) {
      result.status = interruption_gate.status();
      emit_terminal_event(event_sink, request, result, timing);
      return result;
    }

    const auto jostle_neighbors = make_jostle_neighbors(
        current_order, request.local_search.deterministic_seed, iteration);
    const auto jostle_best = best_neighbor_from(
        request, current, jostle_neighbors, reevaluation_cache_, decoder_,
        extra_workers, interruption_gate, counters);
    if (jostle_best.interrupted) {
      result.status = interruption_gate.status();
      emit_terminal_event(event_sink, request, result, timing);
      return result;
    }

    SearchMoveKind move_kind = SearchMoveKind::jostle_oscillation;
    auto next_choice = jostle_best;

    if (!jostle_best.improved) {
      const auto insert_neighbors = make_insert_neighbors(
          current_order, request.local_search.deterministic_seed, iteration);
      const auto insert_best = best_neighbor_from(
          request, current, insert_neighbors, reevaluation_cache_, decoder_,
          extra_workers, interruption_gate, counters);
      if (insert_best.interrupted) {
        result.status = interruption_gate.status();
        emit_terminal_event(event_sink, request, result, timing);
        return result;
      }
      if (insert_best.found &&
          (choice_rank(insert_best) > choice_rank(next_choice) ||
           (choice_rank(insert_best) == choice_rank(next_choice) &&
            (!next_choice.found ||
             evaluation_better(insert_best.evaluation,
                               next_choice.evaluation))))) {
        next_choice = insert_best;
        move_kind = SearchMoveKind::one_piece_insert;
      }
    }

    const bool improved = next_choice.improved;

    if (next_choice.found) {
      current = next_choice.evaluation;
      current_order = next_choice.order;
    }

    if (improved) {
      consecutive_non_improving = 0U;
      if (evaluation_better(current, result.best)) {
        result.best = current;
      }
    } else {
      ++consecutive_non_improving;
    }

    result.evaluated_layout_count = counters.evaluated_layout_count;
    result.reevaluation_cache_hits = counters.reevaluation_cache_hits;
    result.iterations_completed = iteration;

    const auto &progress =
        record_progress(result, iteration, request.local_search.max_iterations,
                        move_kind, improved, timing);
    event_sink.emit(SearchStepProgressEvent{.progress = progress});
    if (improved) {
      event_sink.emit(SearchImprovementFoundEvent{.progress = progress});
    }

    if (interruption_gate.poll()) {
      result.status = interruption_gate.status();
      emit_terminal_event(event_sink, request, result, timing);
      return result;
    }

    if (!improved &&
        consecutive_non_improving >= request.local_search.plateau_budget) {
      break;
    }
  }

  emit_terminal_event(event_sink, request, result, timing);
  return result;
}

auto JostleSearch::clear_caches() -> void {
  reevaluation_cache_.clear();
  decoder_.clear_caches();
}

auto JostleSearch::reevaluation_cache_size() const -> std::size_t {
  return reevaluation_cache_.size();
}

} // namespace shiny::nfp::search
