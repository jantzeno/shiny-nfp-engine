#include "search/genetic.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
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
constexpr std::uint32_t kSearchRevision = 2;
constexpr auto kAlgorithmKind = AlgorithmKind::genetic_search;

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

struct PopulationMember {
  std::vector<std::size_t> order{};
  LayoutEvaluation evaluation{};
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
  bool polish{false};
};

struct IndexedPopulationMember {
  std::size_t task_index{0};
  PopulationMember member{};
};

struct BatchEvaluationResult {
  std::vector<PopulationMember> members{};
  SearchCounters counters{};
  bool interrupted{false};
};

struct PopulationInitialization {
  std::vector<PopulationMember> population{};
  SearchCounters counters{};
  bool interrupted{false};
};

struct PolishedPopulationMember {
  PopulationMember member{};
  bool interrupted{false};
};

[[nodiscard]] auto polish_order(
    const SearchRequest &request, const std::vector<std::size_t> &initial_order,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder, SearchCounters &counters,
    detail::RunInterruptionGate &interruption_gate) -> PolishedPopulationMember;

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
      {static_cast<std::uint64_t>(request.bin.base_bin_id),
       request.bin.geometry_revision,
       static_cast<std::uint64_t>(request.policy),
       static_cast<std::uint64_t>(request.max_bin_count),
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
    for (const auto &point : zone.region.outer) {
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.x));
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.y));
    }
  }

  for (const auto &point : request.bin.polygon.outer) {
    seed = shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.x));
    seed = shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.y));
  }
  for (const auto &hole : request.bin.polygon.holes) {
    for (const auto &point : hole) {
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.x));
      seed =
          shiny::nfp::detail::fnv_hash_mix(seed, std::hash<double>{}(point.y));
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

[[nodiscard]] auto select_worker_index(const cache::LayoutEvalKey &key,
                                       std::size_t worker_count)
    -> std::size_t {
  if (worker_count == 0U) {
    return 0U;
  }
  return static_cast<std::size_t>(key.piece_order_hash % worker_count);
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

  std::vector<std::vector<EvaluationTask>> worker_tasks(worker_count);
  for (const auto &task : tasks) {
    worker_tasks[select_worker_index(task.key, worker_count)].push_back(task);
  }

  std::vector<SearchCounters> worker_counters(worker_count);
  std::vector<std::vector<IndexedPopulationMember>> worker_results(
      worker_count);

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

      PopulationMember member{
          .order = task.order,
          .evaluation = evaluate_order(
              request, task.order, task.key, *cache_store, *decoder,
              worker_counters[worker_index], &interruption_gate),
      };
      if (member.evaluation.decode.interrupted) {
        return;
      }

      if (task.polish) {
        auto polished =
            polish_order(request, member.order, *cache_store, *decoder,
                         worker_counters[worker_index], interruption_gate);
        if (polished.interrupted) {
          return;
        }
        member = std::move(polished.member);
      }

      results.push_back(
          {.task_index = task.task_index, .member = std::move(member)});
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

  batch.members.resize(tasks.size());
  for (std::size_t worker_index = 0U; worker_index < worker_count;
       ++worker_index) {
    batch.counters.evaluated_layout_count +=
        worker_counters[worker_index].evaluated_layout_count;
    batch.counters.reevaluation_cache_hits +=
        worker_counters[worker_index].reevaluation_cache_hits;

    for (auto &indexed_member : worker_results[worker_index]) {
      batch.members[indexed_member.task_index] =
          std::move(indexed_member.member);
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
      make_run_summary(result, request.genetic_search.max_generations, timing);
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

[[nodiscard]] auto unique_order_hash(const std::vector<std::size_t> &order)
    -> std::uint64_t {
  auto seed = shiny::nfp::detail::fnv_hash_values(
      {static_cast<std::uint64_t>(order.size())});
  for (const auto value : order) {
    seed = shiny::nfp::detail::fnv_hash_mix(seed,
                                            static_cast<std::uint64_t>(value));
  }
  return seed;
}

auto push_unique_candidate(std::vector<std::vector<std::size_t>> &candidates,
                           std::vector<std::uint64_t> &seen_hashes,
                           std::vector<std::size_t> order) -> void {
  const auto hash = unique_order_hash(order);
  if (std::find(seen_hashes.begin(), seen_hashes.end(), hash) !=
      seen_hashes.end()) {
    return;
  }

  seen_hashes.push_back(hash);
  candidates.push_back(std::move(order));
}

[[nodiscard]] auto seeded_shuffle(std::vector<std::size_t> order,
                                  std::mt19937 &rng)
    -> std::vector<std::size_t> {
  std::shuffle(order.begin(), order.end(), rng);
  return order;
}

[[nodiscard]] auto ordered_crossover(const std::vector<std::size_t> &parent_a,
                                     const std::vector<std::size_t> &parent_b,
                                     std::mt19937 &rng)
    -> std::vector<std::size_t> {
  if (parent_a.size() < 2U) {
    return parent_a;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U,
                                                          parent_a.size() - 1U);
  auto left = distribution(rng);
  auto right = distribution(rng);
  if (left > right) {
    std::swap(left, right);
  }

  std::vector<std::size_t> child(parent_a.size(), parent_a.size());
  std::vector<bool> used(parent_a.size(), false);

  for (std::size_t index = left; index <= right; ++index) {
    child[index] = parent_a[index];
    used[parent_a[index]] = true;
  }

  std::size_t write_index = (right + 1U) % child.size();
  std::size_t read_index = (right + 1U) % parent_b.size();
  while (std::find(child.begin(), child.end(), child.size()) != child.end()) {
    const auto candidate = parent_b[read_index];
    if (!used[candidate]) {
      child[write_index] = candidate;
      used[candidate] = true;
      write_index = (write_index + 1U) % child.size();
    }
    read_index = (read_index + 1U) % parent_b.size();
  }

  return child;
}

auto mutate_order(std::vector<std::size_t> &order, std::mt19937 &rng) -> void {
  if (order.size() < 2U) {
    return;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U,
                                                          order.size() - 1U);
  if ((rng() % 2U) == 0U) {
    auto left = distribution(rng);
    auto right = distribution(rng);
    if (left == right) {
      right = (right + 1U) % order.size();
    }
    if (left > right) {
      std::swap(left, right);
    }
    std::swap(order[left], order[right]);
    return;
  }

  const auto source = distribution(rng);
  auto target = distribution(rng);
  if (source == target) {
    target = (target + 1U) % order.size();
  }
  order = move_piece_to_index(std::move(order), source, target);
}

[[nodiscard]] auto better_member(const PopulationMember &lhs,
                                 const PopulationMember &rhs) -> bool {
  return evaluation_better(lhs.evaluation, rhs.evaluation);
}

[[nodiscard]] auto initialize_population(
    const SearchRequest &request,
    const std::vector<std::size_t> &baseline_order,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder,
    const std::vector<std::unique_ptr<WorkerContext>> &extra_workers,
    detail::RunInterruptionGate &interruption_gate, std::mt19937 &rng)
    -> PopulationInitialization {
  PopulationInitialization initialization{};
  std::vector<std::vector<std::size_t>> candidate_orders;
  std::vector<std::uint64_t> seen_hashes;
  candidate_orders.reserve(request.genetic_search.population_size);
  seen_hashes.reserve(request.genetic_search.population_size);

  push_unique_candidate(candidate_orders, seen_hashes, baseline_order);

  for (const auto &neighbor : make_jostle_neighbors(
           baseline_order, request.genetic_search.deterministic_seed, 1U)) {
    if (candidate_orders.size() >= request.genetic_search.population_size) {
      break;
    }
    push_unique_candidate(candidate_orders, seen_hashes, neighbor);
  }

  for (const auto &neighbor : make_insert_neighbors(
           baseline_order, request.genetic_search.deterministic_seed, 1U)) {
    if (candidate_orders.size() >= request.genetic_search.population_size) {
      break;
    }
    push_unique_candidate(candidate_orders, seen_hashes, neighbor);
  }

  std::size_t shuffle_attempts = 0U;
  const auto max_shuffle_attempts = request.genetic_search.population_size * 8U;
  while (candidate_orders.size() < request.genetic_search.population_size &&
         shuffle_attempts < max_shuffle_attempts) {
    push_unique_candidate(candidate_orders, seen_hashes,
                          seeded_shuffle(baseline_order, rng));
    ++shuffle_attempts;
  }

  while (candidate_orders.size() < request.genetic_search.population_size) {
    candidate_orders.push_back(
        candidate_orders[candidate_orders.size() % seen_hashes.size()]);
  }

  std::vector<EvaluationTask> tasks;
  tasks.reserve(candidate_orders.size());
  for (std::size_t index = 0U; index < candidate_orders.size(); ++index) {
    tasks.push_back({
        .task_index = index,
        .order = candidate_orders[index],
        .key = make_layout_eval_key(request.decoder_request,
                                    candidate_orders[index]),
        .polish = false,
    });
  }

  auto batch = evaluate_task_batch(request, tasks, cache_store, decoder,
                                   extra_workers, interruption_gate);
  if (batch.interrupted) {
    initialization.interrupted = true;
    return initialization;
  }

  initialization.population = std::move(batch.members);
  initialization.counters = batch.counters;

  std::stable_sort(initialization.population.begin(),
                   initialization.population.end(), better_member);
  return initialization;
}

[[nodiscard]] auto
select_parent_index(const std::vector<PopulationMember> &population,
                    std::uint32_t tournament_size, std::mt19937 &rng)
    -> std::size_t {
  std::uniform_int_distribution<std::size_t> distribution(
      0U, population.size() - 1U);
  std::size_t best_index = distribution(rng);
  for (std::uint32_t round = 1U; round < tournament_size; ++round) {
    const auto candidate_index = distribution(rng);
    if (better_member(population[candidate_index], population[best_index])) {
      best_index = candidate_index;
    }
  }
  return best_index;
}

[[nodiscard]] auto polish_order(
    const SearchRequest &request, const std::vector<std::size_t> &initial_order,
    cache::CacheStore<cache::LayoutEvalKey, LayoutEvaluation> &cache_store,
    pack::ConstructiveDecoder &decoder, SearchCounters &counters,
    detail::RunInterruptionGate &interruption_gate)
    -> PolishedPopulationMember {
  auto current_order = initial_order;
  auto current = evaluate_order(request, current_order, cache_store, decoder,
                                counters, &interruption_gate);
  if (current.decode.interrupted) {
    return {.interrupted = true};
  }
  auto best = current;
  auto best_order = current_order;
  std::uint32_t consecutive_non_improving = 0U;

  for (std::uint32_t iteration = 1U;
       iteration <= request.local_search.max_iterations; ++iteration) {
    if (interruption_gate.poll()) {
      return {.interrupted = true};
    }

    const auto jostle_neighbors = make_jostle_neighbors(
        current_order, request.local_search.deterministic_seed, iteration);

    std::optional<PopulationMember> next_member;
    for (const auto &order : jostle_neighbors) {
      if (interruption_gate.poll()) {
        return {.interrupted = true};
      }

      const auto candidate = evaluate_order(
          request, order, cache_store, decoder, counters, &interruption_gate);
      if (candidate.decode.interrupted) {
        return {.interrupted = true};
      }
      if (strictly_better_score(candidate, current) ||
          same_objective_score(candidate, current)) {
        PopulationMember member{.order = order, .evaluation = candidate};
        if (!next_member.has_value() || better_member(member, *next_member)) {
          next_member = std::move(member);
        }
      }
    }

    if (!next_member.has_value()) {
      const auto insert_neighbors = make_insert_neighbors(
          current_order, request.local_search.deterministic_seed, iteration);
      for (const auto &order : insert_neighbors) {
        if (interruption_gate.poll()) {
          return {.interrupted = true};
        }

        const auto candidate = evaluate_order(
            request, order, cache_store, decoder, counters, &interruption_gate);
        if (candidate.decode.interrupted) {
          return {.interrupted = true};
        }
        if (strictly_better_score(candidate, current) ||
            same_objective_score(candidate, current)) {
          PopulationMember member{.order = order, .evaluation = candidate};
          if (!next_member.has_value() || better_member(member, *next_member)) {
            next_member = std::move(member);
          }
        }
      }
    }

    if (!next_member.has_value()) {
      break;
    }

    const bool improved =
        strictly_better_score(next_member->evaluation, current);
    current = next_member->evaluation;
    current_order = next_member->order;

    if (evaluation_better(current, best)) {
      best = current;
      best_order = current_order;
    }

    if (improved) {
      consecutive_non_improving = 0U;
    } else {
      ++consecutive_non_improving;
      if (consecutive_non_improving >= request.local_search.plateau_budget) {
        break;
      }
    }
  }

  return {
      .member = {.order = std::move(best_order), .evaluation = best},
  };
}

} // namespace

auto GeneticSearch::improve(const SearchRequest &request) -> SearchResult {
  SearchResult result{};
  result.algorithm = kAlgorithmKind;
  result.deterministic_seed = request.genetic_search.deterministic_seed;

  if (!request.genetic_search.enabled || !request.genetic_search.is_valid() ||
      !request.local_search.is_valid() ||
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
      .deterministic_seed = request.genetic_search.deterministic_seed,
      .iteration_budget = request.genetic_search.max_generations,
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

  result.evaluated_layout_count = counters.evaluated_layout_count;
  result.reevaluation_cache_hits = counters.reevaluation_cache_hits;
  const auto &baseline_progress =
      record_progress(result, 0U, request.genetic_search.max_generations,
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

  std::mt19937 rng(request.genetic_search.deterministic_seed);
  auto extra_workers =
      make_extra_workers(request.execution.control.worker_count, cache_policy_);
  auto initialization =
      initialize_population(request, baseline_order, reevaluation_cache_,
                            decoder_, extra_workers, interruption_gate, rng);
  if (initialization.interrupted) {
    result.status = interruption_gate.status();
    emit_terminal_event(event_sink, request, result, timing);
    return result;
  }
  counters.evaluated_layout_count +=
      initialization.counters.evaluated_layout_count;
  counters.reevaluation_cache_hits +=
      initialization.counters.reevaluation_cache_hits;
  auto population = std::move(initialization.population);

  std::uint32_t consecutive_plateau_generations = 0U;
  for (std::uint32_t generation = 1U;
       generation <= request.genetic_search.max_generations; ++generation) {
    if (interruption_gate.poll()) {
      result.status = interruption_gate.status();
      emit_terminal_event(event_sink, request, result, timing);
      return result;
    }

    std::stable_sort(population.begin(), population.end(), better_member);
    std::vector<PopulationMember> next_population;
    next_population.reserve(population.size());

    const auto elite_count = std::min<std::size_t>(
        request.genetic_search.elite_count, population.size());
    for (std::size_t index = 0; index < elite_count; ++index) {
      next_population.push_back(population[index]);
    }

    std::vector<EvaluationTask> child_tasks;
    child_tasks.reserve(population.size() - elite_count);
    while (elite_count + child_tasks.size() < population.size()) {
      const auto first_index = select_parent_index(
          population, request.genetic_search.tournament_size, rng);
      const auto second_index = select_parent_index(
          population, request.genetic_search.tournament_size, rng);

      auto child_order = ordered_crossover(population[first_index].order,
                                           population[second_index].order, rng);
      if ((rng() % 100U) < request.genetic_search.mutation_rate_percent) {
        mutate_order(child_order, rng);
      }

      child_tasks.push_back({
          .task_index = child_tasks.size(),
          .order = std::move(child_order),
          .key = {},
          .polish = request.genetic_search.enable_local_search_polish &&
                    request.local_search.max_iterations > 0U,
      });
    }

    if (!child_tasks.empty()) {
      for (auto &task : child_tasks) {
        task.key = make_layout_eval_key(request.decoder_request, task.order);
      }

      auto batch =
          evaluate_task_batch(request, child_tasks, reevaluation_cache_,
                              decoder_, extra_workers, interruption_gate);
      if (batch.interrupted) {
        result.status = interruption_gate.status();
        emit_terminal_event(event_sink, request, result, timing);
        return result;
      }

      counters.evaluated_layout_count += batch.counters.evaluated_layout_count;
      counters.reevaluation_cache_hits +=
          batch.counters.reevaluation_cache_hits;

      for (auto &member : batch.members) {
        next_population.push_back(std::move(member));
      }
    }

    population = std::move(next_population);
    std::stable_sort(population.begin(), population.end(), better_member);

    const bool improved =
        !population.empty() &&
        evaluation_better(population.front().evaluation, result.best);
    if (improved) {
      result.best = population.front().evaluation;
      consecutive_plateau_generations = 0U;
    } else {
      ++consecutive_plateau_generations;
    }

    result.evaluated_layout_count = counters.evaluated_layout_count;
    result.reevaluation_cache_hits = counters.reevaluation_cache_hits;
    result.iterations_completed = generation;

    const auto &progress = record_progress(
        result, generation, request.genetic_search.max_generations,
        SearchMoveKind::genetic_generation, improved, timing);
    event_sink.emit(SearchStepProgressEvent{.progress = progress});
    if (improved) {
      event_sink.emit(SearchImprovementFoundEvent{.progress = progress});
    }

    if (interruption_gate.poll()) {
      result.status = interruption_gate.status();
      emit_terminal_event(event_sink, request, result, timing);
      return result;
    }

    if (consecutive_plateau_generations >=
        request.genetic_search.plateau_generations) {
      break;
    }
  }

  emit_terminal_event(event_sink, request, result, timing);
  return result;
}

auto GeneticSearch::clear_caches() -> void {
  reevaluation_cache_.clear();
  decoder_.clear_caches();
}

auto GeneticSearch::reevaluation_cache_size() const -> std::size_t {
  return reevaluation_cache_.size();
}

} // namespace shiny::nfp::search