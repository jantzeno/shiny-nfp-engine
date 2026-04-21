#include "packing/separator.hpp"

#include <algorithm>
#include <future>
#include <numeric>
#include <optional>
#include <tuple>
#include <vector>

#include "runtime/deterministic_rng.hpp"

namespace shiny::nesting::pack {
namespace {

auto run_separator_worker(const geom::PolygonWithHoles &container,
                          const std::vector<CollisionTrackerItem> &items,
                          const SeparatorConfig &config,
                          const std::uint64_t seed,
                          const std::size_t worker_index) -> SeparationResult {
  runtime::DeterministicRng rng(seed);
  CollisionTracker tracker(container, items);
  std::size_t strikes = 0;
  std::size_t iterations = 0;
  const bool shuffle_item_order = config.worker_count > 1U && worker_index > 0U;

  while (tracker.exact_total_loss() > 0.0 && strikes < config.strike_limit &&
         iterations < config.max_iterations) {
    bool improved = false;
    std::size_t no_improvement = 0;
    std::vector<std::size_t> order(tracker.item_count());
    std::iota(order.begin(), order.end(), 0U);
    if (shuffle_item_order) {
      rng.shuffle(order);
    }

    while (no_improvement < config.iter_no_improvement_limit &&
           iterations < config.max_iterations) {
      bool iteration_improved = false;
      for (const auto item_index : order) {
        if (tracker.container_loss(item_index) <= 0.0) {
          bool colliding = false;
          for (std::size_t other = 0; other < tracker.item_count(); ++other) {
            if (other != item_index && tracker.pair_loss(item_index, other) > 0.0) {
              colliding = true;
              break;
            }
          }
          if (!colliding) {
            continue;
          }
        }

        const auto move = move_item(tracker, item_index, config.mover, rng);
        if (!move.has_value()) {
          continue;
        }
        tracker.register_item_move(item_index, move->polygon);
        iteration_improved = true;
      }

      tracker.update_gls_weights(config.gls_weight_cap);
      ++iterations;
      if (iteration_improved) {
        improved = true;
        no_improvement = 0;
      } else {
        ++no_improvement;
      }
      if (tracker.exact_total_loss() <= 0.0) {
        break;
      }
    }

    if (!improved) {
      ++strikes;
    } else {
      strikes = 0;
    }
  }

  SeparationResult result;
  result.total_loss = tracker.exact_total_loss();
  result.converged = result.total_loss <= 1e-8;
  result.iterations = iterations;
  result.polygons.reserve(tracker.item_count());
  for (std::size_t index = 0; index < tracker.item_count(); ++index) {
    result.polygons.push_back(tracker.item(index).polygon);
  }
  return result;
}

} // namespace

// Sparrow Algorithm 9 — separation loop.
//
// Termination structure (two nested loops + strike counter):
//
//   loop OUTER (strikes < strike_limit):
//     loop INNER (no_improvement < iter_no_improvement_limit):
//       for each item in fixed order:
//         skip if not violating any constraint
//         try move_item; on success, register the move
//       update_gls_weights()  // every inner iteration
//       if any item moved this iter: no_improvement = 0, improved = true
//       else: ++no_improvement
//       break inner if total_loss() <= 0
//     if INNER finished WITHOUT any improvement: ++strikes
//     else: strikes = 0
//   stop when total_loss() <= 0, OR strikes reach limit, OR max_iterations.
//
// IMPORTANT: `total_loss() > 0` is the loop condition but
// `CollisionTracker::compute_pair_loss` returns >0 even for
// non-overlapping pole-intersecting pairs (see Phase 5 review). For
// such inputs the loop runs to `max_iterations` even when no actual
// overlap remains; `converged` will reflect this (`<= 1e-8`).
//
// Multi-worker (`worker_count > 1`):
//   * Worker 0 gets the same seed as the single-worker path; later
//     workers get `seed + worker_id`.
//   * Workers run independently with their own tracker clone; the best
//     `(total_loss, worker_id)` pair wins so equal-loss ties stay
//     deterministic.
//   * Worker 0 preserves the single-worker baseline ordering; later
//     workers reshuffle item order once per outer strike cycle using
//     their own RNG state for additional search diversity.
auto run_separator(const geom::PolygonWithHoles &container,
                   const std::vector<CollisionTrackerItem> &items,
                   const SeparatorConfig &config, const std::uint64_t seed)
    -> SeparationResult {
  const auto worker_count = std::max<std::size_t>(1U, config.worker_count);
  if (worker_count == 1U) {
    return run_separator_worker(container, items, config, seed, 0U);
  }

  std::vector<std::future<SeparationResult>> workers;
  workers.reserve(worker_count);
  for (std::size_t worker = 0; worker < worker_count; ++worker) {
    workers.push_back(std::async(std::launch::async, [&, worker] {
      return run_separator_worker(container, items, config, seed + worker, worker);
    }));
  }

  std::optional<SeparationResult> best;
  std::size_t best_worker_index = 0U;
  for (std::size_t worker_index = 0; worker_index < workers.size(); ++worker_index) {
    auto result = workers[worker_index].get();
    if (!best.has_value() ||
        std::tie(result.total_loss, worker_index) <
            std::tie(best->total_loss, best_worker_index)) {
      best = std::move(result);
      best_worker_index = worker_index;
    }
  }
  return *best;
}

} // namespace shiny::nesting::pack
