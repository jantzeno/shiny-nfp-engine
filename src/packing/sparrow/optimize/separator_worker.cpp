#include "packing/sparrow/optimize/separator_worker.hpp"

#include <algorithm>
#include <thread>
#include <tuple>

#include "packing/sparrow/runtime/rng.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

auto run_separator_workers(const adapters::PortPolygon &container,
                           std::span<const SeparatorItem> items,
                           const SeparatorConfig &config,
                           const std::uint64_t worker_seed_base,
                           const std::size_t worker_count)
    -> SeparatorWorkerRun {
  const auto resolved_worker_count = std::max<std::size_t>(1U, worker_count);

  // Pre-allocate one slot per worker so each std::jthread writes to a unique
  // index without synchronization. The jthread's built-in stop_token is
  // forwarded to run_separator for cooperative cancellation.
  SeparatorWorkerRun run;
  run.workers.resize(resolved_worker_count);

  {
    std::vector<std::jthread> threads;
    threads.reserve(resolved_worker_count);

    for (std::size_t worker_index = 0; worker_index < resolved_worker_count;
         ++worker_index) {
      // Capture worker_seed_base by value so no reference to a stack variable
      // escapes into the async context.
      threads.emplace_back(
          [&container, &items, &config, &run, worker_index,
           seed_base = worker_seed_base](std::stop_token st) {
            runtime::TraceCapture trace;
            const auto worker_seed =
                runtime::derive_worker_seed(seed_base, worker_index);
            runtime::SplitMix64Rng rng(worker_seed);
            run.workers[worker_index] = SeparatorWorkerResult{
                .worker_index = worker_index,
                .worker_seed = worker_seed,
                .result =
                    run_separator(container, items, config, rng, &trace,
                                  std::move(st)),
                .trace = std::move(trace),
            };
          });
    }
    // All jthreads join automatically here when the vector goes out of scope.
  }

  const auto best_it = std::min_element(
      run.workers.begin(), run.workers.end(),
      [](const auto &lhs, const auto &rhs) {
        return std::tie(lhs.result.total_loss, lhs.worker_index) <
               std::tie(rhs.result.total_loss, rhs.worker_index);
      });
  run.best = *best_it;
  return run;
}

} // namespace shiny::nesting::pack::sparrow::optimize
