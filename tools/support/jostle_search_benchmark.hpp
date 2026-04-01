#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <iomanip>
#include <iostream>
#include <string_view>

#include "sample_cases.hpp"

#include "search/jostle.hpp"

namespace shiny::nfp::tooling {

enum class JostleSearchBenchmarkCase : std::uint8_t {
  smoke = 0,
  dense = 1,
};

struct JostleSearchBenchmarkConfig {
  std::size_t runs{50};
  std::uint32_t worker_count{1};
  JostleSearchBenchmarkCase benchmark_case{JostleSearchBenchmarkCase::smoke};
  bool compare_to_serial{false};
};

struct JostleSearchBenchmarkResult {
  AlgorithmKind algorithm{AlgorithmKind::jostle_search};
  std::size_t runs{0};
  std::uint32_t worker_count{1};
  JostleSearchBenchmarkCase benchmark_case{JostleSearchBenchmarkCase::smoke};
  std::size_t piece_count{0};
  double total_ms{0.0};
  std::size_t best_bin_count{0};
  std::size_t best_unplaced_piece_count{0};
  std::size_t iterations_completed{0};
  std::size_t reevaluation_cache_size{0};
  double serial_total_ms{0.0};
  double avg_over_serial_ratio{0.0};
};

struct JostleSearchWorkerScalingBenchmarkConfig {
  std::size_t runs_per_worker{5};
  JostleSearchBenchmarkCase benchmark_case{JostleSearchBenchmarkCase::dense};
};

struct JostleSearchWorkerScalingBenchmarkResult {
  AlgorithmKind algorithm{AlgorithmKind::jostle_search};
  JostleSearchBenchmarkCase benchmark_case{JostleSearchBenchmarkCase::dense};
  std::size_t runs_per_worker{0};
  std::size_t piece_count{0};
  double serial_avg_ms{0.0};
  std::size_t best_parallel_worker_count{0};
  double best_parallel_avg_ms{0.0};
  std::size_t worst_parallel_worker_count{0};
  double worst_parallel_avg_ms{0.0};
  double max_parallel_over_serial_ratio{0.0};
  double parallel_spread_ratio{0.0};
};

template <class Callback>
inline auto measure_jostle_total_ms(std::size_t run_count, Callback &&callback)
    -> double {
  const auto started = std::chrono::steady_clock::now();
  for (std::size_t index = 0; index < run_count; ++index) {
    callback();
  }
  const auto finished = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          finished - started);
  return elapsed.count();
}

inline auto
run_jostle_search_benchmark(const JostleSearchBenchmarkConfig &config)
    -> JostleSearchBenchmarkResult {
  auto request = config.benchmark_case == JostleSearchBenchmarkCase::dense
                     ? make_dense_search_request()
                     : make_search_request();
  request.execution.control.worker_count = config.worker_count;

  search::SearchResult result{};
  std::size_t reevaluation_cache_size = 0U;
  double total_ms = 0.0;

  if (config.benchmark_case == JostleSearchBenchmarkCase::dense) {
    const auto run_once = [&]() {
      search::JostleSearch run_search;
      result = run_search.improve(request);
      reevaluation_cache_size = run_search.reevaluation_cache_size();
    };

    run_once();
    total_ms = measure_jostle_total_ms(config.runs, run_once);
  } else {
    search::JostleSearch search;
    result = search.improve(request);
    total_ms = measure_jostle_total_ms(
        config.runs, [&] { result = search.improve(request); });
    reevaluation_cache_size = search.reevaluation_cache_size();
  }

  auto serial_total_ms = 0.0;
  auto avg_over_serial_ratio = 0.0;
  if (config.compare_to_serial && config.worker_count > 1U) {
    const auto serial_result = run_jostle_search_benchmark({
        .runs = config.runs,
        .worker_count = 1U,
        .benchmark_case = config.benchmark_case,
        .compare_to_serial = false,
    });
    serial_total_ms = serial_result.total_ms;
    if (serial_total_ms > 0.0) {
      avg_over_serial_ratio = total_ms / serial_total_ms;
    }
  }

  return {
      .algorithm = result.algorithm,
      .runs = config.runs,
      .worker_count = config.worker_count,
      .benchmark_case = config.benchmark_case,
      .piece_count = request.decoder_request.pieces.size(),
      .total_ms = total_ms,
      .best_bin_count = result.best.bin_count,
      .best_unplaced_piece_count = result.best.unplaced_piece_count,
      .iterations_completed = result.iterations_completed,
      .reevaluation_cache_size = reevaluation_cache_size,
      .serial_total_ms = serial_total_ms,
      .avg_over_serial_ratio = avg_over_serial_ratio,
  };
}

inline auto run_jostle_search_worker_scaling_benchmark(
    const JostleSearchWorkerScalingBenchmarkConfig &config)
    -> JostleSearchWorkerScalingBenchmarkResult {
  constexpr std::array<std::uint32_t, 6> worker_counts = {1U, 2U, 3U,
                                                          4U, 5U, 6U};

  JostleSearchWorkerScalingBenchmarkResult scaling{};
  scaling.benchmark_case = config.benchmark_case;
  scaling.runs_per_worker = config.runs_per_worker;

  auto best_parallel_avg_ms = std::numeric_limits<double>::infinity();
  auto worst_parallel_avg_ms = 0.0;

  for (const auto worker_count : worker_counts) {
    const auto result = run_jostle_search_benchmark({
        .runs = config.runs_per_worker,
        .worker_count = worker_count,
        .benchmark_case = config.benchmark_case,
    });

    scaling.algorithm = result.algorithm;
    scaling.piece_count = result.piece_count;
    const auto avg_ms = result.total_ms / static_cast<double>(result.runs);

    if (worker_count == 1U) {
      scaling.serial_avg_ms = avg_ms;
      continue;
    }

    if (avg_ms < best_parallel_avg_ms) {
      best_parallel_avg_ms = avg_ms;
      scaling.best_parallel_avg_ms = avg_ms;
      scaling.best_parallel_worker_count = worker_count;
    }
    if (avg_ms > worst_parallel_avg_ms) {
      worst_parallel_avg_ms = avg_ms;
      scaling.worst_parallel_avg_ms = avg_ms;
      scaling.worst_parallel_worker_count = worker_count;
    }
  }

  if (scaling.serial_avg_ms > 0.0) {
    scaling.max_parallel_over_serial_ratio =
        scaling.worst_parallel_avg_ms / scaling.serial_avg_ms;
  }
  if (scaling.best_parallel_avg_ms > 0.0) {
    scaling.parallel_spread_ratio =
        scaling.worst_parallel_avg_ms / scaling.best_parallel_avg_ms;
  }

  return scaling;
}

inline void print_jostle_metric(std::string_view name, double value) {
  std::cout << name << '=' << std::fixed << std::setprecision(6) << value
            << '\n';
}

inline void print_jostle_metric(std::string_view name, std::size_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void print_jostle_metric(std::string_view name, std::uint32_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void print_jostle_search_benchmark_result(
    const JostleSearchBenchmarkResult &result) {
  if (result.benchmark_case == JostleSearchBenchmarkCase::dense) {
    if (result.worker_count > 1U) {
      std::cout << "scenario=jostle_search_parallel_dense_smoke_v1\n";
    } else {
      std::cout << "scenario=jostle_search_dense_smoke_v1\n";
    }
  } else if (result.worker_count > 1U) {
    std::cout << "scenario=jostle_search_parallel_smoke_v1\n";
  } else {
    std::cout << "scenario=jostle_search_smoke_v1\n";
  }
  std::cout << "algorithm=" << to_string(result.algorithm) << '\n';
  print_jostle_metric("runs", result.runs);
  print_jostle_metric("worker_count",
                      static_cast<std::size_t>(result.worker_count));
  print_jostle_metric("piece_count", result.piece_count);
  print_jostle_metric("total_ms", result.total_ms);
  print_jostle_metric("avg_ms",
                      result.total_ms / static_cast<double>(result.runs));
  print_jostle_metric("best_bin_count", result.best_bin_count);
  print_jostle_metric("best_unplaced_piece_count",
                      result.best_unplaced_piece_count);
  print_jostle_metric("iterations_completed", result.iterations_completed);
  print_jostle_metric("reevaluation_cache_size",
                      result.reevaluation_cache_size);
  if (result.serial_total_ms > 0.0) {
    print_jostle_metric("serial_total_ms", result.serial_total_ms);
    print_jostle_metric("serial_avg_ms", result.serial_total_ms /
                                             static_cast<double>(result.runs));
    print_jostle_metric("avg_over_serial_ratio", result.avg_over_serial_ratio);
  }
}

inline void print_jostle_search_worker_scaling_benchmark_result(
    const JostleSearchWorkerScalingBenchmarkResult &result) {
  if (result.benchmark_case == JostleSearchBenchmarkCase::dense) {
    std::cout << "scenario=jostle_search_dense_worker_scaling_v1\n";
  } else {
    std::cout << "scenario=jostle_search_worker_scaling_v1\n";
  }
  std::cout << "algorithm=" << to_string(result.algorithm) << '\n';
  print_jostle_metric("runs_per_worker", result.runs_per_worker);
  print_jostle_metric("piece_count", result.piece_count);
  print_jostle_metric("serial_avg_ms", result.serial_avg_ms);
  print_jostle_metric("best_parallel_worker_count",
                      result.best_parallel_worker_count);
  print_jostle_metric("best_parallel_avg_ms", result.best_parallel_avg_ms);
  print_jostle_metric("worst_parallel_worker_count",
                      result.worst_parallel_worker_count);
  print_jostle_metric("worst_parallel_avg_ms", result.worst_parallel_avg_ms);
  print_jostle_metric("max_parallel_over_serial_ratio",
                      result.max_parallel_over_serial_ratio);
  print_jostle_metric("parallel_spread_ratio", result.parallel_spread_ratio);
}

} // namespace shiny::nfp::tooling