#pragma once

#include <chrono>
#include <cstddef>

#include <iomanip>
#include <iostream>
#include <string_view>

#include "sample_cases.hpp"

#include "search/genetic.hpp"

namespace shiny::nfp::tooling {

struct GeneticSearchBenchmarkConfig {
  std::size_t runs{50};
  std::uint32_t worker_count{1};
  bool compare_to_serial{false};
};

struct GeneticSearchBenchmarkResult {
  AlgorithmKind algorithm{AlgorithmKind::genetic_search};
  std::size_t runs{0};
  std::uint32_t worker_count{1};
  double total_ms{0.0};
  std::size_t best_bin_count{0};
  std::size_t best_unplaced_piece_count{0};
  std::size_t generations_completed{0};
  std::size_t reevaluation_cache_size{0};
  double serial_total_ms{0.0};
  double avg_over_serial_ratio{0.0};
};

template <class Callback>
inline auto measure_genetic_total_ms(std::size_t run_count, Callback &&callback)
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
run_genetic_search_benchmark(const GeneticSearchBenchmarkConfig &config)
    -> GeneticSearchBenchmarkResult {
  search::GeneticSearch search;
  auto request = make_genetic_search_request();
  request.execution.control.worker_count = config.worker_count;
  auto result = search.improve(request);
  const double total_ms = measure_genetic_total_ms(
      config.runs, [&] { result = search.improve(request); });

  auto serial_total_ms = 0.0;
  auto avg_over_serial_ratio = 0.0;
  if (config.compare_to_serial && config.worker_count > 1U) {
    const auto serial_result = run_genetic_search_benchmark({
        .runs = config.runs,
        .worker_count = 1U,
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
      .total_ms = total_ms,
      .best_bin_count = result.best.bin_count,
      .best_unplaced_piece_count = result.best.unplaced_piece_count,
      .generations_completed = result.iterations_completed,
      .reevaluation_cache_size = search.reevaluation_cache_size(),
      .serial_total_ms = serial_total_ms,
      .avg_over_serial_ratio = avg_over_serial_ratio,
  };
}

inline void print_genetic_metric(std::string_view name, double value) {
  std::cout << name << '=' << std::fixed << std::setprecision(6) << value
            << '\n';
}

inline void print_genetic_metric(std::string_view name, std::size_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void print_genetic_metric(std::string_view name, std::uint32_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void print_genetic_search_benchmark_result(
    const GeneticSearchBenchmarkResult &result) {
  if (result.worker_count > 1U) {
    std::cout << "scenario=genetic_search_parallel_smoke_v1\n";
  } else {
    std::cout << "scenario=genetic_search_smoke_v1\n";
  }
  std::cout << "algorithm=" << to_string(result.algorithm) << '\n';
  print_genetic_metric("runs", result.runs);
  print_genetic_metric("worker_count",
                       static_cast<std::size_t>(result.worker_count));
  print_genetic_metric("total_ms", result.total_ms);
  print_genetic_metric("avg_ms",
                       result.total_ms / static_cast<double>(result.runs));
  print_genetic_metric("best_bin_count", result.best_bin_count);
  print_genetic_metric("best_unplaced_piece_count",
                       result.best_unplaced_piece_count);
  print_genetic_metric("generations_completed", result.generations_completed);
  print_genetic_metric("reevaluation_cache_size",
                       result.reevaluation_cache_size);
  if (result.serial_total_ms > 0.0) {
    print_genetic_metric("serial_total_ms", result.serial_total_ms);
    print_genetic_metric("serial_avg_ms", result.serial_total_ms /
                                              static_cast<double>(result.runs));
    print_genetic_metric("avg_over_serial_ratio", result.avg_over_serial_ratio);
  }
}

} // namespace shiny::nfp::tooling