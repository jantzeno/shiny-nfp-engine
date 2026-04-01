#pragma once

#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string_view>

#include "sample_cases.hpp"

#include "packing/masonry.hpp"

namespace shiny::nfp::tooling {

struct MasonryBenchmarkConfig {
  std::size_t runs{200};
};

struct MasonryBenchmarkResult {
  AlgorithmKind algorithm{AlgorithmKind::masonry_builder};
  std::size_t runs{0};
  double total_ms{0.0};
  std::size_t bin_count{0};
  std::size_t unplaced_piece_count{0};
  std::size_t trace_count{0};
  std::size_t progress_count{0};
  std::size_t convex_cache_size{0};
  std::size_t nonconvex_cache_size{0};
  std::size_t decomposition_cache_size{0};
};

template <class Callback>
inline auto measure_total_ms(std::size_t run_count, Callback &&callback)
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

inline auto run_masonry_benchmark(const MasonryBenchmarkConfig &config)
    -> MasonryBenchmarkResult {
  pack::MasonryBuilder builder;
  const auto request = make_masonry_request();
  auto result = builder.build(request);
  const double total_ms =
      measure_total_ms(config.runs, [&] { result = builder.build(request); });

  return {
      .algorithm = result.algorithm,
      .runs = config.runs,
      .total_ms = total_ms,
      .bin_count = result.bins.size(),
      .unplaced_piece_count = result.layout.unplaced_piece_ids.size(),
      .trace_count = result.trace.size(),
      .progress_count = result.progress.size(),
      .convex_cache_size = builder.convex_cache_size(),
      .nonconvex_cache_size = builder.nonconvex_cache_size(),
      .decomposition_cache_size = builder.decomposition_cache_size(),
  };
}

inline void print_metric(std::string_view name, double value) {
  std::cout << name << '=' << std::fixed << std::setprecision(6) << value
            << '\n';
}

inline void print_metric(std::string_view name, std::size_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void
print_masonry_benchmark_result(const MasonryBenchmarkResult &result) {
  std::cout << "scenario=masonry_layout_smoke_v1\n";
  std::cout << "algorithm=" << to_string(result.algorithm) << '\n';
  print_metric("runs", result.runs);
  print_metric("total_ms", result.total_ms);
  print_metric("avg_ms", result.total_ms / static_cast<double>(result.runs));
  print_metric("bin_count", result.bin_count);
  print_metric("unplaced_piece_count", result.unplaced_piece_count);
  print_metric("trace_count", result.trace_count);
  print_metric("progress_count", result.progress_count);
  print_metric("convex_cache_size", result.convex_cache_size);
  print_metric("nonconvex_cache_size", result.nonconvex_cache_size);
  print_metric("decomposition_cache_size", result.decomposition_cache_size);
}

} // namespace shiny::nfp::tooling