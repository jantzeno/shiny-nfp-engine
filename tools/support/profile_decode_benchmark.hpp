#pragma once

#include <chrono>
#include <cstddef>

#include <iomanip>
#include <iostream>
#include <string_view>

#include "sample_cases.hpp"

#include "packing/decoder.hpp"
#include "search/genetic.hpp"
#include "search/jostle.hpp"

namespace shiny::nfp::tooling {

struct ProfileDecodeBenchmarkConfig {
  std::size_t decoder_runs{200};
  std::size_t search_runs{40};
  AlgorithmKind search_algorithm{AlgorithmKind::jostle_search};
};

struct ProfileDecodeBenchmarkResult {
  std::size_t decoder_runs{0};
  double decoder_total_ms{0.0};
  std::size_t decoder_convex_cache_size{0};
  std::size_t decoder_nonconvex_cache_size{0};
  std::size_t decoder_decomposition_cache_size{0};
  std::size_t decoder_unplaced_piece_count{0};

  AlgorithmKind search_algorithm{AlgorithmKind::jostle_search};
  std::size_t search_runs{0};
  double search_total_ms{0.0};
  std::size_t search_best_bin_count{0};
  std::size_t search_best_unplaced_piece_count{0};
  std::size_t search_iterations_completed{0};
  std::size_t search_reevaluation_cache_size{0};
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

inline auto
run_profile_decode_benchmark(const ProfileDecodeBenchmarkConfig &config)
    -> ProfileDecodeBenchmarkResult {
  pack::ConstructiveDecoder decoder;
  const auto decoder_request = make_decoder_request();
  auto decoder_result = decoder.decode(decoder_request);
  const double decoder_total_ms = measure_total_ms(config.decoder_runs, [&] {
    decoder_result = decoder.decode(decoder_request);
  });

  search::SearchResult search_result{};
  double search_total_ms = 0.0;
  std::size_t search_reevaluation_cache_size = 0U;

  if (config.search_algorithm == AlgorithmKind::genetic_search) {
    search::GeneticSearch search;
    const auto search_request = make_genetic_search_request();
    search_result = search.improve(search_request);
    search_total_ms = measure_total_ms(config.search_runs, [&] {
      search_result = search.improve(search_request);
    });
    search_reevaluation_cache_size = search.reevaluation_cache_size();
  } else {
    search::JostleSearch search;
    const auto search_request = make_search_request();
    search_result = search.improve(search_request);
    search_total_ms = measure_total_ms(config.search_runs, [&] {
      search_result = search.improve(search_request);
    });
    search_reevaluation_cache_size = search.reevaluation_cache_size();
  }

  return {
      .decoder_runs = config.decoder_runs,
      .decoder_total_ms = decoder_total_ms,
      .decoder_convex_cache_size = decoder.convex_cache_size(),
      .decoder_nonconvex_cache_size = decoder.nonconvex_cache_size(),
      .decoder_decomposition_cache_size = decoder.decomposition_cache_size(),
      .decoder_unplaced_piece_count =
          decoder_result.layout.unplaced_piece_ids.size(),
      .search_algorithm = search_result.algorithm,
      .search_runs = config.search_runs,
      .search_total_ms = search_total_ms,
      .search_best_bin_count = search_result.best.bin_count,
      .search_best_unplaced_piece_count =
          search_result.best.unplaced_piece_count,
      .search_iterations_completed = search_result.iterations_completed,
      .search_reevaluation_cache_size = search_reevaluation_cache_size,
  };
}

inline void print_metric(std::string_view name, double value) {
  std::cout << name << '=' << std::fixed << std::setprecision(6) << value
            << '\n';
}

inline void print_metric(std::string_view name, std::size_t value) {
  std::cout << name << '=' << value << '\n';
}

inline void print_profile_decode_benchmark_result(
    const ProfileDecodeBenchmarkResult &result) {
  std::cout << "scenario=profile_decode_smoke_v1\n";
  print_metric("decoder_runs", result.decoder_runs);
  print_metric("decoder_total_ms", result.decoder_total_ms);
  print_metric("decoder_avg_ms", result.decoder_total_ms /
                                     static_cast<double>(result.decoder_runs));
  print_metric("decoder_convex_cache_size", result.decoder_convex_cache_size);
  print_metric("decoder_nonconvex_cache_size",
               result.decoder_nonconvex_cache_size);
  print_metric("decoder_decomposition_cache_size",
               result.decoder_decomposition_cache_size);
  print_metric("decoder_unplaced_piece_count",
               result.decoder_unplaced_piece_count);

  std::cout << "algorithm=" << to_string(result.search_algorithm) << '\n';
  print_metric("search_runs", result.search_runs);
  print_metric("search_total_ms", result.search_total_ms);
  print_metric("search_avg_ms", result.search_total_ms /
                                    static_cast<double>(result.search_runs));
  print_metric("search_best_bin_count", result.search_best_bin_count);
  print_metric("search_best_unplaced_piece_count",
               result.search_best_unplaced_piece_count);
  print_metric("search_iterations_completed",
               result.search_iterations_completed);
  print_metric("search_reevaluation_cache_size",
               result.search_reevaluation_cache_size);
}

} // namespace shiny::nfp::tooling