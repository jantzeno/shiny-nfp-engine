#include <catch2/catch_test_macros.hpp>

#include <sstream>
#include <string>

#include "../../tools/support/genetic_search_benchmark.hpp"
#include "../../tools/support/jostle_search_benchmark.hpp"
#include "../../tools/support/masonry_benchmark.hpp"

namespace {

template <class Callback>
auto capture_stdout(Callback &&callback) -> std::string {
  std::ostringstream buffer;
  auto *const original = std::cout.rdbuf(buffer.rdbuf());
  try {
    std::forward<Callback>(callback)();
  } catch (...) {
    std::cout.rdbuf(original);
    throw;
  }
  std::cout.rdbuf(original);
  return buffer.str();
}

} // namespace

TEST_CASE("benchmark support emits canonical algorithm metrics",
          "[benchmark][tooling][algorithm_kind]") {
  const auto jostle_output = capture_stdout([] {
    shiny::nesting::tooling::print_jostle_search_benchmark_result(
        {.algorithm = shiny::nesting::AlgorithmKind::jostle_search,
         .runs = 5,
         .worker_count = 1,
         .benchmark_case =
             shiny::nesting::tooling::JostleSearchBenchmarkCase::smoke,
         .total_ms = 5.0,
         .best_bin_count = 1,
         .best_unplaced_piece_count = 0,
         .iterations_completed = 3,
         .reevaluation_cache_size = 2});
  });
  REQUIRE(jostle_output.find("algorithm=jostle_search") != std::string::npos);
  REQUIRE(jostle_output.find("search_algorithm=") == std::string::npos);

  const auto scaling_output = capture_stdout([] {
    shiny::nesting::tooling::print_jostle_search_worker_scaling_benchmark_result(
        {.algorithm = shiny::nesting::AlgorithmKind::jostle_search,
         .benchmark_case =
             shiny::nesting::tooling::JostleSearchBenchmarkCase::dense,
         .runs_per_worker = 2,
         .piece_count = 8,
         .serial_avg_ms = 10.0,
         .best_parallel_worker_count = 4,
         .best_parallel_avg_ms = 6.0,
         .worst_parallel_worker_count = 2,
         .worst_parallel_avg_ms = 8.0,
         .max_parallel_over_serial_ratio = 0.8,
         .parallel_spread_ratio = 1.333333});
  });
  REQUIRE(scaling_output.find("algorithm=jostle_search") != std::string::npos);
  REQUIRE(scaling_output.find("search_algorithm=") == std::string::npos);

  const auto genetic_output = capture_stdout([] {
    shiny::nesting::tooling::print_genetic_search_benchmark_result(
        {.algorithm = shiny::nesting::AlgorithmKind::genetic_search,
         .runs = 5,
         .worker_count = 1,
         .total_ms = 5.0,
         .best_bin_count = 1,
         .best_unplaced_piece_count = 0,
         .generations_completed = 3,
         .reevaluation_cache_size = 2});
  });
  REQUIRE(genetic_output.find("algorithm=genetic_search") != std::string::npos);
  REQUIRE(genetic_output.find("search_algorithm=") == std::string::npos);

  const auto masonry_output = capture_stdout([] {
    shiny::nesting::tooling::print_masonry_benchmark_result(
        {.algorithm = shiny::nesting::AlgorithmKind::masonry_builder,
         .runs = 5,
         .total_ms = 5.0,
         .bin_count = 1,
         .unplaced_piece_count = 0,
         .trace_count = 3,
         .progress_count = 3,
         .convex_cache_size = 1,
         .nonconvex_cache_size = 2,
         .decomposition_cache_size = 3});
  });
  REQUIRE(masonry_output.find("algorithm=masonry_builder") !=
          std::string::npos);
}