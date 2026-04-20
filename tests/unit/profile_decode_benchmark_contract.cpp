#include <catch2/catch_test_macros.hpp>

#include <sstream>
#include <string>

#include "../../tools/support/profile_decode_benchmark.hpp"

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

TEST_CASE("profile decode benchmark emits canonical algorithm metric",
          "[benchmark][tooling][algorithm_kind]") {
  const auto output = capture_stdout([] {
    shiny::nesting::tooling::print_profile_decode_benchmark_result(
        {.decoder_runs = 10,
         .decoder_total_ms = 1.0,
         .decoder_convex_cache_size = 1,
         .decoder_nonconvex_cache_size = 2,
         .decoder_decomposition_cache_size = 3,
         .decoder_unplaced_piece_count = 0,
         .search_algorithm = shiny::nesting::AlgorithmKind::genetic_search,
         .search_runs = 5,
         .search_total_ms = 2.0,
         .search_best_bin_count = 1,
         .search_best_unplaced_piece_count = 0,
         .search_iterations_completed = 4,
         .search_reevaluation_cache_size = 6});
  });

  REQUIRE(output.find("algorithm=genetic_search") != std::string::npos);
  REQUIRE(output.find("search_algorithm=") == std::string::npos);
}