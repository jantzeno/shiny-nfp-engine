#include <iostream>
#include <string_view>
#include <vector>

#include "../../tools/support/sample_cases.hpp"
#include "search/genetic.hpp"

namespace {

using shiny::nfp::search::GeneticSearch;
using shiny::nfp::search::SearchResult;

void print_piece_order(std::string_view label,
                       const std::vector<std::uint32_t> &piece_order) {
  std::cout << label << ": ";
  for (std::size_t index = 0; index < piece_order.size(); ++index) {
    if (index > 0U) {
      std::cout << ", ";
    }
    std::cout << piece_order[index];
  }
  std::cout << '\n';
}

void print_summary(const shiny::nfp::search::SearchRequest &request,
                   const SearchResult &result) {
  std::cout << "algorithm=" << shiny::nfp::to_string(result.algorithm) << '\n';
  std::cout << "GA enabled: " << (request.genetic_search.enabled ? "yes" : "no")
            << '\n';
  std::cout << "Population size: " << request.genetic_search.population_size
            << '\n';
  std::cout << "Generation budget: " << request.genetic_search.max_generations
            << '\n';
  std::cout << "Deterministic seed: " << result.deterministic_seed << '\n';
  std::cout << "Generations completed: " << result.iterations_completed << '\n';
  std::cout << "Layouts evaluated: " << result.evaluated_layout_count << '\n';
  std::cout << "Reevaluation cache hits: " << result.reevaluation_cache_hits
            << '\n';
  std::cout << "Improved: " << (result.improved() ? "yes" : "no") << '\n';
  std::cout << "Baseline unplaced pieces: "
            << result.baseline.unplaced_piece_count << '\n';
  std::cout << "Best unplaced pieces: " << result.best.unplaced_piece_count
            << '\n';
  std::cout << "Best bin count: " << result.best.bin_count << '\n';
  std::cout << "Best total utilization: " << result.best.total_utilization
            << '\n';
  print_piece_order("Baseline piece order", result.baseline.piece_order);
  print_piece_order("Best piece order", result.best.piece_order);
}

} // namespace

auto main() -> int {
  GeneticSearch search;
  const auto request = shiny::nfp::tooling::make_genetic_search_request();
  const auto result = search.improve(request);
  print_summary(request, result);
  return 0;
}