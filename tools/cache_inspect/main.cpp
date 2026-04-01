#include <cstdlib>

#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "../support/sample_cases.hpp"

#include "packing/decoder.hpp"
#include "search/genetic.hpp"
#include "search/jostle.hpp"

namespace {

enum class InspectMode {
  all,
  decoder,
  search,
};

auto parse_count_argument(const char *value, std::string_view label)
    -> std::size_t {
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value || (end != nullptr && *end != '\0') || parsed == 0UL) {
    throw std::runtime_error("invalid run count for " + std::string(label));
  }
  return static_cast<std::size_t>(parsed);
}

auto parse_mode(std::string_view value) -> InspectMode {
  if (value == "all") {
    return InspectMode::all;
  }
  if (value == "decoder") {
    return InspectMode::decoder;
  }
  if (value == "search") {
    return InspectMode::search;
  }
  throw std::runtime_error("invalid inspect mode");
}

auto parse_search_algorithm(std::string_view value)
    -> shiny::nfp::AlgorithmKind {
  const auto parsed = shiny::nfp::parse_algorithm_kind(value);
  if (!parsed.has_value() ||
      (*parsed != shiny::nfp::AlgorithmKind::jostle_search &&
       *parsed != shiny::nfp::AlgorithmKind::genetic_search)) {
    throw std::runtime_error("invalid search algorithm");
  }
  return *parsed;
}

void print_usage() {
  std::cerr << "usage: shiny_nfp_engine_cache_inspect [all|decoder|search] "
               "[--decoder-runs N] [--search-runs N] "
               "[--search-algorithm jostle_search|genetic_search]\n";
}

void print_decoder_state(const char *label,
                         const shiny::nfp::pack::ConstructiveDecoder &decoder) {
  std::cout << label << ".convex_cache=" << decoder.convex_cache_size() << '\n';
  std::cout << label << ".nonconvex_cache=" << decoder.nonconvex_cache_size()
            << '\n';
  std::cout << label
            << ".decomposition_cache=" << decoder.decomposition_cache_size()
            << '\n';
}

void print_search_state(const char *label,
                        const shiny::nfp::search::JostleSearch &search) {
  std::cout << label
            << ".reevaluation_cache=" << search.reevaluation_cache_size()
            << '\n';
}

void print_search_state(const char *label,
                        const shiny::nfp::search::GeneticSearch &search) {
  std::cout << label
            << ".reevaluation_cache=" << search.reevaluation_cache_size()
            << '\n';
}

void run_decoder_inspection(std::size_t run_count) {
  using shiny::nfp::tooling::make_decoder_request;

  shiny::nfp::pack::ConstructiveDecoder decoder;
  const auto decoder_request = make_decoder_request();
  shiny::nfp::pack::DecoderResult last_result;

  print_decoder_state("decoder.initial", decoder);
  for (std::size_t index = 0; index < run_count; ++index) {
    last_result = decoder.decode(decoder_request);
  }
  print_decoder_state("decoder.final", decoder);
  std::cout << "decoder.runs=" << run_count << '\n';
  std::cout << "decoder.last.unplaced="
            << last_result.layout.unplaced_piece_ids.size() << '\n';
}

void run_search_inspection(std::size_t run_count,
                           shiny::nfp::AlgorithmKind search_algorithm) {
  shiny::nfp::search::SearchResult last_result;

  if (search_algorithm == shiny::nfp::AlgorithmKind::genetic_search) {
    using shiny::nfp::tooling::make_genetic_search_request;

    shiny::nfp::search::GeneticSearch search;
    const auto search_request = make_genetic_search_request();

    print_search_state("search.initial", search);
    for (std::size_t index = 0; index < run_count; ++index) {
      last_result = search.improve(search_request);
    }
    print_search_state("search.final", search);
  } else {
    using shiny::nfp::tooling::make_search_request;

    shiny::nfp::search::JostleSearch search;
    const auto search_request = make_search_request();

    print_search_state("search.initial", search);
    for (std::size_t index = 0; index < run_count; ++index) {
      last_result = search.improve(search_request);
    }
    print_search_state("search.final", search);
  }

  std::cout << "search.algorithm="
            << shiny::nfp::to_string(last_result.algorithm) << '\n';
  std::cout << "search.status=" << static_cast<int>(last_result.status) << '\n';
  std::cout << "search.runs=" << run_count << '\n';
  std::cout << "search.last.best_bin_count=" << last_result.best.bin_count
            << '\n';
  std::cout << "search.last.best_unplaced="
            << last_result.best.unplaced_piece_count << '\n';
}

} // namespace

auto main(int argc, char **argv) -> int {
  try {
    InspectMode mode = InspectMode::all;
    std::size_t decoder_runs = 2;
    std::size_t search_runs = 2;
    auto search_algorithm = shiny::nfp::AlgorithmKind::jostle_search;

    for (int index = 1; index < argc; ++index) {
      const std::string_view argument{argv[index]};
      if (argument == "all" || argument == "decoder" || argument == "search") {
        mode = parse_mode(argument);
        continue;
      }
      if (argument == "--decoder-runs") {
        if (index + 1 >= argc) {
          print_usage();
          return 1;
        }
        decoder_runs = parse_count_argument(argv[++index], argument);
        continue;
      }
      if (argument == "--search-runs") {
        if (index + 1 >= argc) {
          print_usage();
          return 1;
        }
        search_runs = parse_count_argument(argv[++index], argument);
        continue;
      }
      if (argument == "--search-algorithm") {
        if (index + 1 >= argc) {
          print_usage();
          return 1;
        }
        search_algorithm = parse_search_algorithm(argv[++index]);
        continue;
      }

      print_usage();
      return 1;
    }

    if (mode == InspectMode::all || mode == InspectMode::decoder) {
      run_decoder_inspection(decoder_runs);
    }
    if (mode == InspectMode::all || mode == InspectMode::search) {
      run_search_inspection(search_runs, search_algorithm);
    }

    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}