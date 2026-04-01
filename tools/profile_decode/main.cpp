#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "../support/profile_decode_benchmark.hpp"

namespace {

constexpr std::size_t default_decoder_runs = 200;
constexpr std::size_t default_search_runs = 40;

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

auto parse_count_argument(const char *value, std::string_view label)
    -> std::size_t {
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value || (end != nullptr && *end != '\0') || parsed == 0UL) {
    throw std::runtime_error("invalid run count for " + std::string(label));
  }
  return static_cast<std::size_t>(parsed);
}

void print_usage() {
  std::cerr << "usage: shiny_nfp_engine_profile_decode [--decoder-runs N] "
               "[--search-runs N] [--search-algorithm "
               "jostle_search|genetic_search]\n";
}

} // namespace

auto main(int argc, char **argv) -> int {
  try {
    std::size_t decoder_runs = default_decoder_runs;
    std::size_t search_runs = default_search_runs;
    auto search_algorithm = shiny::nfp::AlgorithmKind::jostle_search;

    for (int index = 1; index < argc; ++index) {
      const std::string_view argument{argv[index]};
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

    const auto result = shiny::nfp::tooling::run_profile_decode_benchmark(
        {.decoder_runs = decoder_runs,
         .search_runs = search_runs,
         .search_algorithm = search_algorithm});
    shiny::nfp::tooling::print_profile_decode_benchmark_result(result);

    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}