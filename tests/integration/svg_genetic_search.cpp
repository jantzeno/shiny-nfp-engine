#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>

#include "algorithm_kind.hpp"
#include "search/genetic.hpp"
#include "util/status.hpp"
#include "support/svg_packing_test_support.hpp"

using shiny::nfp::AlgorithmKind;
using shiny::nfp::search::GeneticSearch;
using shiny::nfp::search::GeneticSearchConfig;
using shiny::nfp::search::SearchRunStatus;
using shiny::nfp::test::svg::import_svg_case;
using shiny::nfp::test::svg::kSearchSeed;
using shiny::nfp::test::svg::make_search_request;
using shiny::nfp::test::svg::output_root;
using shiny::nfp::test::svg::require_request_matches_imported_case;
using shiny::nfp::test::svg::require_same_search_result;
using shiny::nfp::test::svg::require_valid_imported_case;
using shiny::nfp::test::svg::require_valid_search_result;
using shiny::nfp::test::svg::select_svg_case_specs;
using shiny::nfp::test::svg::write_layout_svg;

namespace {

auto make_bounded_readiness_genetic_config(
    const shiny::nfp::search::SearchRequest &request,
    std::uint32_t generation_budget, std::uint32_t plateau_budget)
    -> GeneticSearchConfig {
  const auto piece_count =
      static_cast<std::uint32_t>(request.decoder_request.pieces.size());
  const auto population_size = std::max<std::uint32_t>(4U, piece_count + 1U);

  return {
      .max_generations = std::max<std::uint32_t>(2U, generation_budget),
      .population_size = population_size,
      .deterministic_seed = kSearchSeed,
      .mutation_rate_percent = 10,
      .elite_count = std::min<std::uint32_t>(2U, population_size),
      .tournament_size = std::min<std::uint32_t>(3U, population_size),
      .plateau_generations = std::max<std::uint32_t>(1U, plateau_budget),
      .enable_local_search_polish = false,
      .enabled = true,
  };
}

} // namespace

TEST_CASE("normative svg cases improve deterministically under genetic search",
          "[.][integration][svg][genetic][readiness]") {
  const auto specs = select_svg_case_specs(true, "genetic_search");
  REQUIRE_FALSE(specs.empty());

  for (const auto &spec : specs) {
    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      auto request = make_search_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(),
                                            request.decoder_request);
      request.genetic_search = make_bounded_readiness_genetic_config(
          request, spec.search_iterations, spec.search_plateau_budget);

      GeneticSearch first_search;
      const auto first_result = first_search.improve(request);
      GeneticSearch second_search;
      const auto second_result = second_search.improve(request);

      REQUIRE(first_result.algorithm == AlgorithmKind::genetic_search);
      REQUIRE(first_result.status == SearchRunStatus::completed);
      REQUIRE(second_result.status == SearchRunStatus::completed);
      REQUIRE(first_result.best.placed_piece_count >=
              first_result.baseline.placed_piece_count);
      REQUIRE(first_result.best.unplaced_piece_count <=
              first_result.baseline.unplaced_piece_count);
      require_valid_search_result(spec, request, first_result);
      require_same_search_result(first_result, second_result);

      const auto output_path =
          output_root() / (spec.id + ".genetic.packed.svg");
      REQUIRE(write_layout_svg(output_path, first_result.best.decode.layout) ==
              shiny::nfp::util::Status::ok);
      REQUIRE(std::filesystem::exists(output_path));
    }
  }
}
