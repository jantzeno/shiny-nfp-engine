#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <set>
#include <vector>

#include "algorithm_kind.hpp"
#include "search/jostle.hpp"
#include "util/status.hpp"
#include "support/svg_packing_test_support.hpp"

namespace {

using shiny::nesting::AlgorithmKind;
using shiny::nesting::parse_algorithm_kind;
using shiny::nesting::search::JostleSearch;
using shiny::nesting::search::SearchRunStatus;
using shiny::nesting::test::svg::case_supports_algorithm;
using shiny::nesting::test::svg::import_svg_case;
using shiny::nesting::test::svg::load_svg_case_specs;
using shiny::nesting::test::svg::make_search_request;
using shiny::nesting::test::svg::output_root;
using shiny::nesting::test::svg::require_request_matches_imported_case;
using shiny::nesting::test::svg::require_same_search_result;
using shiny::nesting::test::svg::require_valid_imported_case;
using shiny::nesting::test::svg::require_valid_search_result;
using shiny::nesting::test::svg::select_svg_case_specs;
using shiny::nesting::test::svg::SvgPackingCaseSpec;
using shiny::nesting::test::svg::write_layout_svg;

void require_exploratory_cases(
    const std::vector<SvgPackingCaseSpec> &exploratory_specs) {
  REQUIRE_FALSE(exploratory_specs.empty());

  for (const auto &spec : exploratory_specs) {
    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      const auto request = make_search_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(),
                                            request.decoder_request);

      JostleSearch search;
      const auto result = search.improve(request);

      REQUIRE((result.status == SearchRunStatus::completed ||
               result.status == SearchRunStatus::timed_out));
      require_valid_search_result(spec, request, result);
      REQUIRE(result.iterations_completed <= spec.search_iterations);

      if (!result.best.decode.layout.bins.empty()) {
        const auto output_path =
            output_root() / (spec.id + ".exploratory.packed.svg");
        REQUIRE(write_layout_svg(output_path, result.best.decode.layout) ==
                shiny::nesting::util::Status::ok);
        REQUIRE(std::filesystem::exists(output_path));
      }
    }
  }
}

} // namespace

TEST_CASE("normative svg cases pack deterministically into valid layouts",
          "[.][integration][svg][packing][readiness]") {
  const auto specs = select_svg_case_specs(true, "jostle_search");
  REQUIRE_FALSE(specs.empty());

  for (const auto &spec : specs) {
    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      const auto request = make_search_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(),
                                            request.decoder_request);

      JostleSearch first_search;
      const auto first_result = first_search.improve(request);
      JostleSearch second_search;
      const auto second_result = second_search.improve(request);

      REQUIRE(first_result.status == SearchRunStatus::completed);
      REQUIRE(second_result.status == SearchRunStatus::completed);

      require_valid_search_result(spec, request, first_result);
      require_same_search_result(first_result, second_result);
      REQUIRE(first_result.progress.size() >=
              first_result.iterations_completed);
      if (spec.search_iterations > 0U) {
        REQUIRE(first_result.iterations_completed > 0U);
      }

      const auto output_path = output_root() / (spec.id + ".packed.svg");
      REQUIRE(write_layout_svg(output_path, first_result.best.decode.layout) ==
              shiny::nesting::util::Status::ok);
      REQUIRE(std::filesystem::exists(output_path));
    }
  }
}

TEST_CASE("svg readiness manifest keeps normative cases bounded",
          "[.][integration][svg][packing][readiness]") {
  constexpr std::size_t normative_candidate_group_limit = 16U;
  constexpr std::size_t normative_piece_limit = 4U;
  constexpr std::uint32_t normative_iteration_limit = 6U;
  constexpr std::uint32_t normative_plateau_limit = 2U;
  constexpr std::uint32_t exploratory_time_budget_limit_ms = 3000U;

  const auto all_specs = load_svg_case_specs(false);
  const auto normative_specs = load_svg_case_specs(true);

  REQUIRE_FALSE(all_specs.empty());
  REQUIRE_FALSE(normative_specs.empty());

  std::set<std::string> seen_ids;
  std::size_t exploratory_count = 0U;
  std::size_t bounded_exploratory_count = 0U;
  std::size_t slow_exploratory_count = 0U;
  bool exploratory_extends_beyond_normative_bounds = false;

  for (const auto &spec : all_specs) {
    REQUIRE(seen_ids.insert(spec.id).second);
    REQUIRE_FALSE(spec.description.empty());
    REQUIRE_FALSE(spec.source.empty());
    REQUIRE_FALSE(spec.coverage_features.empty());
    REQUIRE_FALSE(spec.algorithm_applicability.empty());

    std::set<AlgorithmKind> seen_algorithms;
    for (const auto &algorithm : spec.algorithm_applicability) {
      const auto parsed = parse_algorithm_kind(algorithm);
      REQUIRE(parsed.has_value());
      REQUIRE(seen_algorithms.insert(*parsed).second);
    }

    if (spec.normative) {
      REQUIRE(spec.acceptance_lane == "readiness");
      REQUIRE_FALSE(spec.require_observable_success_before_interrupt);
      REQUIRE(spec.require_explicit_bed_id);
      REQUIRE(spec.max_candidate_groups <= normative_candidate_group_limit);
      REQUIRE(spec.max_piece_count > 0U);
      REQUIRE(spec.max_piece_count <= normative_piece_limit);
      REQUIRE(spec.max_bin_count == 1U);
      REQUIRE(spec.search_iterations <= normative_iteration_limit);
      REQUIRE(spec.search_plateau_budget <= normative_plateau_limit);
      REQUIRE(spec.execution_time_budget_ms == 0U);
      REQUIRE(case_supports_algorithm(spec, "jostle_search"));
      continue;
    }

    ++exploratory_count;
    REQUIRE(spec.acceptance_lane == "stress");
    REQUIRE(spec.require_observable_success_before_interrupt);
    REQUIRE(spec.execution_time_budget_ms > 0U);
    REQUIRE(spec.execution_time_budget_ms <= exploratory_time_budget_limit_ms);
    if (spec.slow_exploratory) {
      ++slow_exploratory_count;
    } else {
      ++bounded_exploratory_count;
    }
    exploratory_extends_beyond_normative_bounds =
        exploratory_extends_beyond_normative_bounds ||
        !spec.require_explicit_bed_id ||
        spec.max_candidate_groups > normative_candidate_group_limit ||
        spec.max_piece_count == 0U ||
        spec.max_piece_count > normative_piece_limit ||
        spec.search_iterations > normative_iteration_limit ||
        spec.search_plateau_budget > normative_plateau_limit;
  }

  REQUIRE(exploratory_count > 0U);
  REQUIRE(bounded_exploratory_count > 0U);
  REQUIRE(slow_exploratory_count > 0U);
  REQUIRE(exploratory_extends_beyond_normative_bounds);
}

TEST_CASE("exploratory svg cases import into valid partial layouts",
          "[integration][svg][packing][exploratory]") {
  const auto all_specs = select_svg_case_specs(false, "jostle_search");

  std::vector<SvgPackingCaseSpec> exploratory_specs;
  exploratory_specs.reserve(all_specs.size());
  for (const auto &spec : all_specs) {
    if (!spec.normative && !spec.slow_exploratory) {
      exploratory_specs.push_back(spec);
    }
  }

  require_exploratory_cases(exploratory_specs);
}

TEST_CASE("slow exploratory svg cases import into valid partial layouts",
          "[.][integration][svg][packing][exploratory_slow][slow]") {
  const auto all_specs = select_svg_case_specs(false, "jostle_search");

  std::vector<SvgPackingCaseSpec> slow_exploratory_specs;
  slow_exploratory_specs.reserve(all_specs.size());
  for (const auto &spec : all_specs) {
    if (!spec.normative && spec.slow_exploratory) {
      slow_exploratory_specs.push_back(spec);
    }
  }

  require_exploratory_cases(slow_exploratory_specs);
}

TEST_CASE(
    "exploratory svg cases retain the last successful layout when interrupted",
    "[integration][svg][packing][interrupt]") {
  const auto all_specs = select_svg_case_specs(false, "jostle_search");

  for (const auto &spec : all_specs) {
    if (spec.normative || !spec.require_observable_success_before_interrupt ||
        spec.slow_exploratory) {
      continue;
    }

    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      const auto request = make_search_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(),
                                            request.decoder_request);

      JostleSearch search;
      const auto result = search.improve(request);

      REQUIRE((result.status == SearchRunStatus::completed ||
               result.status == SearchRunStatus::timed_out));
      require_valid_search_result(spec, request, result);
      if (result.status == SearchRunStatus::timed_out) {
        REQUIRE_FALSE(result.progress.empty());
        REQUIRE(result.progress.front().iteration == 0U);
        REQUIRE(result.best.placed_piece_count >= spec.min_placed_piece_count);
      }
    }
  }
}
