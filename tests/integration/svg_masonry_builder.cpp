#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "packing/masonry.hpp"
#include "util/status.hpp"
#include "support/svg_packing_test_support.hpp"

namespace {

using shiny::nfp::pack::MasonryBuilder;
using shiny::nfp::test::svg::import_svg_case;
using shiny::nfp::test::svg::make_masonry_request;
using shiny::nfp::test::svg::output_root;
using shiny::nfp::test::svg::require_request_matches_imported_case;
using shiny::nfp::test::svg::require_same_masonry_result;
using shiny::nfp::test::svg::require_valid_imported_case;
using shiny::nfp::test::svg::require_valid_masonry_result;
using shiny::nfp::test::svg::select_svg_case_specs;
using shiny::nfp::test::svg::SvgPackingCaseSpec;
using shiny::nfp::test::svg::write_layout_svg;

void require_masonry_cases(const std::vector<SvgPackingCaseSpec> &specs,
                           std::string_view suffix) {
  REQUIRE_FALSE(specs.empty());

  for (const auto &spec : specs) {
    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      const auto request = make_masonry_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(),
                                            request.decoder_request);

      MasonryBuilder first_builder;
      const auto first_result = first_builder.build(request);
      MasonryBuilder second_builder;
      const auto second_result = second_builder.build(request);

      require_valid_masonry_result(spec, request, first_result);
      require_same_masonry_result(first_result, second_result);

      const auto output_path =
          output_root() / (spec.id + "." + std::string(suffix) + ".svg");
      REQUIRE(write_layout_svg(output_path, first_result.layout) ==
              shiny::nfp::util::Status::ok);
      REQUIRE(std::filesystem::exists(output_path));
    }
  }
}

} // namespace

TEST_CASE("applicable normative svg cases build deterministic masonry layouts",
          "[.][integration][svg][masonry][readiness]") {
  std::vector<SvgPackingCaseSpec> readiness_specs;
  for (const auto &spec : select_svg_case_specs(true, "masonry_builder")) {
    readiness_specs.push_back(spec);
  }

  require_masonry_cases(readiness_specs, "masonry.packed");
}

TEST_CASE(
    "applicable bounded stress svg cases build deterministic masonry layouts",
    "[integration][svg][masonry][stress]") {
  std::vector<SvgPackingCaseSpec> stress_specs;
  for (const auto &spec : select_svg_case_specs(false, "masonry_builder")) {
    if (!spec.normative && !spec.slow_exploratory) {
      stress_specs.push_back(spec);
    }
  }

  require_masonry_cases(stress_specs, "masonry.exploratory");
}
