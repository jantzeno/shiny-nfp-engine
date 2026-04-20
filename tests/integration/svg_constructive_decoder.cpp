#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "packing/decoder.hpp"
#include "util/status.hpp"
#include "support/svg_packing_test_support.hpp"

using shiny::nesting::pack::ConstructiveDecoder;
using shiny::nesting::test::svg::import_svg_case;
using shiny::nesting::test::svg::make_decoder_request;
using shiny::nesting::test::svg::output_root;
using shiny::nesting::test::svg::require_request_matches_imported_case;
using shiny::nesting::test::svg::require_same_decoder_result;
using shiny::nesting::test::svg::require_valid_decoder_result;
using shiny::nesting::test::svg::require_valid_imported_case;
using shiny::nesting::test::svg::select_svg_case_specs;
using shiny::nesting::test::svg::write_layout_svg;

TEST_CASE("normative svg cases decode deterministically into valid layouts",
          "[.][integration][svg][decoder][readiness]") {
  const auto specs = select_svg_case_specs(true, "constructive_decoder");
  REQUIRE_FALSE(specs.empty());

  for (const auto &spec : specs) {
    DYNAMIC_SECTION(spec.id) {
      const auto imported = import_svg_case(spec);
      REQUIRE(imported.ok());
      require_valid_imported_case(spec, imported.value());

      const auto request = make_decoder_request(spec, imported.value());
      require_request_matches_imported_case(spec, imported.value(), request);

      ConstructiveDecoder first_decoder;
      const auto first_result = first_decoder.decode(request);
      ConstructiveDecoder second_decoder;
      const auto second_result = second_decoder.decode(request);

      REQUIRE_FALSE(first_result.interrupted);
      REQUIRE_FALSE(second_result.interrupted);
      require_valid_decoder_result(spec, request, first_result);
      require_same_decoder_result(first_result, second_result);

      const auto output_path =
          output_root() / (spec.id + ".decoder.packed.svg");
      REQUIRE(write_layout_svg(output_path, first_result.layout) ==
              shiny::nesting::util::Status::ok);
      REQUIRE(std::filesystem::exists(output_path));
    }
  }
}
