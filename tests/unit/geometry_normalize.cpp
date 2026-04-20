#include <catch2/catch_test_macros.hpp>

#include <string>

#include "geometry/normalize.hpp"
#include "geometry/types.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::geom::normalize_polygon;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_polygon_equal;
using shiny::nesting::test::require_ring_equal;

} // namespace

TEST_CASE("geometry normalization fixtures",
          "[geometry][normalize][fixtures]") {
  const auto root = load_fixture_file("geometry/polygon_normalization.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "polygon_normalization");

      const auto expected = parse_polygon(fixture.get_child("expected"));
      const auto input_polygon =
          parse_polygon(fixture.get_child("inputs.polygon"));
      const auto normalized_polygon = normalize_polygon(input_polygon);
      require_polygon_equal(normalized_polygon, expected);

      if (input_polygon.holes.empty()) {
        const Polygon normalized_simple =
            normalize_polygon(Polygon{.outer = input_polygon.outer});
        require_ring_equal(normalized_simple.outer, expected.outer);
      }
    }
  }
}