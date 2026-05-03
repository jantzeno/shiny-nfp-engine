#include <catch2/catch_test_macros.hpp>

#include <string>

#include "io/svg_polygonize.hpp"
#include "support/fixture_test_support.hpp"

namespace {

auto polygon_signed_area(const shiny::nesting::geom::Ring &ring)
    -> long double {
  long double area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    area += static_cast<long double>(ring[index].x()) * ring[next_index].y() -
            static_cast<long double>(ring[next_index].x()) * ring[index].y();
  }
  return area / 2.0L;
}

} // namespace

TEST_CASE("svg path polygonization covers line and curve commands",
          "[io][svg]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("io/svg_paths.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      const auto result = shiny::nesting::io::polygonize_svg_path(
          fixture.get<std::string>("path"),
          {.curve_flattening_tolerance =
               fixture.get<double>("tolerance", 0.25)});

      REQUIRE(result.has_value());
      REQUIRE(result.value().holes().size() ==
              fixture.get<std::size_t>("expected.hole_count", 0));
      REQUIRE(result.value().outer().size() >=
              fixture.get<std::size_t>("expected.min_vertex_count", 4));
      REQUIRE(result.value().outer().front() ==
              shiny::nesting::test::parse_point(
                  fixture.get_child("expected.first_vertex")));
      REQUIRE(polygon_signed_area(result.value().outer()) > 0.0L);

      if (const auto expected_polygon =
              fixture.get_child_optional("expected.polygon")) {
        shiny::nesting::test::require_polygon_equal(
            result.value(),
            shiny::nesting::test::parse_polygon(*expected_polygon));
      }
    }
  }
}

TEST_CASE("svg path polygonization rejects invalid open paths", "[io][svg]") {
  const auto result =
      shiny::nesting::io::polygonize_svg_path("M 0 0 L 4 0 L 4 2", {});
  REQUIRE(result.error() == shiny::nesting::util::Status::invalid_input);
}