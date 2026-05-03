#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "packing/sparrow/adapters/geometry_adapter.hpp"
#include "packing/sparrow/config.hpp"
#include "packing/sparrow/quantify/overlap_proxy.hpp"
#include "support/sparrow_harness.hpp"

namespace {

using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::sparrow::ParityTestDomain;
using shiny::nesting::pack::sparrow::adapters::to_engine_polygon;
using shiny::nesting::pack::sparrow::adapters::to_port_polygon;

auto polygon_with_hole() -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}},
      {Ring{{2.0, 2.0}, {6.0, 2.0}, {6.0, 6.0}, {2.0, 6.0}}});
}

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto concave_l_shape() -> PolygonWithHoles {
  return PolygonWithHoles(Ring{
      {0.0, 0.0}, {6.0, 0.0}, {6.0, 2.0}, {2.0, 2.0}, {2.0, 6.0}, {0.0, 6.0}});
}

} // namespace

TEST_CASE("geometry adapter round-trips polygon with holes through port representation",
          "[sparrow][quantify]") {
  const auto original = polygon_with_hole();
  const auto port_polygon = to_port_polygon(original);
  const auto round_trip = to_engine_polygon(port_polygon);

  CHECK(round_trip == original);
}

TEST_CASE("quantify parity ledger entry points to the correct catch2 target and fixture families",
          "[sparrow][quantify]") {
  const auto ledger = shiny::nesting::pack::sparrow::port_ledger();
  CHECK(std::any_of(ledger.begin(), ledger.end(), [](const auto &entry) {
    return entry.domain == ParityTestDomain::quantify &&
           entry.catch2_target == "tests/unit/sparrow/sparrow_quantify.cpp";
  }));

  const auto fixtures = shiny::nesting::test::sparrow::fixture_manifest();
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.domain == ParityTestDomain::quantify && fixture.convex;
  }));
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.domain == ParityTestDomain::quantify &&
           fixture.near_touching;
  }));
}

TEST_CASE("overlap proxy quantifies convex and concave piece overlap",
          "[sparrow][quantify]") {
  const auto convex_overlap_or =
      shiny::nesting::pack::sparrow::quantify::quantify_overlap(
          to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0)),
          to_port_polygon(rectangle(2.0, 2.0, 6.0, 6.0)));
  REQUIRE(convex_overlap_or.has_value());
  CHECK(convex_overlap_or.value().has_overlap);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      convex_overlap_or.value().overlap_area, 4.0));

  const auto concave_overlap_or =
      shiny::nesting::pack::sparrow::quantify::quantify_overlap(
          to_port_polygon(concave_l_shape()),
          to_port_polygon(rectangle(1.0, 1.0, 4.0, 4.0)));
  REQUIRE(concave_overlap_or.has_value());
  CHECK(concave_overlap_or.value().has_overlap);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      concave_overlap_or.value().overlap_area, 5.0));
}

TEST_CASE("overlap proxy reports zero overlap for holes and near-touching pieces",
          "[sparrow][quantify]") {
  const auto hole_overlap_or =
      shiny::nesting::pack::sparrow::quantify::quantify_overlap(
          to_port_polygon(polygon_with_hole()),
          to_port_polygon(rectangle(3.0, 3.0, 5.0, 5.0)));
  REQUIRE(hole_overlap_or.has_value());
  CHECK_FALSE(hole_overlap_or.value().has_overlap);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      hole_overlap_or.value().overlap_area, 0.0));

  const auto near_touching_overlap_or =
      shiny::nesting::pack::sparrow::quantify::quantify_overlap(
          to_port_polygon(rectangle(0.0, 0.0, 2.0, 2.0)),
          to_port_polygon(rectangle(2.0, 0.0, 4.0, 2.0)));
  REQUIRE(near_touching_overlap_or.has_value());
  CHECK_FALSE(near_touching_overlap_or.value().has_overlap);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      near_touching_overlap_or.value().overlap_area, 0.0));
}