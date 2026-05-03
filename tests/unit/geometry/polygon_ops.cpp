#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/operations/convex_hull.hpp"
#include "geometry/operations/merge_region.hpp"
#include "geometry/operations/simplify.hpp"
#include "geometry/queries/normalize.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::geom::compute_convex_hull;
using shiny::nesting::geom::make_merged_region;
using shiny::nesting::geom::merge_polygon_into_region;
using shiny::nesting::geom::MergedRegion;
using shiny::nesting::geom::normalize_polygon;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::simplify_polygon;
using shiny::nesting::geom::simplify_polygon_douglas_peucker;
using shiny::nesting::geom::union_polygons;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_polygon_equal;
using shiny::nesting::test::require_ring_equal;

auto parse_polygon_list(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<PolygonWithHoles> {
  std::vector<PolygonWithHoles> polygons;
  for (const auto &child : node) {
    polygons.push_back(parse_polygon(child.second));
  }
  return polygons;
}

void require_polygon_list_equal(const std::vector<PolygonWithHoles> &actual,
                                const std::vector<PolygonWithHoles> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require_polygon_equal(actual[index], expected[index]);
  }
}

auto signed_area(const shiny::nesting::geom::Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area +=
        static_cast<long double>(ring[index].x()) * ring[next_index].y() -
        static_cast<long double>(ring[next_index].x()) * ring[index].y();
  }

  return twice_area / 2.0L;
}

auto area_sign(const PolygonWithHoles &polygon) -> int {
  const auto area = signed_area(polygon.outer());
  if (area > 0.0L) {
    return 1;
  }
  if (area < 0.0L) {
    return -1;
  }
  return 0;
}

} // namespace

TEST_CASE("collinear simplification fixtures",
          "[polygon-ops][simplify][fixtures]") {
  const auto root =
      load_fixture_file("polygon_ops/collinear_simplification.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "collinear_simplification");

      const auto input_polygon =
          parse_polygon(fixture.get_child("inputs.polygon"));
      const auto expected = parse_polygon(fixture.get_child("expected"));
      const auto normalized_input = normalize_polygon(input_polygon);
      const auto simplified = simplify_polygon(input_polygon);

      require_polygon_equal(simplified, expected);
      REQUIRE(area_sign(simplified) == area_sign(normalized_input));
      REQUIRE(simplified.holes().size() == normalized_input.holes().size());

      if (input_polygon.holes().empty()) {
        const Polygon simplified_simple = simplify_polygon(
            shiny::nesting::geom::Polygon(input_polygon.outer()));
        require_ring_equal(simplified_simple.outer(), expected.outer());
      }
    }
  }
}

TEST_CASE("convex hull fixtures", "[polygon-ops][convex-hull][fixtures]") {
  const auto root = load_fixture_file("polygon_ops/convex_hull.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "convex_hull");

      const auto input_polygon =
          parse_polygon(fixture.get_child("inputs.polygon"));
      const auto expected = parse_polygon(fixture.get_child("expected"));

      const auto polygon_hull = compute_convex_hull(input_polygon);
      require_polygon_equal(
          shiny::nesting::geom::PolygonWithHoles(polygon_hull.outer()),
          expected);

      const auto simple_hull = compute_convex_hull(
          shiny::nesting::geom::Polygon(input_polygon.outer()));
      require_polygon_equal(
          shiny::nesting::geom::PolygonWithHoles(simple_hull.outer()),
          shiny::nesting::geom::PolygonWithHoles(expected.outer()));
    }
  }
}

TEST_CASE("douglas-peucker simplification removes shallow jogs from polygon",
          "[polygon-ops][simplify][douglas-peucker]") {
  const Polygon input(shiny::nesting::geom::Ring{{0.0, 0.0},
                                                 {4.0, 0.0},
                                                 {4.0, 0.2},
                                                 {8.0, 0.0},
                                                 {12.0, 0.0},
                                                 {12.0, 6.0},
                                                 {0.0, 6.0}});

  const Polygon simplified = simplify_polygon_douglas_peucker(input, 0.25);

  require_ring_equal(simplified.outer(),
                     shiny::nesting::geom::Polygon(
                         shiny::nesting::geom::Ring{
                             {0.0, 0.0}, {12.0, 0.0}, {12.0, 6.0}, {0.0, 6.0}})
                         .outer());
}

TEST_CASE("douglas-peucker simplification preserves polygon holes",
          "[polygon-ops][simplify][douglas-peucker]") {
  const PolygonWithHoles input(shiny::nesting::geom::Ring{{0.0, 0.0},
                                                          {6.0, 0.0},
                                                          {6.0, 0.1},
                                                          {12.0, 0.0},
                                                          {12.0, 10.0},
                                                          {0.0, 10.0}},
                               {shiny::nesting::geom::Ring{{3.0, 3.0},
                                                           {6.0, 3.0},
                                                           {6.0, 3.1},
                                                           {9.0, 3.0},
                                                           {9.0, 7.0},
                                                           {3.0, 7.0}}});

  const PolygonWithHoles simplified =
      simplify_polygon_douglas_peucker(input, 0.2);

  require_polygon_equal(
      simplified, shiny::nesting::geom::PolygonWithHoles(
                      shiny::nesting::geom::Ring{
                          {0.0, 0.0}, {12.0, 0.0}, {12.0, 10.0}, {0.0, 10.0}},
                      {shiny::nesting::geom::Ring{
                          {3.0, 3.0}, {3.0, 7.0}, {9.0, 7.0}, {9.0, 3.0}}}));
}

TEST_CASE("polygon union fixtures", "[polygon-ops][boolean][fixtures]") {
  const auto root = load_fixture_file("polygon_ops/polygon_union.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "polygon_union");

      const auto lhs = parse_polygon(fixture.get_child("inputs.lhs"));
      const auto rhs = parse_polygon(fixture.get_child("inputs.rhs"));
      const auto expected =
          parse_polygon_list(fixture.get_child("expected.regions"));

      require_polygon_list_equal(union_polygons(lhs, rhs), expected);
      require_polygon_list_equal(
          merge_polygon_into_region(make_merged_region(lhs), rhs).regions,
          expected);
    }
  }
}

TEST_CASE("merged region bridge merges prior disjoint components",
          "[polygon-ops][merge-region]") {
  const PolygonWithHoles lhs(shiny::nesting::geom::Ring{
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}});
  const PolygonWithHoles rhs(shiny::nesting::geom::Ring{
      {4.0, 0.0}, {6.0, 0.0}, {6.0, 2.0}, {4.0, 2.0}});
  const PolygonWithHoles bridge(shiny::nesting::geom::Ring{
      {2.0, 0.0}, {4.0, 0.0}, {4.0, 2.0}, {2.0, 2.0}});

  MergedRegion region = make_merged_region(lhs);
  region = merge_polygon_into_region(region, rhs);
  REQUIRE(region.regions.size() == 2U);

  region = merge_polygon_into_region(region, bridge);
  REQUIRE(region.regions.size() == 1U);
  require_polygon_equal(
      region.regions.front(),
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          {0.0, 0.0}, {6.0, 0.0}, {6.0, 2.0}, {0.0, 2.0}}));
}
