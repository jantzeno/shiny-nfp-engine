#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "geometry/normalize.hpp"
#include "polygon_ops/boolean_ops.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nfp::geom::Point2;
using shiny::nfp::geom::PolygonWithHoles;
using shiny::nfp::poly::union_polygons;
using shiny::nfp::test::load_fixture_file;
using shiny::nfp::test::parse_polygon;
using shiny::nfp::test::require_fixture_metadata;
using shiny::nfp::test::require_polygon_equal;

auto parse_polygon_list(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<PolygonWithHoles> {
  std::vector<PolygonWithHoles> polygons;
  for (const auto &child : node) {
    polygons.push_back(parse_polygon(child.second));
  }
  return polygons;
}

auto point_less(const Point2 &lhs, const Point2 &rhs) -> bool {
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  return lhs.y < rhs.y;
}

auto ring_less(const shiny::nfp::geom::Ring &lhs,
               const shiny::nfp::geom::Ring &rhs) -> bool {
  if (lhs.empty()) {
    return !rhs.empty();
  }
  if (rhs.empty()) {
    return false;
  }
  if (point_less(lhs.front(), rhs.front())) {
    return true;
  }
  if (point_less(rhs.front(), lhs.front())) {
    return false;
  }
  return lhs.size() < rhs.size();
}

auto polygon_less(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs)
    -> bool {
  return ring_less(lhs.outer, rhs.outer);
}

auto canonicalize_polygon(const PolygonWithHoles &polygon) -> PolygonWithHoles {
  auto normalized = shiny::nfp::geom::normalize_polygon(polygon);
  std::sort(normalized.holes.begin(), normalized.holes.end(), ring_less);
  return normalized;
}

auto canonicalize_polygons(const std::vector<PolygonWithHoles> &polygons)
    -> std::vector<PolygonWithHoles> {
  std::vector<PolygonWithHoles> normalized;
  normalized.reserve(polygons.size());
  for (const auto &polygon : polygons) {
    normalized.push_back(canonicalize_polygon(polygon));
  }
  std::sort(normalized.begin(), normalized.end(), polygon_less);
  return normalized;
}

void require_polygon_list_equal(const std::vector<PolygonWithHoles> &actual,
                                const std::vector<PolygonWithHoles> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require_polygon_equal(actual[index], expected[index]);
  }
}

} // namespace

TEST_CASE("polygon union topology regressions",
          "[topology][polygon-union][regression]") {
  const auto root =
      load_fixture_file("topology/polygon_union_regressions.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "polygon_union_topology");

      const auto lhs = parse_polygon(fixture.get_child("inputs.lhs"));
      const auto rhs = parse_polygon(fixture.get_child("inputs.rhs"));
      const auto expected = canonicalize_polygons(
          parse_polygon_list(fixture.get_child("expected.regions")));
      const auto actual = canonicalize_polygons(union_polygons(lhs, rhs));

      require_polygon_list_equal(actual, expected);
    }
  }
}