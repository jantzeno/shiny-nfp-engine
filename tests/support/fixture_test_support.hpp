#pragma once

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "geometry/types.hpp"

namespace shiny::nesting::test {

namespace pt = boost::property_tree;

inline auto fixture_root() -> std::filesystem::path {
  return std::filesystem::path{SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT};
}

inline auto load_fixture_file(std::string_view relative_path) -> pt::ptree {
  pt::ptree root;
  pt::read_json(
      (fixture_root() / std::filesystem::path{relative_path}).string(), root);
  return root;
}

inline void require_fixture_metadata(const pt::ptree &fixture,
                                     std::string_view kind) {
  REQUIRE_FALSE(fixture.get<std::string>("id").empty());
  REQUIRE(fixture.get<std::string>("kind") == kind);
}

inline auto parse_point(const pt::ptree &node) -> geom::Point2 {
  std::vector<double> coordinates;
  for (const auto &child : node) {
    coordinates.push_back(child.second.get_value<double>());
  }

  if (coordinates.size() != 2U) {
    throw std::runtime_error(
        "point fixture must contain exactly two coordinates");
  }

  return {coordinates[0], coordinates[1]};
}

inline auto parse_segment(const pt::ptree &node) -> geom::Segment2 {
  return {parse_point(node.get_child("start")),
          parse_point(node.get_child("end"))};
}

inline auto parse_ring(const pt::ptree &node) -> geom::Ring {
  geom::Ring ring;
  for (const auto &child : node) {
    ring.push_back(parse_point(child.second));
  }
  return ring;
}

inline auto parse_polygon(const pt::ptree &node) -> geom::PolygonWithHoles {
  geom::PolygonWithHoles polygon;
  polygon.outer() = parse_ring(node.get_child("outer"));

  if (const auto holes = node.get_child_optional("holes")) {
    for (const auto &child : *holes) {
      polygon.holes().push_back(parse_ring(child.second));
    }
  }

  return polygon;
}

inline void require_point_equal(const geom::Point2 &actual,
                                const geom::Point2 &expected) {
  REQUIRE(actual.x() == expected.x());
  REQUIRE(actual.y() == expected.y());
}

inline void require_ring_equal(const geom::Ring &actual,
                               const geom::Ring &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require_point_equal(actual[index], expected[index]);
  }
}

inline void require_polygon_equal(const geom::PolygonWithHoles &actual,
                                  const geom::PolygonWithHoles &expected) {
  require_ring_equal(actual.outer(), expected.outer());
  REQUIRE(actual.holes().size() == expected.holes().size());
  for (std::size_t index = 0; index < actual.holes().size(); ++index) {
    require_ring_equal(actual.holes()[index], expected.holes()[index]);
  }
}

inline auto parse_expected_points(const pt::ptree &node)
    -> std::vector<geom::Point2> {
  std::vector<geom::Point2> points;
  if (const auto expected_points = node.get_child_optional("points")) {
    for (const auto &child : *expected_points) {
      points.push_back(parse_point(child.second));
    }
  }
  return points;
}

} // namespace shiny::nesting::test
