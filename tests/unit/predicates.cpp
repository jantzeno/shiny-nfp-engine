#include <catch2/catch_test_macros.hpp>
#include <string_view>

#include "predicates/classify.hpp"
#include "predicates/orientation.hpp"
#include "predicates/point_location.hpp"
#include "predicates/segment_intersection.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::geom::Point2;
using shiny::nesting::pred::BoundaryRelation;
using shiny::nesting::pred::Orientation;
using shiny::nesting::pred::OrientationQuery;
using shiny::nesting::pred::PointLocation;
using shiny::nesting::pred::SegmentContactKind;

auto parse_orientation(std::string_view value) -> Orientation {
  if (value == "left_turn") {
    return Orientation::left_turn;
  }
  if (value == "right_turn") {
    return Orientation::right_turn;
  }
  if (value == "collinear") {
    return Orientation::collinear;
  }
  throw std::runtime_error("unknown orientation fixture value");
}

auto parse_segment_contact_kind(std::string_view value) -> SegmentContactKind {
  if (value == "disjoint") {
    return SegmentContactKind::disjoint;
  }
  if (value == "proper_intersection") {
    return SegmentContactKind::proper_intersection;
  }
  if (value == "endpoint_touch") {
    return SegmentContactKind::endpoint_touch;
  }
  if (value == "collinear_overlap") {
    return SegmentContactKind::collinear_overlap;
  }
  if (value == "parallel_disjoint") {
    return SegmentContactKind::parallel_disjoint;
  }
  throw std::runtime_error("unknown segment contact kind fixture value");
}

auto parse_boundary_relation(std::string_view value) -> BoundaryRelation {
  if (value == "off_boundary") {
    return BoundaryRelation::off_boundary;
  }
  if (value == "on_edge_interior") {
    return BoundaryRelation::on_edge_interior;
  }
  if (value == "on_vertex") {
    return BoundaryRelation::on_vertex;
  }
  throw std::runtime_error("unknown boundary relation fixture value");
}

auto parse_point_location(std::string_view value) -> PointLocation {
  if (value == "exterior") {
    return PointLocation::exterior;
  }
  if (value == "boundary") {
    return PointLocation::boundary;
  }
  if (value == "interior") {
    return PointLocation::interior;
  }
  throw std::runtime_error("unknown point location fixture value");
}

} // namespace

TEST_CASE("orientation fixtures", "[predicates][orientation][fixtures]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("predicates/orientation.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(fixture, "orientation");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = shiny::nesting::pred::orient(OrientationQuery{
          shiny::nesting::test::parse_point(inputs.get_child("a")),
          shiny::nesting::test::parse_point(inputs.get_child("b")),
          shiny::nesting::test::parse_point(inputs.get_child("c")),
      });

      REQUIRE(result ==
              parse_orientation(expected.get<std::string>("orientation")));
    }
  }
}

TEST_CASE("segment contact fixtures",
          "[predicates][segment-contact][fixtures]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("predicates/segment_contact.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(fixture, "segment_contact");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = shiny::nesting::pred::classify_segment_contact(
          shiny::nesting::test::parse_segment(inputs.get_child("lhs")),
          shiny::nesting::test::parse_segment(inputs.get_child("rhs")));

      REQUIRE(result.kind ==
              parse_segment_contact_kind(expected.get<std::string>("kind")));
      REQUIRE(result.point_count == expected.get<std::uint8_t>("point_count"));
      REQUIRE(result.a_vertex_contact ==
              expected.get<bool>("a_vertex_contact"));
      REQUIRE(result.b_vertex_contact ==
              expected.get<bool>("b_vertex_contact"));

      const auto expected_points =
          shiny::nesting::test::parse_expected_points(expected);
      std::vector<Point2> actual_points;
      for (std::size_t index = 0; index < result.point_count; ++index) {
        actual_points.push_back(result.points[index]);
      }
      REQUIRE(actual_points.size() == expected_points.size());
      for (std::size_t index = 0; index < actual_points.size(); ++index) {
        shiny::nesting::test::require_point_equal(actual_points[index],
                                              expected_points[index]);
      }
    }
  }
}

TEST_CASE("point-on-segment fixtures",
          "[predicates][point-on-segment][fixtures]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("predicates/point_on_segment.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(fixture, "point_on_segment");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = shiny::nesting::pred::locate_point_on_segment(
          shiny::nesting::test::parse_point(inputs.get_child("point")),
          shiny::nesting::test::parse_segment(inputs.get_child("segment")));

      REQUIRE(result.relation ==
              parse_boundary_relation(expected.get<std::string>("relation")));
      REQUIRE(result.parametric_t == expected.get<double>("parametric_t"));
    }
  }
}

TEST_CASE("point-in-ring fixtures",
          "[predicates][point-location][ring][fixtures]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("predicates/point_in_ring.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(fixture, "point_in_ring");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = shiny::nesting::pred::locate_point_in_ring(
          shiny::nesting::test::parse_point(inputs.get_child("point")),
          shiny::nesting::test::parse_ring(inputs.get_child("ring")));

      REQUIRE(result.location ==
              parse_point_location(expected.get<std::string>("location")));
      REQUIRE(result.edge_index ==
              expected.get<std::int32_t>("edge_index", -1));
      REQUIRE(result.vertex_index ==
              expected.get<std::int32_t>("vertex_index", -1));
    }
  }
}

TEST_CASE("point-in-polygon fixtures",
          "[predicates][point-location][polygon][fixtures]") {
  const auto root =
      shiny::nesting::test::load_fixture_file("predicates/point_in_polygon.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(fixture, "point_in_polygon");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = shiny::nesting::pred::locate_point_in_polygon(
          shiny::nesting::test::parse_point(inputs.get_child("point")),
          shiny::nesting::test::parse_polygon(inputs.get_child("polygon")));

      REQUIRE(result.location ==
              parse_point_location(expected.get<std::string>("location")));
      REQUIRE(result.inside_hole == expected.get<bool>("inside_hole", false));
      REQUIRE(result.hole_index ==
              expected.get<std::int32_t>("hole_index", -1));
      REQUIRE(result.boundary_edge_index ==
              expected.get<std::int32_t>("boundary_edge_index", -1));
      REQUIRE(result.boundary_vertex_index ==
              expected.get<std::int32_t>("boundary_vertex_index", -1));
    }
  }
}

TEST_CASE("lexicographic start-vertex fixtures",
          "[predicates][normalize][fixtures]") {
  const auto root = shiny::nesting::test::load_fixture_file(
      "predicates/lexicographic_min_vertex_index.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      shiny::nesting::test::require_fixture_metadata(
          fixture, "lexicographic_min_vertex_index");

      const auto result = shiny::nesting::pred::lexicographic_min_vertex_index(
          shiny::nesting::test::parse_ring(fixture.get_child("inputs.ring")));

      REQUIRE(result == fixture.get<std::size_t>("expected.min_vertex_index"));
    }
  }
}