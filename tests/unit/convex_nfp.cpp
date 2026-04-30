#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"
#include "predicates/point_location.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using Catch::Approx;
using shiny::nesting::AlgorithmKind;
using shiny::nesting::build_convex_edge_sequence;
using shiny::nesting::compute_convex_ifp;
using shiny::nesting::compute_convex_nfp;
using shiny::nesting::ConvexEdgeSequence;
using shiny::nesting::ConvexIfpRequest;
using shiny::nesting::ConvexNfpRequest;
using shiny::nesting::NfpFeatureKind;
using shiny::nesting::NfpLoop;
using shiny::nesting::NfpResult;
using shiny::nesting::order_convex_nfp_vertices;
using shiny::nesting::pred::PointLocation;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::parse_ring;
using shiny::nesting::test::parse_segment;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_point_equal;
using shiny::nesting::test::require_ring_equal;

auto parse_ring_list(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Ring> {
  std::vector<shiny::nesting::geom::Ring> rings;
  for (const auto &child : node) {
    rings.push_back(parse_ring(child.second));
  }
  return rings;
}

auto parse_points(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Point2> {
  std::vector<shiny::nesting::geom::Point2> points;
  for (const auto &child : node) {
    points.push_back(shiny::nesting::test::parse_point(child.second));
  }
  return points;
}

auto parse_optional_points(const shiny::nesting::test::pt::ptree &node,
                           std::string_view key)
    -> std::vector<shiny::nesting::geom::Point2> {
  std::vector<shiny::nesting::geom::Point2> points;
  if (const auto child = node.get_child_optional(std::string{key})) {
    points.reserve(child->size());
    for (const auto &entry : *child) {
      points.push_back(parse_point(entry.second));
    }
  }
  return points;
}

void require_edge_sequence_equal(
    const ConvexEdgeSequence &actual,
    const std::vector<shiny::nesting::geom::Vector2> &expected_edges,
    const std::vector<std::size_t> &expected_indices) {
  REQUIRE(actual.edges.size() == expected_edges.size());
  REQUIRE(actual.source_indices == expected_indices);
  for (std::size_t index = 0; index < actual.edges.size(); ++index) {
    REQUIRE(actual.edges[index].x() == expected_edges[index].x());
    REQUIRE(actual.edges[index].y() == expected_edges[index].y());
  }
}

auto parse_segments(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Segment2> {
  std::vector<shiny::nesting::geom::Segment2> segments;
  for (const auto &child : node) {
    segments.push_back(parse_segment(child.second));
  }
  return segments;
}

void require_segments_equal(
    const std::vector<shiny::nesting::geom::Segment2> &actual,
    const std::vector<shiny::nesting::geom::Segment2> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require_point_equal(actual[index].start, expected[index].start);
    require_point_equal(actual[index].end, expected[index].end);
  }
}

void require_ring_approx_equal(const shiny::nesting::geom::Ring &actual,
                               const shiny::nesting::geom::Ring &expected,
                               double margin) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    REQUIRE(actual[index].x() == Approx(expected[index].x()).margin(margin));
    REQUIRE(actual[index].y() == Approx(expected[index].y()).margin(margin));
  }
}

auto build_polygon_from_result(const NfpResult &result)
    -> shiny::nesting::geom::PolygonWithHoles {
  shiny::nesting::geom::PolygonWithHoles polygon{};
  for (const auto &loop : result.loops) {
    if (loop.kind == NfpFeatureKind::outer_loop) {
      polygon.outer() = loop.vertices;
    } else if (loop.kind == NfpFeatureKind::hole) {
      polygon.holes().push_back(loop.vertices);
    }
  }
  return polygon;
}

auto midpoint(const shiny::nesting::geom::Point2 &lhs,
              const shiny::nesting::geom::Point2 &rhs)
    -> shiny::nesting::geom::Point2 {
  return shiny::nesting::geom::Point2((lhs.x() + rhs.x()) / 2.0,
                                      (lhs.y() + rhs.y()) / 2.0);
}

auto translate_ring(const shiny::nesting::geom::Ring &ring,
                    const shiny::nesting::geom::Point2 &translation)
    -> shiny::nesting::geom::Ring {
  shiny::nesting::geom::Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back(
        {point.x() + translation.x(), point.y() + translation.y()});
  }
  return translated;
}

auto squared_distance(const shiny::nesting::geom::Point2 &lhs,
                      const shiny::nesting::geom::Point2 &rhs) -> double {
  const auto dx = lhs.x() - rhs.x();
  const auto dy = lhs.y() - rhs.y();
  return dx * dx + dy * dy;
}

auto distance_to_segment(const shiny::nesting::geom::Point2 &point,
                         const shiny::nesting::geom::Segment2 &segment)
    -> double {
  const auto dx = segment.end.x() - segment.start.x();
  const auto dy = segment.end.y() - segment.start.y();
  const auto length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) {
    return std::sqrt(squared_distance(point, segment.start));
  }

  const auto projection = ((point.x() - segment.start.x()) * dx +
                           (point.y() - segment.start.y()) * dy) /
                          length_squared;
  const auto clamped = std::clamp(projection, 0.0, 1.0);
  const shiny::nesting::geom::Point2 closestPoint2(
      segment.start.x() + clamped * dx, segment.start.y() + clamped * dy);
  return std::sqrt(squared_distance(point, closest));
}

auto is_near_polygon_boundary(
    const shiny::nesting::geom::Point2 &point,
    const shiny::nesting::geom::PolygonWithHoles &polygon, double tolerance)
    -> bool {
  auto ring_contains_near_edge = [&](const shiny::nesting::geom::Ring &ring) {
    for (std::size_t index = 0; index < ring.size(); ++index) {
      const auto next_index = (index + 1U) % ring.size();
      if (distance_to_segment(point, {ring[index], ring[next_index]}) <=
          tolerance) {
        return true;
      }
    }
    return false;
  };

  if (ring_contains_near_edge(polygon.outer())) {
    return true;
  }

  for (const auto &hole : polygon.holes()) {
    if (ring_contains_near_edge(hole)) {
      return true;
    }
  }
  return false;
}

void require_translation_feasible(
    const shiny::nesting::geom::PolygonWithHoles &container,
    const shiny::nesting::geom::PolygonWithHoles &piece,
    const shiny::nesting::geom::Point2 &translation) {
  const auto translated_piece = translate_ring(piece.outer(), translation);

  for (const auto &point : translated_piece) {
    const auto location =
        shiny::nesting::pred::locate_point_in_polygon(point, container);
    if (location.inside_hole) {
      REQUIRE((location.location == PointLocation::boundary ||
               is_near_polygon_boundary(point, container, 1e-4)));
    } else {
      REQUIRE((location.location != PointLocation::exterior ||
               is_near_polygon_boundary(point, container, 1e-4)));
    }
  }

  for (std::size_t index = 0; index < translated_piece.size(); ++index) {
    const auto next_index = (index + 1U) % translated_piece.size();
    const auto probe =
        midpoint(translated_piece[index], translated_piece[next_index]);
    const auto location =
        shiny::nesting::pred::locate_point_in_polygon(probe, container);
    if (location.inside_hole) {
      REQUIRE((location.location == PointLocation::boundary ||
               is_near_polygon_boundary(probe, container, 1e-4)));
    } else {
      REQUIRE((location.location != PointLocation::exterior ||
               is_near_polygon_boundary(probe, container, 1e-4)));
    }
  }
}

} // namespace

TEST_CASE("convex edge sequences use deterministic angular traversal",
          "[nfp][convex][ordering]") {
  const shiny::nesting::geom::Ring rectangle{
      {4.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}, {4.0, 0.0}};
  const auto rectangle_sequence = build_convex_edge_sequence(rectangle);
  require_edge_sequence_equal(
      rectangle_sequence, {{4.0, 0.0}, {0.0, 2.0}, {-4.0, 0.0}, {0.0, -2.0}},
      {0U, 1U, 2U, 3U});

  const shiny::nesting::geom::Ring hexagon{{3.0, 2.0}, {1.0, 2.0}, {0.0, 1.0},
                                           {1.0, 0.0}, {3.0, 0.0}, {4.0, 1.0}};
  const auto hexagon_sequence = build_convex_edge_sequence(hexagon);
  require_edge_sequence_equal(hexagon_sequence,
                              {{2.0, 0.0},
                               {1.0, 1.0},
                               {-1.0, 1.0},
                               {-2.0, 0.0},
                               {-1.0, -1.0},
                               {1.0, -1.0}},
                              {1U, 2U, 3U, 4U, 5U, 0U});
}

TEST_CASE("convex nfp vertex ordering follows outgoing edge angle",
          "[nfp][convex][ordering]") {
  const NfpResult result{
      .loops = {NfpLoop{
          .vertices =
              {{-1.0, 0.0}, {0.0, -1.0}, {2.0, -1.0}, {2.0, 2.0}, {-1.0, 2.0}},
          .kind = NfpFeatureKind::outer_loop}},
      .algorithm = AlgorithmKind::convex_nfp,
      .normalized = true,
  };

  const auto ordered = order_convex_nfp_vertices(result);
  REQUIRE(ordered.size() == 5U);

  const std::vector<shiny::nesting::geom::Point2> expected_points{
      {0.0, -1.0}, {2.0, -1.0}, {2.0, 2.0}, {-1.0, 2.0}, {-1.0, 0.0},
  };
  const std::vector<std::size_t> expected_indices{1U, 2U, 3U, 4U, 0U};
  const std::vector<double> expected_polar_keys{
      0.0,
      std::numbers::pi_v<double> / 2.0,
      std::numbers::pi_v<double>,
      3.0 * std::numbers::pi_v<double> / 2.0,
      7.0 * std::numbers::pi_v<double> / 4.0,
  };

  for (std::size_t index = 0; index < ordered.size(); ++index) {
    REQUIRE(ordered[index].point.x() == expected_points[index].x());
    REQUIRE(ordered[index].point.y() == expected_points[index].y());
    REQUIRE(ordered[index].source_edge_index == expected_indices[index]);
    REQUIRE(ordered[index].polar_key == Approx(expected_polar_keys[index]));
  }
}

TEST_CASE("convex nfp fixtures", "[nfp][convex][fixtures]") {
  const auto root = load_fixture_file("nfp/convex_nfp.json");
  REQUIRE(root.get<std::string>("algorithm") == "convex_nfp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "convex_nfp");

      const auto inputs = fixture.get_child("inputs");
      const auto result = compute_convex_nfp(ConvexNfpRequest{
          .piece_a_id = inputs.get<std::uint32_t>("piece_a_id"),
          .piece_b_id = inputs.get<std::uint32_t>("piece_b_id"),
          .convex_a = parse_ring(inputs.get_child("convex_a")),
          .convex_b = parse_ring(inputs.get_child("convex_b")),
          .rotation_a = {.degrees = inputs.get<double>("rotation_a")},
          .rotation_b = {.degrees = inputs.get<double>("rotation_b")},
      });

      const auto expected_loops =
          parse_ring_list(fixture.get_child("expected.outer_loops"));

      REQUIRE(result.algorithm == AlgorithmKind::convex_nfp);
      REQUIRE(result.normalized);
      REQUIRE(result.loops.size() == expected_loops.size());

      for (std::size_t index = 0; index < result.loops.size(); ++index) {
        REQUIRE(result.loops[index].kind == NfpFeatureKind::outer_loop);
        require_ring_equal(result.loops[index].vertices, expected_loops[index]);
      }
    }
  }
}

TEST_CASE("convex nfp perfect fit fixtures",
          "[nfp][convex][perfect-fit][fixtures]") {
  const auto root = load_fixture_file("nfp/perfect_fit.json");
  REQUIRE(root.get<std::string>("algorithm") == "convex_nfp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "convex_nfp_perfect_fit");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = compute_convex_nfp(ConvexNfpRequest{
          .piece_a_id = inputs.get<std::uint32_t>("piece_a_id"),
          .piece_b_id = inputs.get<std::uint32_t>("piece_b_id"),
          .convex_a = parse_ring(inputs.get_child("convex_a")),
          .convex_b = parse_ring(inputs.get_child("convex_b")),
          .rotation_a = {.degrees = inputs.get<double>("rotation_a")},
          .rotation_b = {.degrees = inputs.get<double>("rotation_b")},
      });

      const auto expected_points =
          parse_points(expected.get_child("perfect_fit_points"));
      const auto expected_segments =
          parse_segments(expected.get_child("perfect_sliding_segments"));

      REQUIRE(result.normalized);
      REQUIRE(result.algorithm == AlgorithmKind::convex_nfp);
      REQUIRE(result.perfect_fit_points.size() == expected_points.size());
      REQUIRE(result.perfect_sliding_segments.size() ==
              expected_segments.size());

      for (std::size_t index = 0; index < result.perfect_fit_points.size();
           ++index) {
        require_point_equal(result.perfect_fit_points[index],
                            expected_points[index]);
      }
      require_segments_equal(result.perfect_sliding_segments,
                             expected_segments);
    }
  }
}

TEST_CASE("convex nfp perfect sliding fixtures",
          "[nfp][convex][perfect-sliding][fixtures]") {
  const auto root = load_fixture_file("nfp/perfect_sliding.json");
  REQUIRE(root.get<std::string>("algorithm") == "convex_nfp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "convex_nfp_perfect_sliding");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto result = compute_convex_nfp(ConvexNfpRequest{
          .piece_a_id = inputs.get<std::uint32_t>("piece_a_id"),
          .piece_b_id = inputs.get<std::uint32_t>("piece_b_id"),
          .convex_a = parse_ring(inputs.get_child("convex_a")),
          .convex_b = parse_ring(inputs.get_child("convex_b")),
          .rotation_a = {.degrees = inputs.get<double>("rotation_a")},
          .rotation_b = {.degrees = inputs.get<double>("rotation_b")},
      });

      const auto expected_points =
          parse_points(expected.get_child("perfect_fit_points"));
      const auto expected_segments =
          parse_segments(expected.get_child("perfect_sliding_segments"));

      REQUIRE(result.normalized);
      REQUIRE(result.algorithm == AlgorithmKind::convex_nfp);
      REQUIRE(result.perfect_fit_points.size() == expected_points.size());
      REQUIRE(result.perfect_sliding_segments.size() ==
              expected_segments.size());

      for (std::size_t index = 0; index < result.perfect_fit_points.size();
           ++index) {
        require_point_equal(result.perfect_fit_points[index],
                            expected_points[index]);
      }
      require_segments_equal(result.perfect_sliding_segments,
                             expected_segments);
    }
  }
}

TEST_CASE("convex ifp fixtures", "[nfp][convex][ifp][fixtures]") {
  const auto root = load_fixture_file("nfp/convex_ifp.json");
  REQUIRE(root.get<std::string>("algorithm") == "convex_ifp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "convex_ifp");

      const auto inputs = fixture.get_child("inputs");
      const auto container = parse_polygon(inputs.get_child("container"));
      const auto piece = parse_polygon(inputs.get_child("piece"));
      const auto expected = parse_polygon(fixture.get_child("expected"));
      const auto result = compute_convex_ifp(ConvexIfpRequest{
          .container_id = inputs.get<std::uint32_t>("container_id"),
          .piece_id = inputs.get<std::uint32_t>("piece_id"),
          .container = container,
          .piece = piece,
          .container_rotation = {.degrees =
                                     inputs.get<double>("container_rotation")},
          .piece_rotation = {.degrees = inputs.get<double>("piece_rotation")},
      });

      REQUIRE(result.algorithm == AlgorithmKind::convex_ifp);
      REQUIRE(result.normalized);

      if (expected.outer().empty()) {
        REQUIRE(result.loops.empty());
        continue;
      }

      const auto actual = build_polygon_from_result(result);
      const auto interior_samples = parse_optional_points(
          fixture.get_child("expected"), "interior_samples");
      const auto hole_samples =
          parse_optional_points(fixture.get_child("expected"), "hole_samples");

      REQUIRE(actual.holes().size() == expected.holes().size());
      require_ring_approx_equal(actual.outer(), expected.outer(), 1e-4);
      for (std::size_t index = 0; index < actual.holes().size(); ++index) {
        require_ring_approx_equal(actual.holes()[index],
                                  expected.holes()[index], 1e-4);
      }

      for (const auto &loop : result.loops) {
        for (const auto &point : loop.vertices) {
          require_translation_feasible(container, piece, point);
        }
      }

      for (const auto &sample : interior_samples) {
        const auto location =
            shiny::nesting::pred::locate_point_in_polygon(sample, actual);
        REQUIRE(location.location == PointLocation::interior);
        REQUIRE_FALSE(location.inside_hole);
        require_translation_feasible(container, piece, sample);
      }

      for (const auto &sample : hole_samples) {
        const auto location =
            shiny::nesting::pred::locate_point_in_polygon(sample, actual);
        REQUIRE(location.location == PointLocation::exterior);
        REQUIRE(location.inside_hole);
      }
    }
  }
}