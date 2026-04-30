#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "nfp/nonconvex_nfp.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::ArrangementGraph;
using shiny::nesting::ExtractionStatus;
using shiny::nesting::GraphEdge;
using shiny::nesting::GraphEdgeKind;
using shiny::nesting::GraphVertex;
using shiny::nesting::GraphVertexKind;
using shiny::nesting::NfpFeatureKind;
using shiny::nesting::NfpLoop;
using shiny::nesting::NonconvexNfpRequest;
using shiny::nesting::cache::AlgorithmRevision;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::parse_ring;
using shiny::nesting::test::parse_segment;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::require_point_equal;
using shiny::nesting::test::require_ring_equal;

auto point_less(const shiny::nesting::geom::Point2 &lhs,
                const shiny::nesting::geom::Point2 &rhs) -> bool {
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  return lhs.y < rhs.y;
}

auto ring_signed_area(const shiny::nesting::geom::Ring &ring) -> long double {
  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

auto parse_status(std::string_view value) -> ExtractionStatus {
  if (value == "success") {
    return ExtractionStatus::success;
  }
  if (value == "empty") {
    return ExtractionStatus::empty;
  }
  if (value == "invalid_graph") {
    return ExtractionStatus::invalid_graph;
  }

  throw std::runtime_error("unknown extraction status fixture value");
}

auto parse_vertex_kind(std::string_view value) -> GraphVertexKind {
  if (value == "original_vertex") {
    return GraphVertexKind::original_vertex;
  }
  if (value == "intersection_vertex") {
    return GraphVertexKind::intersection_vertex;
  }
  if (value == "midpoint_vertex") {
    return GraphVertexKind::midpoint_vertex;
  }

  throw std::runtime_error("unknown graph vertex kind fixture value");
}

auto parse_edge_kind(std::string_view value) -> GraphEdgeKind {
  if (value == "boundary_edge") {
    return GraphEdgeKind::boundary_edge;
  }
  if (value == "duplicate_edge") {
    return GraphEdgeKind::duplicate_edge;
  }
  if (value == "pruned_edge") {
    return GraphEdgeKind::pruned_edge;
  }

  throw std::runtime_error("unknown graph edge kind fixture value");
}

auto parse_rings(const shiny::nesting::test::pt::ptree &node)
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
    points.push_back(parse_point(child.second));
  }
  return points;
}

auto parse_segments(const shiny::nesting::test::pt::ptree &node)
    -> std::vector<shiny::nesting::geom::Segment2> {
  std::vector<shiny::nesting::geom::Segment2> segments;
  for (const auto &child : node) {
    segments.push_back(parse_segment(child.second));
  }
  return segments;
}

auto count_intersection_vertices(const ArrangementGraph &graph) -> std::size_t {
  std::size_t count = 0U;
  for (const auto &vertex : graph.vertices) {
    if (vertex.kind == GraphVertexKind::intersection_vertex) {
      ++count;
    }
  }
  return count;
}

auto count_pruned_edges(const ArrangementGraph &graph) -> std::size_t {
  std::size_t count = 0U;
  for (const auto &edge : graph.edges) {
    if (edge.kind == GraphEdgeKind::pruned_edge) {
      ++count;
    }
  }
  return count;
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

void require_points_equal(
    const std::vector<shiny::nesting::geom::Point2> &actual,
    const std::vector<shiny::nesting::geom::Point2> &expected) {
  REQUIRE(actual.size() == expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require_point_equal(actual[index], expected[index]);
  }
}

void require_loops_equal(
    const std::vector<NfpLoop> &actual,
    const std::vector<shiny::nesting::geom::Ring> &expected,
    NfpFeatureKind kind) {
  std::vector<shiny::nesting::geom::Ring> filtered_actual;
  for (const auto &loop : actual) {
    if (loop.kind == kind) {
      filtered_actual.push_back(loop.vertices);
    }
  }

  REQUIRE(filtered_actual.size() == expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto require_contract_normalized =
        [kind](const shiny::nesting::geom::Ring &ring) {
          REQUIRE(ring.size() >= 3U);
          for (std::size_t lhs = 0; lhs < ring.size(); ++lhs) {
            for (std::size_t rhs = lhs + 1U; rhs < ring.size(); ++rhs) {
              const auto duplicate_vertex =
                  ring[lhs].x == ring[rhs].x && ring[lhs].y == ring[rhs].y;
              REQUIRE_FALSE(duplicate_vertex);
            }
          }
          for (std::size_t vertex_index = 1; vertex_index < ring.size();
               ++vertex_index) {
            REQUIRE_FALSE(point_less(ring[vertex_index], ring.front()));
          }

          const auto area = ring_signed_area(ring);
          if (kind == NfpFeatureKind::outer_loop) {
            REQUIRE(area > 0.0L);
          } else if (kind == NfpFeatureKind::hole) {
            REQUIRE(area < 0.0L);
          }
        };

    require_contract_normalized(filtered_actual[index]);
    require_contract_normalized(expected[index]);
    require_ring_equal(filtered_actual[index], expected[index]);
  }
}

auto parse_graph(const shiny::nesting::test::pt::ptree &node)
    -> ArrangementGraph {
  ArrangementGraph graph{};

  for (const auto &child : node.get_child("vertices")) {
    const auto &vertex = child.second;
    graph.vertices.push_back(GraphVertex{
        .id = vertex.get<std::uint32_t>("id"),
        .point = parse_point(vertex.get_child("point")),
        .kind = parse_vertex_kind(vertex.get<std::string>("kind")),
    });
  }

  for (const auto &child : node.get_child("edges")) {
    const auto &edge = child.second;
    graph.edges.push_back(GraphEdge{
        .from = edge.get<std::uint32_t>("from"),
        .to = edge.get<std::uint32_t>("to"),
        .kind = parse_edge_kind(edge.get<std::string>("kind")),
    });
  }

  if (const auto points = node.get_child_optional("perfect_fit_points")) {
    graph.perfect_fit_points = parse_points(*points);
  }
  if (const auto segments =
          node.get_child_optional("perfect_sliding_segments")) {
    graph.perfect_sliding_segments = parse_segments(*segments);
  }
  graph.pruned = node.get("pruned", false);
  return graph;
}

} // namespace

TEST_CASE("nonconvex pair fixtures", "[nfp][nonconvex][fixtures]") {
  const auto root = load_fixture_file("nfp/nonconvex_graph_pairs.json");
  REQUIRE(root.get<std::string>("algorithm") == "nonconvex_graph_nfp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "nonconvex_graph_pair");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const NonconvexNfpRequest request{
          .piece_a_id = inputs.get<std::uint32_t>("piece_a_id"),
          .piece_b_id = inputs.get<std::uint32_t>("piece_b_id"),
          .piece_a = parse_polygon(inputs.get_child("piece_a")),
          .piece_b = parse_polygon(inputs.get_child("piece_b")),
          .rotation_a = {.degrees = inputs.get<double>("rotation_a")},
          .rotation_b = {.degrees = inputs.get<double>("rotation_b")},
          .algorithm_revision = AlgorithmRevision{inputs.get<std::uint32_t>(
              "algorithm_revision")},
      };

      const auto result = shiny::nesting::compute_nonconvex_graph_nfp(request);

      REQUIRE(result.status ==
              parse_status(expected.get<std::string>("status")));
      REQUIRE(result.graph.pruned);
      REQUIRE(result.graph.vertices.size() ==
              expected.get<std::size_t>("graph_vertex_count"));
      REQUIRE(result.graph.edges.size() ==
              expected.get<std::size_t>("graph_edge_count"));
      REQUIRE(count_intersection_vertices(result.graph) ==
              expected.get<std::size_t>("graph_intersection_vertex_count"));
      REQUIRE(count_pruned_edges(result.graph) ==
              expected.get<std::size_t>("graph_pruned_edge_count"));

      for (const auto &edge : result.graph.edges) {
        REQUIRE(edge.from < result.graph.vertices.size());
        REQUIRE(edge.to < result.graph.vertices.size());
      }

      require_loops_equal(result.result.loops,
                          parse_rings(expected.get_child("outer_loops")),
                          NfpFeatureKind::outer_loop);
      require_loops_equal(result.result.loops,
                          parse_rings(expected.get_child("holes")),
                          NfpFeatureKind::hole);
      require_points_equal(
          result.result.perfect_fit_points,
          parse_points(expected.get_child("perfect_fit_points")));
      require_segments_equal(
          result.result.perfect_sliding_segments,
          parse_segments(expected.get_child("perfect_sliding_segments")));
    }
  }
}

TEST_CASE("nonconvex extraction fixtures",
          "[nfp][nonconvex][fixtures][extract]") {
  const auto root = load_fixture_file("nfp/nonconvex_graph_extract.json");
  REQUIRE(root.get<std::string>("algorithm") == "nonconvex_graph_nfp");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "nonconvex_graph_extract");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto graph = parse_graph(inputs.get_child("graph"));
      const auto result = shiny::nesting::extract_nfp_from_graph(graph);

      REQUIRE(result.algorithm ==
              shiny::nesting::AlgorithmKind::nonconvex_graph_nfp);
      REQUIRE(result.normalized);
      require_loops_equal(result.loops,
                          parse_rings(expected.get_child("outer_loops")),
                          NfpFeatureKind::outer_loop);
      require_loops_equal(result.loops,
                          parse_rings(expected.get_child("holes")),
                          NfpFeatureKind::hole);
      require_points_equal(
          result.perfect_fit_points,
          parse_points(expected.get_child("perfect_fit_points")));
      require_segments_equal(
          result.perfect_sliding_segments,
          parse_segments(expected.get_child("perfect_sliding_segments")));
    }
  }
}