#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "decomposition/decomposition_result.hpp"
#include "nfp/engine.hpp"
#include "nfp/nonconvex_nfp.hpp"

namespace {

auto count_vertex_kind(const shiny::nfp::ArrangementGraph &graph,
                       shiny::nfp::GraphVertexKind kind) -> std::size_t {
  return static_cast<std::size_t>(
      std::count_if(graph.vertices.begin(), graph.vertices.end(),
                    [kind](const shiny::nfp::GraphVertex &vertex) {
                      return vertex.kind == kind;
                    }));
}

auto count_edge_kind(const shiny::nfp::ArrangementGraph &graph,
                     shiny::nfp::GraphEdgeKind kind) -> std::size_t {
  return static_cast<std::size_t>(std::count_if(
      graph.edges.begin(), graph.edges.end(),
      [kind](const shiny::nfp::GraphEdge &edge) { return edge.kind == kind; }));
}

void require_valid_vertex_references(
    const shiny::nfp::ArrangementGraph &graph) {
  for (const auto &edge : graph.edges) {
    REQUIRE(edge.from < graph.vertices.size());
    REQUIRE(edge.to < graph.vertices.size());
  }
}

} // namespace

TEST_CASE("arrangement graph inserts intersection and midpoint vertices",
          "[nfp][nonconvex][graph]") {
  const shiny::nfp::NonconvexNfpRequest request{
      .piece_a_id = 201,
      .piece_b_id = 202,
      .piece_a = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 3.0},
                            {0.0, 3.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 3.0},
                            {0.0, 3.0}}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
      .algorithm_revision = shiny::nfp::cache::AlgorithmRevision{1},
  };
  const shiny::nfp::decomp::DecompositionResult piece_a{
      .components =
          {
              {.outer = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 1.0}, {0.0, 1.0}},
               .source_component_index = 0,
               .normalized = true},
              {.outer = {{0.0, 1.0}, {1.0, 1.0}, {1.0, 3.0}, {0.0, 3.0}},
               .source_component_index = 1,
               .normalized = true},
          },
      .validity = shiny::nfp::decomp::DecompositionValidity::valid,
      .signed_area = 5.0,
  };
  const shiny::nfp::decomp::DecompositionResult piece_b = piece_a;

  const auto graph =
      shiny::nfp::detail::build_arrangement_graph_from_decompositions(
          request, piece_a, piece_b);
  REQUIRE_FALSE(graph.pruned);
  REQUIRE_FALSE(graph.vertices.empty());
  REQUIRE_FALSE(graph.edges.empty());
  REQUIRE(count_vertex_kind(
              graph, shiny::nfp::GraphVertexKind::intersection_vertex) > 0U);
  REQUIRE(count_vertex_kind(graph,
                            shiny::nfp::GraphVertexKind::midpoint_vertex) > 0U);
  require_valid_vertex_references(graph);
}

TEST_CASE("pruning suppresses duplicate geometric edges",
          "[nfp][nonconvex][graph]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 2,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto pruned = shiny::nfp::prune_arrangement_graph(graph);
  REQUIRE(pruned.pruned);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::duplicate_edge) ==
          0U);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::pruned_edge) ==
          2U);
}

TEST_CASE("pruning suppresses antiparallel duplicate geometric edges",
          "[nfp][nonconvex][graph]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto pruned = shiny::nfp::prune_arrangement_graph(graph);
  REQUIRE(pruned.pruned);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::pruned_edge) ==
          2U);
}

TEST_CASE("pruning drops coincident spans with occupancy on both sides",
          "[nfp][nonconvex][graph]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 3,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 4,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::intersection_vertex},
              {.id = 5,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::intersection_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 4,
               .to = 5,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto pruned = shiny::nfp::prune_arrangement_graph(graph);
  REQUIRE(pruned.pruned);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::boundary_edge) ==
          0U);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::pruned_edge) ==
          3U);
}

TEST_CASE("pruning canonicalizes nearly coincident duplicate edges",
          "[nfp][nonconvex][graph]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {2.0000000004, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto pruned = shiny::nfp::prune_arrangement_graph(graph);
  REQUIRE(pruned.pruned);
  REQUIRE(count_edge_kind(pruned, shiny::nfp::GraphEdgeKind::pruned_edge) ==
          2U);
}

TEST_CASE("extract_nfp_from_graph drops midpoint-only collinear artifacts",
          "[nfp][nonconvex][extract]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {1.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 2,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {2.0, 1.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 4,
               .point = {2.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 5,
               .point = {1.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
              {.id = 6,
               .point = {0.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 7,
               .point = {0.0, 1.0},
               .kind = shiny::nfp::GraphVertexKind::midpoint_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 2,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 3,
               .to = 4,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 4,
               .to = 5,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 5,
               .to = 6,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 6,
               .to = 7,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 7,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto result = shiny::nfp::extract_nfp_from_graph(graph);
  REQUIRE(result.algorithm == shiny::nfp::AlgorithmKind::nonconvex_graph_nfp);
  REQUIRE(result.normalized);
  REQUIRE(result.loops.size() == 1U);
  REQUIRE(result.loops.front().vertices.size() == 4U);
}

TEST_CASE("extract_nfp_from_graph ignores pruned duplicate edges",
          "[nfp][nonconvex][extract]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {2.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {0.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 2,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 3,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::pruned_edge},
          },
      .pruned = true,
  };

  const auto result = shiny::nfp::extract_nfp_from_graph(graph);
  REQUIRE(result.algorithm == shiny::nfp::AlgorithmKind::nonconvex_graph_nfp);
  REQUIRE(result.normalized);
  REQUIRE(result.loops.size() == 1U);
  REQUIRE(result.loops.front().vertices.size() == 4U);
}

TEST_CASE("extract_nfp_from_graph classifies nested loops as holes",
          "[nfp][nonconvex][extract]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {5.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {5.0, 5.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {0.0, 5.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 4,
               .point = {1.0, 1.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 5,
               .point = {4.0, 1.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 6,
               .point = {4.0, 4.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 7,
               .point = {1.0, 4.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 2,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 3,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 4,
               .to = 5,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 5,
               .to = 6,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 6,
               .to = 7,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 7,
               .to = 4,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
  };

  const auto result = shiny::nfp::extract_nfp_from_graph(graph);
  REQUIRE(result.loops.size() == 2U);
  REQUIRE(result.loops[0].kind == shiny::nfp::NfpFeatureKind::outer_loop);
  REQUIRE(result.loops[1].kind == shiny::nfp::NfpFeatureKind::hole);
}

TEST_CASE("extract_nfp_from_graph filters interior degenerate features",
          "[nfp][nonconvex][extract]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {2.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {0.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 2,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 3,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
      .perfect_fit_points = {{1.0, 1.0}},
      .perfect_sliding_segments = {{{0.0, 0.0}, {2.0, 0.0}}},
  };

  const auto result = shiny::nfp::extract_nfp_from_graph(graph);
  REQUIRE(result.loops.size() == 1U);
  REQUIRE(result.perfect_fit_points.empty());
  REQUIRE(result.perfect_sliding_segments.size() == 1U);
}

TEST_CASE("extract_nfp_from_graph keeps boundary degenerate features",
          "[nfp][nonconvex][extract]") {
  const shiny::nfp::ArrangementGraph graph{
      .vertices =
          {
              {.id = 0,
               .point = {0.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 1,
               .point = {2.0, 0.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 2,
               .point = {2.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
              {.id = 3,
               .point = {0.0, 2.0},
               .kind = shiny::nfp::GraphVertexKind::original_vertex},
          },
      .edges =
          {
              {.from = 0,
               .to = 1,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 1,
               .to = 2,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 2,
               .to = 3,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
              {.from = 3,
               .to = 0,
               .kind = shiny::nfp::GraphEdgeKind::boundary_edge},
          },
      .perfect_fit_points = {{0.0, 1.0}},
      .perfect_sliding_segments = {{{0.0, 0.0}, {2.0, 0.0}}},
  };

  const auto result = shiny::nfp::extract_nfp_from_graph(graph);
  REQUIRE(result.loops.size() == 1U);
  REQUIRE(result.perfect_fit_points.size() == 1U);
  REQUIRE(result.perfect_sliding_segments.size() == 1U);
}

TEST_CASE("nonconvex nfp engine reuses cached decomposition inputs",
          "[nfp][nonconvex][engine]") {
  shiny::nfp::NfpEngine engine{};

  const shiny::nfp::NonconvexNfpRequest request{
      .piece_a_id = 101,
      .piece_b_id = 102,
      .piece_a = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {2.0, 1.0},
                            {2.0, 3.0},
                            {0.0, 3.0}}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 90.0},
      .algorithm_revision = shiny::nfp::cache::AlgorithmRevision{9},
  };

  const auto first = engine.compute_nonconvex_graph_nfp(
      request, shiny::nfp::cache::GeometryRevision{5},
      shiny::nfp::cache::GeometryRevision{7});
  REQUIRE(engine.decomposition_cache_size() == 2U);
  REQUIRE(engine.nonconvex_cache_size() == 1U);
  REQUIRE(first.graph.pruned);
  REQUIRE_FALSE(first.graph.vertices.empty());
  REQUIRE_FALSE(first.graph.edges.empty());
  REQUIRE(first.status == shiny::nfp::ExtractionStatus::success);
  REQUIRE_FALSE(first.result.loops.empty());
  REQUIRE(first.result.algorithm ==
          shiny::nfp::AlgorithmKind::nonconvex_graph_nfp);
  REQUIRE(first.result.normalized);

  const auto second = engine.compute_nonconvex_graph_nfp(
      request, shiny::nfp::cache::GeometryRevision{5},
      shiny::nfp::cache::GeometryRevision{7});
  REQUIRE(engine.decomposition_cache_size() == 2U);
  REQUIRE(engine.nonconvex_cache_size() == 1U);
  REQUIRE(second.status == first.status);
  REQUIRE(second.graph.vertices.size() == first.graph.vertices.size());
  REQUIRE(second.graph.edges.size() == first.graph.edges.size());
  REQUIRE(second.result.loops.size() == first.result.loops.size());
}

TEST_CASE("nonconvex cache key changes with algorithm revision",
          "[nfp][nonconvex][cache]") {
  shiny::nfp::NfpEngine engine{};

  shiny::nfp::NonconvexNfpRequest request{
      .piece_a_id = 301,
      .piece_b_id = 302,
      .piece_a = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {2.0, 1.0},
                            {2.0, 3.0},
                            {0.0, 3.0}}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
      .algorithm_revision = shiny::nfp::cache::AlgorithmRevision{1},
  };

  const auto first = engine.compute_nonconvex_graph_nfp(
      request, shiny::nfp::cache::GeometryRevision{2},
      shiny::nfp::cache::GeometryRevision{3});
  REQUIRE(engine.nonconvex_cache_size() == 1U);
  REQUIRE(first.status == shiny::nfp::ExtractionStatus::success);

  request.algorithm_revision = shiny::nfp::cache::AlgorithmRevision{2};
  const auto revised = engine.compute_nonconvex_graph_nfp(
      request, shiny::nfp::cache::GeometryRevision{2},
      shiny::nfp::cache::GeometryRevision{3});
  REQUIRE(engine.nonconvex_cache_size() == 2U);
  REQUIRE(revised.status == shiny::nfp::ExtractionStatus::success);
}

TEST_CASE("nonconvex engine does not cache interrupted requests",
          "[nfp][nonconvex][cache][cancellation]") {
  shiny::nfp::NfpEngine engine{};

  const shiny::nfp::NonconvexNfpRequest request{
      .piece_a_id = 401,
      .piece_b_id = 402,
      .piece_a = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {2.0, 1.0},
                            {2.0, 3.0},
                            {0.0, 3.0}}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
      .algorithm_revision = shiny::nfp::cache::AlgorithmRevision{3},
  };

  const auto interrupted = engine.compute_nonconvex_graph_nfp(
      request, shiny::nfp::cache::GeometryRevision{8},
      shiny::nfp::cache::GeometryRevision{9}, []() { return true; });

  REQUIRE(interrupted.graph.vertices.empty());
  REQUIRE(interrupted.graph.edges.empty());
  REQUIRE(interrupted.result.loops.empty());
  REQUIRE(engine.decomposition_cache_size() == 0U);
  REQUIRE(engine.nonconvex_cache_size() == 0U);
}