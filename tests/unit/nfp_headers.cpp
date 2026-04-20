#include <catch2/catch_test_macros.hpp>

#include <type_traits>
#include <utility>

#include "cache/stores.hpp"
#include "nfp/cache_keys.hpp"
#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/engine.hpp"
#include "nfp/nonconvex_nfp.hpp"
#include "nfp/orbital_verifier.hpp"

TEST_CASE("nfp headers expose the planned milestone 3, 5, and 6 surfaces",
          "[nfp][headers]") {
  using shiny::nesting::AlgorithmKind;
  using shiny::nesting::ArrangementGraph;
  using shiny::nesting::ConvexEdgeSequence;
  using shiny::nesting::ConvexIfpRequest;
  using shiny::nesting::ConvexNfpRequest;
  using shiny::nesting::ConvexOrderingVertex;
  using shiny::nesting::ExtractionStatus;
  using shiny::nesting::GraphEdge;
  using shiny::nesting::GraphEdgeKind;
  using shiny::nesting::GraphVertex;
  using shiny::nesting::GraphVertexKind;
  using shiny::nesting::NfpEngine;
  using shiny::nesting::NfpFeatureKind;
  using shiny::nesting::NfpLoop;
  using shiny::nesting::NfpResult;
  using shiny::nesting::NonconvexNfpRequest;
  using shiny::nesting::NonconvexNfpResult;
  using shiny::nesting::OrbitalFeasibleTranslation;
  using shiny::nesting::OrbitalLoopTrace;
  using shiny::nesting::OrbitalToucher;
  using shiny::nesting::OrbitalTouchKind;
  using shiny::nesting::OrbitalVerifierResult;
  using shiny::nesting::OrbitalVerifierStatus;
  using shiny::nesting::cache::AlgorithmRevision;
  using shiny::nesting::cache::CacheStore;
  using shiny::nesting::cache::GeometryRevision;
  using shiny::nesting::cache::NonconvexNfpCacheKey;
  using shiny::nesting::cache::PairRotationKey;

  const NfpLoop loop{
      .vertices = {{0.0, 0.0}, {2.0, 0.0}, {1.0, 1.0}},
      .kind = NfpFeatureKind::outer_loop,
  };

  const NfpResult result{
      .loops = {loop},
      .perfect_fit_points = {{1.0, 2.0}},
      .perfect_sliding_segments = {{{0.0, 0.0}, {2.0, 0.0}}},
      .algorithm = AlgorithmKind::convex_nfp,
      .normalized = true,
  };

  REQUIRE(result.loops.size() == 1U);
  REQUIRE(result.loops.front().kind == NfpFeatureKind::outer_loop);
  REQUIRE(result.perfect_fit_points.size() == 1U);
  REQUIRE(result.perfect_sliding_segments.size() == 1U);
  REQUIRE(result.algorithm == AlgorithmKind::convex_nfp);
  REQUIRE(result.normalized);

  const ConvexNfpRequest convex_nfp_request{
      .piece_a_id = 11,
      .piece_b_id = 29,
      .convex_a = {{0.0, 0.0}, {3.0, 0.0}, {0.0, 2.0}},
      .convex_b = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      .rotation_a = {.degrees = 90.0},
      .rotation_b = {.degrees = 270.0},
  };
  REQUIRE(convex_nfp_request.piece_a_id == 11U);
  REQUIRE(convex_nfp_request.piece_b_id == 29U);
  REQUIRE(convex_nfp_request.convex_a.size() == 3U);
  REQUIRE(convex_nfp_request.rotation_a.degrees == 90.0);
  REQUIRE(convex_nfp_request.rotation_b.degrees == 270.0);

  const ConvexIfpRequest convex_ifp_request{
      .container_id = 7,
      .piece_id = 9,
      .container = {.outer = {{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}}},
      .piece = {.outer = {{0.0, 0.0}, {2.0, 0.0}, {1.0, 1.0}}},
      .container_rotation = {.degrees = 0.0},
      .piece_rotation = {.degrees = 180.0},
  };
  REQUIRE(convex_ifp_request.container_id == 7U);
  REQUIRE(convex_ifp_request.piece_id == 9U);
  REQUIRE(convex_ifp_request.container.outer.size() == 4U);
  REQUIRE(convex_ifp_request.piece_rotation.degrees == 180.0);

  const NonconvexNfpRequest nonconvex_request{
      .piece_a_id = 41,
      .piece_b_id = 42,
      .piece_a = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {2.0, 0.0},
                            {2.0, 2.0},
                            {1.0, 1.0},
                            {0.0, 2.0}}},
      .rotation_a = {.degrees = 180.0},
      .rotation_b = {.degrees = 90.0},
      .algorithm_revision = AlgorithmRevision{3},
  };
  REQUIRE(nonconvex_request.piece_a_id == 41U);
  REQUIRE(nonconvex_request.piece_b.outer.size() == 5U);
  REQUIRE(nonconvex_request.algorithm_revision.value == 3U);

  const GraphVertex graph_vertex{
      .id = 7,
      .point = {4.0, 6.0},
      .kind = GraphVertexKind::midpoint_vertex,
  };
  const GraphEdge graph_edge{
      .from = 7,
      .to = 9,
      .kind = GraphEdgeKind::duplicate_edge,
  };
  const ArrangementGraph graph{
      .vertices = {graph_vertex},
      .edges = {graph_edge},
      .perfect_fit_points = {{1.0, 2.0}},
      .perfect_sliding_segments = {{{0.0, 0.0}, {1.0, 0.0}}},
      .pruned = true,
  };
  const NonconvexNfpResult nonconvex_result{
      .result = {.algorithm = AlgorithmKind::nonconvex_graph_nfp,
                 .normalized = true},
      .graph = graph,
      .status = ExtractionStatus::success,
  };
  REQUIRE(graph_vertex.id == 7U);
  REQUIRE(graph_vertex.kind == GraphVertexKind::midpoint_vertex);
  REQUIRE(graph_edge.kind == GraphEdgeKind::duplicate_edge);
  REQUIRE(graph.perfect_fit_points.size() == 1U);
  REQUIRE(graph.perfect_sliding_segments.size() == 1U);
  REQUIRE(graph.pruned);
  REQUIRE(nonconvex_result.result.algorithm ==
          AlgorithmKind::nonconvex_graph_nfp);
  REQUIRE(nonconvex_result.status == ExtractionStatus::success);

  const OrbitalToucher toucher{
      .translation = {1.0, 1.0},
      .kind = OrbitalTouchKind::vertex_edge,
      .contact_points = {{1.0, 0.0}},
  };
  const OrbitalFeasibleTranslation feasible_translation{
      .segment = {{0.0, 0.0}, {2.0, 0.0}},
      .trimmed = false,
  };
  const OrbitalLoopTrace trace{
      .start = {0.0, 0.0},
      .kind = NfpFeatureKind::hole,
      .segment_count = 3,
      .completed = true,
  };
  const OrbitalVerifierResult orbital_result{
      .result = {.algorithm = AlgorithmKind::orbital_verifier,
                 .normalized = true},
      .touchers = {toucher},
      .feasible_translations = {feasible_translation},
      .traces = {trace},
      .inner_loop_starts = {{0.0, 0.0}},
      .status = OrbitalVerifierStatus::success,
      .completed = true,
  };
  REQUIRE(toucher.kind == OrbitalTouchKind::vertex_edge);
  REQUIRE(feasible_translation.segment.start.x == 0.0);
  REQUIRE(trace.kind == NfpFeatureKind::hole);
  REQUIRE(orbital_result.result.algorithm == AlgorithmKind::orbital_verifier);
  REQUIRE(orbital_result.status == OrbitalVerifierStatus::success);
  REQUIRE(orbital_result.completed);

  const ConvexOrderingVertex vertex{
      .point = {4.0, 6.0},
      .source_edge_index = 2,
      .polar_key = 0.75,
  };
  const ConvexEdgeSequence sequence{
      .edges = {{1.0, 0.0}, {0.0, 1.0}},
      .source_indices = {0U, 1U},
  };
  const PairRotationKey pair_key{
      .piece_a_id = 11,
      .rotation_a = {.degrees = 90.0},
      .piece_b_id = 29,
      .rotation_b = {.degrees = 270.0},
      .inside = false,
      .geometry_a_revision = GeometryRevision{4},
      .geometry_b_revision = GeometryRevision{5},
  };
  const NonconvexNfpCacheKey nonconvex_key{
      .pair =
          {
              .piece_a_id = 41,
              .rotation_a = {.degrees = 180.0},
              .piece_b_id = 42,
              .rotation_b = {.degrees = 90.0},
              .inside = false,
              .geometry_a_revision = GeometryRevision{6},
              .geometry_b_revision = GeometryRevision{7},
          },
      .algorithm_revision = AlgorithmRevision{3},
  };
  CacheStore<PairRotationKey, NfpResult> cache_store{};
  CacheStore<NonconvexNfpCacheKey, NonconvexNfpResult> nonconvex_cache_store{};
  NfpEngine engine{};
  REQUIRE(vertex.source_edge_index == 2U);
  REQUIRE(sequence.edges.size() == sequence.source_indices.size());
  REQUIRE(pair_key.geometry_a_revision.value == 4U);
  REQUIRE(nonconvex_key.algorithm_revision.value == 3U);
  REQUIRE(cache_store.size() == 0U);
  REQUIRE(nonconvex_cache_store.size() == 0U);
  REQUIRE(engine.nonconvex_cache_size() == 0U);
  REQUIRE(engine.decomposition_cache_size() == 0U);

  STATIC_REQUIRE(std::is_same_v<decltype(shiny::nesting::compute_convex_nfp(
                                    std::declval<const ConvexNfpRequest &>())),
                                NfpResult>);
  STATIC_REQUIRE(std::is_same_v<
                 decltype(shiny::nesting::compute_convex_nfp(
                     std::declval<const ConvexNfpRequest &>(),
                     std::declval<GeometryRevision>(),
                     std::declval<GeometryRevision>(),
                     std::declval<CacheStore<PairRotationKey, NfpResult> &>())),
                 NfpResult>);
  STATIC_REQUIRE(std::is_same_v<decltype(shiny::nesting::compute_convex_ifp(
                                    std::declval<const ConvexIfpRequest &>())),
                                NfpResult>);
  STATIC_REQUIRE(std::is_same_v<
                 decltype(shiny::nesting::compute_convex_ifp(
                     std::declval<const ConvexIfpRequest &>(),
                     std::declval<GeometryRevision>(),
                     std::declval<GeometryRevision>(),
                     std::declval<CacheStore<PairRotationKey, NfpResult> &>())),
                 NfpResult>);
  STATIC_REQUIRE(
      std::is_same_v<
          decltype(shiny::nesting::build_convex_edge_sequence(
              std::declval<std::span<const shiny::nesting::geom::Point2>>())),
          ConvexEdgeSequence>);
  STATIC_REQUIRE(std::is_same_v<decltype(shiny::nesting::order_convex_nfp_vertices(
                                    std::declval<const NfpResult &>())),
                                std::vector<ConvexOrderingVertex>>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nesting::build_arrangement_graph(
                         std::declval<const NonconvexNfpRequest &>())),
                     ArrangementGraph>);
  STATIC_REQUIRE(std::is_same_v<decltype(shiny::nesting::prune_arrangement_graph(
                                    std::declval<ArrangementGraph>())),
                                ArrangementGraph>);
  STATIC_REQUIRE(std::is_same_v<decltype(shiny::nesting::extract_nfp_from_graph(
                                    std::declval<const ArrangementGraph &>())),
                                NfpResult>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nesting::compute_nonconvex_graph_nfp(
                         std::declval<const NonconvexNfpRequest &>())),
                     NonconvexNfpResult>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nesting::compute_orbital_verifier_nfp(
                         std::declval<const NonconvexNfpRequest &>())),
                     OrbitalVerifierResult>);
  STATIC_REQUIRE(
      std::is_same_v<
          decltype(std::declval<NfpEngine &>().compute_nonconvex_graph_nfp(
              std::declval<const NonconvexNfpRequest &>(),
              std::declval<GeometryRevision>(),
              std::declval<GeometryRevision>())),
          NonconvexNfpResult>);
  STATIC_REQUIRE(
      std::is_same_v<
          decltype(std::declval<NfpEngine &>().compute_orbital_verifier_nfp(
              std::declval<const NonconvexNfpRequest &>(),
              std::declval<GeometryRevision>(),
              std::declval<GeometryRevision>())),
          OrbitalVerifierResult>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nesting::cache::make_nonconvex_nfp_cache_key(
                         std::declval<const NonconvexNfpRequest &>(),
                         std::declval<GeometryRevision>(),
                         std::declval<GeometryRevision>())),
                     NonconvexNfpCacheKey>);
}