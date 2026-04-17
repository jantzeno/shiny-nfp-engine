#include "nfp/engine.hpp"

#include <utility>

#include "logging/shiny_log.hpp"
#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"

namespace shiny::nfp {

auto NfpEngine::compute_convex_nfp(const ConvexNfpRequest &request,
                                   cache::GeometryRevision geometry_a_revision,
                                   cache::GeometryRevision geometry_b_revision)
    -> NfpResult {
  SHINY_DEBUG(
      "compute_convex_nfp: piece_a_id={} piece_b_id={} rot_a={} rot_b={} rev_a={} rev_b={}",
      request.piece_a_id, request.piece_b_id, request.rotation_a.degrees,
      request.rotation_b.degrees, geometry_a_revision.value,
      geometry_b_revision.value);
  return shiny::nfp::compute_convex_nfp(request, geometry_a_revision,
                                        geometry_b_revision, convex_cache_);
}

auto NfpEngine::compute_convex_ifp(const ConvexIfpRequest &request,
                                   cache::GeometryRevision geometry_a_revision,
                                   cache::GeometryRevision geometry_b_revision)
    -> NfpResult {
  SHINY_DEBUG(
      "compute_convex_ifp: container_id={} piece_id={} rot_container={} rot_piece={} rev_container={} rev_piece={}",
      request.container_id, request.piece_id, request.container_rotation.degrees,
      request.piece_rotation.degrees, geometry_a_revision.value,
      geometry_b_revision.value);
  return shiny::nfp::compute_convex_ifp(request, geometry_a_revision,
                                        geometry_b_revision, convex_cache_);
}

auto NfpEngine::decompose_polygon(const decomp::DecompositionRequest &request,
                                  cache::GeometryRevision geometry_revision)
    -> decomp::DecompositionResult {
  SHINY_DEBUG("decompose_polygon: piece_id={} rev={} algorithm={}",
              request.piece_id, geometry_revision.value,
              static_cast<std::uint32_t>(request.algorithm));
  return decomposition_engine_.decompose_polygon(request, geometry_revision);
}

auto NfpEngine::compute_nonconvex_graph_nfp(
    const NonconvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    const std::function<bool()> &interruption_requested) -> NonconvexNfpResult {
  SHINY_DEBUG(
      "compute_nonconvex_graph_nfp: piece_a_id={} piece_b_id={} rev_a={} rev_b={} outer_a_points={} outer_b_points={}",
      request.piece_a_id, request.piece_b_id, geometry_a_revision.value,
      geometry_b_revision.value, request.piece_a.outer.size(),
      request.piece_b.outer.size());
  const auto cache_key = cache::make_nonconvex_nfp_cache_key(
      request, geometry_a_revision, geometry_b_revision);
  if (const auto *cached = nonconvex_cache_.get(cache_key); cached != nullptr) {
    SHINY_TRACE("compute_nonconvex_graph_nfp: cache hit piece_a_id={} piece_b_id={}",
                request.piece_a_id, request.piece_b_id);
    return *cached;
  }
  SHINY_TRACE("compute_nonconvex_graph_nfp: cache miss piece_a_id={} piece_b_id={}",
              request.piece_a_id, request.piece_b_id);

  if (interruption_requested && interruption_requested()) {
    SHINY_DEBUG(
        "compute_nonconvex_graph_nfp: interrupted before decomposition piece_a_id={} piece_b_id={}",
        request.piece_a_id, request.piece_b_id);
    return {};
  }

  const decomp::DecompositionRequest request_a{
      .piece_id = request.piece_a_id,
      .polygon = request.piece_a,
      .rotation = request.rotation_a,
      .algorithm =
          decomp::DecompositionAlgorithm::cgal_optimal_convex_partition,
  };
  const decomp::DecompositionRequest request_b{
      .piece_id = request.piece_b_id,
      .polygon = request.piece_b,
      .rotation = request.rotation_b,
      .algorithm =
          decomp::DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto piece_a =
      decomposition_engine_.decompose_polygon(request_a, geometry_a_revision);
  const auto piece_b =
      decomposition_engine_.decompose_polygon(request_b, geometry_b_revision);
  SHINY_TRACE(
      "compute_nonconvex_graph_nfp: decomposed piece_a_parts={} piece_b_parts={}",
      piece_a.components.size(), piece_b.components.size());

  if (interruption_requested && interruption_requested()) {
    SHINY_DEBUG(
        "compute_nonconvex_graph_nfp: interrupted after decomposition piece_a_id={} piece_b_id={}",
        request.piece_a_id, request.piece_b_id);
    return {};
  }

  auto graph = detail::build_arrangement_graph_from_decompositions(
      request, piece_a, piece_b, interruption_requested);
  SHINY_TRACE(
      "compute_nonconvex_graph_nfp: graph built vertices={} edges={}",
      graph.vertices.size(), graph.edges.size());
  auto pruned_graph = detail::prune_arrangement_graph_with_interruption(
      std::move(graph), interruption_requested);
  SHINY_TRACE(
      "compute_nonconvex_graph_nfp: graph pruned vertices={} edges={}",
      pruned_graph.vertices.size(), pruned_graph.edges.size());
  auto result = detail::extract_nfp_from_graph_with_interruption(
      pruned_graph, interruption_requested);
  const auto status =
      pruned_graph.vertices.empty() || pruned_graph.edges.empty()
          ? ExtractionStatus::empty
          : (result.loops.empty() ? ExtractionStatus::invalid_graph
                                  : ExtractionStatus::success);

  NonconvexNfpResult computed{
      .result = std::move(result),
      .graph = std::move(pruned_graph),
      .status = status,
  };
  SHINY_DEBUG(
      "compute_nonconvex_graph_nfp: completed status={} loops={} cached={} piece_a_id={} piece_b_id={}",
      static_cast<std::uint32_t>(computed.status),
      computed.result.loops.size(),
      (!interruption_requested || !interruption_requested()) ? 1 : 0,
      request.piece_a_id, request.piece_b_id);
  if (!interruption_requested || !interruption_requested()) {
    nonconvex_cache_.put(cache_key, computed);
  }
  return computed;
}

auto NfpEngine::compute_orbital_verifier_nfp(
    const NonconvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision) -> OrbitalVerifierResult {
  SHINY_DEBUG(
      "compute_orbital_verifier_nfp: piece_a_id={} piece_b_id={} rev_a={} rev_b={}",
      request.piece_a_id, request.piece_b_id, geometry_a_revision.value,
      geometry_b_revision.value);
  const decomp::DecompositionRequest request_a{
      .piece_id = request.piece_a_id,
      .polygon = request.piece_a,
      .rotation = request.rotation_a,
      .algorithm =
          decomp::DecompositionAlgorithm::cgal_optimal_convex_partition,
  };
  const decomp::DecompositionRequest request_b{
      .piece_id = request.piece_b_id,
      .polygon = request.piece_b,
      .rotation = request.rotation_b,
      .algorithm =
          decomp::DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto piece_a =
      decomposition_engine_.decompose_polygon(request_a, geometry_a_revision);
  const auto piece_b =
      decomposition_engine_.decompose_polygon(request_b, geometry_b_revision);
  SHINY_TRACE(
      "compute_orbital_verifier_nfp: decomposed piece_a_parts={} piece_b_parts={}",
      piece_a.components.size(), piece_b.components.size());
  return detail::compute_orbital_verifier_from_decompositions(request, piece_a,
                                                              piece_b);
}

auto NfpEngine::clear_convex_cache() -> void { convex_cache_.clear(); }

auto NfpEngine::clear_nonconvex_cache() -> void { nonconvex_cache_.clear(); }

auto NfpEngine::clear_decomposition_cache() -> void {
  decomposition_engine_.clear_cache();
}

auto NfpEngine::clear_caches() -> void {
  clear_convex_cache();
  clear_nonconvex_cache();
  clear_decomposition_cache();
}

auto NfpEngine::convex_cache_size() const -> std::size_t {
  return convex_cache_.size();
}

auto NfpEngine::nonconvex_cache_size() const -> std::size_t {
  return nonconvex_cache_.size();
}

auto NfpEngine::decomposition_cache_size() const -> std::size_t {
  return decomposition_engine_.cache_size();
}

} // namespace shiny::nfp
