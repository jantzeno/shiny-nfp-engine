#include "nfp/engine.hpp"

#include <utility>

#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"

namespace shiny::nfp {

auto NfpEngine::compute_convex_nfp(const ConvexNfpRequest &request,
                                   cache::GeometryRevision geometry_a_revision,
                                   cache::GeometryRevision geometry_b_revision)
    -> NfpResult {
  return shiny::nfp::compute_convex_nfp(request, geometry_a_revision,
                                        geometry_b_revision, convex_cache_);
}

auto NfpEngine::compute_convex_ifp(const ConvexIfpRequest &request,
                                   cache::GeometryRevision geometry_a_revision,
                                   cache::GeometryRevision geometry_b_revision)
    -> NfpResult {
  return shiny::nfp::compute_convex_ifp(request, geometry_a_revision,
                                        geometry_b_revision, convex_cache_);
}

auto NfpEngine::decompose_polygon(const decomp::DecompositionRequest &request,
                                  cache::GeometryRevision geometry_revision)
    -> decomp::DecompositionResult {
  return decomposition_engine_.decompose_polygon(request, geometry_revision);
}

auto NfpEngine::compute_nonconvex_graph_nfp(
    const NonconvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision,
    const std::function<bool()> &interruption_requested) -> NonconvexNfpResult {
  const auto cache_key = cache::make_nonconvex_nfp_cache_key(
      request, geometry_a_revision, geometry_b_revision);
  if (const auto *cached = nonconvex_cache_.get(cache_key); cached != nullptr) {
    return *cached;
  }

  if (interruption_requested && interruption_requested()) {
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

  if (interruption_requested && interruption_requested()) {
    return {};
  }

  auto graph = detail::build_arrangement_graph_from_decompositions(
      request, piece_a, piece_b, interruption_requested);
  auto pruned_graph = detail::prune_arrangement_graph_with_interruption(
      std::move(graph), interruption_requested);
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
  if (!interruption_requested || !interruption_requested()) {
    nonconvex_cache_.put(cache_key, computed);
  }
  return computed;
}

auto NfpEngine::compute_orbital_verifier_nfp(
    const NonconvexNfpRequest &request,
    cache::GeometryRevision geometry_a_revision,
    cache::GeometryRevision geometry_b_revision) -> OrbitalVerifierResult {
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