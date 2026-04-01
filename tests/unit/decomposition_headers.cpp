#include <catch2/catch_test_macros.hpp>

#include <type_traits>
#include <utility>

#include "cache/stores.hpp"
#include "decomposition/decompose.hpp"
#include "decomposition/decomposition_result.hpp"
#include "nfp/engine.hpp"

TEST_CASE("decomposition headers expose the planned milestone 4 surface",
          "[decomposition][headers]") {
  using shiny::nfp::NfpEngine;
  using shiny::nfp::cache::CacheStore;
  using shiny::nfp::cache::GeometryRevision;
  using shiny::nfp::cache::PieceRotationKey;
  using shiny::nfp::decomp::ConvexComponent;
  using shiny::nfp::decomp::DecompositionAlgorithm;
  using shiny::nfp::decomp::DecompositionEngine;
  using shiny::nfp::decomp::DecompositionRequest;
  using shiny::nfp::decomp::DecompositionResult;
  using shiny::nfp::decomp::DecompositionValidity;

  const ConvexComponent component{
      .outer = {{0.0, 0.0}, {4.0, 0.0}, {0.0, 2.0}},
      .source_component_index = 3,
      .normalized = true,
  };
  const DecompositionResult result{
      .components = {component},
      .validity = DecompositionValidity::valid,
      .signed_area = 4.0,
      .cached = false,
  };
  const DecompositionRequest request{
      .piece_id = 17,
      .polygon = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .rotation = {.degrees = 180.0},
      .algorithm = DecompositionAlgorithm::cgal_approx_convex_partition,
  };
  CacheStore<PieceRotationKey, DecompositionResult> cache_store{};
  DecompositionEngine decomposition_engine{};
  NfpEngine nfp_engine{};

  REQUIRE(component.outer.size() == 3U);
  REQUIRE(component.source_component_index == 3U);
  REQUIRE(component.normalized);
  REQUIRE(result.components.size() == 1U);
  REQUIRE(result.validity == DecompositionValidity::valid);
  REQUIRE(result.signed_area == 4.0);
  REQUIRE_FALSE(result.cached);
  REQUIRE(request.piece_id == 17U);
  REQUIRE(request.rotation.degrees == 180.0);
  REQUIRE(request.algorithm ==
          DecompositionAlgorithm::cgal_approx_convex_partition);
  REQUIRE(cache_store.size() == 0U);
  REQUIRE(decomposition_engine.cache_size() == 0U);
  REQUIRE(nfp_engine.convex_cache_size() == 0U);
  REQUIRE(nfp_engine.nonconvex_cache_size() == 0U);
  REQUIRE(nfp_engine.decomposition_cache_size() == 0U);

  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nfp::decomp::decompose_polygon(
                         std::declval<const DecompositionRequest &>())),
                     DecompositionResult>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(shiny::nfp::decomp::decompose_polygon(
                         std::declval<const DecompositionRequest &>(),
                         std::declval<GeometryRevision>(),
                         std::declval<CacheStore<PieceRotationKey,
                                                 DecompositionResult> &>())),
                     DecompositionResult>);
  STATIC_REQUIRE(std::is_same_v<
                 decltype(shiny::nfp::decomp::validate_decomposition(
                     std::declval<const shiny::nfp::geom::PolygonWithHoles &>(),
                     std::declval<const DecompositionResult &>())),
                 DecompositionValidity>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(decomposition_engine.decompose_polygon(
                         std::declval<const DecompositionRequest &>(),
                         std::declval<GeometryRevision>())),
                     DecompositionResult>);
  STATIC_REQUIRE(
      std::is_same_v<decltype(nfp_engine.decompose_polygon(
                         std::declval<const DecompositionRequest &>(),
                         std::declval<GeometryRevision>())),
                     DecompositionResult>);
}