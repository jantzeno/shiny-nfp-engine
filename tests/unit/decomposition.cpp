#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "cache/stores.hpp"
#include "decomposition/decompose.hpp"
#include "geometry/normalize.hpp"
#include "nfp/cache_keys.hpp"
#include "nfp/convex_nfp.hpp"
#include "nfp/engine.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nfp::ConvexNfpRequest;
using shiny::nfp::NfpEngine;
using shiny::nfp::NonconvexNfpRequest;
using shiny::nfp::cache::AlgorithmRevision;
using shiny::nfp::cache::CacheStore;
using shiny::nfp::cache::GeometryRevision;
using shiny::nfp::cache::PieceRotationKey;
using shiny::nfp::decomp::decompose_polygon;
using shiny::nfp::decomp::DecompositionAlgorithm;
using shiny::nfp::decomp::DecompositionRequest;
using shiny::nfp::decomp::DecompositionResult;
using shiny::nfp::decomp::DecompositionValidity;
using shiny::nfp::decomp::validate_decomposition;
using shiny::nfp::geom::normalize_polygon;
using shiny::nfp::test::load_fixture_file;
using shiny::nfp::test::parse_polygon;
using shiny::nfp::test::parse_ring;
using shiny::nfp::test::require_fixture_metadata;
using shiny::nfp::test::require_ring_equal;

constexpr auto epsilon = 1.0e-9;

auto parse_algorithm(std::string_view value) -> DecompositionAlgorithm {
  if (value == "cgal_optimal_convex_partition") {
    return DecompositionAlgorithm::cgal_optimal_convex_partition;
  }
  if (value == "cgal_approx_convex_partition") {
    return DecompositionAlgorithm::cgal_approx_convex_partition;
  }

  throw std::runtime_error("unknown decomposition algorithm fixture value");
}

auto parse_validity(std::string_view value) -> DecompositionValidity {
  if (value == "unknown") {
    return DecompositionValidity::unknown;
  }
  if (value == "valid") {
    return DecompositionValidity::valid;
  }
  if (value == "invalid_topology") {
    return DecompositionValidity::invalid_topology;
  }
  if (value == "invalid_orientation") {
    return DecompositionValidity::invalid_orientation;
  }
  if (value == "area_mismatch") {
    return DecompositionValidity::area_mismatch;
  }
  if (value == "nonconvex_component") {
    return DecompositionValidity::nonconvex_component;
  }

  throw std::runtime_error("unknown decomposition validity fixture value");
}

auto parse_ring_list(const shiny::nfp::test::pt::ptree &node)
    -> std::vector<shiny::nfp::geom::Ring> {
  std::vector<shiny::nfp::geom::Ring> rings;
  for (const auto &child : node) {
    rings.push_back(parse_ring(child.second));
  }
  return rings;
}

auto point_less(const shiny::nfp::geom::Point2 &lhs,
                const shiny::nfp::geom::Point2 &rhs) -> bool {
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
  if (lhs.size() != rhs.size()) {
    return lhs.size() < rhs.size();
  }

  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (point_less(lhs[index], rhs[index])) {
      return true;
    }
    if (point_less(rhs[index], lhs[index])) {
      return false;
    }
  }

  return false;
}

auto inverse_rotate_point(const shiny::nfp::geom::Point2 &point,
                          int rotation_degrees) -> shiny::nfp::geom::Point2 {
  switch (((rotation_degrees % 360) + 360) % 360) {
  case 0:
    return point;
  case 90:
    return {.x = point.y, .y = -point.x};
  case 180:
    return {.x = -point.x, .y = -point.y};
  case 270:
    return {.x = -point.y, .y = point.x};
  default:
    throw std::runtime_error(
        "rotation stability fixtures require right-angle rotations");
  }
}

auto canonicalize_components(const DecompositionResult &result,
                             int rotation_degrees)
    -> std::vector<shiny::nfp::geom::Ring> {
  std::vector<shiny::nfp::geom::Ring> canonical_components;
  canonical_components.reserve(result.components.size());

  for (const auto &component : result.components) {
    shiny::nfp::geom::Ring rotated;
    rotated.reserve(component.outer.size());
    for (const auto &point : component.outer) {
      rotated.push_back(inverse_rotate_point(point, rotation_degrees));
    }
    canonical_components.push_back(
        normalize_polygon(shiny::nfp::geom::Polygon{.outer = rotated}).outer);
  }

  std::sort(canonical_components.begin(), canonical_components.end(),
            ring_less);
  return canonical_components;
}

void require_decomposition_equal(const DecompositionResult &actual,
                                 const DecompositionResult &expected) {
  REQUIRE(actual.validity == expected.validity);
  REQUIRE(std::fabs(actual.signed_area - expected.signed_area) <= epsilon);
  REQUIRE(actual.components.size() == expected.components.size());

  for (std::size_t index = 0; index < actual.components.size(); ++index) {
    REQUIRE(actual.components[index].source_component_index ==
            expected.components[index].source_component_index);
    REQUIRE(actual.components[index].normalized ==
            expected.components[index].normalized);
    require_ring_equal(actual.components[index].outer,
                       expected.components[index].outer);
  }
}

} // namespace

TEST_CASE("decomposition fixtures", "[decomposition][fixtures]") {
  const auto root = load_fixture_file("decomposition/decompose.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "decomposition");

      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto expected_validity =
          parse_validity(expected.get<std::string>("validity"));
      const auto request = DecompositionRequest{
          .piece_id = inputs.get<std::uint32_t>("piece_id"),
          .polygon = parse_polygon(inputs.get_child("polygon")),
          .rotation = {.degrees = inputs.get<double>("rotation_degrees", 0.0)},
          .algorithm = parse_algorithm(inputs.get<std::string>("algorithm")),
      };

      const auto result = decompose_polygon(request);

      REQUIRE(result.validity == expected_validity);
      REQUIRE(result.components.size() ==
              expected.get<std::size_t>("component_count"));
      REQUIRE(std::fabs(result.signed_area -
                        expected.get<double>("signed_area")) <= epsilon);
      REQUIRE_FALSE(result.cached);

      for (const auto &component : result.components) {
        REQUIRE(component.normalized);
        REQUIRE(component.outer.size() >= 3U);
      }

      if (const auto expected_components =
              expected.get_child_optional("normalized_components")) {
        auto actual_rings = canonicalize_components(result, 0);
        auto expected_rings = parse_ring_list(*expected_components);
        std::sort(expected_rings.begin(), expected_rings.end(), ring_less);

        REQUIRE(actual_rings.size() == expected_rings.size());
        for (std::size_t index = 0; index < expected_rings.size(); ++index) {
          require_ring_equal(actual_rings[index], expected_rings[index]);
        }
      }

      REQUIRE(validate_decomposition(request.polygon, result) ==
              expected_validity);
    }
  }
}

TEST_CASE("decomposition rotation variants are stable in canonical space",
          "[decomposition][fixtures][rotation]") {
  const auto root = load_fixture_file("decomposition/decompose.json");

  std::map<std::string, std::vector<shiny::nfp::geom::Ring>> baseline_by_group;
  std::map<std::string, std::string> baseline_id_by_group;

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto stability_group =
        fixture.get_optional<std::string>("stability_group");
    if (!stability_group) {
      continue;
    }

    const auto inputs = fixture.get_child("inputs");
    const auto request = DecompositionRequest{
        .piece_id = inputs.get<std::uint32_t>("piece_id"),
        .polygon = parse_polygon(inputs.get_child("polygon")),
        .rotation = {.degrees = inputs.get<double>("rotation_degrees", 0.0)},
        .algorithm = parse_algorithm(inputs.get<std::string>("algorithm")),
    };
    const auto result = decompose_polygon(request);
    const auto canonical_components = canonicalize_components(
        result, static_cast<int>(std::lround(request.rotation.degrees)));

    const auto [it, inserted] =
        baseline_by_group.emplace(*stability_group, canonical_components);
    if (inserted) {
      baseline_id_by_group.emplace(*stability_group,
                                   fixture.get<std::string>("id"));
      continue;
    }

    DYNAMIC_SECTION(fixture.get<std::string>("id")) {
      REQUIRE(canonical_components.size() == it->second.size());
      for (std::size_t index = 0; index < canonical_components.size();
           ++index) {
        require_ring_equal(canonical_components[index], it->second[index]);
      }
    }
  }
}

TEST_CASE("convex decomposition returns a single normalized component",
          "[decomposition][convex]") {
  const DecompositionRequest request{
      .piece_id = 41,
      .polygon = {.outer = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 2.0}, {0.0, 2.0}}},
      .rotation = {.degrees = 0.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto result = decompose_polygon(request);

  REQUIRE(result.validity == DecompositionValidity::valid);
  REQUIRE(result.components.size() == 1U);
  REQUIRE_FALSE(result.cached);
  REQUIRE(result.components.front().normalized);
  require_ring_equal(
      result.components.front().outer,
      shiny::nfp::geom::Ring{{0.0, 0.0}, {5.0, 0.0}, {5.0, 2.0}, {0.0, 2.0}});
  REQUIRE(validate_decomposition(request.polygon, result) ==
          DecompositionValidity::valid);
}

TEST_CASE("concave decomposition produces valid convex components",
          "[decomposition][concave]") {
  const DecompositionRequest request{
      .piece_id = 42,
      .polygon = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .rotation = {.degrees = 90.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto result = decompose_polygon(request);

  REQUIRE(result.validity == DecompositionValidity::valid);
  REQUIRE(result.components.size() >= 2U);
  REQUIRE_FALSE(result.cached);
  for (const auto &component : result.components) {
    REQUIRE(component.normalized);
    REQUIRE(component.outer.size() >= 3U);
  }
  REQUIRE(validate_decomposition(request.polygon, result) ==
          DecompositionValidity::valid);
}

TEST_CASE("decomposition falls back for occupied-region roof polygon",
          "[decomposition][concave][regression]") {
  const DecompositionRequest request{
      .piece_id = 43,
      .polygon = {.outer = {{0.0, 2.0},
                            {2.0, 2.0},
                            {2.0, 0.0},
                            {5.0, 0.0},
                            {5.0, 2.0},
                            {3.0, 2.0},
                            {1.0, 4.0}}},
      .rotation = {.degrees = 0.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto result = decompose_polygon(request);

  REQUIRE(result.validity == DecompositionValidity::valid);
  REQUIRE(result.components.size() >= 2U);
  REQUIRE_FALSE(result.cached);
  REQUIRE(std::fabs(result.signed_area - 9.0) <= epsilon);
  for (const auto &component : result.components) {
    REQUIRE(component.normalized);
    REQUIRE(component.outer.size() >= 3U);
  }
  REQUIRE(validate_decomposition(request.polygon, result) ==
          DecompositionValidity::valid);
}

TEST_CASE("hole-aware decomposition returns valid convex components",
          "[decomposition][holes]") {
  const DecompositionRequest request{
      .piece_id = 51,
      .polygon = {.outer = {{0.0, 0.0}, {6.0, 0.0}, {6.0, 6.0}, {0.0, 6.0}},
                  .holes = {{{2.0, 2.0}, {2.0, 4.0}, {4.0, 4.0}, {4.0, 2.0}}}},
      .rotation = {.degrees = 0.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto result = decompose_polygon(request);
  REQUIRE(result.validity == DecompositionValidity::valid);
  REQUIRE(result.components.size() >= 4U);
  REQUIRE_FALSE(result.cached);
  REQUIRE(std::fabs(result.signed_area - 32.0) <= epsilon);
  for (const auto &component : result.components) {
    REQUIRE(component.normalized);
    REQUIRE(component.outer.size() >= 3U);
  }
  REQUIRE(validate_decomposition(request.polygon, result) ==
          DecompositionValidity::valid);
}

TEST_CASE("decomposition cache uses piece rotation keys and revisions",
          "[decomposition][cache]") {
  CacheStore<PieceRotationKey, DecompositionResult> cache_store{};
  const DecompositionRequest request{
      .piece_id = 50,
      .polygon = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .rotation = {.degrees = 180.0},
      .algorithm = DecompositionAlgorithm::cgal_approx_convex_partition,
  };

  const auto first =
      decompose_polygon(request, GeometryRevision{7}, cache_store);
  REQUIRE(cache_store.size() == 1U);
  REQUIRE_FALSE(first.cached);

  const auto second =
      decompose_polygon(request, GeometryRevision{7}, cache_store);
  REQUIRE(cache_store.size() == 1U);
  REQUIRE(second.cached);
  require_decomposition_equal(second, first);

  const auto revised =
      decompose_polygon(request, GeometryRevision{8}, cache_store);
  REQUIRE(cache_store.size() == 2U);
  REQUIRE_FALSE(revised.cached);
  require_decomposition_equal(revised, first);
}

TEST_CASE("invalid decomposition input is rejected",
          "[decomposition][invalid]") {
  const DecompositionRequest request{
      .piece_id = 52,
      .polygon = {.outer = {{0.0, 0.0}, {1.0, 0.0}}},
      .rotation = {.degrees = 0.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };

  const auto result = decompose_polygon(request);
  REQUIRE(result.validity == DecompositionValidity::invalid_topology);
  REQUIRE(result.components.empty());
}

TEST_CASE("nfp engine owns convex, nonconvex, and decomposition caches",
          "[nfp][engine][cache]") {
  NfpEngine engine{};

  const ConvexNfpRequest convex_request{
      .piece_a_id = 61,
      .piece_b_id = 62,
      .convex_a = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 2.0}, {0.0, 2.0}},
      .convex_b = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
  };
  const DecompositionRequest decomposition_request{
      .piece_id = 63,
      .polygon = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .rotation = {.degrees = 0.0},
      .algorithm = DecompositionAlgorithm::cgal_optimal_convex_partition,
  };
  const NonconvexNfpRequest nonconvex_request{
      .piece_a_id = 64,
      .piece_b_id = 65,
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
      .algorithm_revision = AlgorithmRevision{4},
  };

  const auto first_nfp = engine.compute_convex_nfp(
      convex_request, GeometryRevision{1}, GeometryRevision{2});
  const auto second_nfp = engine.compute_convex_nfp(
      convex_request, GeometryRevision{1}, GeometryRevision{2});
  REQUIRE(engine.convex_cache_size() == 1U);
  REQUIRE(second_nfp.loops.size() == first_nfp.loops.size());

  const auto first_decomposition =
      engine.decompose_polygon(decomposition_request, GeometryRevision{5});
  const auto second_decomposition =
      engine.decompose_polygon(decomposition_request, GeometryRevision{5});
  REQUIRE(engine.decomposition_cache_size() == 1U);
  REQUIRE_FALSE(first_decomposition.cached);
  REQUIRE(second_decomposition.cached);
  require_decomposition_equal(second_decomposition, first_decomposition);

  const auto first_nonconvex = engine.compute_nonconvex_graph_nfp(
      nonconvex_request, GeometryRevision{8}, GeometryRevision{9});
  const auto second_nonconvex = engine.compute_nonconvex_graph_nfp(
      nonconvex_request, GeometryRevision{8}, GeometryRevision{9});
  REQUIRE(engine.nonconvex_cache_size() == 1U);
  REQUIRE(second_nonconvex.status == first_nonconvex.status);
  REQUIRE(second_nonconvex.result.loops.size() ==
          first_nonconvex.result.loops.size());

  engine.clear_caches();
  REQUIRE(engine.convex_cache_size() == 0U);
  REQUIRE(engine.nonconvex_cache_size() == 0U);
  REQUIRE(engine.decomposition_cache_size() == 0U);
}