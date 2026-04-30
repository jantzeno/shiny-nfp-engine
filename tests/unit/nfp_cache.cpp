#include <catch2/catch_test_macros.hpp>

#include "cache/stores.hpp"
#include "nfp/cache_keys.hpp"
#include "nfp/convex_ifp.hpp"
#include "nfp/convex_nfp.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::compute_convex_ifp;
using shiny::nesting::compute_convex_nfp;
using shiny::nesting::ConvexIfpRequest;
using shiny::nesting::ConvexNfpRequest;
using shiny::nesting::NfpResult;
using shiny::nesting::cache::CacheStore;
using shiny::nesting::cache::GeometryRevision;
using shiny::nesting::cache::make_pair_rotation_key;
using shiny::nesting::cache::PairRotationKey;
using shiny::nesting::test::require_point_equal;
using shiny::nesting::test::require_ring_equal;

void require_result_equal(const NfpResult &actual, const NfpResult &expected) {
  REQUIRE(actual.algorithm == expected.algorithm);
  REQUIRE(actual.normalized == expected.normalized);
  REQUIRE(actual.loops.size() == expected.loops.size());
  REQUIRE(actual.perfect_fit_points.size() ==
          expected.perfect_fit_points.size());
  REQUIRE(actual.perfect_sliding_segments.size() ==
          expected.perfect_sliding_segments.size());

  for (std::size_t index = 0; index < actual.loops.size(); ++index) {
    REQUIRE(actual.loops[index].kind == expected.loops[index].kind);
    require_ring_equal(actual.loops[index].vertices,
                       expected.loops[index].vertices);
  }

  for (std::size_t index = 0; index < actual.perfect_fit_points.size();
       ++index) {
    require_point_equal(actual.perfect_fit_points[index],
                        expected.perfect_fit_points[index]);
  }

  for (std::size_t index = 0; index < actual.perfect_sliding_segments.size();
       ++index) {
    require_point_equal(actual.perfect_sliding_segments[index].start,
                        expected.perfect_sliding_segments[index].start);
    require_point_equal(actual.perfect_sliding_segments[index].end,
                        expected.perfect_sliding_segments[index].end);
  }
}

} // namespace

TEST_CASE("convex nfp cached overload reuses pair rotation entries",
          "[nfp][cache][convex]") {
  CacheStore<PairRotationKey, NfpResult> cache_store{};
  const ConvexNfpRequest request{
      .piece_a_id = 21,
      .piece_b_id = 22,
      .convex_a = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {0.0, 1.0}},
      .convex_b = {{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
  };

  const auto first = compute_convex_nfp(request, GeometryRevision{1},
                                        GeometryRevision{2}, cache_store);
  REQUIRE(cache_store.size() == 1U);

  const auto key =
      make_pair_rotation_key(request, GeometryRevision{1}, GeometryRevision{2});
  const auto *cached = cache_store.get(key);
  REQUIRE(cached != nullptr);
  require_result_equal(*cached, first);

  const auto second = compute_convex_nfp(request, GeometryRevision{1},
                                         GeometryRevision{2}, cache_store);
  REQUIRE(cache_store.size() == 1U);
  require_result_equal(second, first);

  const auto revised = compute_convex_nfp(request, GeometryRevision{3},
                                          GeometryRevision{2}, cache_store);
  REQUIRE(cache_store.size() == 2U);
  require_result_equal(revised, first);
}

TEST_CASE("convex ifp cached overload keeps inside queries distinct",
          "[nfp][cache][ifp]") {
  CacheStore<PairRotationKey, NfpResult> cache_store{};

  const ConvexNfpRequest nfp_request{
      .piece_a_id = 31,
      .piece_b_id = 32,
      .convex_a = {{0.0, 0.0}, {6.0, 0.0}, {6.0, 5.0}, {0.0, 5.0}},
      .convex_b = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {0.0, 1.0}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 0.0},
  };
  const auto nfp_result = compute_convex_nfp(nfp_request, GeometryRevision{7},
                                             GeometryRevision{9}, cache_store);
  REQUIRE(cache_store.size() == 1U);

  const ConvexIfpRequest ifp_request{
      .container_id = 31,
      .piece_id = 32,
      .container =
          shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
              {0.0, 0.0}, {6.0, 0.0}, {6.0, 5.0}, {0.0, 5.0}}),
      .piece =
          shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
              {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {0.0, 1.0}}),
      .container_rotation = {.degrees = 0.0},
      .piece_rotation = {.degrees = 0.0},
  };
  const auto ifp_result = compute_convex_ifp(ifp_request, GeometryRevision{7},
                                             GeometryRevision{9}, cache_store);
  REQUIRE(cache_store.size() == 2U);

  const auto nfp_key = make_pair_rotation_key(nfp_request, GeometryRevision{7},
                                              GeometryRevision{9});
  const auto ifp_key = make_pair_rotation_key(ifp_request, GeometryRevision{7},
                                              GeometryRevision{9});
  REQUIRE_FALSE(nfp_key == ifp_key);
  REQUIRE_FALSE(nfp_key.inside);
  REQUIRE(ifp_key.inside);

  const auto *cached_nfp = cache_store.get(nfp_key);
  const auto *cached_ifp = cache_store.get(ifp_key);
  REQUIRE(cached_nfp != nullptr);
  REQUIRE(cached_ifp != nullptr);
  require_result_equal(*cached_nfp, nfp_result);
  require_result_equal(*cached_ifp, ifp_result);
}