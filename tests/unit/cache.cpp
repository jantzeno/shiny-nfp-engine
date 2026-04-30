#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "cache/cache_policy.hpp"
#include "cache/lru_cache.hpp"
#include "cache/nfp_cache.hpp"
#include "cache/penetration_depth_cache.hpp"
#include "geometry/normalize.hpp"

namespace cn = shiny::nesting::cache;

TEST_CASE("LruCache evicts least recently used entries when bounded",
          "[cache][lru]") {
  cn::LruCache<int, std::string> store(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 2});

  store.put(1, "one");
  store.put(2, "two");
  store.put(3, "three");

  REQUIRE(store.size() == 2U);
  REQUIRE(store.get(1) == nullptr);
  REQUIRE(*store.get(2) == "two");
  REQUIRE(*store.get(3) == "three");
}

TEST_CASE("LruCache promotes accessed entries before eviction",
          "[cache][lru]") {
  cn::LruCache<int, std::string> store(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 2});

  store.put(1, "one");
  store.put(2, "two");
  REQUIRE(*store.get(1) == "one");

  store.put(3, "three");

  REQUIRE(*store.get(1) == "one");
  REQUIRE(store.get(2) == nullptr);
  REQUIRE(*store.get(3) == "three");
}

TEST_CASE("LruCache shared handles outlive eviction", "[cache][lru]") {
  cn::LruCache<int, std::string> store(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 1});

  store.put(1, "one");
  auto handle = store.get(1);
  REQUIRE(handle != nullptr);

  store.put(2, "two");
  REQUIRE(store.get(1) == nullptr);
  REQUIRE(*handle == "one");
}

TEST_CASE("LruCache clear resets storage and lru state", "[cache][lru]") {
  cn::LruCache<int, std::string> store(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 2});

  store.put(1, "one");
  store.put(2, "two");
  store.clear();

  REQUIRE(store.size() == 0U);
  REQUIRE(store.get(1) == nullptr);

  store.put(3, "three");
  REQUIRE(store.size() == 1U);
  REQUIRE(*store.get(3) == "three");
}

TEST_CASE("NfpCache round-trips polygon payloads with accuracy metadata",
          "[cache][nfp]") {
  cn::NfpCache nfp_cache(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 2});
  const auto key = cn::make_nfp_cache_key(1U, 2U, 0.0, 90.0);
  const cn::NfpCacheValue value{
      .polygons = {shiny::nesting::geom::normalize_polygon(
          shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
              {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}))},
      .accuracy = cn::NfpCacheAccuracy::exact,
      .status = shiny::nesting::util::Status::ok,
  };
  nfp_cache.put(key, value);
  const auto cached = nfp_cache.get(key);
  REQUIRE(cached != nullptr);
  REQUIRE(cached->polygons.size() == 1U);
  REQUIRE(cached->accuracy == cn::NfpCacheAccuracy::exact);
  REQUIRE(cached->status == shiny::nesting::util::Status::ok);
}

TEST_CASE("NfpCache key construction normalises rotations", "[cache][nfp]") {
  const auto a = cn::make_nfp_cache_key(1U, 2U, 0.0, 90.0);
  const auto b = cn::make_nfp_cache_key(1U, 2U, 360.0, 450.0);
  REQUIRE(a == b);
}

TEST_CASE("NfpCache distinguishes exact and fallback entry kinds",
          "[cache][nfp]") {
  const auto exact =
      cn::make_nfp_cache_key(1U, 2U, 0.0, 90.0, cn::NfpCacheEntryKind::exact);
  const auto fallback = cn::make_nfp_cache_key(
      1U, 2U, 0.0, 90.0, cn::NfpCacheEntryKind::conservative_bbox_fallback);
  REQUIRE(exact != fallback);
}

TEST_CASE("PenetrationDepthCache round-trips scalar payloads",
          "[cache][penetration_depth]") {
  cn::PenetrationDepthCache pd_cache(
      {.policy = cn::CachePolicy::lru_bounded, .max_entries = 2});
  const auto key = cn::make_penetration_depth_cache_key(
      10U, shiny::nesting::geom::Point2(1.0, 2.0));
  pd_cache.put(key, 3.25);
  const auto cached = pd_cache.get(key);
  REQUIRE(cached != nullptr);
  REQUIRE(*cached == 3.25);
}
