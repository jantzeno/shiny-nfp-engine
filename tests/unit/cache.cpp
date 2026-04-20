#include <catch2/catch_test_macros.hpp>

#include <string>

#include "cache/cache_policy.hpp"
#include "cache/stores.hpp"

TEST_CASE("cache store evicts least recently used entries when bounded",
          "[cache][lru]") {
  shiny::nesting::cache::CacheStore<int, std::string> cache_store(
      {.policy = shiny::nesting::cache::CachePolicy::lru_bounded,
       .max_entries = 2});

  cache_store.put(1, "one");
  cache_store.put(2, "two");
  cache_store.put(3, "three");

  REQUIRE(cache_store.size() == 2U);
  REQUIRE(cache_store.get(1) == nullptr);
  REQUIRE(*cache_store.get(2) == "two");
  REQUIRE(*cache_store.get(3) == "three");
}

TEST_CASE("cache store promotes accessed entries before eviction",
          "[cache][lru]") {
  shiny::nesting::cache::CacheStore<int, std::string> cache_store(
      {.policy = shiny::nesting::cache::CachePolicy::lru_bounded,
       .max_entries = 2});

  cache_store.put(1, "one");
  cache_store.put(2, "two");
  REQUIRE(*cache_store.get(1) == "one");

  cache_store.put(3, "three");

  REQUIRE(*cache_store.get(1) == "one");
  REQUIRE(cache_store.get(2) == nullptr);
  REQUIRE(*cache_store.get(3) == "three");
}

TEST_CASE("cache store clear resets storage and lru state", "[cache][lru]") {
  shiny::nesting::cache::CacheStore<int, std::string> cache_store(
      {.policy = shiny::nesting::cache::CachePolicy::lru_bounded,
       .max_entries = 2});

  cache_store.put(1, "one");
  cache_store.put(2, "two");
  cache_store.clear();

  REQUIRE(cache_store.size() == 0U);
  REQUIRE(cache_store.get(1) == nullptr);

  cache_store.put(3, "three");
  REQUIRE(cache_store.size() == 1U);
  REQUIRE(*cache_store.get(3) == "three");
}