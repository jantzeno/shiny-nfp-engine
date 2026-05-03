#include <catch2/catch_test_macros.hpp>

#include "io/or_dataset_json.hpp"
#include "support/fixture_test_support.hpp"

TEST_CASE("OR-Datasets strip schema loads", "[io][or-datasets]") {
  const auto dataset = shiny::nesting::io::load_or_dataset(
      shiny::nesting::test::fixture_root() / "or_datasets/strip_dataset.json");

  REQUIRE(dataset.ok());
  REQUIRE(dataset.value().name == "fixture_strip");
  REQUIRE(dataset.value().uses_strip_container());
  REQUIRE_FALSE(dataset.value().uses_explicit_bins());
  REQUIRE(dataset.value().items.size() == 2);
  REQUIRE(dataset.value().items.front().allowed_orientations ==
          std::vector<double>{0.0, 180.0});
}

TEST_CASE("OR-Datasets explicit-bin schema loads", "[io][or-datasets]") {
  const auto dataset = shiny::nesting::io::load_or_dataset(
      shiny::nesting::test::fixture_root() /
      "or_datasets/explicit_bins_dataset.json");

  REQUIRE(dataset.ok());
  REQUIRE(dataset.value().name == "fixture_bins");
  REQUIRE_FALSE(dataset.value().uses_strip_container());
  REQUIRE(dataset.value().uses_explicit_bins());
  REQUIRE(dataset.value().bins.size() == 2);
  REQUIRE(dataset.value().bins.front().zones.size() == 1);
  REQUIRE(dataset.value().bins.front().polygon.holes().size() == 1);
}
