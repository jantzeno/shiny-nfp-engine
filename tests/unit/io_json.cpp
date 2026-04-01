#include <catch2/catch_test_macros.hpp>

#include <filesystem>

#include "io/json.hpp"
#include "support/fixture_test_support.hpp"

namespace {

namespace fs = std::filesystem;

auto make_temp_path(std::string_view leaf_name) -> fs::path {
  return fs::temp_directory_path() / "shiny_nfp_engine_io_tests" /
         fs::path{leaf_name};
}

} // namespace

TEST_CASE("json polygon sets load and round trip", "[io][json]") {
  const auto fixture_path =
      shiny::nfp::test::fixture_root() / fs::path{"io/polygon_set.json"};
  const auto loaded = shiny::nfp::io::load_polygon_set(fixture_path);

  REQUIRE(loaded.ok());
  REQUIRE(loaded.value().size() == 2U);

  const auto output_path = make_temp_path("polygon_roundtrip.json");
  REQUIRE(shiny::nfp::io::save_polygon_set(output_path, loaded.value()) ==
          shiny::nfp::util::Status::ok);

  const auto roundtrip = shiny::nfp::io::load_polygon_set(output_path);
  REQUIRE(roundtrip.ok());
  REQUIRE(roundtrip.value().size() == loaded.value().size());

  for (std::size_t index = 0; index < loaded.value().size(); ++index) {
    shiny::nfp::test::require_polygon_equal(roundtrip.value()[index],
                                            loaded.value()[index]);
  }
}

TEST_CASE("json io rejects traversal and persists layout outputs",
          "[io][json]") {
  REQUIRE(
      shiny::nfp::io::load_polygon_set(fs::path{"../escape.json"}).status() ==
      shiny::nfp::util::Status::invalid_input);

  const auto layout_path = make_temp_path("layout.json");
  const auto cut_plan_path = make_temp_path("cut_plan.json");

  shiny::nfp::pack::PlacedPiece placed_piece{};
  placed_piece.placement = {
      .piece_id = 7,
      .bin_id = 1,
      .translation = {1.0, 1.0},
  };
  placed_piece.polygon = {
      .outer = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}},
  };

  shiny::nfp::pack::LayoutBin bin{};
  bin.bin_id = 1;
  bin.container = {
      .outer = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}},
  };
  bin.placements.push_back(placed_piece);

  shiny::nfp::pack::Layout layout{};
  layout.bins.push_back(bin);
  layout.unplaced_piece_ids.push_back(9);

  const shiny::nfp::pack::CutPlan cut_plan{
      .segments = {{.bin_id = 1,
                    .piece_id = 7,
                    .segment = {{0.0, 0.0}, {2.0, 0.0}}}},
      .raw_cut_length = 2.0,
      .total_cut_length = 2.0,
      .removed_cut_length = 0.0,
  };

  REQUIRE(shiny::nfp::io::save_layout(layout_path, layout) ==
          shiny::nfp::util::Status::ok);
  REQUIRE(shiny::nfp::io::save_cut_plan(cut_plan_path, cut_plan) ==
          shiny::nfp::util::Status::ok);
  REQUIRE(fs::exists(layout_path));
  REQUIRE(fs::exists(cut_plan_path));
}