#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "api/request_builder.hpp"
#include "geometry/operations/boolean_ops.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/queries/sanitize.hpp"
#include "geometry/queries/validity.hpp"
#include "geometry/transforms/transform.hpp"
#include "io/json.hpp"
#include "io/layout_svg.hpp"
#include "packing/layout.hpp"
#include "request.hpp"
#include "solve.hpp"
#include "util/status.hpp"

namespace {

using Catch::Approx;
using shiny::nesting::BinRequest;
using shiny::nesting::CandidateStrategy;
using shiny::nesting::IrregularOptions;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceOrdering;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionSearchConfig;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::api::NestingRequestBuilder;
using shiny::nesting::api::SolveControlBuilder;
using shiny::nesting::geom::DiscreteRotationSet;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::ResolvedRotation;
using shiny::nesting::geom::Vector2;
using shiny::nesting::util::Status;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          shiny::nesting::geom::Point2(min_x, min_y),
          shiny::nesting::geom::Point2(max_x, min_y),
          shiny::nesting::geom::Point2(max_x, max_y),
          shiny::nesting::geom::Point2(min_x, max_y),
      }));
}

auto base_request() -> NestingRequest {
  return NestingRequestBuilder{}
      .with_strategy(StrategyKind::sequential_backtrack)
      .with_default_rotations(DiscreteRotationSet{{0.0}})
      .add_bin(BinRequest{
          .bin_id = 1,
          .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
      })
      .add_piece(PieceRequest{
          .piece_id = 10,
          .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
      })
      .build();
}

auto temp_path(const std::string_view leaf) -> std::filesystem::path {
  return std::filesystem::temp_directory_path() /
         "shiny_nesting_engine_current_surface_contracts" /
         std::filesystem::path{leaf};
}

auto read_file(const std::filesystem::path &path) -> std::string {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>{input},
          std::istreambuf_iterator<char>{}};
}

auto layout_signature(const shiny::nesting::pack::Layout &layout)
    -> std::vector<std::pair<std::uint32_t, std::uint32_t>> {
  std::vector<std::pair<std::uint32_t, std::uint32_t>> signature;
  for (const auto &entry : layout.placement_trace) {
    signature.emplace_back(entry.piece_id, entry.bin_id);
  }
  return signature;
}

} // namespace

TEST_CASE("api contracts reject invalid request families",
          "[contracts][api][invalid]") {
  struct InvalidCase {
    std::string_view id;
    NestingRequest request;
    bool invalid_before_normalization{true};
  };

  auto duplicate_piece = base_request();
  duplicate_piece.pieces.push_back(duplicate_piece.pieces.front());

  auto duplicate_bin = base_request();
  duplicate_bin.bins.push_back(duplicate_bin.bins.front());

  auto zero_quantity = base_request();
  zero_quantity.pieces.front().quantity = 0;

  auto invalid_piece_polygon = base_request();
  invalid_piece_polygon.pieces.front().polygon =
      shiny::nesting::geom::PolygonWithHoles(
          shiny::nesting::geom::Ring{{0.0, 0.0}, {1.0, 0.0}});

  auto selected_bin_missing = base_request();
  selected_bin_missing.execution.selected_bin_ids = {999};

  auto impossible_rotation = base_request();
  impossible_rotation.pieces.front().allowed_rotations =
      DiscreteRotationSet{{90.0}};

  auto invalid_irregular_options = base_request();
  invalid_irregular_options.execution.strategy =
      StrategyKind::sequential_backtrack;
  invalid_irregular_options.execution.irregular.max_candidate_points = 0;

  auto empty_bins = base_request();
  empty_bins.bins.clear();

  auto empty_pieces = base_request();
  empty_pieces.pieces.clear();

  auto zero_stock = base_request();
  zero_stock.bins.front().stock = 0;

  auto invalid_exclusion = base_request();
  invalid_exclusion.bins.front().exclusion_zones.push_back(
      shiny::nesting::place::BedExclusionZone{
          .zone_id = 55,
          .region = shiny::nesting::geom::Polygon(
              shiny::nesting::geom::Ring{{0.0, 0.0}, {1.0, 0.0}}),
      });

  auto invalid_spacing = base_request();
  invalid_spacing.execution.part_spacing = -0.25;

  auto invalid_preprocess = base_request();
  invalid_preprocess.preprocess.simplify_epsilon =
      std::numeric_limits<double>::infinity();

  auto invalid_production_config = base_request();
  invalid_production_config.execution.strategy =
      StrategyKind::metaheuristic_search;
  invalid_production_config.execution.production.population_size = 2;
  invalid_production_config.execution.production.elite_count = 1;
  invalid_production_config.execution.production.mutant_count = 1;

  const std::vector<InvalidCase> cases{
      {"duplicate-piece-id", duplicate_piece},
      {"duplicate-bin-id", duplicate_bin},
      {"zero-quantity", zero_quantity},
      {"invalid-piece-polygon", invalid_piece_polygon, false},
      {"selected-bin-missing", selected_bin_missing, false},
      {"piece-rotation-outside-defaults", impossible_rotation},
      {"invalid-irregular-options", invalid_irregular_options},
      {"empty-bins", empty_bins, false},
      {"empty-pieces", empty_pieces, false},
      {"zero-stock", zero_stock},
      {"invalid-exclusion-zone", invalid_exclusion},
      {"invalid-part-spacing", invalid_spacing},
      {"invalid-preprocess", invalid_preprocess},
      {"invalid-production-config", invalid_production_config},
  };

  for (const auto &test_case : cases) {
    DYNAMIC_SECTION(test_case.id) {
      if (test_case.invalid_before_normalization) {
        REQUIRE_FALSE(test_case.request.is_valid());
      }
      REQUIRE_FALSE(shiny::nesting::normalize_request(test_case.request).ok());
      REQUIRE(shiny::nesting::normalize_request(test_case.request).status() ==
              Status::invalid_input);
    }
  }
}

TEST_CASE("api contracts accept edge-legal requests",
          "[contracts][api][positive]") {
  auto exact_fit = base_request();
  exact_fit.bins.front().polygon = rectangle(0.0, 0.0, 2.0, 2.0);
  exact_fit.pieces.front().polygon = rectangle(0.0, 0.0, 2.0, 2.0);

  auto quantity_and_pinning = base_request();
  quantity_and_pinning.bins = {
      {.bin_id = 1, .polygon = rectangle(0.0, 0.0, 10.0, 10.0)},
      {.bin_id = 2, .polygon = rectangle(0.0, 0.0, 10.0, 10.0), .stock = 2},
  };
  quantity_and_pinning.execution.selected_bin_ids = {2};
  quantity_and_pinning.pieces.front().quantity = 2;
  quantity_and_pinning.pieces.front().allowed_bin_ids = {2};
  quantity_and_pinning.pieces.front().allowed_rotations =
      DiscreteRotationSet{{0.0}};

  const std::vector<NestingRequest> cases{exact_fit, quantity_and_pinning};
  for (const auto &request : cases) {
    REQUIRE(request.is_valid());
    const auto normalized = shiny::nesting::normalize_request(request);
    REQUIRE(normalized.ok());
    const auto result = shiny::nesting::solve(
        request, SolveControlBuilder{}.with_random_seed(0).build());
    REQUIRE(result.ok());
    REQUIRE(result.value().stop_reason == StopReason::completed);
    REQUIRE(result.value().layout_valid());
    REQUIRE(result.value().all_parts_placed());
  }
}

TEST_CASE(
    "request normalization preserves revisions, selected bins, and expansion",
    "[contracts][api][normalize]") {
  auto request = base_request();
  request.execution.selected_bin_ids = {2, 2};
  request.bins = {
      {.bin_id = 1, .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
      {.bin_id = 2, .polygon = rectangle(0.0, 0.0, 6.0, 6.0), .stock = 2},
  };
  request.pieces.front().quantity = 2;
  request.pieces.front().allowed_bin_ids = {2, 2};

  const auto normalized = shiny::nesting::normalize_request(request);

  REQUIRE(normalized.ok());
  REQUIRE(normalized.value().request.execution.selected_bin_ids ==
          std::vector<std::uint32_t>{2U});
  REQUIRE(normalized.value().request.bins.size() == 1U);
  REQUIRE(normalized.value().request.bins.front().geometry_revision != 0U);
  REQUIRE(normalized.value().request.pieces.front().geometry_revision != 0U);
  REQUIRE(normalized.value().expanded_bins.size() == 2U);
  REQUIRE(normalized.value().expanded_pieces.size() == 2U);
}

TEST_CASE("geometry invariants stay stable under normalization and transforms",
          "[contracts][geometry][property]") {
  const PolygonWithHoles noisy(shiny::nesting::geom::Ring{
      {0.0, 0.0}, {0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}, {0.0, 0.0}});

  const auto sanitized = shiny::nesting::geom::sanitize_polygon(noisy);
  REQUIRE(sanitized.duplicate_vertices >= 2U);
  REQUIRE(sanitized.zero_length_edges >= 1U);
  REQUIRE(shiny::nesting::geom::validate_polygon(sanitized.polygon).is_valid());

  const auto sanitized_again =
      shiny::nesting::geom::sanitize_polygon(sanitized.polygon);
  REQUIRE(sanitized_again.duplicate_vertices == 0U);
  REQUIRE(sanitized_again.zero_length_edges == 0U);
  REQUIRE(shiny::nesting::geom::polygon_area(sanitized_again.polygon) ==
          Approx(12.0));

  const auto translated =
      shiny::nesting::geom::translate(sanitized.polygon, Vector2{7.0, -3.0});
  const auto rotated = shiny::nesting::geom::rotate(
      sanitized.polygon, ResolvedRotation{.degrees = 90.0});
  const auto mirrored = shiny::nesting::geom::mirror(sanitized.polygon);

  REQUIRE(shiny::nesting::geom::polygon_area(translated) == Approx(12.0));
  REQUIRE(shiny::nesting::geom::polygon_area(rotated) == Approx(12.0));
  REQUIRE(shiny::nesting::geom::polygon_area(mirrored) == Approx(12.0));
  REQUIRE(shiny::nesting::geom::normalize_polygon(sanitized.polygon).outer() ==
          shiny::nesting::geom::normalize_polygon(
              shiny::nesting::geom::normalize_polygon(sanitized.polygon))
              .outer());
}

TEST_CASE("boolean topology identities preserve area",
          "[contracts][geometry][topology]") {
  const auto a = rectangle(0.0, 0.0, 4.0, 4.0);
  const auto b = rectangle(2.0, 0.0, 6.0, 4.0);
  const auto far = rectangle(10.0, 10.0, 11.0, 11.0);

  const auto union_ab = shiny::nesting::geom::union_polygons(a, b);
  const auto intersection_ab =
      shiny::nesting::geom::intersection_polygons(a, b);
  const auto difference_aa = shiny::nesting::geom::difference_polygons(a, a);
  const auto intersection_far =
      shiny::nesting::geom::intersection_polygons(a, far);

  REQUIRE(shiny::nesting::geom::polygon_area_sum(union_ab) +
              shiny::nesting::geom::polygon_area_sum(intersection_ab) ==
          Approx(shiny::nesting::geom::polygon_area(a) +
                 shiny::nesting::geom::polygon_area(b)));
  REQUIRE(difference_aa.empty());
  REQUIRE(intersection_far.empty());
  REQUIRE(shiny::nesting::geom::polygon_distance(a, far) > 0.0);
}

TEST_CASE("packing and search contracts preserve bins, seeds, and validity",
          "[contracts][packing][search]") {
  auto request = base_request();
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.part_spacing = 0.0;
  request.execution.irregular = IrregularOptions{
      .candidate_strategy = CandidateStrategy::anchor_vertex,
      .max_candidate_points = 32,
      .piece_ordering = PieceOrdering::input,
      .enable_backtracking = true,
  };
  request.bins = {
      {.bin_id = 11, .polygon = rectangle(0.0, 0.0, 8.0, 4.0)},
      {.bin_id = 22, .polygon = rectangle(0.0, 0.0, 8.0, 4.0)},
  };
  request.execution.selected_bin_ids = {22};
  request.pieces = {
      {.piece_id = 101,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
       .allowed_bin_ids = {22}},
      {.piece_id = 102,
       .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
       .allowed_bin_ids = {22}},
  };

  const auto first = shiny::nesting::solve(request, SolveControlBuilder{}
                                                        .with_random_seed(17)
                                                        .with_operation_limit(4)
                                                        .build());
  const auto second =
      shiny::nesting::solve(request, SolveControlBuilder{}
                                         .with_random_seed(17)
                                         .with_operation_limit(4)
                                         .build());

  REQUIRE(first.ok());
  REQUIRE(second.ok());
  REQUIRE(first.value().effective_seed == 17U);
  REQUIRE(first.value().stop_reason == StopReason::operation_limit_reached);
  REQUIRE(first.value().layout_valid());
  REQUIRE(first.value().layout.bins.size() == 1U);
  REQUIRE(first.value().layout.bins.front().bin_id == 22U);
  REQUIRE(first.value().layout.bins.front().placements.size() == 2U);
  REQUIRE(first.value().layout.unplaced_piece_ids.empty());
  REQUIRE(layout_signature(first.value().layout) ==
          layout_signature(second.value().layout));
}

TEST_CASE(
    "partial placement contracts report unplaced pieces deterministically",
    "[contracts][packing][negative]") {
  auto request = base_request();
  request.execution.strategy = StrategyKind::sequential_backtrack;
  request.execution.default_rotations = DiscreteRotationSet{{0.0}};
  request.bins.front().polygon = rectangle(0.0, 0.0, 3.0, 3.0);
  request.pieces = {
      {.piece_id = 10, .polygon = rectangle(0.0, 0.0, 3.0, 3.0)},
      {.piece_id = 11, .polygon = rectangle(0.0, 0.0, 4.0, 4.0)},
  };

  const auto result = shiny::nesting::solve(request);

  REQUIRE(result.ok());
  REQUIRE(result.value().stop_reason == StopReason::completed);
  REQUIRE(result.value().layout_valid());
  REQUIRE_FALSE(result.value().all_parts_placed());
  REQUIRE(result.value().layout.bins.size() == 1U);
  REQUIRE(result.value().layout.bins.front().placements.size() == 1U);
  REQUIRE(result.value().layout.unplaced_piece_ids ==
          std::vector<std::uint32_t>{11U});
}

TEST_CASE("IO artifacts expose deterministic IDs and reject unsafe paths",
          "[contracts][io][artifacts]") {
  const auto solved = shiny::nesting::solve(base_request());
  REQUIRE(solved.ok());

  const auto layout_path = temp_path("layout.json");
  const auto svg_path = temp_path("layout.svg");
  REQUIRE(shiny::nesting::io::save_layout(layout_path, solved.value().layout) ==
          Status::ok);
  REQUIRE(shiny::nesting::io::save_layout_svg(
              svg_path, solved.value().layout) == Status::ok);
  REQUIRE(shiny::nesting::io::save_layout_svg(temp_path("layout.txt"),
                                              solved.value().layout) ==
          Status::invalid_input);

  const auto json = read_file(layout_path);
  const auto svg = read_file(svg_path);
  REQUIRE(json.find("\"kind\": \"layout\"") != std::string::npos);
  REQUIRE(json.find("\"placement_count\"") != std::string::npos);
  REQUIRE(json.find("\"unplaced_piece_ids\"") != std::string::npos);
  REQUIRE(json.find("\"bin_id\"") != std::string::npos);
  REQUIRE(json.find("\"piece_id\"") != std::string::npos);
  REQUIRE(svg.find("<svg xmlns=\"http://www.w3.org/2000/svg\"") !=
          std::string::npos);
  REQUIRE(svg.find("viewBox=\"") != std::string::npos);
  REQUIRE(svg.find("id=\"bin-1\"") != std::string::npos);
  REQUIRE(svg.find("id=\"piece-10\"") != std::string::npos);
  REQUIRE(svg.find("data-bin-id=\"1\"") != std::string::npos);
}
