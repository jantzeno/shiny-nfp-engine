#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "geometry/queries/normalize.hpp"
#include "internal/legacy_solve.hpp"
#include "request.hpp"
#include "result.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::CandidateStrategy;
using shiny::nesting::NestingRequest;
using shiny::nesting::OptimizerKind;
using shiny::nesting::PieceOrdering;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProductionOptimizerKind;
using shiny::nesting::SolveControl;
using shiny::nesting::StopReason;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      }));
}

auto l_shape() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          {0.0, 0.0},
          {3.0, 0.0},
          {3.0, 1.0},
          {1.0, 1.0},
          {1.0, 3.0},
          {0.0, 3.0},
      }));
}

auto triangle() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      PolygonWithHoles(Ring{
          {0.0, 0.0},
          {3.0, 0.0},
          {1.5, 2.6},
      }));
}

auto convex_pentagon() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      PolygonWithHoles(Ring{
          {0.5, 0.0},
          {2.5, 0.0},
          {3.0, 1.5},
          {1.5, 3.0},
          {0.0, 1.5},
      }));
}

auto concave_notch() -> PolygonWithHoles {
  // T-shape: rectangular body with a rectangular notch cut from the top centre.
  return shiny::nesting::geom::normalize_polygon(
      PolygonWithHoles(Ring{
          {0.0, 0.0},
          {4.0, 0.0},
          {4.0, 3.0},
          {2.5, 3.0},
          {2.5, 1.5},
          {1.5, 1.5},
          {1.5, 3.0},
          {0.0, 3.0},
      }));
}

auto concave_bin() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
          {0.0, 0.0},
          {8.0, 0.0},
          {8.0, 3.0},
          {3.0, 3.0},
          {3.0, 8.0},
          {0.0, 8.0},
      }));
}

auto count_placements(const shiny::nesting::NestingResult &result)
    -> std::size_t {
  std::size_t count = 0;
  for (const auto &bin : result.layout.bins) {
    count += bin.placements.size();
  }
  return count;
}

auto placements_per_bin(const shiny::nesting::NestingResult &result)
    -> std::map<std::uint32_t, std::size_t> {
  std::map<std::uint32_t, std::size_t> counts;
  for (const auto &bin : result.layout.bins) {
    counts[bin.bin_id] += bin.placements.size();
  }
  return counts;
}

auto solve_checked(const NestingRequest &request,
                   const SolveControl &control = {})
    -> shiny::nesting::NestingResult {
  REQUIRE(request.is_valid());
  const auto solved = shiny::nesting::solve(request, control);
  REQUIRE(solved.ok());
  return solved.value();
}

auto targeted_request() -> NestingRequest {
  NestingRequest request;
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0, 90.0}};
  request.execution.irregular.candidate_strategy =
      CandidateStrategy::nfp_hybrid;
  request.execution.irregular.piece_ordering = PieceOrdering::input;
  request.execution.irregular.max_candidate_points = 128;
  return request;
}

auto production_smoke_request() -> NestingRequest {
  auto request = targeted_request();
  request.execution.strategy = StrategyKind::metaheuristic_search;
  request.execution.default_rotations = {{0.0}};
  request.execution.production.population_size = 10;
  request.execution.production.elite_count = 2;
  request.execution.production.mutant_count = 2;
  request.execution.production.max_iterations = 4;
  request.execution.production.polishing_passes = 0;
  request.execution.production.diversification_swaps = 1;
  request.execution.production.infeasible_pool_capacity = 2;
  request.execution.production.infeasible_rollback_after = 1;
  request.bins.push_back(BinRequest{
      .bin_id = 500,
      .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 501,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 502,
      .polygon = rectangle(0.0, 0.0, 2.0, 4.0),
  });
  return request;
}

} // namespace

TEST_CASE("representative integration layouts cover core placement shapes",
          "[integration][representative-layouts]") {
  SECTION("single-bin strip nesting") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 100,
        .polygon = rectangle(0.0, 0.0, 12.0, 3.0),
    });
    request.pieces = {
        {.piece_id = 10, .polygon = rectangle(0.0, 0.0, 4.0, 3.0)},
        {.piece_id = 11, .polygon = rectangle(0.0, 0.0, 4.0, 3.0)},
        {.piece_id = 12, .polygon = rectangle(0.0, 0.0, 4.0, 3.0)},
    };

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(count_placements(result) == 3U);
    REQUIRE(result.layout.bins.size() == 1U);
  }

  SECTION("multi-bin explicit assignment") {
    auto request = targeted_request();
    request.bins = {
        {.bin_id = 201, .polygon = rectangle(0.0, 0.0, 5.0, 5.0)},
        {.bin_id = 202, .polygon = rectangle(0.0, 0.0, 5.0, 5.0)},
    };
    request.execution.selected_bin_ids = {201, 202};
    request.pieces = {
        {.piece_id = 21,
         .polygon = rectangle(0.0, 0.0, 5.0, 5.0),
         .allowed_bin_ids = {201}},
        {.piece_id = 22,
         .polygon = rectangle(0.0, 0.0, 5.0, 5.0),
         .allowed_bin_ids = {202}},
    };

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(placements_per_bin(result) ==
            std::map<std::uint32_t, std::size_t>{{201, 1U}, {202, 1U}});
  }

  SECTION("dense rectangle packing") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 300,
        .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
    });
    for (std::uint32_t id = 31; id < 35; ++id) {
      request.pieces.push_back(PieceRequest{
          .piece_id = id,
          .polygon = rectangle(0.0, 0.0, 2.0, 2.0),
      });
    }

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(result.layout.bins.front().utilization.placement_count == 4U);
  }

  SECTION("concave piece in rectangular bin") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 400,
        .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
    });
    request.pieces.push_back(
        PieceRequest{.piece_id = 41, .polygon = l_shape()});

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(result.layout.bins.front().placements.front().placement.piece_id ==
            41U);
  }

  SECTION("concave bin with concave piece") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 410,
        .polygon = concave_bin(),
    });
    request.pieces.push_back(
        PieceRequest{.piece_id = 42, .polygon = l_shape()});

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(result.layout.bins.front().bin_id == 410U);
  }

  SECTION("non-rectilinear irregular polygon set") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 500,
        .polygon = rectangle(0.0, 0.0, 14.0, 14.0),
    });
    request.pieces = {
        {.piece_id = 51, .polygon = triangle()},
        {.piece_id = 52, .polygon = convex_pentagon()},
        {.piece_id = 53, .polygon = concave_notch()},
        {.piece_id = 54, .polygon = l_shape()},
    };

    const auto result = solve_checked(request);
    REQUIRE(result.is_full_success());
    REQUIRE(result.layout_valid());
    REQUIRE(count_placements(result) == 4U);
  }

  SECTION("impossible layout records unplaced piece") {
    auto request = targeted_request();
    request.bins.push_back(BinRequest{
        .bin_id = 600,
        .polygon = rectangle(0.0, 0.0, 4.0, 4.0),
    });
    request.pieces.push_back(PieceRequest{
        .piece_id = 61,
        .polygon = rectangle(0.0, 0.0, 5.0, 5.0),
    });

    const auto result = solve_checked(request);
    REQUIRE(result.stop_reason == StopReason::completed);
    REQUIRE_FALSE(result.all_parts_placed());
    REQUIRE(result.unplaced_parts() == 1U);
    REQUIRE(result.layout.unplaced_piece_ids ==
            std::vector<std::uint32_t>{61U});
  }
}

TEST_CASE("production optimizer smoke covers readiness strategy breadth",
          "[readiness][optimizer-smoke][production]") {
  // Only brkga is accepted by production_optimizer_is_valid(); the legacy
  // kinds (simulated_annealing, alns, gdrr, lahc) are permanently rejected by
  // request.is_valid() and must not appear in a smoke that calls solve_checked.
  const std::array optimizers{
      std::pair{ProductionOptimizerKind::brkga, OptimizerKind::brkga},
  };

  for (const auto &[optimizer, expected_optimizer] : optimizers) {
    DYNAMIC_SECTION(static_cast<int>(optimizer)) {
      auto request = production_smoke_request();
      request.execution.production_optimizer = optimizer;

      const auto result = solve_checked(
          request, SolveControl{.operation_limit = 8, .random_seed = 17});
      REQUIRE(result.strategy == StrategyKind::metaheuristic_search);
      REQUIRE(result.search.optimizer == expected_optimizer);
      REQUIRE(result.layout_valid());
      REQUIRE(result.all_parts_placed());
      REQUIRE(count_placements(result) == 2U);
      REQUIRE((result.stop_reason == StopReason::completed ||
               result.stop_reason == StopReason::operation_limit_reached));
    }
  }
}
