#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "geometry/normalize.hpp"
#include "geometry/rotation_refinement.hpp"
#include "geometry/rtree_index.hpp"
#include "geometry/transform.hpp"
#include "request.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::geom::Box2;
using shiny::nesting::geom::DiscreteRotationSet;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::RTreeIndex;
using shiny::nesting::geom::RotationIndex;
using shiny::nesting::geom::RotationRange;

auto rectangle(double width, double height) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}},
  });
}

auto right_notch() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(PolygonWithHoles{
      .outer = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}, {2.0, 3.0},
                {2.0, 1.0}, {0.0, 1.0}},
  });
}

auto left_notch() -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      shiny::nesting::geom::mirror(right_notch()));
}

} // namespace

TEST_CASE("rotation ranges materialize evenly sampled angles",
          "[geometry][rotation-range]") {
  const DiscreteRotationSet rotations{
      .range_degrees = RotationRange{.min_degrees = 0.0,
                                     .max_degrees = 90.0,
                                     .step_degrees = 30.0},
  };

  REQUIRE(shiny::nesting::geom::materialize_rotations(rotations) ==
          std::vector<double>{0.0, 30.0, 60.0, 90.0});
  const auto resolved =
      shiny::nesting::geom::resolve_rotation(RotationIndex{2}, rotations);
  REQUIRE(resolved.has_value());
  REQUIRE(resolved->degrees == 60.0);
}

TEST_CASE("rotation refinement sampling stays discrete and explicit",
          "[geometry][rotation-range][refinement]") {
  const auto sampled = shiny::nesting::geom::sample_refinement_range(
      RotationRange{.min_degrees = 0.0, .max_degrees = 90.0, .step_degrees = 30.0});

  REQUIRE(sampled == std::vector<double>{0.0, 30.0, 60.0, 90.0});

  const auto refined = shiny::nesting::geom::local_refinement_angles(
      RotationRange{.min_degrees = 0.0, .max_degrees = 90.0, .step_degrees = 30.0},
      30.0, 2U);

  REQUIRE(refined == std::vector<double>{30.0, 37.5, 45.0, 15.0, 22.5});
}

TEST_CASE("rotation refinement rejects wrap-around candidates outside the range",
          "[geometry][rotation-range][refinement][wrap-around]") {
  // Seed at 0° with a step that lands the lower probe at -1°, which
  // normalize_angle_degrees wraps to 359°. The configured window
  // [0°, 5°] does NOT include 359°, so the wrap-around candidate
  // must be rejected (regression for the bounds-check-before-normalize
  // bug noted in rotation_refinement.hpp).
  const auto refined = shiny::nesting::geom::local_refinement_angles(
      RotationRange{.min_degrees = 0.0, .max_degrees = 5.0, .step_degrees = 2.0},
      0.0, 1U);

  REQUIRE_FALSE(refined.empty());
  REQUIRE(refined.front() == 0.0);
  for (const auto angle : refined) {
    REQUIRE(angle >= 0.0);
    REQUIRE(angle <= 5.0 + 1e-9);
  }
}

TEST_CASE("materialize_rotations bounds pathological ranges",
          "[geometry][rotation-range][bounds]") {
  // Step is technically positive but absurdly small relative to the
  // span — must clamp instead of allocating gigabytes.
  const auto pathological = shiny::nesting::geom::materialize_rotations(
      DiscreteRotationSet{.range_degrees = RotationRange{
                              .min_degrees = 0.0,
                              .max_degrees = 360.0,
                              .step_degrees = 1e-12,
                          }});
  REQUIRE(pathological.empty());
}

TEST_CASE("request normalization accepts rotation ranges",
          "[geometry][rotation-range][request]") {
  NestingRequest request;
  request.execution.default_rotations = {
      .range_degrees = RotationRange{.min_degrees = 0.0,
                                     .max_degrees = 180.0,
                                     .step_degrees = 90.0},
  };
  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = rectangle(5.0, 5.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 7,
      .polygon = rectangle(2.0, 1.0),
  });

  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  REQUIRE(shiny::nesting::geom::materialize_rotations(
              normalized.value().request.execution.default_rotations) ==
          std::vector<double>{0.0, 90.0, 180.0});
}

TEST_CASE("constructive solve refines sampled rotation ranges locally",
          "[geometry][rotation-range][refinement][solve]") {
  NestingRequest request;
  request.execution.strategy = shiny::nesting::StrategyKind::sequential_backtrack;
  request.execution.default_rotations = {
      .range_degrees = RotationRange{.min_degrees = 0.0,
                                     .max_degrees = 90.0,
                                     .step_degrees = 90.0},
  };
  request.bins.push_back(BinRequest{
      .bin_id = 2,
      .polygon = rectangle(3.0, 3.0),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 8,
      .polygon = rectangle(3.2, 0.5),
  });

  const auto result = shiny::nesting::solve(request);
  REQUIRE(result.ok());
  REQUIRE(result.value().layout.placement_trace.size() == 1U);
  const auto refined_angle =
      result.value().layout.placement_trace.front().resolved_rotation.degrees;
  REQUIRE(refined_angle != 0.0);
  REQUIRE(refined_angle != 90.0);
}

TEST_CASE("rtree index queries overlapping bounds",
          "[geometry][rtree]") {
  // TODO: add an rtree microbench (insert / query at scale) — see review
  // Phase 9 §4. Skipped here to keep the unit suite fast.
  RTreeIndex index;
  index.insert(1, Box2{.min = {.x = 0.0, .y = 0.0}, .max = {.x = 4.0, .y = 4.0}});
  index.insert(2, Box2{.min = {.x = 7.0, .y = 7.0}, .max = {.x = 9.0, .y = 9.0}});
  index.insert(3, Box2{.min = {.x = 3.0, .y = 3.0}, .max = {.x = 8.0, .y = 8.0}});

  REQUIRE(index.query(
              Box2{.min = {.x = 2.0, .y = 2.0}, .max = {.x = 6.0, .y = 6.0}}) ==
          std::vector<std::uint32_t>{1U, 3U});
}

TEST_CASE("constructive solve can use mirrored placements when enabled",
          "[geometry][mirror][solve]") {
  shiny::nesting::NestingRequest baseline_request;
  baseline_request.execution.strategy = shiny::nesting::StrategyKind::sequential_backtrack;
  baseline_request.execution.default_rotations = {{0.0}};
  baseline_request.bins.push_back(
      shiny::nesting::BinRequest{.bin_id = 1, .polygon = left_notch()});
  baseline_request.pieces.push_back(
      shiny::nesting::PieceRequest{.piece_id = 10, .polygon = right_notch()});

  const auto baseline = shiny::nesting::solve(baseline_request);
  REQUIRE(baseline.ok());
  REQUIRE(baseline.value().layout.placement_trace.empty());

  auto mirrored_request = baseline_request;
  mirrored_request.pieces.front().allow_mirror = true;

  const auto mirrored = shiny::nesting::solve(mirrored_request);
  REQUIRE(mirrored.ok());
  REQUIRE(mirrored.value().layout.placement_trace.size() == 1U);
  REQUIRE(mirrored.value().layout.bins.front().placements.front().placement.mirrored);
}
