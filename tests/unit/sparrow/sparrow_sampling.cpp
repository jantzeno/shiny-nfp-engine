#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>

#include "packing/bin_identity.hpp"
#include "packing/sparrow/adapters/layout_adapter.hpp"
#include "packing/sparrow/config.hpp"
#include "packing/sparrow/quantify/collision_tracker.hpp"
#include "packing/sparrow/runtime/rng.hpp"
#include "packing/sparrow/sample/coordinate_descent.hpp"
#include "packing/sparrow/sample/search_placement.hpp"
#include "packing/sparrow/sample/uniform_sampler.hpp"
#include "support/sparrow_harness.hpp"

namespace {

using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::sparrow::adapters::to_port_polygon;

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}});
}

auto make_seed_solution() -> shiny::nesting::pack::sparrow::SeedSolution {
  shiny::nesting::pack::Layout layout;
  layout.bins.push_back({
      .bin_id = 1,
      .identity =
          {
              .lifecycle = shiny::nesting::pack::BinLifecycle::user_created,
              .source_request_bin_id = 1,
          },
      .utilization =
          {
              .bin_id = 1,
              .placement_count = 1,
              .occupied_area = 8.0,
              .container_area = 100.0,
              .utilization = 0.08,
          },
  });
  layout.placement_trace.push_back({
      .piece_id = 2,
      .bin_id = 1,
      .resolved_rotation = shiny::nesting::geom::ResolvedRotation{0.0},
      .translation = shiny::nesting::geom::Point2{6.0, 0.0},
      .phase = shiny::nesting::pack::ConstructivePlacementPhase::primary_order,
  });
  return shiny::nesting::pack::sparrow::adapters::to_seed_solution(layout);
}

} // namespace

TEST_CASE("sparrow RNG produces identical sequences from the same seed",
          "[sparrow][sampling]") {
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng lhs(42);
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rhs(42);

  CHECK(lhs.next_u64() == rhs.next_u64());
  CHECK(lhs.next_u64() == rhs.next_u64());
  CHECK(shiny::nesting::pack::sparrow::runtime::derive_worker_seed(42, 0) !=
        shiny::nesting::pack::sparrow::runtime::derive_worker_seed(42, 1));

  const auto control = shiny::nesting::test::sparrow::make_profile_control(99);
  const auto seed_flow = shiny::nesting::pack::sparrow::build_seed_flow_plan(
      control, shiny::nesting::SolveProfile::balanced);
  CHECK(seed_flow.public_seed == 99U);
  CHECK(seed_flow.constructive_seed != seed_flow.worker_seed_base);
}

TEST_CASE("sampler parity ledger entry points to the correct catch2 target and fixture families",
          "[sparrow][sampling]") {
  const auto ledger = shiny::nesting::pack::sparrow::port_ledger();
  CHECK(std::any_of(ledger.begin(), ledger.end(), [](const auto &entry) {
    return entry.domain ==
               shiny::nesting::pack::sparrow::ParityTestDomain::sampler &&
           entry.catch2_target == "tests/unit/sparrow/sparrow_sampling.cpp";
  }));

  const auto fixtures = shiny::nesting::test::sparrow::fixture_manifest();
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.domain ==
               shiny::nesting::pack::sparrow::ParityTestDomain::sampler &&
           fixture.holes;
  }));
}

TEST_CASE("uniform sampler produces bounded samples deterministically on a fixed seed",
          "[sparrow][sampling]") {
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng lhs(77);
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rhs(77);
  constexpr std::array rotations{0.0, 90.0};
  const shiny::nesting::geom::Box2 bounds{{-1.0, 2.0}, {3.0, 6.0}};

  const auto lhs_samples =
      shiny::nesting::pack::sparrow::sample::sample_uniform_placements(
          lhs, bounds, 4U, rotations);
  const auto rhs_samples =
      shiny::nesting::pack::sparrow::sample::sample_uniform_placements(
          rhs, bounds, 4U, rotations);

  REQUIRE(lhs_samples.size() == 4U);
  REQUIRE(lhs_samples.size() == rhs_samples.size());
  for (std::size_t index = 0; index < lhs_samples.size(); ++index) {
    CHECK(shiny::nesting::test::sparrow::nearly_equal(
        lhs_samples[index].translation.x(),
        rhs_samples[index].translation.x()));
    CHECK(shiny::nesting::test::sparrow::nearly_equal(
        lhs_samples[index].translation.y(),
        rhs_samples[index].translation.y()));
    CHECK(lhs_samples[index].rotation_degrees ==
          rhs_samples[index].rotation_degrees);
    CHECK(lhs_samples[index].translation.x() >= bounds.min.x());
    CHECK(lhs_samples[index].translation.x() <= bounds.max.x());
    CHECK(lhs_samples[index].translation.y() >= bounds.min.y());
    CHECK(lhs_samples[index].translation.y() <= bounds.max.y());
  }
}

TEST_CASE("search-placement sample counts scale with the configured solve profile",
          "[sparrow][sampling]") {
  const auto balanced =
      shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::for_profile(
          shiny::nesting::SolveProfile::balanced);
  const auto maximum =
      shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::for_profile(
          shiny::nesting::SolveProfile::maximum_search);

  CHECK(balanced.global_samples == 8U);
  CHECK(balanced.focused_samples == 4U);
  CHECK(maximum.global_samples == 16U);
  CHECK(maximum.focused_samples == 8U);
}

TEST_CASE("search-placement selects the constructive seed position when it has zero loss",
          "[sparrow][sampling]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)),
      {{.piece_id = 1,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
       {.piece_id = 2,
        .polygon = to_port_polygon(rectangle(2.0, 0.0, 6.0, 2.0))}});
  const auto seed_solution = make_seed_solution();
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rng(9);
  constexpr std::array rotations{0.0, 90.0};

  const auto result = shiny::nesting::pack::sparrow::sample::search_placement(
      {
          .tracker = &tracker,
          .moving_index = 1U,
          .seed_solution = &seed_solution,
          .allowed_rotations_degrees = rotations,
          .policy =
              shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::
                  for_profile(shiny::nesting::SolveProfile::balanced),
      },
      rng);

  REQUIRE(result.has_value());
  CHECK(result->sampled_candidates == 8U);
  CHECK(result->focused_candidates == 4U);
  CHECK(result->best_candidate.from_constructive_seed);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      result->best_candidate.translation.x(), 6.0));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      result->best_candidate.translation.y(), 0.0));
  CHECK(result->best_candidate.rotation_degrees == 0.0);
  CHECK(result->best_candidate.weighted_loss == 0.0);
}

TEST_CASE("coordinate descent retains the best sample when no refinement improves overlap",
          "[sparrow][sampling]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)),
      {{.piece_id = 1,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
       {.piece_id = 2,
        .polygon = to_port_polygon(rectangle(6.0, 0.0, 10.0, 2.0))}});

  const auto refined = shiny::nesting::pack::sparrow::sample::refine_placement(
      tracker, 1U,
      {.polygon = rectangle(6.0, 0.0, 10.0, 2.0),
       .translation = shiny::nesting::geom::Point2{6.0, 0.0},
       .rotation_degrees = 0.0,
       .weighted_loss = 0.0,
       .from_constructive_seed = true},
      {.iteration_budget = 4U,
       .translation_step = 1.0,
       .min_translation_step = 0.2,
       .angle_step_degrees = 90.0,
       .min_angle_step_degrees = 45.0,
       .enable_rotation_axis = true});

  CHECK(refined.best_candidate.from_constructive_seed);
  CHECK(refined.best_candidate.weighted_loss == 0.0);
}

TEST_CASE("coordinate descent reduces weighted loss within the configured iteration budget",
          "[sparrow][sampling]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0)),
      {{.piece_id = 1,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 2.0, 4.0))},
       {.piece_id = 2,
        .polygon = to_port_polygon(rectangle(1.0, 1.0, 4.0, 3.0))}});

  const auto refined = shiny::nesting::pack::sparrow::sample::refine_placement(
      tracker, 1U,
      {.polygon = rectangle(1.0, 1.0, 4.0, 3.0),
       .translation = shiny::nesting::geom::Point2{1.0, 1.0},
       .rotation_degrees = 0.0,
       .weighted_loss = 2.0,
       .from_constructive_seed = false},
      {.iteration_budget = 1U,
       .translation_step = 0.5,
       .min_translation_step = 0.1,
       .angle_step_degrees = 90.0,
       .min_angle_step_degrees = 45.0,
       .enable_rotation_axis = true});

  CHECK(refined.iterations_completed == 1U);
  CHECK(refined.stopped_by_iteration_budget);
  CHECK(refined.best_candidate.weighted_loss < 2.0);
}