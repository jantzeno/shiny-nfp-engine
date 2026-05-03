#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "packing/sparrow/adapters/geometry_adapter.hpp"
#include "packing/sparrow/config.hpp"
#include "packing/sparrow/quantify/collision_tracker.hpp"
#include "packing/sparrow/runtime/trace.hpp"
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

} // namespace

TEST_CASE("trace capture stores accepted moves and compression attempts and clears cleanly",
          "[sparrow][tracker]") {
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;
  trace.accepted_moves.push_back({
      .seed = 7,
      .piece_id = 101,
      .bin_id = 5,
      .objective_before = 12.0,
      .objective_after = 3.0,
  });
  trace.infeasible_pool_selections.push_back({
      .seed = 7,
      .piece_id = 101,
      .pool_size = 4,
      .rank = 1,
  });

  REQUIRE_FALSE(trace.empty());
  trace.clear();
  CHECK(trace.empty());
}

TEST_CASE("tracker parity ledger entry points to the correct catch2 target and exclusion fixtures",
          "[sparrow][tracker]") {
  const auto ledger = shiny::nesting::pack::sparrow::port_ledger();
  CHECK(std::any_of(ledger.begin(), ledger.end(), [](const auto &entry) {
    return entry.domain ==
               shiny::nesting::pack::sparrow::ParityTestDomain::tracker &&
           entry.catch2_target == "tests/unit/sparrow/sparrow_tracker.cpp";
  }));

  const auto fixtures = shiny::nesting::test::sparrow::fixture_manifest();
  CHECK(std::any_of(fixtures.begin(), fixtures.end(), [](const auto &fixture) {
    return fixture.domain ==
               shiny::nesting::pack::sparrow::ParityTestDomain::tracker &&
           fixture.exclusion_pressure;
  }));
}

TEST_CASE("collision tracker preallocates pair storage proportional to piece count",
          "[sparrow][tracker]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 8.0, 8.0)),
      {{.piece_id = 101,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
       {.piece_id = 102,
        .polygon = to_port_polygon(rectangle(2.0, 2.0, 6.0, 6.0))},
       {.piece_id = 103,
        .polygon = to_port_polygon(rectangle(6.0, 0.0, 10.0, 4.0))}});

  CHECK(tracker.item_count() == 3U);
  CHECK(tracker.pair_entry_count() == 3U);
  CHECK(tracker.pair_index(0U, 1U) == 0U);
  CHECK(tracker.pair_index(0U, 2U) == 1U);
  CHECK(tracker.pair_index(1U, 2U) == 2U);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(tracker.pair_loss(0U, 1U),
                                                    4.0));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(tracker.container_loss(2U),
                                                    8.0));
  CHECK(
      shiny::nesting::test::sparrow::nearly_equal(tracker.total_loss(), 12.0));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(
      tracker.weighted_total_loss(), 12.0));
}

TEST_CASE("collision tracker updates GLS weights deterministically on polygon move",
          "[sparrow][tracker]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)),
      {{.piece_id = 201,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
       {.piece_id = 202,
        .polygon = to_port_polygon(rectangle(2.0, 0.0, 6.0, 4.0))}});

  tracker.update_gls_weights(1.3);
  CHECK(shiny::nesting::test::sparrow::nearly_equal(tracker.pair_weight(0U, 1U),
                                                    1.3));

  tracker.register_item_polygon(1U,
                                to_port_polygon(rectangle(4.0, 0.0, 8.0, 4.0)));
  tracker.update_gls_weights(1.3);

  CHECK(shiny::nesting::test::sparrow::nearly_equal(tracker.pair_loss(0U, 1U),
                                                    0.0));
  CHECK(shiny::nesting::test::sparrow::nearly_equal(tracker.pair_weight(0U, 1U),
                                                    1.235));
}