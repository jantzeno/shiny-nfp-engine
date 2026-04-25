#include <catch2/catch_test_macros.hpp>

#include <thread>

#include "observer.hpp"
#include "runtime/cancellation.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"

TEST_CASE("deterministic RNG is repeatable", "[runtime][rng]") {
  shiny::nesting::runtime::DeterministicRng first(1234);
  shiny::nesting::runtime::DeterministicRng second(1234);

  REQUIRE(first.next_u64() == second.next_u64());
  REQUIRE(first.uniform_index(10) == second.uniform_index(10));
}

TEST_CASE("timing budget reports expiry", "[runtime][timing]") {
  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget budget(1);

  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  REQUIRE(budget.enabled());
  REQUIRE(budget.expired(stopwatch));
}

TEST_CASE(
    "cancellation probe and progress snapshots carry shared execution state",
    "[runtime][observer]") {
  shiny::nesting::runtime::CancellationSource source;
  const auto token = source.token();
  const auto probe = shiny::nesting::runtime::make_interruption_probe(token);

  REQUIRE_FALSE(token.stop_requested());
  REQUIRE_FALSE(probe());
  source.request_stop();
  REQUIRE(token.stop_requested());
  REQUIRE(probe());

  shiny::nesting::ProgressSnapshot snapshot;
  snapshot.sequence = 4;
  snapshot.placements_successful = 3;
  snapshot.total_requested_parts = 7;
  snapshot.budget.operation_limit_enabled = true;
  snapshot.budget.operation_limit = 25;
  snapshot.budget.operations_completed = 12;
  snapshot.budget.time_limit_enabled = true;
  snapshot.budget.time_limit_milliseconds = 10'000;
  snapshot.budget.elapsed_milliseconds = 3'500;
  snapshot.budget.cancellation_requested = true;
  snapshot.stop_reason = shiny::nesting::StopReason::time_limit_reached;

  REQUIRE(snapshot.sequence == 4);
  REQUIRE(snapshot.placements_successful == 3);
  REQUIRE(snapshot.total_requested_parts == 7);
  REQUIRE(snapshot.budget.operation_limit == 25);
  REQUIRE(snapshot.budget.elapsed_milliseconds == 3'500);
  REQUIRE(snapshot.stop_reason ==
          shiny::nesting::StopReason::time_limit_reached);
}
