#include <catch2/catch_test_macros.hpp>

#include <chrono>
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
  shiny::nesting::runtime::TimeBudget budget(5);

  std::this_thread::sleep_for(std::chrono::milliseconds(15));

  REQUIRE(budget.enabled());
  REQUIRE(budget.expired(stopwatch));
}

TEST_CASE("timing budget reports remaining time without hidden caps",
          "[runtime][timing]") {
  shiny::nesting::runtime::Stopwatch stopwatch;
  shiny::nesting::runtime::TimeBudget unlimited;
  shiny::nesting::runtime::TimeBudget budget(10);

  REQUIRE_FALSE(unlimited.enabled());
  REQUIRE(unlimited.remaining_milliseconds(stopwatch) == 0U);
  REQUIRE(budget.remaining_milliseconds(stopwatch) <= 10U);
  REQUIRE(budget.remaining_milliseconds(stopwatch) > 0U);

  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  REQUIRE(budget.remaining_milliseconds(stopwatch) == 0U);
  REQUIRE(budget.expired(stopwatch));
}

TEST_CASE("cancellation probe and progress snapshots carry shared execution state",
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
  snapshot.stop_reason = shiny::nesting::StopReason::time_limit_reached;
  snapshot.improved = true;

  REQUIRE(snapshot.sequence == 4);
  REQUIRE(snapshot.placements_successful == 3);
  REQUIRE(snapshot.total_requested_parts == 7);
  REQUIRE(snapshot.stop_reason ==
          shiny::nesting::StopReason::time_limit_reached);
  REQUIRE(snapshot.improved);
}

TEST_CASE("cancellation fired before token creation is immediately detectable",
          "[runtime][cancellation]") {
  shiny::nesting::runtime::CancellationSource source;
  source.request_stop();

  const auto token = source.token();
  const auto probe = shiny::nesting::runtime::make_interruption_probe(token);

  REQUIRE(token.stop_requested());
  REQUIRE(probe());
}

TEST_CASE("two CancellationSource instances are independent",
          "[runtime][cancellation]") {
  shiny::nesting::runtime::CancellationSource source_a;
  shiny::nesting::runtime::CancellationSource source_b;

  const auto token_a = source_a.token();
  const auto token_b = source_b.token();

  source_a.request_stop();

  REQUIRE(token_a.stop_requested());
  REQUIRE_FALSE(token_b.stop_requested());
}
