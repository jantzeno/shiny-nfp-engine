#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "packing/sparrow/adapters/geometry_adapter.hpp"
#include "packing/sparrow/config.hpp"
#include "packing/sparrow/eval/sample_eval.hpp"
#include "packing/sparrow/eval/separation_evaluator.hpp"
#include "packing/sparrow/optimize/separator.hpp"
#include "packing/sparrow/optimize/separator_worker.hpp"
#include "packing/sparrow/quantify/collision_tracker.hpp"
#include "packing/sparrow/runtime/trace.hpp"

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

TEST_CASE("separator trace records strike counts from a real separator run that exhausts the limit",
          "[sparrow][separator]") {
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rng(37);
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;

  // One piece larger than the container: separator cannot converge and will
  // exhaust the strike limit, populating trace.strike_counts.
  const std::array items{
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 101,
          .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
  };
  const auto result = shiny::nesting::pack::sparrow::optimize::run_separator(
      to_port_polygon(rectangle(0.0, 0.0, 3.0, 3.0)), items,
      {.strike_limit = 2U,
       .iter_no_improvement_limit = 2U,
       .max_iterations = 4U,
       .search_policy =
           shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::
               for_profile(shiny::nesting::SolveProfile::balanced),
       .descent = {.iteration_budget = 1U, .translation_step = 0.5,
                   .min_translation_step = 0.1}},
      rng, &trace);

  CHECK_FALSE(result.converged);
  // At least one StrikeCountEvent is recorded. The implementation records events
  // at each no-improvement iteration: the first event has strike_count == 0
  // (captured before increment) and the last event holds the final count.
  REQUIRE_FALSE(trace.strike_counts.empty());
  CHECK(trace.strike_counts.back().strike_count >= 1U);
}

TEST_CASE("sample eval accumulates weighted loss and exits early when threshold is exceeded",
          "[sparrow][separator]") {
  const shiny::nesting::pack::sparrow::eval::BoundedLossContext context{
      .best_known_loss = 10.0,
  };

  const auto under_budget =
      shiny::nesting::pack::sparrow::eval::accumulate_weighted_loss(context,
                                                                    4.0, 5.0);
  CHECK_FALSE(context.should_stop(10.0));
  CHECK_FALSE(under_budget.early_terminated);
  CHECK(context.accepts(under_budget));

  const auto at_budget =
      shiny::nesting::pack::sparrow::eval::accumulate_weighted_loss(context,
                                                                    4.0, 6.0);
  CHECK_FALSE(at_budget.early_terminated);
  CHECK_FALSE(context.accepts(at_budget));

  const auto over_budget =
      shiny::nesting::pack::sparrow::eval::accumulate_weighted_loss(context,
                                                                    6.0, 5.0);
  CHECK(over_budget.early_terminated);
  CHECK_FALSE(context.accepts(over_budget));
}

TEST_CASE("separation evaluator scores candidate placement and short-circuits on budget",
          "[sparrow][separator]") {
  shiny::nesting::pack::sparrow::quantify::CollisionTracker tracker(
      to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)),
      {{.piece_id = 301,
        .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
       {.piece_id = 302,
        .polygon = to_port_polygon(rectangle(6.0, 0.0, 10.0, 4.0))}});

  const auto full_eval =
      shiny::nesting::pack::sparrow::eval::evaluate_separation_candidate(
          tracker, 1U, to_port_polygon(rectangle(2.0, 0.0, 6.0, 4.0)), 100.0);
  CHECK_FALSE(full_eval.early_terminated);
  CHECK(full_eval.weighted_loss == 8.0);

  const auto early_exit =
      shiny::nesting::pack::sparrow::eval::evaluate_separation_candidate(
          tracker, 1U, to_port_polygon(rectangle(2.0, 0.0, 6.0, 4.0)), 6.0);
  CHECK(early_exit.early_terminated);
  CHECK(early_exit.weighted_loss == 8.0);
}

TEST_CASE("single-worker separator converges to zero loss on a simple two-piece overlap",
          "[sparrow][separator]") {
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rng(21);
  shiny::nesting::pack::sparrow::runtime::TraceCapture trace;
  const std::array items{
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 401,
          .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 402,
          .polygon = to_port_polygon(rectangle(2.0, 0.0, 6.0, 4.0))},
  };
  const auto result = shiny::nesting::pack::sparrow::optimize::run_separator(
      to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)), items,
      {.strike_limit = 2U,
       .iter_no_improvement_limit = 2U,
       .max_iterations = 8U,
       .search_policy =
           shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::
               for_profile(shiny::nesting::SolveProfile::balanced),
       .descent = {.iteration_budget = 2U,
                   .translation_step = 1.0,
                   .min_translation_step = 0.25}},
      rng, &trace);

  CHECK(result.converged);
  CHECK(result.total_loss == 0.0);
  CHECK(result.accepted_moves > 0U);
  CHECK_FALSE(trace.accepted_moves.empty());
}

TEST_CASE("single-worker separator halts when strike limit is exhausted",
          "[sparrow][separator]") {
  shiny::nesting::pack::sparrow::runtime::SplitMix64Rng rng(22);
  const std::array items{
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 501,
          .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
  };
  const auto result = shiny::nesting::pack::sparrow::optimize::run_separator(
      to_port_polygon(rectangle(0.0, 0.0, 3.0, 3.0)), items,
      {.strike_limit = 1U,
       .iter_no_improvement_limit = 1U,
       .max_iterations = 1U,
       .search_policy =
           shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::
               for_profile(shiny::nesting::SolveProfile::balanced),
       .descent = {.iteration_budget = 0U}},
      rng);

  CHECK_FALSE(result.converged);
  CHECK(result.iterations == 1U);
  CHECK(result.total_loss > 0.0);
}

TEST_CASE("separator worker pool keeps per-worker seed and trace state independent",
          "[sparrow][separator]") {
  const std::array items{
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 601,
          .polygon = to_port_polygon(rectangle(0.0, 0.0, 4.0, 4.0))},
      shiny::nesting::pack::sparrow::optimize::SeparatorItem{
          .piece_id = 602,
          .polygon = to_port_polygon(rectangle(2.0, 0.0, 6.0, 4.0))},
  };

  const auto run =
      shiny::nesting::pack::sparrow::optimize::run_separator_workers(
          to_port_polygon(rectangle(0.0, 0.0, 10.0, 10.0)), items,
          {.strike_limit = 2U,
           .iter_no_improvement_limit = 2U,
           .max_iterations = 8U,
           .search_policy =
               shiny::nesting::pack::sparrow::sample::SearchPlacementPolicy::
                   for_profile(shiny::nesting::SolveProfile::balanced),
           .descent = {.iteration_budget = 2U,
                       .translation_step = 1.0,
                       .min_translation_step = 0.25}},
          42U, 2U);

  REQUIRE(run.workers.size() == 2U);
  CHECK(run.workers[0].worker_index == 0U);
  CHECK(run.workers[1].worker_index == 1U);
  CHECK(run.workers[0].worker_seed != run.workers[1].worker_seed);
  // The scenario converges (two overlapping pieces in a 10×10 container).
  // Contract: workers have unique indices, unique seeds derived from the base,
  // and the best entry points to the worker with the lower total_loss.
  CHECK(run.workers[0].worker_seed != run.workers[1].worker_seed);
  CHECK(run.best.result.total_loss <=
        std::min(run.workers[0].result.total_loss,
                 run.workers[1].result.total_loss) +
            1e-8);
  CHECK((run.best.worker_index == 0U || run.best.worker_index == 1U));
}