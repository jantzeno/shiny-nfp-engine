#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "api/request_builder.hpp"
#include "packing/sparrow/adapters/request_adapter.hpp"
#include "packing/sparrow/multi_bin.hpp"
#include "packing/sparrow/optimize/objective.hpp"
#include "packing/sparrow/search/solution_pool.hpp"
#include "solve.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::ObjectiveMode;
using shiny::nesting::PieceRequest;
using shiny::nesting::ProfileRequest;
using shiny::nesting::ProfileSolveControl;
using shiny::nesting::SolveProfile;
using shiny::nesting::api::ProfileRequestBuilder;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::sparrow::KnapsackMoveClass;

auto rectangle(double width, double height) -> PolygonWithHoles {
  return PolygonWithHoles(
      Ring{{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}});
}

auto make_knapsack_request() -> ProfileRequest {
  return ProfileRequestBuilder{}
      .with_profile(SolveProfile::maximum_search)
      .with_objective_mode(ObjectiveMode::maximize_value)
      .with_time_limit_ms(1'000U)
      .with_allow_part_overflow(false)
      .add_bin(BinRequest{
          .bin_id = 1,
          .polygon = rectangle(10.0, 4.0),
      })
      .add_piece(PieceRequest{
          .piece_id = 10,
          .polygon = rectangle(5.0, 4.0),
          .value = 1.0,
      })
      .add_piece(PieceRequest{
          .piece_id = 11,
          .polygon = rectangle(5.0, 4.0),
          .value = 1.0,
      })
      .add_piece(PieceRequest{
          .piece_id = 12,
          .polygon = rectangle(10.0, 4.0),
          .value = 5.0,
      })
      .build();
}

} // namespace

TEST_CASE("knapsack request adapter maps piece values into the Sparrow instance",
          "[sparrow][integration][knapsack]") {
  const auto adapted_or =
      shiny::nesting::pack::sparrow::adapters::adapt_request(
          make_knapsack_request(), ProfileSolveControl{.random_seed = 7U});
  REQUIRE(adapted_or.ok());

  const auto &instance = adapted_or.value().instance;
  CHECK(instance.objective_mode == ObjectiveMode::maximize_value);
  REQUIRE(instance.pieces.size() == 3U);
  CHECK(instance.pieces[0].value == 1.0);
  CHECK(instance.pieces[1].value == 1.0);
  CHECK(instance.pieces[2].value == 5.0);
}

TEST_CASE("knapsack objective ranks higher-value legal subsets ahead of lower-value ones",
          "[sparrow][integration][knapsack]") {
  shiny::nesting::search::SolutionPoolEntry low_value;
  low_value.metrics = {
      .placed_parts = 2U,
      .placed_value = 2.0,
      .bin_count = 1U,
      .strip_length = 10.0,
      .utilization = 1.0,
      .objective_mode = ObjectiveMode::maximize_value,
  };

  shiny::nesting::search::SolutionPoolEntry high_value;
  high_value.metrics = {
      .placed_parts = 1U,
      .placed_value = 5.0,
      .bin_count = 1U,
      .strip_length = 10.0,
      .utilization = 1.0,
      .objective_mode = ObjectiveMode::maximize_value,
  };

  CHECK(shiny::nesting::search::better_metrics(high_value.metrics,
                                               low_value.metrics));
  CHECK(shiny::nesting::pack::sparrow::optimize::compare_objective(
            shiny::nesting::pack::sparrow::optimize::evaluate_objective(
                high_value),
            shiny::nesting::pack::sparrow::optimize::evaluate_objective(
                low_value)) ==
        shiny::nesting::pack::sparrow::optimize::ObjectiveOrdering::better);
}

TEST_CASE("knapsack move legality enumerates value-swap inject and eject classes under assignment constraints",
          "[sparrow][integration][knapsack]") {
  const auto adapted_or =
      shiny::nesting::pack::sparrow::adapters::adapt_request(
          make_knapsack_request(), ProfileSolveControl{.random_seed = 11U});
  REQUIRE(adapted_or.ok());

  const auto &instance = adapted_or.value().instance;
  const auto low_piece_id = instance.pieces.front().instance.expanded_piece_id;
  const auto high_piece_id = instance.pieces.back().instance.expanded_piece_id;
  const auto bin_id = instance.bins.front().instance.expanded_bin_id;

  const auto free_moves =
      shiny::nesting::pack::sparrow::legal_knapsack_move_classes(
          instance,
          {.active_bin_ids = {bin_id}, .unplaced_piece_ids = {high_piece_id}},
          low_piece_id, bin_id);
  CHECK(std::find(free_moves.begin(), free_moves.end(),
                  KnapsackMoveClass::value_swap) != free_moves.end());
  CHECK(std::find(free_moves.begin(), free_moves.end(),
                  KnapsackMoveClass::inject_unplaced) != free_moves.end());
  CHECK(std::find(free_moves.begin(), free_moves.end(),
                  KnapsackMoveClass::eject_low_value) != free_moves.end());

  auto pinned_request = make_knapsack_request();
  pinned_request.maintain_bed_assignment = true;
  pinned_request.pieces.front().assigned_bin_id = 1U;
  pinned_request.pieces[1].allowed_bin_ids = {1U};
  pinned_request.pieces[2].allowed_bin_ids = {1U};
  const auto pinned_or = shiny::nesting::pack::sparrow::adapters::adapt_request(
      pinned_request, ProfileSolveControl{.random_seed = 11U});
  REQUIRE(pinned_or.ok());
  const auto pinned_piece_id =
      pinned_or.value().instance.pieces.front().instance.expanded_piece_id;
  const auto pinned_bin_id =
      pinned_or.value().instance.bins.front().instance.expanded_bin_id;

  const auto pinned_moves =
      shiny::nesting::pack::sparrow::legal_knapsack_move_classes(
          pinned_or.value().instance,
          {.active_bin_ids = {pinned_bin_id},
           .unplaced_piece_ids = {pinned_or.value()
                                      .instance.pieces.back()
                                      .instance.expanded_piece_id}},
          pinned_piece_id, pinned_bin_id);
  CHECK(std::find(pinned_moves.begin(), pinned_moves.end(),
                  KnapsackMoveClass::value_swap) != pinned_moves.end());
  CHECK(std::find(pinned_moves.begin(), pinned_moves.end(),
                  KnapsackMoveClass::inject_unplaced) == pinned_moves.end());
  CHECK(std::find(pinned_moves.begin(), pinned_moves.end(),
                  KnapsackMoveClass::eject_low_value) == pinned_moves.end());
}

TEST_CASE("maximum-search knapsack solve places the highest-value subset when not all pieces fit",
          "[sparrow][integration][knapsack]") {
  const auto solved = shiny::nesting::solve(
      make_knapsack_request(), ProfileSolveControl{.random_seed = 19U});
  REQUIRE(solved.ok());
  REQUIRE(solved.value().validation.valid);
  REQUIRE(solved.value().layout.placement_trace.size() == 1U);
  CHECK(solved.value().layout.placement_trace.front().piece_id == 12U);
  CHECK(solved.value().layout.unplaced_piece_ids.size() == 2U);
}