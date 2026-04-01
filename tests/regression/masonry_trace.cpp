#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "algorithm_kind.hpp"
#include "search/masonry.hpp"

namespace {

using shiny::nfp::AlgorithmKind;
using shiny::nfp::search::MasonryRunner;
using shiny::nfp::search::MasonryRunRequest;
using shiny::nfp::search::search_event_algorithm_kind;
using shiny::nfp::search::search_event_kind;
using shiny::nfp::search::SearchEvent;
using shiny::nfp::search::SearchEventKind;

} // namespace

TEST_CASE("masonry observer events stay aligned with retained progress",
          "[regression][packing][masonry][observer]") {
  MasonryRunner runner;
  MasonryRunRequest request{};
  request.masonry_request.decoder_request = {
      .bin = {.base_bin_id = 70,
              .polygon = {.outer = {{0.0, 0.0},
                                    {10.0, 0.0},
                                    {10.0, 10.0},
                                    {0.0, 10.0}}},
              .geometry_revision = 700},
      .pieces =
          {
              {.piece_id = 1,
               .polygon =
                   {.outer = {{0.0, 0.0}, {6.0, 0.0}, {6.0, 4.0}, {0.0, 4.0}}},
               .geometry_revision = 1},
              {.piece_id = 2,
               .polygon =
                   {.outer = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}},
               .geometry_revision = 2},
              {.piece_id = 3,
               .polygon =
                   {.outer = {{0.0, 0.0}, {5.0, 0.0}, {5.0, 3.0}, {0.0, 3.0}}},
               .geometry_revision = 3},
          },
      .policy = shiny::nfp::place::PlacementPolicy::bottom_left,
      .config = {.placement =
                     {.allowed_rotations = {.angles_degrees = {0.0, 90.0, 180.0,
                                                               270.0}}}},
      .max_bin_count = 1,
  };
  request.execution.control.capture_timestamps = false;

  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
  };

  const auto result = runner.run(request);

  REQUIRE(result.algorithm == AlgorithmKind::masonry_builder);
  REQUIRE(result.events.size() == live_events.size());
  REQUIRE(result.progress.size() == result.masonry.progress.size());
  REQUIRE(result.progress.size() == 3);
  REQUIRE(result.masonry.layout.placement_trace.size() == 3);

  const std::vector<SearchEventKind> expected = {
      SearchEventKind::run_started,       SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::run_completed,
  };

  for (std::size_t index = 0; index < expected.size(); ++index) {
    REQUIRE(search_event_kind(result.events[index]) == expected[index]);
    REQUIRE(search_event_kind(live_events[index]) == expected[index]);
    REQUIRE(search_event_algorithm_kind(result.events[index]) ==
            AlgorithmKind::masonry_builder);
  }

  for (std::size_t index = 0; index < result.progress.size(); ++index) {
    REQUIRE(result.progress[index].iteration ==
            result.masonry.progress[index].processed_piece_count);
    REQUIRE(result.progress[index].best_placed_piece_count ==
            result.masonry.progress[index].placed_piece_count);
    REQUIRE(result.progress[index].best_unplaced_piece_count ==
            result.masonry.progress[index].unplaced_piece_count);
    REQUIRE(result.progress[index].best_bin_count ==
            result.masonry.progress[index].bin_count);
  }
}