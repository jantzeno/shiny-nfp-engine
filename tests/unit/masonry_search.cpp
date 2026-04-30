#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

#include "algorithm_kind.hpp"
#include "packing/masonry.hpp"
#include "search/masonry.hpp"

namespace {

using shiny::nesting::AlgorithmKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::MasonryRequest;
using shiny::nesting::pack::PackingConfig;
using shiny::nesting::pack::PieceInput;
using shiny::nesting::place::PlacementPolicy;
using shiny::nesting::search::MasonryRunner;
using shiny::nesting::search::MasonryRunRequest;
using shiny::nesting::search::search_event_algorithm_kind;
using shiny::nesting::search::search_event_kind;
using shiny::nesting::search::SearchEvent;
using shiny::nesting::search::SearchEventKind;
using shiny::nesting::search::SearchRunStatus;

auto make_rectangle(double min_x, double min_y, double max_x, double max_y)
    -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      shiny::nesting::geom::Point2(min_x, min_y),
      shiny::nesting::geom::Point2(max_x, min_y),
      shiny::nesting::geom::Point2(max_x, max_y),
      shiny::nesting::geom::Point2(min_x, max_y),
  });
}

auto make_piece(std::uint32_t piece_id, PolygonWithHoles polygon,
                std::uint64_t geometry_revision) -> PieceInput {
  return {
      .piece_id = piece_id,
      .polygon = std::move(polygon),
      .geometry_revision = geometry_revision,
  };
}

} // namespace

TEST_CASE("masonry runner emits canonical observer events",
          "[packing][masonry][observer]") {
  MasonryRunner runner;

  MasonryRunRequest request{};
  request.masonry_request = MasonryRequest{
      .decoder_request =
          {
              .bins = {{
                  .bin_id = 60,
                  .polygon = make_rectangle(0.0, 0.0, 10.0, 10.0),
                  .geometry_revision = 600,
              }},
              .pieces =
                  {
                      make_piece(1, make_rectangle(0.0, 0.0, 6.0, 4.0), 1),
                      make_piece(2, make_rectangle(0.0, 0.0, 4.0, 4.0), 2),
                      make_piece(3, make_rectangle(0.0, 0.0, 5.0, 3.0), 3),
                  },
              .policy = PlacementPolicy::bottom_left,
              .config = PackingConfig{},
          },
  };
  request.execution.control.capture_timestamps = false;

  std::vector<SearchEvent> live_events;
  request.execution.observer.on_event = [&](const SearchEvent &event) {
    live_events.push_back(event);
  };

  const auto result = runner.run(request);

  REQUIRE(result.algorithm == AlgorithmKind::masonry_builder);
  REQUIRE(result.status == SearchRunStatus::completed);
  REQUIRE(result.progress.size() == 3);
  REQUIRE(result.masonry.progress.size() == 3);
  REQUIRE(result.events.size() == live_events.size());

  const std::vector<SearchEventKind> expected_kinds = {
      SearchEventKind::run_started,       SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::step_progress,
      SearchEventKind::improvement_found, SearchEventKind::run_completed,
  };

  REQUIRE(live_events.size() == expected_kinds.size());
  for (std::size_t index = 0; index < expected_kinds.size(); ++index) {
    REQUIRE(search_event_kind(live_events[index]) == expected_kinds[index]);
    REQUIRE(search_event_kind(result.events[index]) == expected_kinds[index]);
    REQUIRE(search_event_algorithm_kind(live_events[index]) ==
            AlgorithmKind::masonry_builder);
  }

  REQUIRE(result.progress[0].iteration == 1);
  REQUIRE(result.progress[1].iteration == 2);
  REQUIRE(result.progress[2].iteration == 3);
  REQUIRE(result.progress[0].improved);
  REQUIRE(result.progress[1].improved);
  REQUIRE(result.progress[2].improved);
}
