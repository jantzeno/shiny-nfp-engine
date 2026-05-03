#include <catch2/catch_test_macros.hpp>

#include "api/dto.hpp"
#include "api/request_builder.hpp"
#include "runtime/cancellation.hpp"
#include "solve.hpp"

namespace {

auto rectangle(double min_x, double min_y, double max_x, double max_y)
    -> shiny::nesting::geom::PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(shiny::nesting::geom::Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

} // namespace

TEST_CASE("README public API examples compile and execute",
          "[readme][api][examples]") {
  using namespace shiny::nesting;

  const auto request = api::ProfileRequestBuilder{}
                           .with_profile(SolveProfile::quick)
                           .add_bin(BinRequest{
                               .bin_id = 1,
                               .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
                           })
                           .add_piece(PieceRequest{
                               .piece_id = 7,
                               .polygon = rectangle(0.0, 0.0, 3.0, 2.0),
                           })
                           .build_checked();
  REQUIRE(request.ok());

  const auto solved = solve(request.value());
  REQUIRE(solved.ok());
  REQUIRE(solved.value().is_full_success());

  const auto production_request =
      api::ProfileRequestBuilder{}
          .with_profile(SolveProfile::balanced)
          .with_time_limit_ms(1)
          .add_bin(BinRequest{
              .bin_id = 1,
              .polygon = rectangle(0.0, 0.0, 10.0, 10.0),
          })
          .add_piece(PieceRequest{
              .piece_id = 7,
              .polygon = rectangle(0.0, 0.0, 3.0, 2.0),
          })
          .build_checked();
  REQUIRE(production_request.ok());

  const auto timed =
      solve(production_request.value(),
            api::ProfileSolveControlBuilder{}.with_random_seed(7).build());
  REQUIRE(timed.ok());
  REQUIRE(timed.value().stop_reason == StopReason::time_limit_reached);

  const auto request_dto = api::to_dto(request.value());
  const auto roundtrip = solve(api::to_request(request_dto),
                               api::to_solve_control(request_dto.control));
  REQUIRE(roundtrip.ok());
  const auto result_dto = api::to_dto(roundtrip.value());
  REQUIRE(result_dto.summary.full_success);

  auto progress_snapshot = ProfileProgressSnapshot{};
  progress_snapshot.profile = SolveProfile::balanced;
  progress_snapshot.phase = ProgressPhase::completed;
  progress_snapshot.phase_detail = "balanced";
  progress_snapshot.current_layout = solved.value().layout;
  progress_snapshot.best_layout = solved.value().layout;
  progress_snapshot.active_bin_id = 1U;
  progress_snapshot.bin_summary = runtime::summarize_bins(
      solved.value().layout, progress_snapshot.active_bin_id);
  progress_snapshot.placed_count = solved.value().placed_parts();
  progress_snapshot.utilization_percent = solved.value().utilization_percent();
  progress_snapshot.elapsed_time_milliseconds = 5U;
  progress_snapshot.remaining_time_milliseconds = std::nullopt;
  progress_snapshot.stop_reason = StopReason::completed;
  progress_snapshot.improved = true;

  const auto progress_dto = api::to_dto(progress_snapshot);
  REQUIRE(progress_dto.profile == SolveProfile::balanced);
  REQUIRE(progress_dto.phase == ProgressPhase::completed);
  REQUIRE(progress_dto.phase_detail == "balanced");
  REQUIRE(progress_dto.active_bin_id == 1U);
  REQUIRE(progress_dto.bin_summary.size() == 1U);
  REQUIRE(progress_dto.placed_count == 1U);
}
