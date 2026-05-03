#include <algorithm>
#include <cstddef>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "fixtures/export_surface/mtg_fixture.hpp"
#include "runtime/cancellation.hpp"

using namespace shiny::nesting;
using namespace shiny::nesting::test::mtg;

namespace {

[[nodiscard]] auto placed_parts(const pack::Layout &layout) -> std::size_t {
  return layout.placement_trace.size();
}

void require_summary_consistency(const NestingResult &result) {
  const auto summary = result.summary();
  REQUIRE(summary.placed_parts == result.placed_parts());
  REQUIRE(summary.unplaced_parts == result.unplaced_parts());
  REQUIRE(summary.layout_valid == result.layout_valid());
  REQUIRE(summary.placed_parts + summary.unplaced_parts == result.total_parts);
}

} // namespace

TEST_CASE("irregular constructive multi-start keeps the best layout after "
          "cancellation",
          "[solve][irregular][multi-start][observer][cancellation]") {
  const auto fixture = load_mtg_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;
  options.selected_bin_ids = {};

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  runtime::CancellationSource cancel_source{};
  std::vector<ProgressSnapshot> observed;
  bool saw_empty_search_pulse = false;
  bool saw_full_layout_snapshot = false;
  std::size_t best_observed_layout = 0;

  SolveControl control{};
  control.operation_limit = 4;
  control.random_seed = 99;
  control.cancellation = cancel_source.token();
  control.on_progress = [&](const ProgressSnapshot &snapshot) {
    observed.push_back(snapshot);

    const std::size_t observed_placed = placed_parts(snapshot.layout);
    best_observed_layout = std::max(best_observed_layout, observed_placed);
    saw_full_layout_snapshot |= observed_placed == fixture.pieces.size();

    if (saw_full_layout_snapshot && snapshot.sequence >= 2U &&
        snapshot.layout.placement_trace.empty()) {
      saw_empty_search_pulse = true;
      cancel_source.request_stop();
    }
  };

  const auto solved = solve(request, control);
  REQUIRE(solved.has_value());

  ExpectedOutcome expected{};
  expected.expected_placed_count = fixture.pieces.size();
  expected.expected_stop_reason = StopReason::cancelled;
  validate_layout(fixture, request, options, solved.value(), expected);

  REQUIRE(observed.size() >= 2U);
  REQUIRE(saw_empty_search_pulse);
  REQUIRE(saw_full_layout_snapshot);
  REQUIRE(best_observed_layout == fixture.pieces.size());
  REQUIRE(placed_parts(solved.value().layout) == best_observed_layout);
  REQUIRE(placed_parts(observed.back().layout) <
          placed_parts(solved.value().layout));
  REQUIRE(solved.value().layout_valid());
  require_summary_consistency(solved.value());
}

TEST_CASE("irregular constructive multi-start rejects unbounded solve control",
          "[solve][irregular][multi-start][budget]") {
  const auto fixture = make_asymmetric_engine_surface_fixture();

  MtgRequestOptions options{};
  options.strategy = StrategyKind::bounding_box;
  options.allow_part_overflow = true;
  options.maintain_bed_assignment = false;

  const auto request = make_request(fixture, options);
  REQUIRE(request.is_valid());

  const auto solved = solve(request, SolveControl{.random_seed = 99});

  REQUIRE_FALSE(solved.ok());
  REQUIRE(solved.status() == util::Status::invalid_input);
}
