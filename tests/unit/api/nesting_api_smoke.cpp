#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <optional>

#include "internal/legacy_solve.hpp"
#include "internal/request_normalization.hpp"

namespace {

using shiny::nesting::BinRequest;
using shiny::nesting::NestingRequest;
using shiny::nesting::PieceRequest;
using shiny::nesting::SolveControl;
using shiny::nesting::StrategyKind;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;

auto unit_square() -> PolygonWithHoles {
  return PolygonWithHoles(Ring{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
}

} // namespace

TEST_CASE("public nesting API round-trip: normalize and solve accept a valid request and return placed results",
          "[api][smoke]") {
  // Aggregate-initialize a NestingRequest using C++23 designated initializers.
  NestingRequest request{};
  request.execution.strategy = StrategyKind::bounding_box;
  request.execution.default_rotations = {{0.0}};

  request.bins.push_back(BinRequest{
      .bin_id = 1,
      .polygon = unit_square(),
  });
  request.pieces.push_back(PieceRequest{
      .piece_id = 1,
      .polygon = PolygonWithHoles(
          Ring{{0.1, 0.1}, {0.4, 0.1}, {0.4, 0.4}, {0.1, 0.4}}),
  });

  // normalize_request returns util::StatusOr (std::expected-style sum type).
  // Test both .ok() and .value() access to exercise the C++23 API patterns.
  const auto normalized = shiny::nesting::normalize_request(request);
  REQUIRE(normalized.ok());
  REQUIRE(normalized.value().expanded_pieces.size() == 1U);
  REQUIRE(normalized.value().expanded_bins.size() == 1U);

  // solve returns util::StatusOr<NestingResult> — field order matches
  // SolveControl declaration (operation_limit before random_seed).
  SolveControl control{};
  control.operation_limit = 10U;
  control.random_seed = 0U;
  const auto result = shiny::nesting::solve(request, control);
  REQUIRE(result.ok());
  REQUIRE(result.value().placed_parts() >= 1U);
  REQUIRE(result.value().validation.valid);

  // std::optional fields compile and are usable: PieceRequest::assigned_bin_id.
  PieceRequest optional_field_piece{
      .piece_id = 2,
      .polygon = unit_square(),
      .assigned_bin_id = std::nullopt,
  };
  REQUIRE_FALSE(optional_field_piece.assigned_bin_id.has_value());
}
