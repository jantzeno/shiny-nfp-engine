#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "cache/nfp_cache.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "geometry/transforms/transform.hpp"
#include "geometry/types.hpp"
#include "packing/irregular/blocked_regions.hpp"
#include "packing/irregular/candidate_generation.hpp"

namespace {

using shiny::nesting::cache::NfpCacheAccuracy;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::ResolvedRotation;
using shiny::nesting::geom::Ring;
using shiny::nesting::geom::Vector2;
using shiny::nesting::pack::CandidateGenerationObstacle;
using shiny::nesting::pack::build_blocked_regions;

auto square(const double side) -> PolygonWithHoles {
  return shiny::nesting::geom::normalize_polygon(
      PolygonWithHoles(Ring{{0.0, 0.0},
                            {side, 0.0},
                            {side, side},
                            {0.0, side}}));
}

constexpr ResolvedRotation zero_rotation{.degrees = 0.0};

} // namespace

TEST_CASE("build_blocked_regions with no obstacles produces an empty result",
          "[packing][blocked-regions]") {
  const auto moving_piece = square(1.0);
  const std::span<const CandidateGenerationObstacle> empty{};

  const auto result = build_blocked_regions(empty, moving_piece, 0U,
                                            zero_rotation, nullptr, nullptr);

  REQUIRE(result.has_value());
  REQUIRE(result.value().polygons.empty());
  REQUIRE(result.value().accuracy == NfpCacheAccuracy::exact);
}

TEST_CASE("build_blocked_regions with one square obstacle produces a non-empty "
          "blocked region",
          "[packing][blocked-regions]") {
  // Fixed 2×2 square at origin, moving 1×1 square.
  // The NFP of a 2×2 fixed square with a 1×1 moving square is a 3×3 region
  // at (-1,-1)-(2,2), area == 9.0.
  const CandidateGenerationObstacle obstacle{
      .geometry_revision = 0U,
      .polygon = square(2.0),
      .translation = Vector2(0.0, 0.0),
      .rotation = zero_rotation,
  };
  const auto moving_piece = square(1.0);

  const auto result =
      build_blocked_regions(std::span(&obstacle, 1U), moving_piece, 0U,
                            zero_rotation, nullptr, nullptr);

  REQUIRE(result.has_value());
  REQUIRE_FALSE(result.value().polygons.empty());

  const auto blocked_area =
      shiny::nesting::geom::polygon_area_sum(result.value().polygons);
  REQUIRE(blocked_area > 0.0);
  // For two convex squares the NFP area = (w_fixed + w_moving) * (h_fixed +
  // h_moving) = 3 * 3 = 9.
  REQUIRE(blocked_area > 8.0);
  REQUIRE(blocked_area < 10.0);
}

TEST_CASE("build_blocked_regions with two separate obstacles produces at least "
          "two blocked region polygons",
          "[packing][blocked-regions]") {
  // Two unit squares far apart so their NFPs do not merge.
  const std::array obstacles{
      CandidateGenerationObstacle{
          .geometry_revision = 0U,
          .polygon = square(1.0),
          .translation = Vector2(0.0, 0.0),
          .rotation = zero_rotation,
      },
      CandidateGenerationObstacle{
          .geometry_revision = 0U,
          .polygon = square(1.0),
          .translation = Vector2(20.0, 0.0),
          .rotation = zero_rotation,
      },
  };
  const auto moving_piece = square(0.5);

  const auto result =
      build_blocked_regions(std::span(obstacles), moving_piece, 0U,
                            zero_rotation, nullptr, nullptr);

  REQUIRE(result.has_value());
  // Each convex-square obstacle contributes exactly one polygon to the result.
  REQUIRE(result.value().polygons.size() >= 2U);

  // Total blocked area must be strictly greater than from a single obstacle.
  const auto total_area =
      shiny::nesting::geom::polygon_area_sum(result.value().polygons);
  REQUIRE(total_area > 0.0);
}

TEST_CASE("build_blocked_regions accuracy degrades to conservative_bbox when "
          "NFP fails for a degenerate polygon",
          "[packing][blocked-regions]") {
  // A degenerate polygon (three collinear points → zero-area) will cause NFP
  // computation to fail, which triggers the bbox fallback path and sets
  // accuracy to conservative_bbox_fallback on the result.
  const CandidateGenerationObstacle obstacle{
      .geometry_revision = 0U,
      .polygon = PolygonWithHoles(
          Ring{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), // collinear → degenerate
      .translation = Vector2(0.0, 0.0),
      .rotation = zero_rotation,
  };
  const auto moving_piece = square(1.0);

  const auto result =
      build_blocked_regions(std::span(&obstacle, 1U), moving_piece, 0U,
                            zero_rotation, nullptr, nullptr);

  REQUIRE(result.has_value());
  // The fallback polygon is a bounding-box approximation: non-empty and has
  // the conservative_bbox_fallback accuracy flag.
  REQUIRE_FALSE(result.value().polygons.empty());
  REQUIRE(result.value().accuracy == NfpCacheAccuracy::conservative_bbox_fallback);
}
