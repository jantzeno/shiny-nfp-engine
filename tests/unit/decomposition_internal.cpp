#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "geometry/decomposition/convex_decomposition.hpp"
#include "geometry/decomposition/triangulation.hpp"
#include "geometry/operations/boolean_ops.hpp"
#include "geometry/operations/greedy_merge.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nesting::decomp::decompose_convex;
using shiny::nesting::decomp::is_convex;
using shiny::nesting::decomp::triangulate_polygon;
using shiny::nesting::geom::compute_bounds;
using shiny::nesting::geom::greedy_pairwise_merge;
using shiny::nesting::geom::normalize_polygon;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::Polygon;
using shiny::nesting::geom::polygon_area;
using shiny::nesting::geom::Ring;
using shiny::nesting::geom::union_polygons;
using shiny::nesting::test::require_ring_equal;

constexpr double kAreaEpsilon = 1.0e-8;

[[nodiscard]] auto polygon_less(const Polygon &lhs, const Polygon &rhs)
    -> bool {
  const auto lhs_bounds = compute_bounds(lhs);
  const auto rhs_bounds = compute_bounds(rhs);
  if (lhs_bounds.min.x() != rhs_bounds.min.x()) {
    return lhs_bounds.min.x() < rhs_bounds.min.x();
  }
  if (lhs_bounds.min.y() != rhs_bounds.min.y()) {
    return lhs_bounds.min.y() < rhs_bounds.min.y();
  }

  const auto lhs_area = polygon_area(lhs);
  const auto rhs_area = polygon_area(rhs);
  if (lhs_area != rhs_area) {
    return lhs_area < rhs_area;
  }

  if (lhs.outer().size() != rhs.outer().size()) {
    return lhs.outer().size() < rhs.outer().size();
  }

  for (std::size_t index = 0; index < lhs.outer().size(); ++index) {
    if (lhs.outer()[index] < rhs.outer()[index]) {
      return true;
    }
    if (rhs.outer()[index] < lhs.outer()[index]) {
      return false;
    }
  }

  return false;
}

[[nodiscard]] auto try_merge_polygons(const Polygon &lhs, const Polygon &rhs)
    -> std::optional<Polygon> {
  const auto merged =
      union_polygons(shiny::nesting::geom::PolygonWithHoles(lhs.outer()),
                     shiny::nesting::geom::PolygonWithHoles(rhs.outer()));
  if (merged.size() != 1U || !merged.front().holes().empty()) {
    return std::nullopt;
  }

  auto candidate =
      normalize_polygon(shiny::nesting::geom::Polygon(merged.front().outer()));
  const auto candidate_area = polygon_area(candidate);
  const auto expected_area = polygon_area(lhs) + polygon_area(rhs);
  if (std::fabs(candidate_area - expected_area) > kAreaEpsilon) {
    return std::nullopt;
  }
  if (!is_convex(candidate)) {
    return std::nullopt;
  }
  return candidate;
}

[[nodiscard]] auto make_staircase_polygon(const std::uint32_t seed) -> Polygon {
  const std::size_t step_count = 2U + static_cast<std::size_t>(seed % 4U);

  std::vector<double> xs;
  xs.reserve(step_count + 1U);
  xs.push_back(0.0);
  std::uint32_t state = seed * 1664525U + 1013904223U;
  for (std::size_t index = 0; index < step_count; ++index) {
    state = state * 1664525U + 1013904223U;
    xs.push_back(xs.back() + 1.0 + static_cast<double>(state % 5U));
  }

  std::vector<double> ys;
  ys.reserve(step_count + 1U);
  ys.push_back(0.0);
  for (std::size_t index = 0; index < step_count; ++index) {
    state = state * 1664525U + 1013904223U;
    ys.push_back(ys.back() + 1.0 + static_cast<double>(state % 4U));
  }

  shiny::nesting::geom::Ring ring;
  ring.reserve(2U * step_count + 2U);
  ring.push_back({0.0, 0.0});
  ring.push_back({xs.back(), 0.0});
  for (std::size_t index = 0; index < step_count; ++index) {
    const auto x_index = step_count - index;
    ring.push_back({xs[x_index], ys[index + 1U]});
    ring.push_back({xs[x_index - 1U], ys[index + 1U]});
  }

  return normalize_polygon(shiny::nesting::geom::Polygon(std::move(ring)));
}

// TODO(perf): add an opt-in micro-benchmark that compares the
// adjacency-aware merge against a restart-greedy baseline on a fixed
// staircase fixture and asserts the runtime ratio. Skipped today
// because the catch2 test harness is not configured with a stable
// timing tolerance suitable for CI.
} // namespace

TEST_CASE("triangulation exposes deterministic adjacency",
          "[decomposition][internal][triangulation]") {
  const Polygon polygon(Ring{
      {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {1.0, 1.0}, {1.0, 4.0}, {0.0, 4.0}});

  auto triangulation_or = triangulate_polygon(polygon);
  REQUIRE(triangulation_or.ok());
  const auto &triangulation = triangulation_or.value();

  REQUIRE(triangulation.triangles.size() == triangulation.neighbours.size());
  REQUIRE(triangulation.triangles.size() >= 2U);

  std::size_t adjacency_count = 0;
  for (std::size_t triangle_index = 0;
       triangle_index < triangulation.neighbours.size(); ++triangle_index) {
    for (const auto neighbour : triangulation.neighbours[triangle_index]) {
      if (neighbour < 0) {
        continue;
      }

      ++adjacency_count;
      REQUIRE(static_cast<std::size_t>(neighbour) <
              triangulation.triangles.size());

      bool symmetric = false;
      for (const auto reverse :
           triangulation.neighbours[static_cast<std::size_t>(neighbour)]) {
        if (reverse == static_cast<std::int32_t>(triangle_index)) {
          symmetric = true;
          break;
        }
      }
      REQUIRE(symmetric);
    }
  }

  REQUIRE(adjacency_count >= 2U);
}

TEST_CASE("regression keeps the deterministic L-shape partition",
          "[decomposition][internal][regression]") {
  const Polygon polygon(Ring{
      {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {1.0, 1.0}, {1.0, 4.0}, {0.0, 4.0}});

  auto pieces_or = decompose_convex(polygon);
  REQUIRE(pieces_or.ok());
  auto pieces = std::move(pieces_or).value();

  REQUIRE(pieces.size() == 2U);
  std::vector<Polygon> expected_pieces{
      Polygon{shiny::nesting::geom::Ring{
          {0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {1.0, 1.0}}},
      Polygon{shiny::nesting::geom::Ring{
          {0.0, 0.0}, {1.0, 1.0}, {1.0, 4.0}, {0.0, 4.0}}},
  };

  std::sort(pieces.begin(), pieces.end(), polygon_less);
  std::sort(expected_pieces.begin(), expected_pieces.end(), polygon_less);
  require_ring_equal(pieces[0].outer(), expected_pieces[0].outer());
  require_ring_equal(pieces[1].outer(), expected_pieces[1].outer());
}

TEST_CASE("adjacency-aware merge never uses more pieces than restart greedy",
          "[decomposition][internal][property]") {
  for (std::uint32_t seed = 0; seed < 50U; ++seed) {
    DYNAMIC_SECTION(seed) {
      const auto polygon = make_staircase_polygon(seed);

      auto actual_or = decompose_convex(polygon);
      REQUIRE(actual_or.ok());
      const auto &actual = actual_or.value();

      auto triangulation_or = triangulate_polygon(polygon);
      REQUIRE(triangulation_or.ok());
      auto greedy_pieces =
          greedy_pairwise_merge(std::move(triangulation_or).value().triangles,
                                [](const Polygon &lhs, const Polygon &rhs) {
                                  return try_merge_polygons(lhs, rhs);
                                });

      REQUIRE(actual.size() <= greedy_pieces.size());
      for (const auto &piece : actual) {
        REQUIRE(is_convex(piece));
      }
    }
  }
}