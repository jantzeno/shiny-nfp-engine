#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "packing/candidate_generation.hpp"
#include "runtime/deterministic_rng.hpp"

namespace {

using shiny::nesting::CandidateStrategy;
using shiny::nesting::ExecutionPolicy;
using shiny::nesting::IrregularOptions;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::pack::CandidateGenerationObstacle;
using shiny::nesting::pack::GeneratedCandidatePoint;
using shiny::nesting::place::PlacementCandidateSource;
using shiny::nesting::place::PlacementStartCorner;
using shiny::nesting::runtime::DeterministicRng;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return {
      .outer = {
          {min_x, min_y},
          {max_x, min_y},
          {max_x, max_y},
          {min_x, max_y},
      },
  };
}

auto make_linear_candidates(const std::size_t count)
    -> std::vector<GeneratedCandidatePoint> {
  std::vector<GeneratedCandidatePoint> points;
  points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    points.push_back({
        .translation =
            Point2{static_cast<double>(count - index - 1U), 0.0},
        .source = PlacementCandidateSource::constructive_boundary,
    });
  }
  return points;
}

auto normalize_candidates(const std::vector<GeneratedCandidatePoint> &points)
    -> std::vector<std::tuple<std::int64_t, std::int64_t, int>> {
  std::vector<std::tuple<std::int64_t, std::int64_t, int>> normalized;
  normalized.reserve(points.size());
  for (const auto &point : points) {
    normalized.emplace_back(
        std::llround(point.translation.x * 1'000'000.0),
        std::llround(point.translation.y * 1'000'000.0),
        static_cast<int>(point.source));
  }
  std::sort(normalized.begin(), normalized.end());
  normalized.erase(std::unique(normalized.begin(), normalized.end()),
                   normalized.end());
  return normalized;
}

} // namespace

TEST_CASE("candidate limiting with max_candidate_points = 1 returns one point",
          "[packing][candidate-generation][limit][edge]") {
  ExecutionPolicy execution;
  execution.irregular.max_candidate_points = 1U;
  execution.irregular.candidate_gaussian_sigma = 0.5;

  auto points = make_linear_candidates(8U);
  DeterministicRng rng(42U);

  shiny::nesting::pack::limit_candidate_points(
      points, execution, PlacementStartCorner::bottom_left, &rng);

  REQUIRE(points.size() == 1U);
}

TEST_CASE("candidate limiting leaves small inputs unchanged",
          "[packing][candidate-generation][limit][edge]") {
  ExecutionPolicy execution;
  execution.irregular.max_candidate_points = 16U;
  execution.irregular.candidate_gaussian_sigma = 0.5;

  auto points = make_linear_candidates(4U);
  const auto sorted_expected = normalize_candidates(points);
  DeterministicRng rng(7U);

  shiny::nesting::pack::limit_candidate_points(
      points, execution, PlacementStartCorner::bottom_left, &rng);

  REQUIRE(points.size() == 4U);
  REQUIRE(normalize_candidates(points) == sorted_expected);
}

TEST_CASE("candidate limiting biases stochastic tail toward higher priority",
          "[packing][candidate-generation][limit]") {
  ExecutionPolicy execution;
  execution.irregular.max_candidate_points = 6U;
  execution.irregular.candidate_gaussian_sigma = 0.25;

  std::array<std::size_t, 12> selection_counts{};
  for (std::uint64_t seed = 0; seed < 512U; ++seed) {
    auto points = make_linear_candidates(selection_counts.size());
    DeterministicRng rng(seed);

    shiny::nesting::pack::limit_candidate_points(
        points, execution, PlacementStartCorner::bottom_left, &rng);

    REQUIRE(points.size() == execution.irregular.max_candidate_points);
    REQUIRE(std::is_sorted(points.begin(), points.end(),
                           [](const GeneratedCandidatePoint &lhs,
                              const GeneratedCandidatePoint &rhs) {
                             return lhs.translation.x < rhs.translation.x;
                           }));
    for (const auto &point : points) {
      ++selection_counts[static_cast<std::size_t>(point.translation.x)];
    }
  }

  REQUIRE(selection_counts[0] == 512U);
  REQUIRE(selection_counts[1] == 512U);
  REQUIRE(selection_counts[2] > selection_counts[11]);
  REQUIRE(selection_counts[3] > selection_counts[10]);
}

TEST_CASE("candidate limiting is deterministic for a fixed seed",
          "[packing][candidate-generation][deterministic]") {
  ExecutionPolicy execution;
  execution.irregular.max_candidate_points = 8U;
  execution.irregular.candidate_gaussian_sigma = 0.5;

  auto lhs = make_linear_candidates(20U);
  auto rhs = make_linear_candidates(20U);
  DeterministicRng lhs_rng(17U);
  DeterministicRng rhs_rng(17U);

  shiny::nesting::pack::limit_candidate_points(
      lhs, execution, PlacementStartCorner::bottom_left, &lhs_rng);
  shiny::nesting::pack::limit_candidate_points(
      rhs, execution, PlacementStartCorner::bottom_left, &rhs_rng);

  REQUIRE(lhs.size() == rhs.size());
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    REQUIRE(lhs[index].translation == rhs[index].translation);
    REQUIRE(lhs[index].source == rhs[index].source);
  }
}

TEST_CASE("irregular options reject non-positive gaussian sigma",
          "[request][irregular][validation]") {
  IrregularOptions options;
  REQUIRE(options.is_valid());

  options.candidate_gaussian_sigma = 0.0;
  REQUIRE_FALSE(options.is_valid());

  options = IrregularOptions{};
  options.candidate_gaussian_sigma = -0.25;
  REQUIRE_FALSE(options.is_valid());

  options = IrregularOptions{};
  options.candidate_gaussian_sigma = std::numeric_limits<double>::infinity();
  REQUIRE_FALSE(options.is_valid());
}

TEST_CASE("hybrid NFP candidate strategy is arrangement union perfect",
          "[packing][candidate-generation][hybrid]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 10.0);
  const auto moving_piece = rectangle(0.0, 0.0, 4.0, 4.0);
  const std::vector<CandidateGenerationObstacle> obstacles{{
      .geometry_revision = 2U,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .translation = {.x = 0.0, .y = 0.0},
      .rotation = {.degrees = 0.0},
  }};
  const std::span<const PolygonWithHoles> exclusion_regions{};
  const std::span<const CandidateGenerationObstacle> obstacle_span{obstacles};

  const auto arrangement = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 1U, {.degrees = 0.0},
      CandidateStrategy::nfp_arrangement);
  const auto perfect = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 1U, {.degrees = 0.0},
      CandidateStrategy::nfp_perfect);
  const auto hybrid = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 1U, {.degrees = 0.0},
      CandidateStrategy::nfp_hybrid);

  REQUIRE(arrangement.ok());
  REQUIRE(perfect.ok());
  REQUIRE(hybrid.ok());

  auto expected = normalize_candidates(arrangement.value());
  const auto normalized_perfect = normalize_candidates(perfect.value());
  expected.insert(expected.end(), normalized_perfect.begin(),
                  normalized_perfect.end());
  std::sort(expected.begin(), expected.end());
  expected.erase(std::unique(expected.begin(), expected.end()), expected.end());

  REQUIRE(normalize_candidates(hybrid.value()) == expected);
}
