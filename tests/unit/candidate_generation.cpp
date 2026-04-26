#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "geometry/transform.hpp"
#include "nfp/ifp.hpp"
#include "nfp/nfp.hpp"
#include "packing/irregular/candidate_generation.hpp"
#include "predicates/point_location.hpp"
#include "runtime/deterministic_rng.hpp"
#include "support/mtg_fixture.hpp"

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
using shiny::nesting::test::mtg::kBed1Id;
using shiny::nesting::test::mtg::load_mtg_fixture_with_actual_polygons;
using shiny::nesting::test::mtg::MtgRequestOptions;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return {
      .outer =
          {
              {min_x, min_y},
              {max_x, min_y},
              {max_x, max_y},
              {min_x, max_y},
          },
  };
}

auto concave_step_piece() -> PolygonWithHoles {
  return {
      .outer =
          {
              {1.0, 0.0},
              {4.0, 0.0},
              {4.0, 1.0},
              {2.0, 1.0},
              {2.0, 3.0},
              {0.0, 3.0},
              {0.0, 2.0},
              {1.0, 2.0},
          },
  };
}

auto make_baseline_actual_polygon_options() -> MtgRequestOptions {
  MtgRequestOptions options{};
  options.strategy = shiny::nesting::StrategyKind::sequential_backtrack;
  options.placement_policy =
      shiny::nesting::place::PlacementPolicy::bottom_left;
  options.irregular.candidate_strategy = CandidateStrategy::anchor_vertex;
  options.maintain_bed_assignment = false;
  options.allow_part_overflow = true;
  options.part_spacing_mm = 0.0;
  options.bed1_start_corner = PlacementStartCorner::bottom_left;
  options.bed2_start_corner = PlacementStartCorner::bottom_left;
  return options;
}

auto solve_baseline_actual_polygon_layout(
    const shiny::nesting::test::mtg::MtgFixture &fixture)
    -> shiny::nesting::NestingResult {
  const auto options = make_baseline_actual_polygon_options();
  auto request = shiny::nesting::test::mtg::make_request(fixture, options);
  REQUIRE(request.is_valid());

  shiny::nesting::SolveControl control{};
  control.time_limit_milliseconds = 60'000U;
  control.random_seed = 0U;

  const auto solved = shiny::nesting::solve(request, control);
  REQUIRE(solved.has_value());
  return solved.value();
}

auto find_placed_piece(const shiny::nesting::NestingResult &result,
                       const std::uint32_t bin_id, const std::uint32_t piece_id)
    -> const shiny::nesting::pack::PlacedPiece * {
  const auto bin_it =
      std::find_if(result.layout.bins.begin(), result.layout.bins.end(),
                   [bin_id](const auto &bin) { return bin.bin_id == bin_id; });
  if (bin_it == result.layout.bins.end()) {
    return nullptr;
  }
  const auto piece_it =
      std::find_if(bin_it->placements.begin(), bin_it->placements.end(),
                   [piece_id](const auto &placed) {
                     return placed.placement.piece_id == piece_id;
                   });
  return piece_it == bin_it->placements.end() ? nullptr : &*piece_it;
}

auto make_linear_candidates(const std::size_t count)
    -> std::vector<GeneratedCandidatePoint> {
  std::vector<GeneratedCandidatePoint> points;
  points.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    points.push_back({
        .translation = Point2{static_cast<double>(count - index - 1U), 0.0},
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
    normalized.emplace_back(std::llround(point.translation.x * 1'000'000.0),
                            std::llround(point.translation.y * 1'000'000.0),
                            static_cast<int>(point.source));
  }
  std::sort(normalized.begin(), normalized.end());
  normalized.erase(std::unique(normalized.begin(), normalized.end()),
                   normalized.end());
  return normalized;
}

auto normalize_translations(std::span<const Point2> points)
    -> std::vector<std::pair<std::int64_t, std::int64_t>> {
  std::vector<std::pair<std::int64_t, std::int64_t>> normalized;
  normalized.reserve(points.size());
  for (const auto &point : points) {
    normalized.emplace_back(std::llround(point.x * 1'000'000.0),
                            std::llround(point.y * 1'000'000.0));
  }
  std::sort(normalized.begin(), normalized.end());
  normalized.erase(std::unique(normalized.begin(), normalized.end()),
                   normalized.end());
  return normalized;
}

auto normalize_candidate_translations(
    const std::vector<GeneratedCandidatePoint> &points)
    -> std::vector<std::pair<std::int64_t, std::int64_t>> {
  std::vector<Point2> translations;
  translations.reserve(points.size());
  for (const auto &point : points) {
    translations.push_back(point.translation);
  }
  return normalize_translations(translations);
}

auto raw_vertex_anchor_translations(const PolygonWithHoles &container,
                                    const PolygonWithHoles &piece)
    -> std::vector<std::pair<std::int64_t, std::int64_t>> {
  std::vector<Point2> translations;
  translations.reserve(container.outer.size() * piece.outer.size());
  for (const auto &container_vertex : container.outer) {
    for (const auto &piece_vertex : piece.outer) {
      translations.push_back({.x = container_vertex.x - piece_vertex.x,
                              .y = container_vertex.y - piece_vertex.y});
    }
  }
  return normalize_translations(translations);
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
      container, exclusion_regions, obstacle_span, moving_piece, 1U,
      {.degrees = 0.0}, CandidateStrategy::nfp_arrangement);
  const auto perfect = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 1U,
      {.degrees = 0.0}, CandidateStrategy::nfp_perfect);
  const auto hybrid = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 1U,
      {.degrees = 0.0}, CandidateStrategy::nfp_hybrid);

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

TEST_CASE("actual-polygon MTG repro piece produces bed-domain candidates",
          "[packing][candidate-generation][actual-polygons][mtg]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto piece_it =
      std::find_if(fixture.pieces.begin(), fixture.pieces.end(),
                   [](const auto &piece) { return piece.piece_id == 4U; });
  REQUIRE(piece_it != fixture.pieces.end());

  const std::span<const PolygonWithHoles> exclusion_regions{};
  const std::span<const CandidateGenerationObstacle> obstacles{};

  const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
      fixture.bed1.polygon, exclusion_regions, obstacles, piece_it->polygon,
      piece_it->piece_id, {.degrees = 0.0}, CandidateStrategy::nfp_perfect);

  INFO("Diagnosed anchor repro first reports no candidate for actual-polygon "
       "piece 4 on bed "
       << kBed1Id
       << "; this pins the lower-level bed-domain candidate "
          "generation for that same piece/bin case.");
  REQUIRE(points.ok());
  REQUIRE_FALSE(points.value().empty());
}

TEST_CASE("NFP perfect-fit candidates stay inside the feasible domain",
          "[packing][candidate-generation][property]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 10.0);
  const auto moving_piece = rectangle(0.0, 0.0, 2.0, 2.0);
  const auto obstacle_polygon = shiny::nesting::geom::translate(
      rectangle(0.0, 0.0, 3.0, 3.0), {.x = 6.0, .y = 6.0});
  const std::vector<CandidateGenerationObstacle> obstacles{{
      .geometry_revision = 2U,
      .polygon = obstacle_polygon,
      .translation = {.x = 6.0, .y = 6.0},
      .rotation = {.degrees = 0.0},
  }};
  const std::span<const PolygonWithHoles> exclusion_regions{};
  const std::span<const CandidateGenerationObstacle> obstacle_span{obstacles};

  const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacle_span, moving_piece, 17U,
      {.degrees = 0.0}, CandidateStrategy::nfp_perfect);
  REQUIRE(points.ok());
  REQUIRE_FALSE(points.value().empty());

  const auto domain = shiny::nesting::nfp::compute_ifp(container, moving_piece);
  REQUIRE(domain.ok());
  const auto blocked_base = shiny::nesting::geom::translate(
      obstacles.front().polygon, {.x = -obstacles.front().translation.x,
                                  .y = -obstacles.front().translation.y});
  auto blocked = shiny::nesting::nfp::compute_nfp(blocked_base, moving_piece);
  REQUIRE(blocked.ok());
  std::vector<PolygonWithHoles> blocked_world;
  blocked_world.reserve(blocked.value().size());
  for (const auto &polygon : blocked.value()) {
    blocked_world.push_back(shiny::nesting::geom::translate(
        polygon, {.x = obstacles.front().translation.x,
                  .y = obstacles.front().translation.y}));
  }
  for (const auto &candidate : points.value()) {
    const bool in_domain = std::any_of(
        domain.value().begin(), domain.value().end(), [&](const auto &region) {
          return shiny::nesting::pred::locate_point_in_polygon(
                     candidate.translation, region)
                     .location != shiny::nesting::pred::PointLocation::exterior;
        });
    const bool inside_blocked_interior = std::any_of(
        blocked_world.begin(), blocked_world.end(), [&](const auto &region) {
          return shiny::nesting::pred::locate_point_in_polygon(
                     candidate.translation, region)
                     .location == shiny::nesting::pred::PointLocation::interior;
        });
    INFO(candidate.translation.x << "," << candidate.translation.y);
    REQUIRE(in_domain);
    REQUIRE_FALSE(inside_blocked_interior);
  }
}

TEST_CASE("actual-polygon pair 7 -> 4 returns candidates directly after NFP "
          "fallback",
          "[packing][candidate-generation][actual-polygons][repro]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto result = solve_baseline_actual_polygon_layout(fixture);
  const auto *fixed = find_placed_piece(result, kBed1Id, 7U);
  const auto *moving = find_placed_piece(result, kBed1Id, 4U);
  REQUIRE(fixed != nullptr);
  REQUIRE(moving != nullptr);
  REQUIRE(std::abs(fixed->placement.translation.x) < 1e-6);
  REQUIRE(std::abs(fixed->placement.translation.y - 1243.4646072387695) < 1e-6);

  const CandidateGenerationObstacle obstacle{
      .geometry_revision = fixed->piece_geometry_revision,
      .polygon = fixed->polygon,
      .translation = {.x = fixed->placement.translation.x,
                      .y = fixed->placement.translation.y},
      .rotation = fixed->resolved_rotation,
  };
  const std::array<CandidateGenerationObstacle, 1U> obstacles{obstacle};
  const auto moving_local = shiny::nesting::geom::translate(
      moving->polygon, {.x = -moving->placement.translation.x,
                        .y = -moving->placement.translation.y});
  const std::span<const PolygonWithHoles> exclusion_regions{};

  for (const auto strategy :
       {CandidateStrategy::nfp_perfect, CandidateStrategy::nfp_arrangement}) {
    INFO("strategy=" << static_cast<int>(strategy));
    const auto status_or = shiny::nesting::pack::generate_nfp_candidate_points(
        fixture.bed1.polygon, exclusion_regions, obstacles, moving_local,
        moving->piece_geometry_revision, moving->resolved_rotation, strategy);
    REQUIRE(status_or.ok());
    REQUIRE_FALSE(status_or.value().empty());
  }
}

TEST_CASE(
    "anchor strategy uses feasible-domain candidates beyond raw vertex anchors",
    "[packing][candidate-generation][concave][anchor]") {
  const auto container = rectangle(0.0, 0.0, 8.0, 8.0);
  const auto moving_piece = concave_step_piece();
  const std::span<const PolygonWithHoles> exclusion_regions{};
  const std::span<const CandidateGenerationObstacle> obstacles{};

  const auto anchor_points =
      shiny::nesting::pack::generate_nfp_candidate_points(
          container, exclusion_regions, obstacles, moving_piece, 99U,
          {.degrees = 0.0}, CandidateStrategy::anchor_vertex);

  REQUIRE(anchor_points.ok());

  const auto candidate_translations =
      normalize_candidate_translations(anchor_points.value());
  const auto vertex_only_translations =
      raw_vertex_anchor_translations(container, moving_piece);
  const auto bottom_left_translation =
      std::pair<std::int64_t, std::int64_t>{0LL, 0LL};

  INFO("The anchor strategy should now reuse the shared feasible-domain model, "
       "so a valid concave bottom-left placement is available even when raw "
       "boundary vertex-to-piece-vertex anchors miss it.");
  REQUIRE(std::find(candidate_translations.begin(),
                    candidate_translations.end(),
                    bottom_left_translation) != candidate_translations.end());
  REQUIRE(std::find(vertex_only_translations.begin(),
                    vertex_only_translations.end(),
                    bottom_left_translation) == vertex_only_translations.end());
  REQUIRE(std::all_of(
      anchor_points.value().begin(), anchor_points.value().end(),
      [](const GeneratedCandidatePoint &point) {
        return point.source == PlacementCandidateSource::constructive_boundary;
      }));
}

TEST_CASE("concave piece produces feasible candidate beyond raw vertex anchors",
          "[packing][candidate-generation][concave][perfect-fit]") {
  const auto container = rectangle(0.0, 0.0, 8.0, 8.0);
  const auto moving_piece = concave_step_piece();
  const std::span<const PolygonWithHoles> exclusion_regions{};
  const std::span<const CandidateGenerationObstacle> obstacles{};

  const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacles, moving_piece, 99U,
      {.degrees = 0.0}, CandidateStrategy::nfp_perfect);

  REQUIRE(points.ok());

  const auto candidate_translations =
      normalize_candidate_translations(points.value());
  const auto vertex_only_translations =
      raw_vertex_anchor_translations(container, moving_piece);
  const auto bottom_left_translation =
      std::pair<std::int64_t, std::int64_t>{0LL, 0LL};

  INFO("This concave piece has no outer vertex at its bounding-box minimum, so "
       "the valid bottom-left placement is not reachable from raw boundary "
       "vertex-to-piece-vertex anchors alone.");
  REQUIRE(std::find(candidate_translations.begin(),
                    candidate_translations.end(),
                    bottom_left_translation) != candidate_translations.end());
  REQUIRE(std::find(vertex_only_translations.begin(),
                    vertex_only_translations.end(),
                    bottom_left_translation) == vertex_only_translations.end());
}
