#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

#include "geometry/transforms/transform.hpp"
#include "nfp/ifp.hpp"
#include "nfp/nfp.hpp"
#include "packing/irregular/blocked_regions.hpp"
#include "packing/irregular/candidate_generation.hpp"
#include "predicates/point_location.hpp"
#include "runtime/deterministic_rng.hpp"
#include "support/fixture_test_support.hpp"
#include "support/mtg_fixture.hpp"

namespace {

using shiny::nesting::CandidateStrategy;
using shiny::nesting::ExecutionPolicy;
using shiny::nesting::IrregularOptions;
using shiny::nesting::geom::Point2;
using shiny::nesting::geom::PolygonWithHoles;
using shiny::nesting::geom::Ring;
using shiny::nesting::pack::CandidateGenerationObstacle;
using shiny::nesting::pack::GeneratedCandidatePoint;
using shiny::nesting::place::PlacementCandidateSource;
using shiny::nesting::place::PlacementStartCorner;
using shiny::nesting::runtime::DeterministicRng;
using shiny::nesting::test::load_fixture_file;
using shiny::nesting::test::parse_point;
using shiny::nesting::test::parse_polygon;
using shiny::nesting::test::require_fixture_metadata;
using shiny::nesting::test::mtg::kBed1Id;
using shiny::nesting::test::mtg::load_mtg_fixture_with_actual_polygons;
using shiny::nesting::test::mtg::MtgRequestOptions;

auto rectangle(const double min_x, const double min_y, const double max_x,
               const double max_y) -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(Ring{
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
  });
}

auto concave_step_piece() -> PolygonWithHoles {
  return shiny::nesting::geom::PolygonWithHoles(Ring{
      {1.0, 0.0},
      {4.0, 0.0},
      {4.0, 1.0},
      {2.0, 1.0},
      {2.0, 3.0},
      {0.0, 3.0},
      {0.0, 2.0},
      {1.0, 2.0},
  });
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
    normalized.emplace_back(std::llround(point.translation.x() * 1'000'000.0),
                            std::llround(point.translation.y() * 1'000'000.0),
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
    normalized.emplace_back(std::llround(point.x() * 1'000'000.0),
                            std::llround(point.y() * 1'000'000.0));
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
  translations.reserve(container.outer().size() * piece.outer().size());
  for (const auto &container_vertex : container.outer()) {
    for (const auto &piece_vertex : piece.outer()) {
      translations.push_back(Point2(container_vertex.x() - piece_vertex.x(),
                                    container_vertex.y() - piece_vertex.y()));
    }
  }
  return normalize_translations(translations);
}

auto parse_obstacles(const shiny::nesting::test::pt::ptree &inputs)
    -> std::vector<CandidateGenerationObstacle> {
  std::vector<CandidateGenerationObstacle> obstacles;
  const auto obstacle_nodes = inputs.get_child_optional("obstacles");
  if (!obstacle_nodes.has_value()) {
    return obstacles;
  }

  for (const auto &obstacle_node : *obstacle_nodes) {
    const auto &obstacle = obstacle_node.second;
    const auto translation = parse_point(obstacle.get_child("translation"));
    const auto base_polygon = parse_polygon(obstacle.get_child("polygon"));
    obstacles.push_back({
        .geometry_revision = obstacle.get<std::uint64_t>("revision"),
        .polygon = shiny::nesting::geom::translate(
            base_polygon,
            shiny::nesting::geom::Vector2(translation.x(), translation.y())),
        .translation =
            shiny::nesting::geom::Vector2(translation.x(), translation.y()),
        .rotation = {.degrees = 0.0},
    });
  }
  return obstacles;
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
                             return lhs.translation.x() < rhs.translation.x();
                           }));
    for (const auto &point : points) {
      ++selection_counts[static_cast<std::size_t>(point.translation.x())];
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
      .translation = shiny::nesting::geom::Vector2(0.0, 0.0),
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
      rectangle(0.0, 0.0, 3.0, 3.0), shiny::nesting::geom::Vector2(6.0, 6.0));
  const std::vector<CandidateGenerationObstacle> obstacles{{
      .geometry_revision = 2U,
      .polygon = obstacle_polygon,
      .translation = shiny::nesting::geom::Vector2(6.0, 6.0),
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
      obstacles.front().polygon,
      shiny::nesting::geom::Vector2(-obstacles.front().translation.x(),
                                    -obstacles.front().translation.y()));
  auto blocked = shiny::nesting::nfp::compute_nfp(blocked_base, moving_piece);
  REQUIRE(blocked.ok());
  std::vector<PolygonWithHoles> blocked_world;
  blocked_world.reserve(blocked.value().size());
  for (const auto &polygon : blocked.value()) {
    blocked_world.push_back(shiny::nesting::geom::translate(
        polygon,
        shiny::nesting::geom::Vector2(obstacles.front().translation.x(),
                                      obstacles.front().translation.y())));
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
    INFO(candidate.translation.x() << "," << candidate.translation.y());
    REQUIRE(in_domain);
    REQUIRE_FALSE(inside_blocked_interior);
  }
}

TEST_CASE("fixture-backed NFP candidate generation covers synthetic geometry "
          "families",
          "[packing][candidate-generation][nfp][fixtures]") {
  const auto root = load_fixture_file("nfp/candidate_geometry_families.json");

  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "candidate_geometry_family");
      const auto inputs = fixture.get_child("inputs");
      const auto expected = fixture.get_child("expected");
      const auto container = parse_polygon(inputs.get_child("container"));
      const auto moving_piece = parse_polygon(inputs.get_child("moving"));
      const auto obstacles = parse_obstacles(inputs);
      const std::span<const PolygonWithHoles> exclusion_regions{};
      const std::span<const CandidateGenerationObstacle> obstacle_span{
          obstacles};

      shiny::nesting::cache::NfpCache cache(
          shiny::nesting::cache::default_nfp_cache_config());
      shiny::nesting::pack::CandidateGenerationDiagnostics diagnostics{};

      const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
          container, exclusion_regions, obstacle_span, moving_piece,
          shiny::nesting::geom::polygon_revision(moving_piece),
          {.degrees = 0.0}, CandidateStrategy::nfp_hybrid, &cache,
          &diagnostics);

      REQUIRE(points.ok());
      REQUIRE(points.value().size() >=
              expected.get<std::size_t>("min_candidates"));
      REQUIRE_FALSE(diagnostics.used_conservative_bbox_fallback());
      if (expected.get<bool>("requires_exact_nfp")) {
        REQUIRE(diagnostics.exact_nfp_computations > 0U);
      }
      REQUIRE(diagnostics.perfect_fit_candidates > 0U);
      REQUIRE(diagnostics.perfect_slide_candidates > 0U);

      const auto blocked = shiny::nesting::pack::build_blocked_regions(
          obstacle_span, moving_piece,
          shiny::nesting::geom::polygon_revision(moving_piece),
          {.degrees = 0.0}, &cache, &diagnostics);
      REQUIRE(blocked.ok());
      REQUIRE_FALSE(blocked.value().polygons.empty());
      REQUIRE(blocked.value().accuracy ==
              shiny::nesting::cache::NfpCacheAccuracy::exact);
    }
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
  REQUIRE(std::abs(fixed->placement.translation.x()) < 1e-6);
  REQUIRE(std::abs(fixed->placement.translation.y() - 1243.4646072387695) <
          1e-6);

  const CandidateGenerationObstacle obstacle{
      .geometry_revision = fixed->piece_geometry_revision,
      .polygon = fixed->polygon,
      .translation = shiny::nesting::geom::Vector2(
          fixed->placement.translation.x(), fixed->placement.translation.y()),
      .rotation = fixed->resolved_rotation,
  };
  const std::array<CandidateGenerationObstacle, 1U> obstacles{obstacle};
  const auto moving_local = shiny::nesting::geom::translate(
      moving->polygon,
      shiny::nesting::geom::Vector2(-moving->placement.translation.x(),
                                    -moving->placement.translation.y()));
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

TEST_CASE("anchor strategy skips obstacle NFP generation by default",
          "[packing][candidate-generation][anchor][contract]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 10.0);
  const auto moving_piece = rectangle(0.0, 0.0, 2.0, 2.0);
  const std::vector<CandidateGenerationObstacle> obstacles{{
      .geometry_revision = 2U,
      .polygon = rectangle(0.0, 0.0, 3.0, 3.0),
      .translation = shiny::nesting::geom::Vector2(4.0, 4.0),
      .rotation = {.degrees = 0.0},
  }};
  const std::span<const PolygonWithHoles> exclusion_regions{};
  shiny::nesting::cache::NfpCache cache(
      shiny::nesting::cache::default_nfp_cache_config());
  shiny::nesting::pack::CandidateGenerationDiagnostics diagnostics{};

  const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacles, moving_piece, 17U,
      {.degrees = 0.0}, CandidateStrategy::anchor_vertex, &cache, &diagnostics);

  REQUIRE(points.ok());
  REQUIRE_FALSE(points.value().empty());
  REQUIRE(diagnostics.exact_nfp_cache_hits == 0U);
  REQUIRE(diagnostics.conservative_bbox_cache_hits == 0U);
  REQUIRE(diagnostics.exact_nfp_computations == 0U);
  REQUIRE(diagnostics.conservative_bbox_fallbacks == 0U);
  REQUIRE_FALSE(diagnostics.used_conservative_bbox_fallback());

  const auto exact_key = shiny::nesting::cache::make_nfp_cache_key(
      2U, 17U, 0.0, 0.0, shiny::nesting::cache::NfpCacheEntryKind::exact);
  const auto fallback_key = shiny::nesting::cache::make_nfp_cache_key(
      2U, 17U, 0.0, 0.0,
      shiny::nesting::cache::NfpCacheEntryKind::conservative_bbox_fallback);
  REQUIRE(cache.get(exact_key) == nullptr);
  REQUIRE(cache.get(fallback_key) == nullptr);
}

TEST_CASE("exact NFP cache entries stay separate from fallback entries",
          "[packing][candidate-generation][cache][contract]") {
  const auto container = rectangle(0.0, 0.0, 10.0, 10.0);
  const auto moving_piece = rectangle(0.0, 0.0, 4.0, 4.0);
  const std::vector<CandidateGenerationObstacle> obstacles{{
      .geometry_revision = 2U,
      .polygon = rectangle(0.0, 0.0, 6.0, 6.0),
      .translation = shiny::nesting::geom::Vector2(0.0, 0.0),
      .rotation = {.degrees = 0.0},
  }};
  const std::span<const PolygonWithHoles> exclusion_regions{};
  shiny::nesting::cache::NfpCache cache(
      shiny::nesting::cache::default_nfp_cache_config());
  shiny::nesting::pack::CandidateGenerationDiagnostics diagnostics{};

  const auto points = shiny::nesting::pack::generate_nfp_candidate_points(
      container, exclusion_regions, obstacles, moving_piece, 17U,
      {.degrees = 0.0}, CandidateStrategy::nfp_perfect, &cache, &diagnostics);

  REQUIRE(points.ok());
  REQUIRE_FALSE(points.value().empty());
  REQUIRE(diagnostics.exact_nfp_computations > 0U);
  REQUIRE(diagnostics.conservative_bbox_fallbacks == 0U);
  REQUIRE(diagnostics.perfect_fit_candidates > 0U);
  REQUIRE(std::all_of(points.value().begin(), points.value().end(),
                      [](const GeneratedCandidatePoint &point) {
                        return point.nfp_accuracy ==
                               shiny::nesting::cache::NfpCacheAccuracy::exact;
                      }));

  const auto exact_key = shiny::nesting::cache::make_nfp_cache_key(
      2U, 17U, 0.0, 0.0, shiny::nesting::cache::NfpCacheEntryKind::exact);
  const auto fallback_key = shiny::nesting::cache::make_nfp_cache_key(
      2U, 17U, 0.0, 0.0,
      shiny::nesting::cache::NfpCacheEntryKind::conservative_bbox_fallback);
  const auto exact_entry = cache.get(exact_key);
  REQUIRE(exact_entry != nullptr);
  REQUIRE(exact_entry->accuracy ==
          shiny::nesting::cache::NfpCacheAccuracy::exact);
  REQUIRE(exact_entry->status == shiny::nesting::util::Status::ok);
  REQUIRE(cache.get(fallback_key) == nullptr);
}

TEST_CASE("bbox fallback cache entries are labeled separately from exact NFPs",
          "[packing][candidate-generation][fallback][cache]") {
  const auto fixture = load_mtg_fixture_with_actual_polygons();
  const auto result = solve_baseline_actual_polygon_layout(fixture);
  const auto *fixed = find_placed_piece(result, kBed1Id, 7U);
  const auto *moving = find_placed_piece(result, kBed1Id, 4U);
  REQUIRE(fixed != nullptr);
  REQUIRE(moving != nullptr);

  const CandidateGenerationObstacle obstacle{
      .geometry_revision = fixed->piece_geometry_revision,
      .polygon = fixed->polygon,
      .translation = shiny::nesting::geom::Vector2(
          fixed->placement.translation.x(), fixed->placement.translation.y()),
      .rotation = fixed->resolved_rotation,
  };
  const std::array<CandidateGenerationObstacle, 1U> obstacles{obstacle};
  const auto moving_local = shiny::nesting::geom::translate(
      moving->polygon,
      shiny::nesting::geom::Vector2(-moving->placement.translation.x(),
                                    -moving->placement.translation.y()));
  const std::span<const PolygonWithHoles> exclusion_regions{};
  shiny::nesting::cache::NfpCache cache(
      shiny::nesting::cache::default_nfp_cache_config());

  shiny::nesting::pack::CandidateGenerationDiagnostics first_diagnostics{};
  const auto first = shiny::nesting::pack::generate_nfp_candidate_points(
      fixture.bed1.polygon, exclusion_regions, obstacles, moving_local,
      moving->piece_geometry_revision, moving->resolved_rotation,
      CandidateStrategy::nfp_perfect, &cache, &first_diagnostics);

  REQUIRE(first.ok());
  REQUIRE_FALSE(first.value().empty());
  REQUIRE(first_diagnostics.conservative_bbox_fallbacks > 0U);
  REQUIRE(first_diagnostics.exact_nfp_cache_hits == 0U);
  REQUIRE(first_diagnostics.conservative_bbox_cache_hits == 0U);
  REQUIRE(first_diagnostics.used_conservative_bbox_fallback());
  REQUIRE(std::all_of(first.value().begin(), first.value().end(),
                      [](const GeneratedCandidatePoint &point) {
                        return point.nfp_accuracy ==
                               shiny::nesting::cache::NfpCacheAccuracy::
                                   conservative_bbox_fallback;
                      }));

  const auto exact_key = shiny::nesting::cache::make_nfp_cache_key(
      fixed->piece_geometry_revision, moving->piece_geometry_revision,
      fixed->resolved_rotation.degrees, moving->resolved_rotation.degrees,
      shiny::nesting::cache::NfpCacheEntryKind::exact);
  const auto fallback_key = shiny::nesting::cache::make_nfp_cache_key(
      fixed->piece_geometry_revision, moving->piece_geometry_revision,
      fixed->resolved_rotation.degrees, moving->resolved_rotation.degrees,
      shiny::nesting::cache::NfpCacheEntryKind::conservative_bbox_fallback);

  REQUIRE(cache.get(exact_key) == nullptr);
  const auto fallback_entry = cache.get(fallback_key);
  REQUIRE(fallback_entry != nullptr);
  REQUIRE(fallback_entry->accuracy ==
          shiny::nesting::cache::NfpCacheAccuracy::conservative_bbox_fallback);
  REQUIRE_FALSE(fallback_entry->polygons.empty());
  REQUIRE(fallback_entry->status != shiny::nesting::util::Status::ok);

  shiny::nesting::pack::CandidateGenerationDiagnostics second_diagnostics{};
  const auto second = shiny::nesting::pack::generate_nfp_candidate_points(
      fixture.bed1.polygon, exclusion_regions, obstacles, moving_local,
      moving->piece_geometry_revision, moving->resolved_rotation,
      CandidateStrategy::nfp_perfect, &cache, &second_diagnostics);

  REQUIRE(second.ok());
  REQUIRE(second_diagnostics.conservative_bbox_cache_hits > 0U);
  REQUIRE(second_diagnostics.exact_nfp_cache_hits == 0U);
  REQUIRE(second_diagnostics.conservative_bbox_fallbacks == 0U);
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

TEST_CASE("candidate diagnostics recorder aggregates reusable totals",
          "[packing][candidate-generation][diagnostics]") {
  shiny::nesting::pack::CandidateDiagnosticsRecorder recorder;
  shiny::nesting::pack::CandidateGenerationDiagnostics first;
  first.exact_nfp_cache_hits = 2;
  first.conservative_bbox_cache_hits = 1;
  first.exact_nfp_computations = 3;
  first.conservative_bbox_fallbacks = 4;
  first.conservative_bbox_fallback_candidates = 5;
  recorder.record(first);

  shiny::nesting::pack::CandidateGenerationDiagnostics second;
  second.exact_nfp_cache_hits = 7;
  second.conservative_bbox_fallback_candidates = 11;
  recorder.record(second);

  const auto &totals = recorder.totals();
  REQUIRE(totals.exact_nfp_cache_hits == 9U);
  REQUIRE(totals.conservative_bbox_cache_hits == 1U);
  REQUIRE(totals.exact_nfp_computations == 3U);
  REQUIRE(totals.conservative_bbox_fallbacks == 4U);
  REQUIRE(totals.conservative_bbox_candidate_points == 16U);

  recorder.reset();
  REQUIRE(recorder.totals().exact_nfp_cache_hits == 0U);
}
