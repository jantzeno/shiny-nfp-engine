#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "nfp/engine.hpp"
#include "nfp/nonconvex_nfp.hpp"
#include "nfp/orbital_verifier.hpp"
#include "support/fixture_test_support.hpp"

namespace {

using shiny::nfp::NfpFeatureKind;
using shiny::nfp::NfpLoop;
using shiny::nfp::NonconvexNfpRequest;
using shiny::nfp::OrbitalVerifierStatus;
using shiny::nfp::cache::AlgorithmRevision;
using shiny::nfp::test::load_fixture_file;
using shiny::nfp::test::parse_polygon;
using shiny::nfp::test::require_fixture_metadata;
using shiny::nfp::test::require_point_equal;
using shiny::nfp::test::require_ring_equal;

auto point_less(const shiny::nfp::geom::Point2 &lhs,
                const shiny::nfp::geom::Point2 &rhs) -> bool {
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  }
  return lhs.y < rhs.y;
}

TEST_CASE("orbital verifier engine reuses cached decomposition inputs",
          "[nfp][orbital][engine]") {
  shiny::nfp::NfpEngine engine{};

  const NonconvexNfpRequest request{
      .piece_a_id = 101,
      .piece_b_id = 102,
      .piece_a = {.outer = {{0.0, 0.0},
                            {4.0, 0.0},
                            {4.0, 1.0},
                            {1.0, 1.0},
                            {1.0, 4.0},
                            {0.0, 4.0}}},
      .piece_b = {.outer = {{0.0, 0.0},
                            {3.0, 0.0},
                            {3.0, 1.0},
                            {2.0, 1.0},
                            {2.0, 3.0},
                            {0.0, 3.0}}},
      .rotation_a = {.degrees = 0.0},
      .rotation_b = {.degrees = 90.0},
      .algorithm_revision = AlgorithmRevision{9},
  };

  const auto first = engine.compute_orbital_verifier_nfp(
      request, shiny::nfp::cache::GeometryRevision{5},
      shiny::nfp::cache::GeometryRevision{7});
  REQUIRE(engine.decomposition_cache_size() == 2U);
  REQUIRE(engine.nonconvex_cache_size() == 0U);
  REQUIRE(first.status == OrbitalVerifierStatus::success);
  REQUIRE(first.result.algorithm ==
          shiny::nfp::AlgorithmKind::orbital_verifier);
  REQUIRE(first.result.normalized);
  REQUIRE_FALSE(first.result.loops.empty());

  const auto second = engine.compute_orbital_verifier_nfp(
      request, shiny::nfp::cache::GeometryRevision{5},
      shiny::nfp::cache::GeometryRevision{7});
  REQUIRE(engine.decomposition_cache_size() == 2U);
  REQUIRE(engine.nonconvex_cache_size() == 0U);
  REQUIRE(second.status == first.status);
  REQUIRE(second.result.loops.size() == first.result.loops.size());
  REQUIRE(second.traces.size() == first.traces.size());
}

auto ring_signed_area(const shiny::nfp::geom::Ring &ring) -> long double {
  long double twice_area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    twice_area += static_cast<long double>(ring[index].x) * ring[next_index].y -
                  static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return twice_area / 2.0L;
}

auto segment_less(const shiny::nfp::geom::Segment2 &lhs,
                  const shiny::nfp::geom::Segment2 &rhs) -> bool {
  const auto lhs_start = point_less(lhs.start, lhs.end) ? lhs.start : lhs.end;
  const auto lhs_end = point_less(lhs.start, lhs.end) ? lhs.end : lhs.start;
  const auto rhs_start = point_less(rhs.start, rhs.end) ? rhs.start : rhs.end;
  const auto rhs_end = point_less(rhs.start, rhs.end) ? rhs.end : rhs.start;

  if (!((lhs_start.x == rhs_start.x) && (lhs_start.y == rhs_start.y))) {
    return point_less(lhs_start, rhs_start);
  }
  return point_less(lhs_end, rhs_end);
}

void require_points_equal(
    const std::vector<shiny::nfp::geom::Point2> &actual,
    const std::vector<shiny::nfp::geom::Point2> &expected) {
  auto sorted_actual = actual;
  auto sorted_expected = expected;
  std::sort(sorted_actual.begin(), sorted_actual.end(), point_less);
  std::sort(sorted_expected.begin(), sorted_expected.end(), point_less);

  REQUIRE(sorted_actual.size() == sorted_expected.size());
  for (std::size_t index = 0; index < sorted_actual.size(); ++index) {
    require_point_equal(sorted_actual[index], sorted_expected[index]);
  }
}

void require_segments_equal(
    const std::vector<shiny::nfp::geom::Segment2> &actual,
    const std::vector<shiny::nfp::geom::Segment2> &expected) {
  auto normalized = [](std::vector<shiny::nfp::geom::Segment2> segments) {
    for (auto &segment : segments) {
      if (point_less(segment.end, segment.start)) {
        std::swap(segment.start, segment.end);
      }
    }
    std::sort(segments.begin(), segments.end(), segment_less);
    return segments;
  };

  const auto sorted_actual = normalized(actual);
  const auto sorted_expected = normalized(expected);
  REQUIRE(sorted_actual.size() == sorted_expected.size());
  for (std::size_t index = 0; index < sorted_actual.size(); ++index) {
    require_point_equal(sorted_actual[index].start,
                        sorted_expected[index].start);
    require_point_equal(sorted_actual[index].end, sorted_expected[index].end);
  }
}

void require_loops_equal(const std::vector<NfpLoop> &actual,
                         const std::vector<NfpLoop> &expected,
                         NfpFeatureKind kind) {
  std::vector<shiny::nfp::geom::Ring> actual_rings;
  std::vector<shiny::nfp::geom::Ring> expected_rings;
  for (const auto &loop : actual) {
    if (loop.kind == kind) {
      actual_rings.push_back(loop.vertices);
    }
  }
  for (const auto &loop : expected) {
    if (loop.kind == kind) {
      expected_rings.push_back(loop.vertices);
    }
  }

  REQUIRE(actual_rings.size() == expected_rings.size());
  for (std::size_t index = 0; index < actual_rings.size(); ++index) {
    REQUIRE(actual_rings[index].size() >= 3U);
    REQUIRE(expected_rings[index].size() >= 3U);
    for (std::size_t vertex_index = 1;
         vertex_index < actual_rings[index].size(); ++vertex_index) {
      REQUIRE_FALSE(point_less(actual_rings[index][vertex_index],
                               actual_rings[index].front()));
    }
    const auto actual_area = ring_signed_area(actual_rings[index]);
    const auto expected_area = ring_signed_area(expected_rings[index]);
    if (kind == NfpFeatureKind::outer_loop) {
      REQUIRE(actual_area > 0.0L);
      REQUIRE(expected_area > 0.0L);
    } else if (kind == NfpFeatureKind::hole) {
      REQUIRE(actual_area < 0.0L);
      REQUIRE(expected_area < 0.0L);
    }
    require_ring_equal(actual_rings[index], expected_rings[index]);
  }
}

auto find_graph_fixture(const shiny::nfp::test::pt::ptree &root,
                        std::string_view fixture_id)
    -> const shiny::nfp::test::pt::ptree & {
  for (const auto &fixture_node : root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    if (fixture.get<std::string>("id") == fixture_id) {
      return fixture;
    }
  }

  throw std::runtime_error("missing graph fixture for orbital verifier test");
}

auto build_request(const shiny::nfp::test::pt::ptree &inputs)
    -> NonconvexNfpRequest {
  return {
      .piece_a_id = inputs.get<std::uint32_t>("piece_a_id"),
      .piece_b_id = inputs.get<std::uint32_t>("piece_b_id"),
      .piece_a = parse_polygon(inputs.get_child("piece_a")),
      .piece_b = parse_polygon(inputs.get_child("piece_b")),
      .rotation_a = {.degrees = inputs.get<double>("rotation_a")},
      .rotation_b = {.degrees = inputs.get<double>("rotation_b")},
      .algorithm_revision =
          AlgorithmRevision{inputs.get<std::uint32_t>("algorithm_revision")},
  };
}

} // namespace

TEST_CASE("orbital verifier fixtures match graph extraction topology",
          "[nfp][orbital][fixtures]") {
  const auto orbital_root =
      load_fixture_file("nfp/orbital_verifier_pairs.json");
  REQUIRE(orbital_root.get<std::string>("algorithm") == "orbital_verifier");
  const auto graph_root = load_fixture_file("nfp/nonconvex_graph_pairs.json");

  for (const auto &fixture_node : orbital_root.get_child("fixtures")) {
    const auto &fixture = fixture_node.second;
    const auto id = fixture.get<std::string>("id");

    DYNAMIC_SECTION(id) {
      require_fixture_metadata(fixture, "orbital_verifier_pair");

      const auto graph_fixture_id =
          fixture.get<std::string>("inputs.graph_fixture_id");
      const auto &graph_fixture =
          find_graph_fixture(graph_root, graph_fixture_id);
      const auto request = build_request(graph_fixture.get_child("inputs"));

      const auto graph_result =
          shiny::nfp::compute_nonconvex_graph_nfp(request);
      const auto orbital_result =
          shiny::nfp::compute_orbital_verifier_nfp(request);

      REQUIRE(graph_result.status == shiny::nfp::ExtractionStatus::success);
      REQUIRE(orbital_result.status == OrbitalVerifierStatus::success);
      REQUIRE(orbital_result.completed);
      REQUIRE(orbital_result.result.algorithm ==
              shiny::nfp::AlgorithmKind::orbital_verifier);
      REQUIRE(orbital_result.result.normalized);
      REQUIRE_FALSE(orbital_result.touchers.empty());
      REQUIRE_FALSE(orbital_result.feasible_translations.empty());
      REQUIRE(orbital_result.traces.size() ==
              orbital_result.result.loops.size());
      for (const auto &trace : orbital_result.traces) {
        REQUIRE(trace.completed);
        REQUIRE(trace.segment_count >= 3U);
      }

      require_loops_equal(orbital_result.result.loops,
                          graph_result.result.loops,
                          NfpFeatureKind::outer_loop);
      require_loops_equal(orbital_result.result.loops,
                          graph_result.result.loops, NfpFeatureKind::hole);
      require_points_equal(orbital_result.result.perfect_fit_points,
                           graph_result.result.perfect_fit_points);
      require_segments_equal(orbital_result.result.perfect_sliding_segments,
                             graph_result.result.perfect_sliding_segments);

      REQUIRE(orbital_result.inner_loop_starts.size() ==
              fixture.get<std::size_t>("expected.inner_loop_start_count"));
      std::size_t hole_count = 0U;
      std::vector<shiny::nfp::geom::Point2> expected_inner_loop_starts;
      for (const auto &loop : orbital_result.result.loops) {
        if (loop.kind == NfpFeatureKind::hole) {
          ++hole_count;
        }
      }
      for (const auto &loop : graph_result.result.loops) {
        if (loop.kind == NfpFeatureKind::hole) {
          expected_inner_loop_starts.push_back(loop.vertices.front());
        }
      }
      REQUIRE(orbital_result.inner_loop_starts.size() == hole_count);
      require_points_equal(orbital_result.inner_loop_starts,
                           expected_inner_loop_starts);
    }
  }
}