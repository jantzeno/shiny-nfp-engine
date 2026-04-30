#include "geometry/queries/sanitize.hpp"

#include <cmath>

#include "geometry/operations/simplify.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"

namespace shiny::nesting::geom {
namespace {

constexpr double kZeroLengthEdgeEpsilon = 1e-12;
constexpr double kSliverRingAreaEpsilon = 1e-10;

auto count_ring_duplicates(std::span<const Point2> ring) -> std::size_t {
  if (ring.empty()) {
    return 0U;
  }

  std::size_t duplicates = 0U;
  for (std::size_t index = 1; index < ring.size(); ++index) {
    if (ring[index] == ring[index - 1U]) {
      ++duplicates;
    }
  }
  if (ring.size() > 1U && ring.front() == ring.back()) {
    ++duplicates;
  }
  return duplicates;
}

auto count_zero_length_edges(std::span<const Point2> ring) -> std::size_t {
  if (ring.size() < 2U) {
    return 0U;
  }

  std::size_t zero_edges = 0U;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    if (point_distance(ring[index], ring[(index + 1U) % ring.size()]) <=
        kZeroLengthEdgeEpsilon) {
      ++zero_edges;
    }
  }
  return zero_edges;
}

auto count_sliver_ring(std::span<const Point2> ring) -> std::size_t {
  if (ring.size() < 3U) {
    return 1U;
  }
  return std::fabs(ring_signed_area(ring)) <= kSliverRingAreaEpsilon ? 1U : 0U;
}

} // namespace

auto sanitize_polygon(const PolygonWithHoles &polygon) -> PolygonSanitization {
  PolygonSanitization report;
  for_each_ring(polygon, [&](std::span<const Point2> ring) {
    report.duplicate_vertices += count_ring_duplicates(ring);
    report.zero_length_edges += count_zero_length_edges(ring);
    report.sliver_rings += count_sliver_ring(ring);
  });

  report.polygon =
      normalize_polygon(simplify_polygon(normalize_polygon(polygon)));
  return report;
}

} // namespace shiny::nesting::geom