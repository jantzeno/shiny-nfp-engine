#include "polygon_ops/simplify.hpp"

#include <algorithm>
#include <vector>

#include "geometry/normalize.hpp"
#include "predicates/orientation.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nfp::poly {
namespace {

[[nodiscard]] auto simplify_normalized_ring(const geom::Ring &ring)
    -> geom::Ring {
  if (ring.size() < 4U) {
    return ring;
  }

  geom::Ring simplified = ring;

  while (simplified.size() >= 4U) {
    std::vector<bool> remove_vertex(simplified.size(), false);
    std::size_t removed_count = 0;

    for (std::size_t index = 0; index < simplified.size(); ++index) {
      const auto prev_index =
          (index + simplified.size() - 1U) % simplified.size();
      const auto next_index = (index + 1U) % simplified.size();

      const auto orientation = pred::orient(
          {simplified[prev_index], simplified[index], simplified[next_index]});
      if (orientation != pred::Orientation::collinear) {
        continue;
      }

      const auto on_segment = pred::locate_point_on_segment(
          simplified[index],
          geom::Segment2{simplified[prev_index], simplified[next_index]});
      if (on_segment.relation == pred::BoundaryRelation::off_boundary) {
        continue;
      }

      remove_vertex[index] = true;
      ++removed_count;
    }

    if (removed_count == 0U || simplified.size() - removed_count < 3U) {
      break;
    }

    geom::Ring next_ring;
    next_ring.reserve(simplified.size() - removed_count);
    for (std::size_t index = 0; index < simplified.size(); ++index) {
      if (!remove_vertex[index]) {
        next_ring.push_back(simplified[index]);
      }
    }

    simplified = std::move(next_ring);
  }

  return simplified;
}

} // namespace

auto simplify_collinear_ring(std::span<const geom::Point2> ring) -> geom::Ring {
  const auto normalized = geom::normalize_polygon(
      geom::Polygon{.outer = geom::Ring(ring.begin(), ring.end())});
  const auto simplified = simplify_normalized_ring(normalized.outer);
  return geom::normalize_polygon(geom::Polygon{.outer = simplified}).outer;
}

auto simplify_polygon(const geom::Polygon &polygon) -> geom::Polygon {
  const auto normalized = geom::normalize_polygon(polygon);
  return geom::normalize_polygon(
      geom::Polygon{.outer = simplify_normalized_ring(normalized.outer)});
}

auto simplify_polygon(const geom::PolygonWithHoles &polygon)
    -> geom::PolygonWithHoles {
  const auto normalized = geom::normalize_polygon(polygon);

  geom::PolygonWithHoles simplified{};
  simplified.outer = simplify_normalized_ring(normalized.outer);
  simplified.holes.reserve(normalized.holes.size());

  for (const auto &hole : normalized.holes) {
    simplified.holes.push_back(simplify_normalized_ring(hole));
  }

  return geom::normalize_polygon(simplified);
}

} // namespace shiny::nfp::poly