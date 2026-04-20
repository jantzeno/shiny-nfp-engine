#include "predicates/classify.hpp"

#include "geometry/detail/point_compare.hpp"

namespace shiny::nesting::pred {

auto lexicographic_min_vertex_index(std::span<const geom::Point2> ring)
    -> std::size_t {
  if (ring.empty()) {
    return 0;
  }

  std::size_t min_index = 0;
  for (std::size_t index = 1; index < ring.size(); ++index) {
    if (detail::point_less(ring[index], ring[min_index])) {
      min_index = index;
    }
  }

  return min_index;
}

} // namespace shiny::nesting::pred