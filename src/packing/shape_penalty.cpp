#include "packing/shape_penalty.hpp"

#include <cmath>

#include "geometry/polygon.hpp"
#include "polygon_ops/convex_hull.hpp"

namespace shiny::nesting::pack {

// Pair penalty scaling factor: λ_ab = (hull_area(a) · hull_area(b))^(1/4)
//   (= sqrt(sqrt(A) · sqrt(B)) as written, identical algebra).
// Multiplied into the actual overlap area, this scales the cost of a
// collision by the geometric mean of the two pieces' "size scale".
// Penalises a few large-piece collisions more than many small-piece
// collisions of the same total area, which discourages the separator
// from accumulating swarms of tiny overlaps in lieu of resolving big
// ones. (Sparrow `overlap/penalty.rs`.)
auto shape_penalty(const geom::PolygonWithHoles &lhs,
                   const geom::PolygonWithHoles &rhs) -> double {
  const auto lhs_hull_area =
      std::max(0.0, geom::polygon_area(poly::compute_convex_hull(lhs)));
  const auto rhs_hull_area =
      std::max(0.0, geom::polygon_area(poly::compute_convex_hull(rhs)));
  return std::sqrt(std::sqrt(lhs_hull_area) * std::sqrt(rhs_hull_area));
}

} // namespace shiny::nesting::pack
