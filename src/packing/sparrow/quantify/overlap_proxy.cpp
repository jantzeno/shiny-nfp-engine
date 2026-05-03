#include "packing/sparrow/quantify/overlap_proxy.hpp"

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/polygon.hpp"

namespace shiny::nesting::pack::sparrow::quantify {

namespace {

constexpr double kOverlapAreaEpsilon = 1e-9;

} // namespace

auto quantify_overlap(const adapters::PortPolygon &lhs,
                      const adapters::PortPolygon &rhs)
    -> util::StatusOr<OverlapProxyResult> {
  const auto intersection_or = geom::try_intersection_polygons(
      adapters::to_engine_polygon(lhs), adapters::to_engine_polygon(rhs));
  if (!intersection_or.ok()) {
    return intersection_or.status();
  }

  const double overlap_area = geom::polygon_area_sum(intersection_or.value());
  return OverlapProxyResult{
      .overlap_area = overlap_area,
      .has_overlap = overlap_area > kOverlapAreaEpsilon,
  };
}

} // namespace shiny::nesting::pack::sparrow::quantify