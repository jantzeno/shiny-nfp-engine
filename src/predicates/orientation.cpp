#include "predicates/orientation.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace shiny::nfp::pred {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

[[nodiscard]] auto to_cgal_point(const geom::Point2 &point) -> Kernel::Point_2 {
  return {point.x, point.y};
}

} // namespace

auto orient(const OrientationQuery &query) -> Orientation {
  const auto cgal_orientation = CGAL::orientation(
      to_cgal_point(query.a), to_cgal_point(query.b), to_cgal_point(query.c));

  switch (cgal_orientation) {
  case CGAL::LEFT_TURN:
    return Orientation::left_turn;
  case CGAL::RIGHT_TURN:
    return Orientation::right_turn;
  case CGAL::COLLINEAR:
    return Orientation::collinear;
  }

  return Orientation::collinear;
}

} // namespace shiny::nfp::pred