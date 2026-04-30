#include "predicates/orientation.hpp"

#include <algorithm>
#include <cmath>

namespace shiny::nesting::pred {
namespace {

constexpr double kOrientationEpsilon = 1e-12;

[[nodiscard]] auto orientation_det(const OrientationQuery &query) -> double {
  const auto ab_x = query.b.x() - query.a.x();
  const auto ab_y = query.b.y() - query.a.y();
  const auto ac_x = query.c.x() - query.a.x();
  const auto ac_y = query.c.y() - query.a.y();
  return ab_x * ac_y - ab_y * ac_x;
}

[[nodiscard]] auto orientation_tolerance(const OrientationQuery &query)
    -> double {
  const auto ab_x = std::abs(query.b.x() - query.a.x());
  const auto ab_y = std::abs(query.b.y() - query.a.y());
  const auto ac_x = std::abs(query.c.x() - query.a.x());
  const auto ac_y = std::abs(query.c.y() - query.a.y());
  const auto scale = std::max({1.0, ab_x, ab_y, ac_x, ac_y});
  return kOrientationEpsilon * scale * scale;
}

} // namespace

auto orient(const OrientationQuery &query) -> Orientation {
  const auto det = orientation_det(query);
  const auto tolerance = orientation_tolerance(query);
  if (det > tolerance) {
    return Orientation::left_turn;
  }
  if (det < -tolerance) {
    return Orientation::right_turn;
  }

  return Orientation::collinear;
}

} // namespace shiny::nesting::pred