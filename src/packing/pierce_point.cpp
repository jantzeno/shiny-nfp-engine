#include "packing/pierce_point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry/polygon.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kLeadRadiusFactor = 0.25;
constexpr double kLeadRadiusCap = 0.5;
constexpr double kLeadRadiusFloor = 1e-6;

[[nodiscard]] auto normalized_vector(const geom::Point2 &from,
                                     const geom::Point2 &to) -> geom::Vector2 {
  const auto dx = to.x() - from.x();
  const auto dy = to.y() - from.y();
  const auto length = std::sqrt(dx * dx + dy * dy);
  if (length <= kLeadRadiusFloor) {
    return {};
  }
  return {dx / length, dy / length};
}

[[nodiscard]] auto add_scaled(const geom::Point2 &point,
                              const geom::Vector2 &vector, double scale)
    -> geom::Point2 {
  return {point.x() + vector.x() * scale, point.y() + vector.y() * scale};
}

[[nodiscard]] auto midpoint(const geom::Point2 &lhs, const geom::Point2 &rhs)
    -> geom::Point2 {
  return {(lhs.x() + rhs.x()) / 2.0, (lhs.y() + rhs.y()) / 2.0};
}

[[nodiscard]] auto right_normal(const geom::Vector2 &tangent) -> geom::Vector2 {
  return {tangent.y(), -tangent.x()};
}

[[nodiscard]] auto tangent_turn_penalty(const geom::Vector2 &incoming_tangent,
                                        const geom::Vector2 &outgoing_tangent)
    -> double {
  const auto dot = std::clamp(incoming_tangent.x() * outgoing_tangent.x() +
                                  incoming_tangent.y() * outgoing_tangent.y(),
                              -1.0, 1.0);
  return 1.0 - dot;
}

struct PierceCandidate {
  geom::Point2 point{};
  geom::Vector2 incoming_tangent{};
  geom::Vector2 outgoing_tangent{};
  double radius{0.0};
  double concavity_penalty{0.0};
  std::size_t stable_index{0};
};

// `right_normal` of a CCW-wound ring's edge points outward (away from
// the interior). For CW rings the same formula points inward — so the
// lead arc would be drawn into the part. Compute the ring's signed
// area sign once and use it as a multiplier when constructing arcs.
[[nodiscard]] auto outward_sign(std::span<const geom::Point2> ring) -> double {
  const double signed_area = geom::ring_signed_area(ring);
  return signed_area >= 0.0 ? 1.0 : -1.0;
}

[[nodiscard]] auto build_lead_in(const geom::Point2 &pierce_point,
                                 const geom::Vector2 &outgoing_tangent,
                                 double radius, double outward)
    -> CutContourOrder::LeadArc {
  if (radius <= kLeadRadiusFloor ||
      (outgoing_tangent.x() == 0.0 && outgoing_tangent.y() == 0.0)) {
    return {};
  }

  const auto base_normal = right_normal(outgoing_tangent);
  const geom::Vector2 normal(base_normal.x() * outward,
                             base_normal.y() * outward);
  const auto center = add_scaled(pierce_point, normal, radius);
  return {
      .enabled = true,
      .start = add_scaled(center, outgoing_tangent, -radius),
      .end = pierce_point,
      .center = center,
      .clockwise = outward >= 0.0,
  };
}

[[nodiscard]] auto build_lead_out(const geom::Point2 &pierce_point,
                                  const geom::Vector2 &incoming_tangent,
                                  double radius, double outward)
    -> CutContourOrder::LeadArc {
  if (radius <= kLeadRadiusFloor ||
      (incoming_tangent.x() == 0.0 && incoming_tangent.y() == 0.0)) {
    return {};
  }

  const auto base_normal = right_normal(incoming_tangent);
  const geom::Vector2 normal(base_normal.x() * outward,
                             base_normal.y() * outward);
  const auto center = add_scaled(pierce_point, normal, radius);
  return {
      .enabled = true,
      .start = pierce_point,
      .end = add_scaled(center, incoming_tangent, radius),
      .center = center,
      .clockwise = outward >= 0.0,
  };
}

} // namespace

// Pierce point + lead-in/lead-out arc selection (Plan §12.3).
//
// Strategy: evaluate both vertices and edge midpoints, then pick the
// candidate that minimises `travel_distance + local_turn_penalty`.
// Midpoints have zero corner penalty and tangents aligned with the
// host edge; vertices use the incoming/outgoing ring tangents. Both
// lead arcs are tangent to the cut path so the cutter enters/exits
// without a corner. Radius is capped at 0.5 units and floored at 25%
// of the shorter attached geometry: the shorter adjacent edge for
// vertices, and half the host edge for midpoints.
//
// `right_normal` returns the outward normal assuming the ring is wound
// counter-clockwise (the convention used elsewhere in the engine).
// If a ring is wound the other way, the lead arc will be drawn into
// the part instead of the scrap — relies on upstream ring orientation
// being consistent.
auto select_pierce_plan(const CutContour &contour,
                        const geom::Point2 &previous_exit) -> PiercePlan {
  PiercePlan plan{};
  if (contour.ring.empty()) {
    plan.pierce_point = previous_exit;
    plan.exit_point = previous_exit;
    return plan;
  }

  if (contour.ring.size() < 3U) {
    const auto best =
        std::min_element(contour.ring.begin(), contour.ring.end(),
                         [&](const auto &lhs, const auto &rhs) {
                           return geom::squared_distance(lhs, previous_exit) <
                                  geom::squared_distance(rhs, previous_exit);
                         });
    plan.pierce_point = *best;
    plan.exit_point = plan.pierce_point;
    return plan;
  }

  const auto ring_span =
      std::span<const geom::Point2>(contour.ring.data(), contour.ring.size());
  const auto bounds = geom::compute_bounds(ring_span);
  const auto concavity_weight =
      std::hypot(geom::box_width(bounds), geom::box_height(bounds)) * 0.1;

  std::vector<PierceCandidate> candidates;
  candidates.reserve(contour.ring.size() * 2U);
  std::size_t stable_index = 0;
  for (std::size_t index = 0; index < contour.ring.size(); ++index) {
    const auto previous_index =
        (index + contour.ring.size() - 1U) % contour.ring.size();
    const auto next_index = (index + 1U) % contour.ring.size();
    const auto incoming_tangent =
        normalized_vector(contour.ring[previous_index], contour.ring[index]);
    const auto outgoing_tangent =
        normalized_vector(contour.ring[index], contour.ring[next_index]);
    const auto incoming_length =
        geom::point_distance(contour.ring[previous_index], contour.ring[index]);
    const auto outgoing_length =
        geom::point_distance(contour.ring[index], contour.ring[next_index]);

    candidates.push_back({
        .point = contour.ring[index],
        .incoming_tangent = incoming_tangent,
        .outgoing_tangent = outgoing_tangent,
        .radius = std::min(kLeadRadiusCap,
                           std::min(incoming_length, outgoing_length) *
                               kLeadRadiusFactor),
        .concavity_penalty =
            tangent_turn_penalty(incoming_tangent, outgoing_tangent),
        .stable_index = stable_index++,
    });

    if (outgoing_length <= kLeadRadiusFloor) {
      continue;
    }
    candidates.push_back({
        .point = midpoint(contour.ring[index], contour.ring[next_index]),
        .incoming_tangent = outgoing_tangent,
        .outgoing_tangent = outgoing_tangent,
        .radius = std::min(kLeadRadiusCap,
                           (outgoing_length * 0.5) * kLeadRadiusFactor),
        .concavity_penalty = 0.0,
        .stable_index = stable_index++,
    });
  }

  std::size_t best_index = 0;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto score =
        geom::point_distance(previous_exit, candidates[index].point) +
        concavity_weight * candidates[index].concavity_penalty;
    if (score + kLeadRadiusFloor < best_score ||
        (std::abs(score - best_score) <= kLeadRadiusFloor &&
         candidates[index].stable_index <
             candidates[best_index].stable_index)) {
      best_index = index;
      best_score = score;
    }
  }

  const auto &best = candidates[best_index];
  plan.pierce_point = best.point;
  plan.exit_point = plan.pierce_point;

  const double outward = outward_sign(ring_span);
  plan.lead_in = build_lead_in(plan.pierce_point, best.outgoing_tangent,
                               best.radius, outward);
  plan.lead_out = build_lead_out(plan.pierce_point, best.incoming_tangent,
                                 best.radius, outward);
  if (plan.lead_out.enabled) {
    plan.exit_point = plan.lead_out.end;
  }
  return plan;
}

auto select_pierce_point(const CutContour &contour,
                         const geom::Point2 &previous_exit) -> geom::Point2 {
  return select_pierce_plan(contour, previous_exit).pierce_point;
}

} // namespace shiny::nesting::pack
