#include "packing/layout.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "packing/common_edge.hpp"
#include "packing/cutting_sequence.hpp"
#include "packing/pierce_point.hpp"
#include "predicates/orientation.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kLengthEpsilon = 1e-9;
constexpr double kCoordinateScale = 1000000000.0;

struct CutSegmentRecord {
  CutSegment segment{};
  std::size_t ordinal{0};
};

[[nodiscard]] auto coordinate_key(double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * kCoordinateScale));
}

[[nodiscard]] auto canonical_point_before(const geom::Point2 &lhs,
                                          const geom::Point2 &rhs) -> bool {
  const auto lhs_x = coordinate_key(lhs.x());
  const auto rhs_x = coordinate_key(rhs.x());
  if (lhs_x != rhs_x) {
    return lhs_x < rhs_x;
  }
  return coordinate_key(lhs.y()) < coordinate_key(rhs.y());
}

[[nodiscard]] auto ordered_endpoints(const geom::Segment2 &segment)
    -> std::pair<geom::Point2, geom::Point2> {
  if (canonical_point_before(segment.end, segment.start)) {
    return {segment.end, segment.start};
  }
  return {segment.start, segment.end};
}

[[nodiscard]] auto segment_length(const geom::Segment2 &segment) -> double {
  const auto dx = segment.end.x() - segment.start.x();
  const auto dy = segment.end.y() - segment.start.y();
  return std::sqrt(dx * dx + dy * dy);
}

[[nodiscard]] auto point_on_segment(const geom::Point2 &point,
                                    const geom::Segment2 &segment) -> bool {
  return pred::locate_point_on_segment(point, segment).relation !=
         pred::BoundaryRelation::off_boundary;
}

[[nodiscard]] auto are_collinear(const geom::Segment2 &lhs,
                                 const geom::Segment2 &rhs) -> bool {
  return pred::orient({lhs.start, lhs.end, rhs.start}) ==
             pred::Orientation::collinear &&
         pred::orient({lhs.start, lhs.end, rhs.end}) ==
             pred::Orientation::collinear;
}

[[nodiscard]] auto segment_projection_axis(const geom::Segment2 &segment)
    -> bool {
  return std::fabs(segment.end.x() - segment.start.x()) >=
         std::fabs(segment.end.y() - segment.start.y());
}

[[nodiscard]] auto project_point(const geom::Point2 &point, bool use_x_axis)
    -> double {
  return use_x_axis ? point.x() : point.y();
}

[[nodiscard]] auto segment_covers_segment(const geom::Segment2 &candidate,
                                          const geom::Segment2 &covered)
    -> bool {
  return point_on_segment(covered.start, candidate) &&
         point_on_segment(covered.end, candidate);
}

[[nodiscard]] auto
segments_cover_segment(const geom::Segment2 &covered,
                       std::span<const geom::Segment2> candidates) -> bool {
  if (covered.start.x() == covered.end.x() &&
      covered.start.y() == covered.end.y()) {
    return false;
  }

  const auto use_x_axis = segment_projection_axis(covered);
  const auto covered_start = project_point(covered.start, use_x_axis);
  const auto covered_end = project_point(covered.end, use_x_axis);
  const auto covered_min = std::min(covered_start, covered_end);
  const auto covered_max = std::max(covered_start, covered_end);

  std::vector<std::pair<double, double>> intervals;
  intervals.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    if (!are_collinear(candidate, covered)) {
      continue;
    }

    const auto candidate_start = project_point(candidate.start, use_x_axis);
    const auto candidate_end = project_point(candidate.end, use_x_axis);
    const auto interval_start =
        std::max(covered_min, std::min(candidate_start, candidate_end));
    const auto interval_end =
        std::min(covered_max, std::max(candidate_start, candidate_end));
    if (interval_end + kLengthEpsilon < interval_start) {
      continue;
    }

    intervals.emplace_back(interval_start, interval_end);
  }

  if (intervals.empty()) {
    return false;
  }

  std::sort(intervals.begin(), intervals.end(),
            [](const auto &lhs, const auto &rhs) {
              if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
              }
              return lhs.second < rhs.second;
            });

  auto covered_until = intervals.front().second;
  if (intervals.front().first > covered_min + kLengthEpsilon) {
    return false;
  }

  for (std::size_t index = 1; index < intervals.size(); ++index) {
    if (intervals[index].first > covered_until + kLengthEpsilon) {
      return false;
    }
    covered_until = std::max(covered_until, intervals[index].second);
    if (covered_until >= covered_max - kLengthEpsilon) {
      return true;
    }
  }

  return covered_until >= covered_max - kLengthEpsilon;
}

[[nodiscard]] auto cut_segment_less(const CutSegmentRecord &lhs,
                                    const CutSegmentRecord &rhs) -> bool {
  if (lhs.segment.bin_id != rhs.segment.bin_id) {
    return lhs.segment.bin_id < rhs.segment.bin_id;
  }
  if (lhs.segment.piece_id != rhs.segment.piece_id) {
    return lhs.segment.piece_id < rhs.segment.piece_id;
  }
  if (lhs.segment.from_hole != rhs.segment.from_hole) {
    return !lhs.segment.from_hole && rhs.segment.from_hole;
  }

  const auto [lhs_first, lhs_second] = ordered_endpoints(lhs.segment.segment);
  const auto [rhs_first, rhs_second] = ordered_endpoints(rhs.segment.segment);
  if (canonical_point_before(lhs_first, rhs_first)) {
    return true;
  }
  if (canonical_point_before(rhs_first, lhs_first)) {
    return false;
  }
  if (canonical_point_before(lhs_second, rhs_second)) {
    return true;
  }
  if (canonical_point_before(rhs_second, lhs_second)) {
    return false;
  }
  return lhs.ordinal < rhs.ordinal;
}

void append_ring_segments(std::vector<CutSegmentRecord> &segments,
                          std::uint32_t bin_id, std::uint32_t piece_id,
                          const geom::Ring &ring, bool from_hole,
                          std::size_t &ordinal) {
  if (ring.size() < 2U) {
    return;
  }

  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    const geom::Segment2 segment{ring[index], ring[next_index]};
    if (segment.start.x() == segment.end.x() &&
        segment.start.y() == segment.end.y()) {
      continue;
    }

    segments.push_back({
        .segment = {.bin_id = bin_id,
                    .piece_id = piece_id,
                    .segment = segment,
                    .from_hole = from_hole},
        .ordinal = ordinal++,
    });
  }
}

[[nodiscard]] auto collect_raw_segments(const Layout &layout)
    -> std::vector<CutSegmentRecord> {
  std::vector<CutSegmentRecord> segments;
  std::size_t ordinal = 0;

  for (const auto &bin : layout.bins) {
    for (const auto &piece : bin.placements) {
      append_ring_segments(segments, bin.bin_id, piece.placement.piece_id,
                           piece.polygon.outer(), false, ordinal);
      for (const auto &hole : piece.polygon.holes()) {
        append_ring_segments(segments, bin.bin_id, piece.placement.piece_id,
                             hole, true, ordinal);
      }
    }
  }

  return segments;
}

[[nodiscard]] auto collect_boundary_segments(const LayoutBin &bin)
    -> std::vector<geom::Segment2> {
  std::vector<geom::Segment2> segments;

  const auto append_ring = [&segments](const geom::Ring &ring) {
    if (ring.size() < 2U) {
      return;
    }
    for (std::size_t index = 0; index < ring.size(); ++index) {
      const auto next_index = (index + 1U) % ring.size();
      const geom::Segment2 segment{ring[index], ring[next_index]};
      if (segment.start.x() == segment.end.x() &&
          segment.start.y() == segment.end.y()) {
        continue;
      }
      segments.push_back(segment);
    }
  };

  for (const auto &region : bin.occupied.regions) {
    append_ring(region.outer());
    for (const auto &hole : region.holes()) {
      append_ring(hole);
    }
  }

  return segments;
}

[[nodiscard]] auto
boundary_covers_segment(const geom::Segment2 &segment,
                        std::span<const geom::Segment2> boundaries,
                        const LaserCutOptimizationConfig &config) -> bool {
  if (!config.require_exact_collinearity) {
    return segments_cover_segment(segment, boundaries);
  }

  return std::any_of(boundaries.begin(), boundaries.end(),
                     [&segment](const auto &boundary) {
                       return segment_covers_segment(boundary, segment);
                     });
}

[[nodiscard]] auto
removable_segment(const CutSegmentRecord &candidate,
                  std::span<const CutSegmentRecord> raw_segments,
                  std::span<const geom::Segment2> boundary_segments,
                  const LaserCutOptimizationConfig &config) -> bool {
  if (config.preserve_visible_notches &&
      boundary_covers_segment(candidate.segment.segment, boundary_segments,
                              config)) {
    return false;
  }

  const auto candidate_length = segment_length(candidate.segment.segment);
  std::vector<geom::Segment2> relaxed_cover_segments;

  for (const auto &other : raw_segments) {
    if (&other == &candidate ||
        other.segment.bin_id != candidate.segment.bin_id) {
      continue;
    }

    if (!config.require_exact_collinearity) {
      relaxed_cover_segments.push_back(other.segment.segment);
    }

    if (!segment_covers_segment(other.segment.segment,
                                candidate.segment.segment)) {
      continue;
    }

    const auto other_length = segment_length(other.segment.segment);
    if (other_length > candidate_length + kLengthEpsilon) {
      return true;
    }

    if (std::fabs(other_length - candidate_length) <= kLengthEpsilon &&
        cut_segment_less(other, candidate)) {
      return true;
    }
  }

  if (!config.require_exact_collinearity &&
      segments_cover_segment(candidate.segment.segment,
                             relaxed_cover_segments)) {
    return true;
  }

  return false;
}

} // namespace

auto build_cut_plan(const Layout &layout,
                    const LaserCutOptimizationConfig &config) -> CutPlan {
  auto raw_segments = collect_raw_segments(layout);
  std::sort(raw_segments.begin(), raw_segments.end(), cut_segment_less);

  CutPlan plan{};
  for (const auto &entry : raw_segments) {
    plan.raw_cut_length += segment_length(entry.segment.segment);
  }

  if (config.mode == SharedCutOptimizationMode::off) {
    for (const auto &entry : raw_segments) {
      plan.segments.push_back(entry.segment);
    }
    plan.total_cut_length = plan.raw_cut_length;
  } else {
    for (const auto &bin : layout.bins) {
      const auto boundaries = collect_boundary_segments(bin);
      std::vector<CutSegmentRecord> bin_segments;
      for (const auto &entry : raw_segments) {
        if (entry.segment.bin_id == bin.bin_id) {
          bin_segments.push_back(entry);
        }
      }

      for (const auto &entry : bin_segments) {
        if (!removable_segment(entry, bin_segments, boundaries, config)) {
          plan.segments.push_back(entry.segment);
          plan.total_cut_length += segment_length(entry.segment.segment);
        }
      }
    }
  }

  if (config.mode != SharedCutOptimizationMode::off) {
    plan.segments = detect_common_edges(std::move(plan.segments));
  }
  plan.total_cut_length = 0.0;
  for (const auto &segment : plan.segments) {
    plan.total_cut_length += segment_length(segment.segment);
  }
  plan.removed_cut_length = plan.raw_cut_length - plan.total_cut_length;

  const auto sequence = build_cutting_sequence(layout);
  geom::Point2 previous_exit{};
  for (std::size_t index = 0; index < sequence.size(); ++index) {
    const auto pierce_plan = select_pierce_plan(sequence[index], previous_exit);
    plan.contour_order.push_back({
        .bin_id = sequence[index].bin_id,
        .piece_id = sequence[index].piece_id,
        .from_hole = sequence[index].from_hole,
        .order_index = index,
        .pierce_point = pierce_plan.pierce_point,
        .lead_in = pierce_plan.lead_in,
        .lead_out = pierce_plan.lead_out,
    });
    previous_exit = pierce_plan.exit_point;
  }

  return plan;
}

} // namespace shiny::nesting::pack
