#include "packing/common_edge.hpp"

#include <algorithm>
#include <cmath>
#include <compare>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "geometry/polygon.hpp"
#include "geometry/rtree_index.hpp"
#include "predicates/orientation.hpp"
#include "predicates/point_location.hpp"

namespace shiny::nesting::pack {
namespace {

constexpr double kCoordinateScale = 1000000.0;
constexpr double kSegmentEpsilon = 1e-9;

[[nodiscard]] auto point_on_segment(const geom::Point2 &point,
                                    const geom::Segment2 &segment) -> bool {
  return pred::locate_point_on_segment(point, segment).relation !=
         pred::BoundaryRelation::off_boundary;
}

[[nodiscard]] auto covers(const geom::Segment2 &candidate,
                          const geom::Segment2 &covered) -> bool {
  return point_on_segment(covered.start, candidate) &&
         point_on_segment(covered.end, candidate);
}

[[nodiscard]] auto coordinate_key(const double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * kCoordinateScale));
}

[[nodiscard]] auto segment_bounds(const geom::Segment2 &segment) -> geom::Box2 {
  return {geom::Point2{std::min(segment.start.x(), segment.end.x()),
                       std::min(segment.start.y(), segment.end.y())},
          geom::Point2{std::max(segment.start.x(), segment.end.x()),
                       std::max(segment.start.y(), segment.end.y())}};
}

[[nodiscard]] auto are_collinear(const geom::Segment2 &lhs,
                                 const geom::Segment2 &rhs) -> bool {
  return pred::orient({lhs.start, lhs.end, rhs.start}) ==
             pred::Orientation::collinear &&
         pred::orient({lhs.start, lhs.end, rhs.end}) ==
             pred::Orientation::collinear;
}

[[nodiscard]] auto canonical_direction(const geom::Segment2 &segment)
    -> std::optional<geom::Vector2> {
  const auto dx = segment.end.x() - segment.start.x();
  const auto dy = segment.end.y() - segment.start.y();
  const auto length = std::hypot(dx, dy);
  if (length <= kSegmentEpsilon) {
    return std::nullopt;
  }

  geom::Vector2 direction{dx / length, dy / length};
  if (direction.x() < -kSegmentEpsilon ||
      (std::abs(direction.x()) <= kSegmentEpsilon && direction.y() < 0.0)) {
    direction.set_x(-direction.x());
    direction.set_y(-direction.y());
  }
  return direction;
}

[[nodiscard]] auto project_onto_line(const geom::Point2 &point,
                                     const geom::Point2 &origin,
                                     const geom::Vector2 &direction) -> double {
  return (point.x() - origin.x()) * direction.x() +
         (point.y() - origin.y()) * direction.y();
}

struct LineKey {
  std::uint32_t bin_id{0};
  std::int64_t direction_x{0};
  std::int64_t direction_y{0};
  std::int64_t offset{0};

  auto operator<=>(const LineKey &) const = default;
};

[[nodiscard]] auto line_key(const CutSegment &segment)
    -> std::optional<LineKey> {
  const auto direction = canonical_direction(segment.segment);
  if (!direction.has_value()) {
    return std::nullopt;
  }

  const geom::Vector2 normal{-direction->y(), direction->x()};
  const auto offset = normal.x() * segment.segment.start.x() +
                      normal.y() * segment.segment.start.y();
  return LineKey{
      .bin_id = segment.bin_id,
      .direction_x = coordinate_key(direction->x()),
      .direction_y = coordinate_key(direction->y()),
      .offset = coordinate_key(offset),
  };
}

struct IndexedSegment {
  CutSegment segment{};
  std::size_t original_index{0};
};

struct ExactLineGroup {
  IndexedSegment reference{};
  std::vector<IndexedSegment> segments{};
};

struct MergedInterval {
  double start{0.0};
  double end{0.0};
  geom::Point2 start_point{};
  geom::Point2 end_point{};
  std::size_t representative_index{0};
};

[[nodiscard]] auto merge_collinear_segments(std::vector<CutSegment> segments)
    -> std::vector<IndexedSegment> {
  std::map<LineKey, std::vector<ExactLineGroup>> grouped_segments;
  std::vector<IndexedSegment> merged_segments;
  merged_segments.reserve(segments.size());

  for (std::size_t index = 0; index < segments.size(); ++index) {
    IndexedSegment indexed{.segment = segments[index], .original_index = index};
    const auto key = line_key(indexed.segment);
    if (!key.has_value()) {
      merged_segments.push_back(indexed);
      continue;
    }

    auto &groups = grouped_segments[*key];
    const auto group_it = std::find_if(
        groups.begin(), groups.end(), [&](const ExactLineGroup &group) {
          return are_collinear(group.reference.segment.segment,
                               indexed.segment.segment);
        });
    if (group_it == groups.end()) {
      groups.push_back({
          .reference = indexed,
          .segments = {indexed},
      });
      continue;
    }
    group_it->segments.push_back(indexed);
  }

  for (const auto &[unused_key, groups] : grouped_segments) {
    for (const auto &group : groups) {
      const auto direction =
          canonical_direction(group.reference.segment.segment);
      if (!direction.has_value()) {
        merged_segments.push_back(group.reference);
        continue;
      }

      const auto origin = group.reference.segment.segment.start;
      std::vector<MergedInterval> intervals;
      intervals.reserve(group.segments.size());
      for (const auto &entry : group.segments) {
        auto start =
            project_onto_line(entry.segment.segment.start, origin, *direction);
        auto end =
            project_onto_line(entry.segment.segment.end, origin, *direction);
        auto start_point = entry.segment.segment.start;
        auto end_point = entry.segment.segment.end;
        if (end < start) {
          std::swap(start, end);
          std::swap(start_point, end_point);
        }

        intervals.push_back({
            .start = start,
            .end = end,
            .start_point = start_point,
            .end_point = end_point,
            .representative_index = entry.original_index,
        });
      }

      std::sort(intervals.begin(), intervals.end(),
                [](const MergedInterval &lhs, const MergedInterval &rhs) {
                  if (lhs.start != rhs.start) {
                    return lhs.start < rhs.start;
                  }
                  if (lhs.end != rhs.end) {
                    return lhs.end < rhs.end;
                  }
                  return lhs.representative_index < rhs.representative_index;
                });

      auto emit_interval = [&](const MergedInterval &interval) {
        auto segment = segments[interval.representative_index];
        segment.segment =
            geom::Segment2{interval.start_point, interval.end_point};
        merged_segments.push_back({
            .segment = segment,
            .original_index = interval.representative_index,
        });
      };

      MergedInterval current = intervals.front();
      for (std::size_t index = 1; index < intervals.size(); ++index) {
        if (intervals[index].start <= current.end + kSegmentEpsilon) {
          if (intervals[index].end > current.end + kSegmentEpsilon) {
            current.end = intervals[index].end;
            current.end_point = intervals[index].end_point;
          }
          current.representative_index =
              std::min(current.representative_index,
                       intervals[index].representative_index);
          continue;
        }

        emit_interval(current);
        current = intervals[index];
      }
      emit_interval(current);
    }
  }

  std::sort(merged_segments.begin(), merged_segments.end(),
            [](const IndexedSegment &lhs, const IndexedSegment &rhs) {
              return lhs.original_index < rhs.original_index;
            });
  return merged_segments;
}

} // namespace

// Common-edge dedup (Plan §12.2): when two pieces share a boundary,
// the cutter only needs to traverse it once. This pass first groups
// same-bin collinear segments, merges overlapping or abutting coverage
// into maximal intervals, then uses an R-tree broad phase to drop any
// remaining fully covered duplicates.
auto detect_common_edges(std::vector<CutSegment> segments)
    -> std::vector<CutSegment> {
  auto merged_segments = merge_collinear_segments(std::move(segments));
  geom::RTreeIndex segment_index;
  std::vector<geom::Box2> bounds;
  bounds.reserve(merged_segments.size());
  for (std::size_t index = 0; index < merged_segments.size(); ++index) {
    bounds.push_back(segment_bounds(merged_segments[index].segment.segment));
    segment_index.insert(static_cast<std::uint32_t>(index), bounds.back());
  }

  std::vector<CutSegment> filtered;
  filtered.reserve(merged_segments.size());
  for (std::size_t index = 0; index < merged_segments.size(); ++index) {
    bool redundant = false;
    for (const auto candidate_id : segment_index.query(bounds[index])) {
      const auto other = static_cast<std::size_t>(candidate_id);
      if (index == other || merged_segments[index].segment.bin_id !=
                                merged_segments[other].segment.bin_id) {
        continue;
      }

      if (covers(merged_segments[other].segment.segment,
                 merged_segments[index].segment.segment)) {
        const auto same_start =
            point_on_segment(merged_segments[other].segment.segment.start,
                             merged_segments[index].segment.segment);
        const auto same_end =
            point_on_segment(merged_segments[other].segment.segment.end,
                             merged_segments[index].segment.segment);
        if (same_start && same_end) {
          redundant = merged_segments[other].original_index <
                      merged_segments[index].original_index;
          if (redundant) {
            break;
          }
          continue;
        }
        redundant = true;
        break;
      }
    }
    if (!redundant) {
      filtered.push_back(merged_segments[index].segment);
    }
  }
  return filtered;
}

} // namespace shiny::nesting::pack
