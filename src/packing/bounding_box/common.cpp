#include "packing/bounding_box/common.hpp"

#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

#include "geometry/queries/normalize.hpp"
#include "packing/common.hpp"

namespace shiny::nesting::pack {

auto point_for_start_corner(const geom::Point2 &point,
                            const geom::Box2 &container,
                            const place::PlacementStartCorner start_corner)
    -> geom::Point2 {
  const bool on_right =
      start_corner == place::PlacementStartCorner::bottom_right ||
      start_corner == place::PlacementStartCorner::top_right;
  const bool on_top = start_corner == place::PlacementStartCorner::top_left ||
                      start_corner == place::PlacementStartCorner::top_right;
  return {on_right ? container.min.x() + (container.max.x() - point.x())
                   : point.x(),
          on_top ? container.min.y() + (container.max.y() - point.y())
                 : point.y()};
}

auto box_for_start_corner(const geom::Box2 &box, const geom::Box2 &container,
                          const place::PlacementStartCorner start_corner)
    -> geom::Box2 {
  return normalize_box(
      point_for_start_corner(box.min, container, start_corner),
      point_for_start_corner(box.max, container, start_corner));
}

auto unique_sorted_values(std::vector<double> values) -> std::vector<double> {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end(),
                           [](double lhs, double rhs) {
                             return almost_equal(lhs, rhs);
                           }),
               values.end());
  return values;
}

auto overlaps_any_occupied_bounds(std::span<const geom::Box2> occupied_bounds,
                                  const geom::Box2 &candidate_bounds) -> bool {
  return overlaps_any_occupied_bounds(occupied_bounds, candidate_bounds, 0.0);
}

auto overlaps_any_occupied_bounds(std::span<const geom::Box2> occupied_bounds,
                                  const geom::Box2 &candidate_bounds,
                                  const double spacing) -> bool {
  return std::any_of(occupied_bounds.begin(), occupied_bounds.end(),
                     [&](const geom::Box2 &occupied_bounds_entry) {
                       return boxes_violate_spacing(
                           candidate_bounds, occupied_bounds_entry, spacing);
                     });
}

auto split_free_rectangles(std::vector<geom::Box2> &free_rectangles,
                           const geom::Box2 &used_bounds) -> void {
  std::vector<geom::Box2> updated;
  updated.reserve(free_rectangles.size() * 4U + 1U);
  for (const geom::Box2 &free_rectangle : free_rectangles) {
    if (!boxes_overlap_interior(free_rectangle, used_bounds)) {
      updated.push_back(free_rectangle);
      continue;
    }

    const geom::Box2 intersection{
        geom::Point2{std::max(free_rectangle.min.x(), used_bounds.min.x()),
                     std::max(free_rectangle.min.y(), used_bounds.min.y())},
        geom::Point2{std::min(free_rectangle.max.x(), used_bounds.max.x()),
                     std::min(free_rectangle.max.y(), used_bounds.max.y())},
    };

    const geom::Box2 left{
        free_rectangle.min,
        geom::Point2{intersection.min.x(), free_rectangle.max.y()}};
    const geom::Box2 right{
        geom::Point2{intersection.max.x(), free_rectangle.min.y()},
        free_rectangle.max};
    const geom::Box2 bottom{
        geom::Point2{free_rectangle.min.x(), free_rectangle.min.y()},
        geom::Point2{free_rectangle.max.x(), intersection.min.y()}};
    const geom::Box2 top{
        geom::Point2{free_rectangle.min.x(), intersection.max.y()},
        geom::Point2{free_rectangle.max.x(), free_rectangle.max.y()}};

    for (const geom::Box2 &candidate : {left, right, bottom, top}) {
      if (box_has_area(candidate)) {
        updated.push_back(candidate);
      }
    }
  }

  std::vector<geom::Box2> reduced;
  reduced.reserve(updated.size());
  for (std::size_t index = 0; index < updated.size(); ++index) {
    bool contained = false;
    for (std::size_t other_index = 0; other_index < updated.size();
         ++other_index) {
      if (index == other_index) {
        continue;
      }
      if (contains_box(updated[other_index], updated[index])) {
        contained = true;
        break;
      }
    }
    if (!contained) {
      reduced.push_back(updated[index]);
    }
  }

  bool merged_any = true;
  while (merged_any) {
    merged_any = false;
    for (std::size_t index = 0; index < reduced.size() && !merged_any;
         ++index) {
      for (std::size_t other_index = index + 1; other_index < reduced.size();
           ++other_index) {
        const geom::Box2 &lhs = reduced[index];
        const geom::Box2 &rhs = reduced[other_index];

        const bool same_vertical_span =
            almost_equal(lhs.min.y(), rhs.min.y()) &&
            almost_equal(lhs.max.y(), rhs.max.y());
        const bool horizontally_adjacent =
            almost_equal(lhs.max.x(), rhs.min.x()) ||
            almost_equal(rhs.max.x(), lhs.min.x());
        if (same_vertical_span && horizontally_adjacent) {
          reduced[index] = geom::Box2{
              geom::Point2{std::min(lhs.min.x(), rhs.min.x()), lhs.min.y()},
              geom::Point2{std::max(lhs.max.x(), rhs.max.x()), lhs.max.y()},
          };
          reduced.erase(reduced.begin() +
                        static_cast<std::ptrdiff_t>(other_index));
          merged_any = true;
          break;
        }

        const bool same_horizontal_span =
            almost_equal(lhs.min.x(), rhs.min.x()) &&
            almost_equal(lhs.max.x(), rhs.max.x());
        const bool vertically_adjacent =
            almost_equal(lhs.max.y(), rhs.min.y()) ||
            almost_equal(rhs.max.y(), lhs.min.y());
        if (same_horizontal_span && vertically_adjacent) {
          reduced[index] = geom::Box2{
              geom::Point2{lhs.min.x(), std::min(lhs.min.y(), rhs.min.y())},
              geom::Point2{lhs.max.x(), std::max(lhs.max.y(), rhs.max.y())},
          };
          reduced.erase(reduced.begin() +
                        static_cast<std::ptrdiff_t>(other_index));
          merged_any = true;
          break;
        }
      }
    }
  }

  free_rectangles = std::move(reduced);
}

auto total_container_area(const DecoderResult &result) -> double {
  double total = 0.0;
  for (const BinState &bin : result.bins) {
    total += bin.utilization.container_area;
  }
  return total;
}

auto total_occupied_area(const DecoderResult &result) -> double {
  double total = 0.0;
  for (const BinState &bin : result.bins) {
    total += bin.utilization.occupied_area;
  }
  return total;
}

auto overall_utilization(const DecoderResult &result) -> double {
  const double container_area = total_container_area(result);
  if (container_area <= kCoordinateSnap) {
    return 0.0;
  }
  return total_occupied_area(result) / container_area;
}

auto decode_result_better(const DecoderResult &lhs, const DecoderResult &rhs)
    -> bool {
  const std::size_t lhs_placed = lhs.layout.placement_trace.size();
  const std::size_t rhs_placed = rhs.layout.placement_trace.size();
  if (lhs_placed != rhs_placed) {
    return lhs_placed > rhs_placed;
  }

  const bool lhs_completed = !lhs.interrupted;
  const bool rhs_completed = !rhs.interrupted;
  if (lhs_completed != rhs_completed) {
    return lhs_completed;
  }

  const double lhs_utilization = overall_utilization(lhs);
  const double rhs_utilization = overall_utilization(rhs);
  if (!almost_equal(lhs_utilization, rhs_utilization)) {
    return lhs_utilization > rhs_utilization;
  }

  const std::size_t lhs_bins_used = lhs.layout.bins.size();
  const std::size_t rhs_bins_used = rhs.layout.bins.size();
  if (lhs_bins_used != rhs_bins_used) {
    return lhs_bins_used < rhs_bins_used;
  }

  return false;
}

auto resulting_envelope_area(const BinPackingState &state,
                             const geom::Box2 &candidate_bounds) -> double {
  double min_x = candidate_bounds.min.x();
  double max_x = candidate_bounds.max.x();
  double max_y = candidate_bounds.max.y();
  for (const auto &shelf : state.shelves) {
    min_x = std::min(min_x, shelf.used_min_x);
    max_x = std::max(max_x, shelf.used_max_x);
    max_y = std::max(max_y, shelf.y + shelf.height);
  }

  return std::max(0.0, max_x - min_x) *
         std::max(0.0, max_y - state.container_bounds.min.y());
}

auto start_corner_on_right(const place::PlacementStartCorner start_corner)
    -> bool {
  return start_corner == place::PlacementStartCorner::bottom_right ||
         start_corner == place::PlacementStartCorner::top_right;
}

auto start_corner_on_top(const place::PlacementStartCorner start_corner)
    -> bool {
  return start_corner == place::PlacementStartCorner::top_left ||
         start_corner == place::PlacementStartCorner::top_right;
}

auto primary_edge_distance(const geom::Box2 &container_bounds,
                           const geom::Box2 &piece_bounds,
                           const place::PlacementStartCorner start_corner)
    -> double {
  if (start_corner_on_top(start_corner)) {
    return container_bounds.max.y() - piece_bounds.max.y();
  }
  return piece_bounds.min.y() - container_bounds.min.y();
}

auto secondary_edge_distance(const geom::Box2 &container_bounds,
                             const geom::Box2 &piece_bounds,
                             const place::PlacementStartCorner start_corner)
    -> double {
  if (start_corner_on_right(start_corner)) {
    return container_bounds.max.x() - piece_bounds.max.x();
  }
  return piece_bounds.min.x() - container_bounds.min.x();
}

auto better_candidate(const BinPackingState &state,
                      const PlacementCandidate &lhs,
                      const PlacementCandidate &rhs,
                      place::PlacementPolicy policy) -> bool {
  const double lhs_primary =
      primary_edge_distance(state.container_bounds, lhs.translated_bounds,
                            state.bin_state.start_corner);
  const double rhs_primary =
      primary_edge_distance(state.container_bounds, rhs.translated_bounds,
                            state.bin_state.start_corner);
  const double lhs_secondary =
      secondary_edge_distance(state.container_bounds, lhs.translated_bounds,
                              state.bin_state.start_corner);
  const double rhs_secondary =
      secondary_edge_distance(state.container_bounds, rhs.translated_bounds,
                              state.bin_state.start_corner);
  switch (policy) {
  case place::PlacementPolicy::bottom_left:
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::minimum_length:
    if (!almost_equal(lhs.translated_bounds.max.x(),
                      rhs.translated_bounds.max.x())) {
      return lhs.translated_bounds.max.x() < rhs.translated_bounds.max.x();
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  case place::PlacementPolicy::maximum_utilization:
    if (!almost_equal(lhs.resulting_utilization, rhs.resulting_utilization)) {
      return lhs.resulting_utilization > rhs.resulting_utilization;
    }
    if (!almost_equal(lhs_primary, rhs_primary)) {
      return lhs_primary < rhs_primary;
    }
    if (!almost_equal(lhs_secondary, rhs_secondary)) {
      return lhs_secondary < rhs_secondary;
    }
    break;
  }

  if (lhs.placement.rotation_index.value !=
      rhs.placement.rotation_index.value) {
    return lhs.placement.rotation_index.value <
           rhs.placement.rotation_index.value;
  }

  return false;
}

auto make_empty_bin(const BinInput &bin) -> BinPackingState {
  BinPackingState state{};
  state.bin_state.bin_id = bin.bin_id;
  state.bin_state.container = geom::normalize_polygon(bin.polygon);
  state.bin_state.container_geometry_revision = bin.geometry_revision;
  state.bin_state.start_corner = bin.start_corner;
  state.bin_state.utilization = summarize_bin(state.bin_state);
  state.container_bounds = compute_bounds(state.bin_state.container);
  state.free_rectangles.push_back(state.container_bounds);
  return state;
}

} // namespace shiny::nesting::pack
