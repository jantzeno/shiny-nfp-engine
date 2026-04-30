#include "geometry/operations/merge_region.hpp"

#include <algorithm>

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/queries/normalize.hpp"

namespace shiny::nesting::geom {
namespace {

[[nodiscard]] auto polygon_less(const PolygonWithHoles &lhs,
                                const PolygonWithHoles &rhs) -> bool {
  if (lhs.outer().empty()) {
    return !rhs.outer().empty();
  }
  if (rhs.outer().empty()) {
    return false;
  }

  if (lhs.outer().front().x() != rhs.outer().front().x()) {
    return lhs.outer().front().x() < rhs.outer().front().x();
  }
  if (lhs.outer().front().y() != rhs.outer().front().y()) {
    return lhs.outer().front().y() < rhs.outer().front().y();
  }
  if (lhs.outer().size() != rhs.outer().size()) {
    return lhs.outer().size() < rhs.outer().size();
  }
  return lhs.holes().size() < rhs.holes().size();
}

void sort_regions(std::vector<PolygonWithHoles> &regions) {
  std::sort(regions.begin(), regions.end(), polygon_less);
}

} // namespace

auto make_merged_region(const PolygonWithHoles &polygon) -> MergedRegion {
  if (polygon.outer().empty()) {
    return {};
  }

  return {.regions = {normalize_polygon(polygon)}};
}

auto merge_region(const PolygonWithHoles &lhs, const PolygonWithHoles &rhs)
    -> MergedRegion {
  return merge_polygon_into_region(make_merged_region(lhs), rhs);
}

auto merge_polygon_into_region(const MergedRegion &region,
                               const PolygonWithHoles &polygon)
    -> MergedRegion {
  if (polygon.outer().empty()) {
    return region;
  }

  PolygonWithHoles accumulator = normalize_polygon(polygon);
  std::vector<PolygonWithHoles> pending = region.regions;

  bool merged_any = false;
  do {
    merged_any = false;
    std::vector<PolygonWithHoles> next_pending;
    next_pending.reserve(pending.size());

    for (const auto &existing : pending) {
      const auto merged = union_polygons(existing, accumulator);
      if (merged.size() == 1U) {
        accumulator = merged.front();
        merged_any = true;
      } else {
        next_pending.push_back(existing);
      }
    }

    pending = std::move(next_pending);
  } while (merged_any);

  pending.push_back(std::move(accumulator));
  sort_regions(pending);
  return {.regions = std::move(pending)};
}

} // namespace shiny::nesting::geom