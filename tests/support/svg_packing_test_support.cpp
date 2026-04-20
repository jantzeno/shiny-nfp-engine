#include "support/svg_packing_test_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>

#include <catch2/catch_test_macros.hpp>

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"

#include "algorithm_kind.hpp"
#include "geometry/normalize.hpp"
#include "predicates/classify.hpp"
#include "predicates/point_location.hpp"
#include "predicates/segment_intersection.hpp"
#include "search/observer.hpp"
#include "support/fixture_test_support.hpp"

namespace shiny::nesting::test::svg {
namespace {

namespace fs = std::filesystem;

using geom::Point2;
using geom::Polygon;
using geom::PolygonWithHoles;
using geom::Ring;
using geom::Segment2;
using pack::BinInput;
using pack::DecoderRequest;
using pack::DecoderResult;
using pack::Layout;
using pack::MasonryProgressSnapshot;
using pack::MasonryRequest;
using pack::MasonryResult;
using pack::PackingConfig;
using place::BedExclusionZone;
using place::BedGrainDirection;
using place::PartGrainCompatibility;
using place::PlacementPolicy;
using pred::PointLocation;
using pred::SegmentContactKind;
using search::SearchRequest;
using search::SearchResult;
using search::SearchRunStatus;
using util::Status;
using util::StatusOr;

constexpr double kCurveFlatteningTolerance = 1.0;
constexpr double kCoordinateTolerance = 1e-9;

std::mutex g_live_progress_mutex{};

struct ContourInfo {
  Ring ring{};
  double area{0.0};
};

auto source_root() -> fs::path {
  return fixture_root().parent_path() / "files";
}

auto case_input_path(const SvgPackingCaseSpec &spec) -> fs::path {
  return source_root() / spec.input_path;
}

[[nodiscard]] auto live_progress_enabled() -> bool {
  const char *value = std::getenv("SHINY_NFP_ENGINE_TEST_PROGRESS");
  if (value == nullptr) {
    return false;
  }

  const std::string_view mode{value};
  return mode == "1" || mode == "true" || mode == "svg" || mode == "readiness";
}

[[nodiscard]] auto move_kind_label(search::SearchMoveKind kind)
    -> std::string_view {
  using search::SearchMoveKind;

  switch (kind) {
  case SearchMoveKind::none:
    return "none";
  case SearchMoveKind::jostle_oscillation:
    return "jostle_oscillation";
  case SearchMoveKind::one_piece_insert:
    return "one_piece_insert";
  case SearchMoveKind::genetic_generation:
    return "genetic_generation";
  }

  return "unknown";
}

auto emit_live_progress_line(std::string_view line) -> void {
  std::scoped_lock lock(g_live_progress_mutex);

  if (FILE *tty = std::fopen("/dev/tty", "a"); tty != nullptr) {
    std::fprintf(tty, "%.*s\n", static_cast<int>(line.size()), line.data());
    std::fflush(tty);
    std::fclose(tty);
    return;
  }

  std::fprintf(stderr, "%.*s\n", static_cast<int>(line.size()), line.data());
  std::fflush(stderr);
}

auto install_live_progress_observer(const SvgPackingCaseSpec &spec,
                                    SearchRequest &request) -> void {
  if (!live_progress_enabled()) {
    return;
  }

  emit_live_progress_line("svg_case=" + spec.id + " event=case_started");
  request.execution.observer.on_event = [case_id = spec.id](
                                            const search::SearchEvent &event) {
    using search::SearchCancellationAcknowledgedEvent;
    using search::SearchEventKind;
    using search::SearchImprovementFoundEvent;
    using search::SearchRunCompletedEvent;
    using search::SearchRunStartedEvent;
    using search::SearchStepProgressEvent;
    using search::SearchTimeoutReachedEvent;
    using search::search_event_kind;

    std::ostringstream stream;
    stream << "svg_case=" << case_id << ' ';

    switch (search_event_kind(event)) {
    case SearchEventKind::run_started: {
      const auto &payload = std::get<SearchRunStartedEvent>(event);
      stream << "event=run_started algorithm="
             << to_string(payload.algorithm_kind)
             << " seed=" << payload.deterministic_seed
             << " iteration_budget=" << payload.iteration_budget
             << " piece_count=" << payload.piece_count;
      break;
    }
    case SearchEventKind::step_progress:
    case SearchEventKind::improvement_found: {
      const auto &payload =
          search_event_kind(event) == SearchEventKind::improvement_found
              ? std::get<SearchImprovementFoundEvent>(event).progress
              : std::get<SearchStepProgressEvent>(event).progress;
      stream << "event="
             << (search_event_kind(event) == SearchEventKind::improvement_found
                     ? "improvement_found"
                     : "step_progress")
             << " iteration=" << payload.iteration << '/'
             << payload.iteration_budget
             << " move=" << move_kind_label(payload.move_kind)
             << " placed=" << payload.best_placed_piece_count
             << " unplaced=" << payload.best_unplaced_piece_count
             << " bins=" << payload.best_bin_count
             << " evals=" << payload.evaluated_layout_count
             << " cache_hits=" << payload.reevaluation_cache_hits
             << " utilization=" << payload.best_total_utilization
             << " elapsed_ms=" << payload.elapsed_ms;
      break;
    }
    case SearchEventKind::run_completed: {
      const auto &payload = std::get<SearchRunCompletedEvent>(event).summary;
      stream << "event=run_completed iterations="
             << payload.iterations_completed
             << " placed=" << payload.best_placed_piece_count
             << " unplaced=" << payload.best_unplaced_piece_count
             << " bins=" << payload.best_bin_count
             << " elapsed_ms=" << payload.elapsed_ms;
      break;
    }
    case SearchEventKind::timeout_reached: {
      const auto &payload = std::get<SearchTimeoutReachedEvent>(event).summary;
      stream << "event=timeout_reached iterations="
             << payload.iterations_completed
             << " placed=" << payload.best_placed_piece_count
             << " unplaced=" << payload.best_unplaced_piece_count
             << " bins=" << payload.best_bin_count
             << " elapsed_ms=" << payload.elapsed_ms;
      break;
    }
    case SearchEventKind::cancellation_acknowledged: {
      const auto &payload =
          std::get<SearchCancellationAcknowledgedEvent>(event).summary;
      stream << "event=cancellation_acknowledged iterations="
             << payload.iterations_completed
             << " placed=" << payload.best_placed_piece_count
             << " unplaced=" << payload.best_unplaced_piece_count
             << " bins=" << payload.best_bin_count
             << " elapsed_ms=" << payload.elapsed_ms;
      break;
    }
    }

    emit_live_progress_line(stream.str());
  };
}

auto install_live_progress_observer(const SvgPackingCaseSpec &spec,
                                    MasonryRequest &request) -> void {
  if (!live_progress_enabled()) {
    return;
  }

  emit_live_progress_line("svg_case=" + spec.id +
                          " event=case_started algorithm=masonry_builder");
  request.observer.on_progress =
      [case_id = spec.id](const MasonryProgressSnapshot &progress) {
        std::ostringstream stream;
        stream << "svg_case=" << case_id << " event=step_progress algorithm="
               << to_string(progress.algorithm_kind)
               << " processed=" << progress.processed_piece_count << '/'
               << progress.piece_count
               << " current_piece=" << progress.current_piece_id
               << " placed=" << progress.placed_piece_count
               << " unplaced=" << progress.unplaced_piece_count
               << " bins=" << progress.bin_count
               << " utilization=" << progress.total_utilization;
        emit_live_progress_line(stream.str());
      };
}

auto parse_bed_grain_direction(std::string_view value) -> BedGrainDirection {
  if (value == "unrestricted") {
    return BedGrainDirection::unrestricted;
  }
  if (value == "along_x") {
    return BedGrainDirection::along_x;
  }
  if (value == "along_y") {
    return BedGrainDirection::along_y;
  }
  throw std::runtime_error("unknown svg packing bed grain direction");
}

auto parse_part_grain_compatibility(std::string_view value)
    -> PartGrainCompatibility {
  if (value == "unrestricted") {
    return PartGrainCompatibility::unrestricted;
  }
  if (value == "parallel_to_bed") {
    return PartGrainCompatibility::parallel_to_bed;
  }
  if (value == "perpendicular_to_bed") {
    return PartGrainCompatibility::perpendicular_to_bed;
  }
  throw std::runtime_error("unknown svg packing grain compatibility");
}

auto parse_exclusion_zones(const pt::ptree &node)
    -> std::vector<BedExclusionZone> {
  std::vector<BedExclusionZone> zones;
  for (const auto &child : node) {
    zones.push_back({
        .zone_id = child.second.get<std::uint32_t>("zone_id", 0),
        .region = {.outer = parse_ring(child.second.get_child("region"))},
    });
  }
  return zones;
}

template <typename OptionalNode>
auto parse_string_list(const OptionalNode &node) -> std::vector<std::string> {
  std::vector<std::string> values;
  if (!node) {
    return values;
  }

  for (const auto &entry : *node) {
    values.push_back(entry.second.template get_value<std::string>());
  }
  return values;
}

[[nodiscard]] auto string_list_contains(const std::vector<std::string> &values,
                                        std::string_view needle) -> bool {
  return std::find(values.begin(), values.end(), needle) != values.end();
}

auto signed_area(const Ring &ring) -> long double {
  if (ring.size() < 3U) {
    return 0.0L;
  }

  long double area = 0.0L;
  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    area += static_cast<long double>(ring[index].x) * ring[next_index].y -
            static_cast<long double>(ring[next_index].x) * ring[index].y;
  }
  return area / 2.0L;
}

auto absolute_area(const Ring &ring) -> double {
  return std::abs(static_cast<double>(signed_area(ring)));
}

auto midpoint(const Point2 &lhs, const Point2 &rhs) -> Point2 {
  return {.x = (lhs.x + rhs.x) * 0.5, .y = (lhs.y + rhs.y) * 0.5};
}

auto point_line_distance(const Point2 &point, const Point2 &line_start,
                         const Point2 &line_end) -> double {
  const auto dx = line_end.x - line_start.x;
  const auto dy = line_end.y - line_start.y;
  if (dx == 0.0 && dy == 0.0) {
    const auto px = point.x - line_start.x;
    const auto py = point.y - line_start.y;
    return std::sqrt(px * px + py * py);
  }

  const auto numerator =
      std::fabs(dy * point.x - dx * point.y + line_end.x * line_start.y -
                line_end.y * line_start.x);
  return numerator / std::sqrt(dx * dx + dy * dy);
}

void append_cubic_points(std::vector<Point2> &ring, const Point2 &start,
                         const Point2 &control_a, const Point2 &control_b,
                         const Point2 &end, double tolerance, int depth = 0) {
  const auto flatness = std::max(point_line_distance(control_a, start, end),
                                 point_line_distance(control_b, start, end));
  if (depth >= 12 || flatness <= tolerance) {
    ring.push_back(end);
    return;
  }

  const auto start_a = midpoint(start, control_a);
  const auto a_b = midpoint(control_a, control_b);
  const auto b_end = midpoint(control_b, end);
  const auto left_mid = midpoint(start_a, a_b);
  const auto right_mid = midpoint(a_b, b_end);
  const auto split = midpoint(left_mid, right_mid);

  append_cubic_points(ring, start, start_a, left_mid, split, tolerance,
                      depth + 1);
  append_cubic_points(ring, split, right_mid, b_end, end, tolerance, depth + 1);
}

auto normalize_ring(Ring ring) -> std::optional<Ring> {
  auto polygon = geom::normalize_polygon(Polygon{.outer = ring});
  if (polygon.outer.size() < 3U) {
    return std::nullopt;
  }
  if (absolute_area(polygon.outer) <= kMinimumArea) {
    return std::nullopt;
  }
  return polygon.outer;
}

auto flatten_path(const NSVGpath &path) -> std::optional<Ring> {
  if (path.closed == 0 || path.npts < 4) {
    return std::nullopt;
  }

  Ring ring;
  ring.reserve(static_cast<std::size_t>(path.npts));
  const auto *points = path.pts;
  ring.push_back({.x = points[0], .y = points[1]});

  for (int index = 0; index < path.npts - 1; index += 3) {
    const auto *segment = &path.pts[index * 2];
    const Point2 start{.x = segment[0], .y = segment[1]};
    const Point2 control_a{.x = segment[2], .y = segment[3]};
    const Point2 control_b{.x = segment[4], .y = segment[5]};
    const Point2 end{.x = segment[6], .y = segment[7]};
    append_cubic_points(ring, start, control_a, control_b, end,
                        kCurveFlatteningTolerance);
  }

  return normalize_ring(std::move(ring));
}

auto extract_contours(const NSVGshape &shape) -> std::vector<ContourInfo> {
  std::vector<ContourInfo> contours;
  for (const auto *path = shape.paths; path != nullptr; path = path->next) {
    auto ring = flatten_path(*path);
    if (!ring.has_value()) {
      continue;
    }
    contours.push_back(
        {.ring = std::move(*ring), .area = absolute_area(*ring)});
  }
  return contours;
}

auto shape_area_hint(const NSVGshape &shape) -> double {
  return std::max(0.0F, shape.bounds[2] - shape.bounds[0]) *
         std::max(0.0F, shape.bounds[3] - shape.bounds[1]);
}

auto group_key_for_shape(std::string_view shape_id, std::size_t shape_index)
    -> std::string {
  if (shape_id.empty()) {
    return "shape-" + std::to_string(shape_index);
  }
  if (shape_id == "bed") {
    return "bed";
  }

  if (shape_id.rfind("path-", 0) == 0U) {
    const auto second_dash = shape_id.find('-', 5U);
    if (second_dash != std::string_view::npos && second_dash > 5U) {
      return "artwork-" + std::string(shape_id.substr(5U, second_dash - 5U));
    }
  }

  return std::string(shape_id);
}

auto build_polygons_from_contours(std::vector<ContourInfo> contours)
    -> std::vector<PolygonWithHoles> {
  std::stable_sort(contours.begin(), contours.end(),
                   [](const auto &lhs, const auto &rhs) {
                     if (lhs.area != rhs.area) {
                       return lhs.area > rhs.area;
                     }
                     return lhs.ring < rhs.ring;
                   });

  std::vector<PolygonWithHoles> polygons;
  for (const auto &contour : contours) {
    bool attached = false;
    for (auto &polygon : polygons) {
      const auto outer_location =
          pred::locate_point_in_ring(contour.ring.front(), polygon.outer);
      if (outer_location.location == PointLocation::exterior) {
        continue;
      }

      bool inside_existing_hole = false;
      for (const auto &hole : polygon.holes) {
        const auto hole_location =
            pred::locate_point_in_ring(contour.ring.front(), hole);
        if (hole_location.location != PointLocation::exterior) {
          inside_existing_hole = true;
          break;
        }
      }

      if (!inside_existing_hole) {
        polygon.holes.push_back(contour.ring);
        attached = true;
        break;
      }
    }

    if (!attached) {
      polygons.push_back({.outer = contour.ring});
    }
  }

  for (auto &polygon : polygons) {
    polygon = geom::normalize_polygon(polygon);
  }
  return polygons;
}

auto bounds_for_ring(const Ring &ring) -> std::array<double, 4> {
  std::array<double, 4> bounds{ring.front().x, ring.front().y, ring.front().x,
                               ring.front().y};
  for (const auto &point : ring) {
    bounds[0] = std::min(bounds[0], point.x);
    bounds[1] = std::min(bounds[1], point.y);
    bounds[2] = std::max(bounds[2], point.x);
    bounds[3] = std::max(bounds[3], point.y);
  }
  return bounds;
}

auto translate_ring(const Ring &ring, double dx, double dy) -> Ring {
  Ring translated;
  translated.reserve(ring.size());
  for (const auto &point : ring) {
    translated.push_back({.x = point.x + dx, .y = point.y + dy});
  }
  return translated;
}

auto translate_polygon(const PolygonWithHoles &polygon, double dx, double dy)
    -> PolygonWithHoles {
  PolygonWithHoles translated{};
  translated.outer = translate_ring(polygon.outer, dx, dy);
  translated.holes.reserve(polygon.holes.size());
  for (const auto &hole : polygon.holes) {
    translated.holes.push_back(translate_ring(hole, dx, dy));
  }
  return geom::normalize_polygon(translated);
}

auto nearly_equal(double lhs, double rhs) -> bool {
  return std::abs(lhs - rhs) <= kCoordinateTolerance;
}

void require_same_ring(const Ring &lhs, const Ring &rhs) {
  REQUIRE(lhs.size() == rhs.size());
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    REQUIRE(nearly_equal(lhs[index].x, rhs[index].x));
    REQUIRE(nearly_equal(lhs[index].y, rhs[index].y));
  }
}

void require_same_polygon(const PolygonWithHoles &lhs,
                          const PolygonWithHoles &rhs) {
  require_same_ring(lhs.outer, rhs.outer);
  REQUIRE(lhs.holes.size() == rhs.holes.size());
  for (std::size_t index = 0; index < lhs.holes.size(); ++index) {
    require_same_ring(lhs.holes[index], rhs.holes[index]);
  }
}

template <typename Fn> void for_each_segment(const Ring &ring, Fn &&fn) {
  if (ring.size() < 2U) {
    return;
  }

  for (std::size_t index = 0; index < ring.size(); ++index) {
    const auto next_index = (index + 1U) % ring.size();
    fn(Segment2{.start = ring[index], .end = ring[next_index]});
  }
}

template <typename Fn>
void for_each_polygon_ring(const PolygonWithHoles &polygon, Fn &&fn) {
  fn(polygon.outer);
  for (const auto &hole : polygon.holes) {
    fn(hole);
  }
}

auto rings_have_proper_intersection(const Ring &lhs, const Ring &rhs) -> bool {
  bool intersects = false;
  for_each_segment(lhs, [&](const Segment2 &lhs_segment) {
    if (intersects) {
      return;
    }

    for_each_segment(rhs, [&](const Segment2 &rhs_segment) {
      if (intersects) {
        return;
      }

      const auto contact =
          pred::classify_segment_contact(lhs_segment, rhs_segment);
      if (contact.kind == SegmentContactKind::proper_intersection) {
        intersects = true;
      }
    });
  });
  return intersects;
}

auto point_inside_container(const Point2 &point,
                            const PolygonWithHoles &container) -> bool {
  const auto location = pred::locate_point_in_polygon(point, container);
  return location.location != PointLocation::exterior && !location.inside_hole;
}

auto polygon_crosses_container_boundary(const PolygonWithHoles &polygon,
                                        const PolygonWithHoles &container)
    -> bool {
  bool crosses = false;
  for_each_polygon_ring(polygon, [&](const Ring &candidate_ring) {
    if (crosses) {
      return;
    }

    if (rings_have_proper_intersection(candidate_ring, container.outer)) {
      crosses = true;
      return;
    }

    for (const auto &hole : container.holes) {
      if (rings_have_proper_intersection(candidate_ring, hole)) {
        crosses = true;
        return;
      }
    }
  });
  return crosses;
}

auto polygon_within_container(const PolygonWithHoles &polygon,
                              const PolygonWithHoles &container) -> bool {
  if (polygon_crosses_container_boundary(polygon, container)) {
    return false;
  }

  bool all_inside = true;
  for_each_polygon_ring(polygon, [&](const Ring &ring) {
    if (!all_inside) {
      return;
    }

    for (const auto &point : ring) {
      if (!point_inside_container(point, container)) {
        all_inside = false;
        return;
      }
    }
  });
  return all_inside;
}

auto polygons_overlap_interior(const PolygonWithHoles &lhs,
                               const PolygonWithHoles &rhs) -> bool {
  bool has_proper_intersection = false;
  for_each_polygon_ring(lhs, [&](const Ring &lhs_ring) {
    if (has_proper_intersection) {
      return;
    }

    for_each_polygon_ring(rhs, [&](const Ring &rhs_ring) {
      if (has_proper_intersection) {
        return;
      }
      if (rings_have_proper_intersection(lhs_ring, rhs_ring)) {
        has_proper_intersection = true;
      }
    });
  });
  if (has_proper_intersection) {
    return true;
  }

  for (const auto &point : lhs.outer) {
    const auto location = pred::locate_point_in_polygon(point, rhs);
    if (location.location == PointLocation::interior && !location.inside_hole) {
      return true;
    }
  }

  for (const auto &point : rhs.outer) {
    const auto location = pred::locate_point_in_polygon(point, lhs);
    if (location.location == PointLocation::interior && !location.inside_hole) {
      return true;
    }
  }

  return false;
}

void require_layout_consistency(const DecoderRequest &request,
                                const Layout &layout,
                                std::size_t placed_piece_count,
                                std::size_t unplaced_piece_count) {
  REQUIRE(layout.placement_trace.size() == placed_piece_count);
  REQUIRE(layout.unplaced_piece_ids.size() == unplaced_piece_count);
  REQUIRE(placed_piece_count + unplaced_piece_count == request.pieces.size());

  std::size_t total_placed = 0U;
  for (const auto &bin : layout.bins) {
    total_placed += bin.placements.size();
    REQUIRE(bin.utilization.placement_count == bin.placements.size());
    REQUIRE(bin.utilization.utilization >= 0.0);

    for (std::size_t lhs_index = 0; lhs_index < bin.placements.size();
         ++lhs_index) {
      const auto &placed = bin.placements[lhs_index];
      REQUIRE(polygon_within_container(placed.polygon, bin.container));

      const auto piece_it = std::find_if(
          request.pieces.begin(), request.pieces.end(), [&](const auto &piece) {
            return piece.piece_id == placed.placement.piece_id;
          });
      REQUIRE(piece_it != request.pieces.end());

      const auto resolved_rotation = place::resolve_rotation(
          placed.placement.rotation_index, request.config.placement);
      REQUIRE(resolved_rotation.has_value());
      REQUIRE(place::grain_compatibility_allows_rotation(
          *resolved_rotation, request.config.placement.bed_grain_direction,
          piece_it->grain_compatibility));

      for (const auto &zone : request.config.placement.exclusion_zones) {
        const PolygonWithHoles exclusion_zone{.outer = zone.region.outer};
        REQUIRE_FALSE(
            polygons_overlap_interior(placed.polygon, exclusion_zone));
      }

      for (std::size_t rhs_index = lhs_index + 1U;
           rhs_index < bin.placements.size(); ++rhs_index) {
        REQUIRE_FALSE(
            polygons_overlap_interior(bin.placements[lhs_index].polygon,
                                      bin.placements[rhs_index].polygon));
      }
    }
  }

  REQUIRE(total_placed == placed_piece_count);
}

auto svg_path_data(const PolygonWithHoles &polygon) -> std::string {
  auto append_ring = [](std::ostringstream &stream, const Ring &ring) {
    if (ring.empty()) {
      return;
    }

    stream << "M " << ring.front().x << ' ' << ring.front().y;
    for (std::size_t index = 1; index < ring.size(); ++index) {
      stream << " L " << ring[index].x << ' ' << ring[index].y;
    }
    stream << " Z ";
  };

  std::ostringstream stream;
  append_ring(stream, polygon.outer);
  for (const auto &hole : polygon.holes) {
    append_ring(stream, hole);
  }
  return stream.str();
}

} // namespace

auto output_root() -> std::filesystem::path {
  return fixture_root().parent_path().parent_path() / "build" / "test" /
         "svg_packing";
}

auto load_svg_case_specs(bool normative_only)
    -> std::vector<SvgPackingCaseSpec> {
  const auto manifest = load_fixture_file("integration/svg_packing_cases.json");

  std::vector<SvgPackingCaseSpec> specs;
  for (const auto &entry : manifest.get_child("cases")) {
    const auto &node = entry.second;
    require_fixture_metadata(node, "svg_packing_case");

    SvgPackingCaseSpec spec{};
    spec.id = node.get<std::string>("id");
    spec.description = node.get<std::string>("description");
    spec.source = node.get<std::string>("source", spec.source);
    spec.input_path = fs::path{node.get<std::string>("input")};
    spec.normative = node.get<bool>("normative", true);
    spec.slow_exploratory = node.get<bool>("slow_exploratory", false);
    spec.acceptance_lane = node.get<std::string>(
        "acceptance_lane", spec.normative ? "readiness" : "stress");
    spec.coverage_features =
        parse_string_list(node.get_child_optional("coverage_features"));
    spec.algorithm_applicability =
        parse_string_list(node.get_child_optional("algorithm_applicability"));
    if (spec.algorithm_applicability.empty()) {
      spec.algorithm_applicability.push_back("jostle_search");
    }
    spec.require_observable_success_before_interrupt = node.get<bool>(
        "require_observable_success_before_interrupt", !spec.normative);
    spec.require_explicit_bed_id =
        node.get<bool>("require_explicit_bed_id", true);
    spec.max_candidate_groups = node.get<std::size_t>(
        "max_candidate_groups", spec.max_candidate_groups);
    spec.max_piece_count =
        node.get<std::size_t>("max_piece_count", spec.max_piece_count);
    spec.min_piece_area =
        node.get<double>("min_piece_area", spec.min_piece_area);
    spec.allowed_rotations_degrees.clear();
    if (const auto rotations =
            node.get_child_optional("allowed_rotations_degrees")) {
      for (const auto &rotation_entry : *rotations) {
        spec.allowed_rotations_degrees.push_back(
            rotation_entry.second.get_value<double>());
      }
    }
    if (spec.allowed_rotations_degrees.empty()) {
      spec.allowed_rotations_degrees.push_back(0.0);
    }
    if (const auto grain_direction =
            node.get_optional<std::string>("bed_grain_direction")) {
      spec.bed_grain_direction = parse_bed_grain_direction(*grain_direction);
    }
    if (const auto exclusion_zones =
            node.get_child_optional("exclusion_zones")) {
      spec.exclusion_zones = parse_exclusion_zones(*exclusion_zones);
    }
    if (const auto piece_grain =
            node.get_child_optional("piece_grain_compatibility")) {
      for (const auto &grain_entry : *piece_grain) {
        spec.piece_grain_compatibility.emplace(
            grain_entry.first,
            parse_part_grain_compatibility(
                grain_entry.second.get_value<std::string>()));
      }
    }
    spec.max_bin_count =
        node.get<std::size_t>("max_bin_count", spec.max_bin_count);
    spec.min_placed_piece_count = node.get<std::size_t>(
        "min_placed_piece_count", spec.min_placed_piece_count);
    spec.require_full_placement =
        node.get<bool>("require_full_placement", spec.require_full_placement);
    spec.search_iterations =
        node.get<std::uint32_t>("search_iterations", spec.search_iterations);
    spec.search_plateau_budget = node.get<std::uint32_t>(
        "search_plateau_budget", spec.search_plateau_budget);
    spec.execution_time_budget_ms = node.get<std::uint32_t>(
        "execution_time_budget_ms", spec.execution_time_budget_ms);

    if (normative_only && !spec.normative) {
      continue;
    }
    specs.push_back(std::move(spec));
  }

  std::sort(specs.begin(), specs.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.id < rhs.id; });
  return specs;
}

auto case_supports_algorithm(const SvgPackingCaseSpec &spec,
                             std::string_view algorithm) -> bool {
  return string_list_contains(spec.algorithm_applicability, algorithm);
}

auto select_svg_case_specs(bool normative_only, std::string_view algorithm)
    -> std::vector<SvgPackingCaseSpec> {
  auto specs = load_svg_case_specs(normative_only);
  specs.erase(std::remove_if(specs.begin(), specs.end(),
                             [&](const auto &spec) {
                               return !case_supports_algorithm(spec, algorithm);
                             }),
              specs.end());
  return specs;
}

auto import_svg_case(const SvgPackingCaseSpec &spec)
    -> StatusOr<ImportedSvgCase> {
  const auto path = case_input_path(spec);
  const auto path_string = path.string();
  std::unique_ptr<NSVGimage, void (*)(NSVGimage *)> image(
      nsvgParseFromFile(path_string.c_str(), "px", 96.0F), nsvgDelete);
  if (!image) {
    return Status::computation_failed;
  }

  const NSVGshape *bed_shape = nullptr;
  double largest_area_hint = -1.0;
  std::unordered_map<std::string, double> group_area_hints;

  std::size_t shape_index = 0;
  for (const auto *shape = image->shapes; shape != nullptr;
       shape = shape->next, ++shape_index) {
    const auto area_hint = shape_area_hint(*shape);
    if (std::string_view{shape->id} == "bed") {
      bed_shape = shape;
    }
    if (!spec.require_explicit_bed_id && area_hint > largest_area_hint) {
      largest_area_hint = area_hint;
      if (bed_shape == nullptr) {
        bed_shape = shape;
      }
    }

    const auto key = group_key_for_shape(shape->id, shape_index);
    group_area_hints[key] += area_hint;
  }

  if (bed_shape == nullptr) {
    return Status::invalid_input;
  }

  const auto bed_group_key = group_key_for_shape(bed_shape->id, 0U);

  std::vector<std::pair<std::string, double>> ranked_groups;
  ranked_groups.reserve(group_area_hints.size());
  for (const auto &[key, area_hint] : group_area_hints) {
    if (key == "bed" || key == bed_group_key) {
      continue;
    }
    ranked_groups.emplace_back(key, area_hint);
  }

  std::stable_sort(ranked_groups.begin(), ranked_groups.end(),
                   [](const auto &lhs, const auto &rhs) {
                     if (lhs.second != rhs.second) {
                       return lhs.second > rhs.second;
                     }
                     return lhs.first < rhs.first;
                   });

  const bool has_artwork_groups = std::any_of(
      ranked_groups.begin(), ranked_groups.end(),
      [](const auto &entry) { return entry.first.rfind("artwork-", 0) == 0U; });
  const auto group_limit =
      has_artwork_groups
          ? ranked_groups.size()
          : std::min(spec.max_candidate_groups, ranked_groups.size());

  std::set<std::string> selected_groups;
  for (std::size_t index = 0;
       index < ranked_groups.size() && index < group_limit; ++index) {
    selected_groups.insert(ranked_groups[index].first);
  }

  ImportedSvgCase imported{.source_path = path};
  std::unordered_map<std::string, std::vector<ContourInfo>> grouped_contours;

  shape_index = 0;
  for (const auto *shape = image->shapes; shape != nullptr;
       shape = shape->next, ++shape_index) {
    if (shape == bed_shape) {
      const auto bed_contours = extract_contours(*shape);
      auto bed_polygons = build_polygons_from_contours(bed_contours);
      if (bed_polygons.empty()) {
        return Status::invalid_input;
      }
      std::stable_sort(bed_polygons.begin(), bed_polygons.end(),
                       [](const auto &lhs, const auto &rhs) {
                         return absolute_area(lhs.outer) >
                                absolute_area(rhs.outer);
                       });
      imported.bed = std::move(bed_polygons.front());
      continue;
    }

    const auto key = group_key_for_shape(shape->id, shape_index);
    if (!selected_groups.contains(key)) {
      continue;
    }

    auto contours = extract_contours(*shape);
    if (contours.empty()) {
      continue;
    }

    auto &bucket = grouped_contours[key];
    std::move(contours.begin(), contours.end(), std::back_inserter(bucket));
  }

  if (imported.bed.outer.empty()) {
    return Status::invalid_input;
  }

  for (auto &[key, contours] : grouped_contours) {
    auto polygons = build_polygons_from_contours(std::move(contours));
    std::stable_sort(
        polygons.begin(), polygons.end(), [](const auto &lhs, const auto &rhs) {
          return absolute_area(lhs.outer) > absolute_area(rhs.outer);
        });

    for (std::size_t index = 0; index < polygons.size(); ++index) {
      const auto area = absolute_area(polygons[index].outer);
      if (area <= std::max(kMinimumArea, spec.min_piece_area)) {
        continue;
      }
      auto piece_key = key;
      if (polygons.size() > 1U) {
        piece_key += "-" + std::to_string(index + 1U);
      }
      imported.pieces.push_back({.key = std::move(piece_key),
                                 .polygon = std::move(polygons[index]),
                                 .area = area});
    }
  }

  std::stable_sort(imported.pieces.begin(), imported.pieces.end(),
                   [](const auto &lhs, const auto &rhs) {
                     if (lhs.area != rhs.area) {
                       return lhs.area > rhs.area;
                     }
                     return lhs.key < rhs.key;
                   });

  if (imported.pieces.empty()) {
    return Status::invalid_input;
  }

  return imported;
}

auto make_decoder_request(const SvgPackingCaseSpec &spec,
                          const ImportedSvgCase &svg_case) -> DecoderRequest {
  const auto bed_bounds = bounds_for_ring(svg_case.bed.outer);
  const auto translated_bed =
      translate_polygon(svg_case.bed, -bed_bounds[0], -bed_bounds[1]);
  const auto translated_bed_bounds = bounds_for_ring(translated_bed.outer);
  const auto bed_width = translated_bed_bounds[2] - translated_bed_bounds[0];
  const auto bed_height = translated_bed_bounds[3] - translated_bed_bounds[1];
  const auto effective_bin_count =
      spec.max_bin_count == 0U
          ? std::max<std::size_t>(svg_case.pieces.size(), 1U)
          : spec.max_bin_count;
  const auto expand_bins = [&](const BinInput &base_bin)
      -> std::vector<BinInput> {
    std::vector<BinInput> bins;
    bins.reserve(effective_bin_count);
    for (std::size_t index = 0; index < effective_bin_count; ++index) {
      BinInput bin = base_bin;
      bin.bin_id = base_bin.bin_id + static_cast<std::uint32_t>(index);
      bins.push_back(std::move(bin));
    }
    return bins;
  };

  DecoderRequest request{
      .bins = expand_bins({.bin_id = 1,
                           .polygon = translated_bed,
                           .geometry_revision = 1}),
      .policy = PlacementPolicy::bottom_left,
      .config =
          PackingConfig{
              .placement =
                  {
                      .allowed_rotations = {.angles_degrees =
                                                spec.allowed_rotations_degrees},
                      .bed_grain_direction = spec.bed_grain_direction,
                      .explore_concave_candidates = true,
                      .exclusion_zones = spec.exclusion_zones,
                  },
              .enable_hole_first_placement = false,
          },
  };

  std::vector<const ImportedPiece *> fitting_pieces;
  fitting_pieces.reserve(svg_case.pieces.size());
  for (const auto &piece : svg_case.pieces) {
    const auto piece_bounds = bounds_for_ring(piece.polygon.outer);
    const auto piece_width = piece_bounds[2] - piece_bounds[0];
    const auto piece_height = piece_bounds[3] - piece_bounds[1];
    if (piece_width > bed_width || piece_height > bed_height) {
      continue;
    }

    fitting_pieces.push_back(&piece);
  }

  std::stable_sort(fitting_pieces.begin(), fitting_pieces.end(),
                   [](const ImportedPiece *lhs, const ImportedPiece *rhs) {
                     if (lhs->area != rhs->area) {
                       return lhs->area > rhs->area;
                     }
                     return lhs->key < rhs->key;
                   });

  const auto piece_limit =
      spec.max_piece_count == 0U
          ? fitting_pieces.size()
          : std::min(spec.max_piece_count, fitting_pieces.size());
  request.pieces.reserve(piece_limit);
  for (std::size_t piece_index = 0; piece_index < fitting_pieces.size() &&
                                    request.pieces.size() < piece_limit;
       ++piece_index) {
    const auto piece_bounds =
        bounds_for_ring(fitting_pieces[piece_index]->polygon.outer);
    const auto translated_piece =
        translate_polygon(fitting_pieces[piece_index]->polygon,
                          -piece_bounds[0], -piece_bounds[1]);

    request.pieces.push_back(
        {.piece_id = static_cast<std::uint32_t>(piece_index + 1U),
         .polygon = translated_piece,
         .geometry_revision = static_cast<std::uint64_t>(piece_index + 1U),
         .grain_compatibility = [&]() {
           const auto &piece_key = fitting_pieces[piece_index]->key;
           const auto it = spec.piece_grain_compatibility.find(piece_key);
           if (it == spec.piece_grain_compatibility.end()) {
             return PartGrainCompatibility::unrestricted;
           }
           return it->second;
         }()});
  }

  return request;
}

auto make_search_request(const SvgPackingCaseSpec &spec,
                         const ImportedSvgCase &svg_case) -> SearchRequest {
  SearchRequest request{};
  request.decoder_request = make_decoder_request(spec, svg_case);
  request.local_search = {
      .max_iterations = spec.search_iterations,
      .deterministic_seed = kSearchSeed,
      .plateau_budget = spec.search_plateau_budget,
  };
  request.execution.control.worker_count = 4U;
  request.execution.control.time_budget_ms = spec.execution_time_budget_ms;
  install_live_progress_observer(spec, request);
  return request;
}

auto make_masonry_request(const SvgPackingCaseSpec &spec,
                          const ImportedSvgCase &svg_case) -> MasonryRequest {
  MasonryRequest request{};
  request.decoder_request = make_decoder_request(spec, svg_case);
  install_live_progress_observer(spec, request);
  return request;
}

auto write_layout_svg(const std::filesystem::path &path, const Layout &layout)
    -> Status {
  if (layout.bins.empty() || layout.bins.front().container.outer.empty()) {
    return Status::invalid_input;
  }

  fs::create_directories(path.parent_path());
  std::ofstream stream(path);
  if (!stream) {
    return Status::computation_failed;
  }

  const auto bounds = bounds_for_ring(layout.bins.front().container.outer);
  const auto width = std::max(1.0, bounds[2] - bounds[0]);
  const auto height = std::max(1.0, bounds[3] - bounds[1]);
  const std::array<std::string_view, 8> palette{
      "#cf5c36", "#2f4858", "#33658a", "#86bbd8",
      "#758e4f", "#f6ae2d", "#8d6a9f", "#4f6d7a",
  };

  stream << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"" << bounds[0]
         << ' ' << bounds[1] << ' ' << width << ' ' << height
         << "\" fill-rule=\"evenodd\">\n";
  stream << "  <rect x=\"" << bounds[0] << "\" y=\"" << bounds[1]
         << "\" width=\"" << width << "\" height=\"" << height
         << "\" fill=\"#ffffff\" />\n";
  stream << "  <path d=\"" << svg_path_data(layout.bins.front().container)
         << "\" fill=\"none\" stroke=\"#101820\" stroke-width=\"1\" />\n";

  for (std::size_t index = 0; index < layout.bins.front().placements.size();
       ++index) {
    const auto &placed = layout.bins.front().placements[index];
    stream << "  <path d=\"" << svg_path_data(placed.polygon) << "\" fill=\""
           << palette[index % palette.size()]
           << "\" fill-opacity=\"0.55\" stroke=\"#0f172a\" "
              "stroke-width=\"0.75\" />\n";
  }

  stream << "</svg>\n";
  return Status::ok;
}

void require_valid_imported_case(const SvgPackingCaseSpec &spec,
                                 const ImportedSvgCase &imported) {
  REQUIRE_FALSE(spec.id.empty());
  REQUIRE_FALSE(spec.description.empty());
  REQUIRE_FALSE(spec.source.empty());
  REQUIRE_FALSE(spec.acceptance_lane.empty());
  REQUIRE_FALSE(spec.coverage_features.empty());
  REQUIRE_FALSE(spec.algorithm_applicability.empty());
  REQUIRE((spec.acceptance_lane == "readiness" ||
           spec.acceptance_lane == "stress" || spec.acceptance_lane == "soak"));

  REQUIRE_FALSE(imported.source_path.empty());
  REQUIRE_FALSE(imported.bed.outer.empty());
  REQUIRE_FALSE(imported.pieces.empty());

  const auto minimum_piece_area = std::max(kMinimumArea, spec.min_piece_area);
  std::set<std::string> seen_piece_keys;
  for (const auto &piece : imported.pieces) {
    REQUIRE_FALSE(piece.key.empty());
    REQUIRE(seen_piece_keys.insert(piece.key).second);
    REQUIRE_FALSE(piece.polygon.outer.empty());
    REQUIRE(piece.area > minimum_piece_area);
    REQUIRE(absolute_area(piece.polygon.outer) > minimum_piece_area);
  }
}

void require_request_matches_imported_case(const SvgPackingCaseSpec &spec,
                                           const ImportedSvgCase &imported,
                                           const DecoderRequest &request) {
  REQUIRE_FALSE(request.bins.empty());
  REQUIRE_FALSE(request.bins.front().polygon.outer.empty());
  REQUIRE_FALSE(request.pieces.empty());
  REQUIRE(request.pieces.size() <= imported.pieces.size());
  REQUIRE(request.pieces.size() >= spec.min_placed_piece_count);
  if (spec.max_piece_count > 0U) {
    REQUIRE(request.pieces.size() <= spec.max_piece_count);
  }

  std::set<std::uint32_t> seen_piece_ids;
  for (const auto &piece : request.pieces) {
    REQUIRE(seen_piece_ids.insert(piece.piece_id).second);
    REQUIRE(piece.geometry_revision ==
            static_cast<std::uint64_t>(piece.piece_id));
    REQUIRE_FALSE(piece.polygon.outer.empty());
  }
}

void require_valid_decoder_result(const SvgPackingCaseSpec &spec,
                                  const DecoderRequest &request,
                                  const DecoderResult &result) {
  const auto effective_bin_count = spec.max_bin_count == 0U
                                       ? std::max<std::size_t>(
                                             request.pieces.size(), 1U)
                                       : spec.max_bin_count;
  require_layout_consistency(request, result.layout,
                             result.layout.placement_trace.size(),
                             result.layout.unplaced_piece_ids.size());
  REQUIRE(result.layout.bins.size() >= 1U);
  REQUIRE(result.layout.bins.size() <= effective_bin_count);
  REQUIRE(result.layout.placement_trace.size() >= spec.min_placed_piece_count);
  if (spec.require_full_placement) {
    REQUIRE(result.layout.unplaced_piece_ids.empty());
  }
}

void require_valid_search_result(const SvgPackingCaseSpec &spec,
                                 const SearchRequest &request,
                                 const SearchResult &result) {
  const auto effective_bin_count = spec.max_bin_count == 0U
                                       ? std::max<std::size_t>(
                                             request.decoder_request.pieces.size(),
                                             1U)
                                       : spec.max_bin_count;
  REQUIRE(result.status != SearchRunStatus::invalid_request);
  REQUIRE(result.deterministic_seed == kSearchSeed);
  REQUIRE(result.evaluated_layout_count > 0U);
  REQUIRE(result.best.placed_piece_count + result.best.unplaced_piece_count ==
          request.decoder_request.pieces.size());
  REQUIRE(result.best.decode.layout.bins.size() == result.best.bin_count);

  require_layout_consistency(request.decoder_request, result.best.decode.layout,
                             result.best.placed_piece_count,
                             result.best.unplaced_piece_count);

  REQUIRE(result.best.bin_count >= 1U);
  REQUIRE(result.best.bin_count <= effective_bin_count);
  REQUIRE(result.best.placed_piece_count >= spec.min_placed_piece_count);
  if (spec.require_full_placement) {
    REQUIRE(result.best.unplaced_piece_count == 0U);
  }
  if (spec.require_observable_success_before_interrupt &&
      (result.status == SearchRunStatus::timed_out ||
       result.status == SearchRunStatus::cancelled)) {
    REQUIRE_FALSE(result.progress.empty());
    REQUIRE(result.progress.front().iteration == 0U);
    REQUIRE(result.progress.back().iteration >= result.iterations_completed);
  }
}

void require_valid_masonry_result(const SvgPackingCaseSpec &spec,
                                  const MasonryRequest &request,
                                  const MasonryResult &result) {
  const auto effective_bin_count = spec.max_bin_count == 0U
                                       ? std::max<std::size_t>(
                                             request.decoder_request.pieces.size(),
                                             1U)
                                       : spec.max_bin_count;
  REQUIRE(result.algorithm == AlgorithmKind::masonry_builder);
  REQUIRE(result.layout.bins.size() == result.bins.size());
  REQUIRE(result.trace.size() == result.layout.placement_trace.size());
  REQUIRE(result.progress.size() == request.decoder_request.pieces.size());

  require_layout_consistency(request.decoder_request, result.layout,
                             result.layout.placement_trace.size(),
                             result.layout.unplaced_piece_ids.size());

  REQUIRE(result.layout.bins.size() >= 1U);
  REQUIRE(result.layout.bins.size() <= effective_bin_count);
  REQUIRE(result.layout.placement_trace.size() >= spec.min_placed_piece_count);
  if (spec.require_full_placement) {
    REQUIRE(result.layout.unplaced_piece_ids.empty());
  }
}

void require_same_layout(const Layout &lhs, const Layout &rhs) {
  REQUIRE(lhs.bins.size() == rhs.bins.size());
  for (std::size_t bin_index = 0; bin_index < lhs.bins.size(); ++bin_index) {
    const auto &lhs_bin = lhs.bins[bin_index];
    const auto &rhs_bin = rhs.bins[bin_index];

    REQUIRE(lhs_bin.bin_id == rhs_bin.bin_id);
    require_same_polygon(lhs_bin.container, rhs_bin.container);
    REQUIRE(lhs_bin.placements.size() == rhs_bin.placements.size());
    for (std::size_t placement_index = 0;
         placement_index < lhs_bin.placements.size(); ++placement_index) {
      const auto &lhs_piece = lhs_bin.placements[placement_index];
      const auto &rhs_piece = rhs_bin.placements[placement_index];
      REQUIRE(lhs_piece.placement.piece_id == rhs_piece.placement.piece_id);
      REQUIRE(lhs_piece.placement.bin_id == rhs_piece.placement.bin_id);
      REQUIRE(nearly_equal(lhs_piece.placement.translation.x,
                           rhs_piece.placement.translation.x));
      REQUIRE(nearly_equal(lhs_piece.placement.translation.y,
                           rhs_piece.placement.translation.y));
      require_same_polygon(lhs_piece.polygon, rhs_piece.polygon);
    }
  }

  REQUIRE(lhs.placement_trace.size() == rhs.placement_trace.size());
  for (std::size_t index = 0; index < lhs.placement_trace.size(); ++index) {
    const auto &lhs_entry = lhs.placement_trace[index];
    const auto &rhs_entry = rhs.placement_trace[index];
    REQUIRE(lhs_entry.piece_id == rhs_entry.piece_id);
    REQUIRE(lhs_entry.bin_id == rhs_entry.bin_id);
    REQUIRE(nearly_equal(lhs_entry.translation.x, rhs_entry.translation.x));
    REQUIRE(nearly_equal(lhs_entry.translation.y, rhs_entry.translation.y));
    REQUIRE(lhs_entry.opened_new_bin == rhs_entry.opened_new_bin);
  }

  REQUIRE(lhs.unplaced_piece_ids == rhs.unplaced_piece_ids);
}

void require_same_decoder_result(const DecoderResult &lhs,
                                 const DecoderResult &rhs) {
  REQUIRE(lhs.interrupted == rhs.interrupted);
  require_same_layout(lhs.layout, rhs.layout);
}

void require_same_search_result(const SearchResult &lhs,
                                const SearchResult &rhs) {
  REQUIRE(lhs.algorithm == rhs.algorithm);
  REQUIRE(lhs.status == rhs.status);
  REQUIRE(lhs.deterministic_seed == rhs.deterministic_seed);
  REQUIRE(lhs.iterations_completed == rhs.iterations_completed);
  REQUIRE(lhs.evaluated_layout_count == rhs.evaluated_layout_count);
  REQUIRE(lhs.reevaluation_cache_hits == rhs.reevaluation_cache_hits);
  REQUIRE(lhs.best.piece_order == rhs.best.piece_order);
  REQUIRE(lhs.best.bin_count == rhs.best.bin_count);
  REQUIRE(lhs.best.placed_piece_count == rhs.best.placed_piece_count);
  REQUIRE(lhs.best.unplaced_piece_count == rhs.best.unplaced_piece_count);
  REQUIRE(nearly_equal(lhs.best.total_utilization, rhs.best.total_utilization));
  REQUIRE(
      nearly_equal(lhs.best.average_utilization, rhs.best.average_utilization));
  REQUIRE(nearly_equal(lhs.best.last_bin_utilization,
                       rhs.best.last_bin_utilization));
  REQUIRE(lhs.progress.size() == rhs.progress.size());
  for (std::size_t index = 0; index < lhs.progress.size(); ++index) {
    const auto &lhs_entry = lhs.progress[index];
    const auto &rhs_entry = rhs.progress[index];
    REQUIRE(lhs_entry.algorithm_kind == rhs_entry.algorithm_kind);
    REQUIRE(lhs_entry.iteration == rhs_entry.iteration);
    REQUIRE(lhs_entry.iteration_budget == rhs_entry.iteration_budget);
    REQUIRE(lhs_entry.move_kind == rhs_entry.move_kind);
    REQUIRE(lhs_entry.improved == rhs_entry.improved);
    REQUIRE(lhs_entry.evaluated_layout_count ==
            rhs_entry.evaluated_layout_count);
    REQUIRE(lhs_entry.reevaluation_cache_hits ==
            rhs_entry.reevaluation_cache_hits);
    REQUIRE(lhs_entry.best_bin_count == rhs_entry.best_bin_count);
    REQUIRE(lhs_entry.best_placed_piece_count ==
            rhs_entry.best_placed_piece_count);
    REQUIRE(lhs_entry.best_unplaced_piece_count ==
            rhs_entry.best_unplaced_piece_count);
    REQUIRE(nearly_equal(lhs_entry.best_total_utilization,
                         rhs_entry.best_total_utilization));
    REQUIRE(lhs_entry.best_piece_order == rhs_entry.best_piece_order);
  }
  REQUIRE(lhs.best.decode.interrupted == rhs.best.decode.interrupted);
  require_same_layout(lhs.best.decode.layout, rhs.best.decode.layout);
}

void require_same_masonry_result(const MasonryResult &lhs,
                                 const MasonryResult &rhs) {
  REQUIRE(lhs.algorithm == rhs.algorithm);
  require_same_layout(lhs.layout, rhs.layout);
  REQUIRE(lhs.trace.size() == rhs.trace.size());
  for (std::size_t index = 0; index < lhs.trace.size(); ++index) {
    const auto &left = lhs.trace[index];
    const auto &right = rhs.trace[index];
    REQUIRE(left.piece_id == right.piece_id);
    REQUIRE(left.bin_id == right.bin_id);
    REQUIRE(left.shelf_index == right.shelf_index);
    REQUIRE(left.rotation_index == right.rotation_index);
    REQUIRE(left.resolved_rotation.degrees == right.resolved_rotation.degrees);
    REQUIRE(nearly_equal(left.translation.x, right.translation.x));
    REQUIRE(nearly_equal(left.translation.y, right.translation.y));
    REQUIRE(left.opened_new_bin == right.opened_new_bin);
    REQUIRE(left.started_new_shelf == right.started_new_shelf);
    REQUIRE(left.inside_hole == right.inside_hole);
    REQUIRE(left.hole_index == right.hole_index);
  }
  REQUIRE(lhs.progress.size() == rhs.progress.size());
}

} // namespace shiny::nesting::test::svg
