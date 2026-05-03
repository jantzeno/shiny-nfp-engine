#include "io/import_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry/operations/simplify.hpp"
#include "geometry/queries/normalize.hpp"

namespace shiny::nesting::io {
namespace detail {

constexpr int kMaxBezierDepth = 12;

[[nodiscard]] auto distance_point_to_line(const geom::Point2 &point,
                                          const geom::Point2 &start,
                                          const geom::Point2 &end) -> double {
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  const double denominator = dx * dx + dy * dy;
  if (denominator <= std::numeric_limits<double>::epsilon()) {
    const double px = point.x() - start.x();
    const double py = point.y() - start.y();
    return std::sqrt(px * px + py * py);
  }

  const double t =
      std::clamp(((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) /
                     denominator,
                 0.0, 1.0);
  const double projected_x = start.x() + dx * t;
  const double projected_y = start.y() + dy * t;
  const double px = point.x() - projected_x;
  const double py = point.y() - projected_y;
  return std::sqrt(px * px + py * py);
}

auto append_cubic_bezier_points(const geom::Point2 &start,
                                const geom::Point2 &control1,
                                const geom::Point2 &control2,
                                const geom::Point2 &end, const double tolerance,
                                std::vector<geom::Point2> &samples,
                                const int depth = 0) -> void {
  const double flatness =
      std::max(distance_point_to_line(control1, start, end),
               distance_point_to_line(control2, start, end));
  if (depth >= kMaxBezierDepth || flatness <= tolerance) {
    samples.push_back(end);
    return;
  }

  const geom::Point2 start_control{(start.x() + control1.x()) * 0.5,
                                   (start.y() + control1.y()) * 0.5};
  const geom::Point2 control_mid{(control1.x() + control2.x()) * 0.5,
                                 (control1.y() + control2.y()) * 0.5};
  const geom::Point2 control_end{(control2.x() + end.x()) * 0.5,
                                 (control2.y() + end.y()) * 0.5};
  const geom::Point2 left_mid{(start_control.x() + control_mid.x()) * 0.5,
                              (start_control.y() + control_mid.y()) * 0.5};
  const geom::Point2 right_mid{(control_mid.x() + control_end.x()) * 0.5,
                               (control_mid.y() + control_end.y()) * 0.5};
  const geom::Point2 split{(left_mid.x() + right_mid.x()) * 0.5,
                           (left_mid.y() + right_mid.y()) * 0.5};

  append_cubic_bezier_points(start, start_control, left_mid, split, tolerance,
                             samples, depth + 1);
  append_cubic_bezier_points(split, right_mid, control_end, end, tolerance,
                             samples, depth + 1);
}

auto append_segment_samples(const ImportedPathSegment &segment,
                            const double tolerance,
                            std::vector<geom::Point2> &samples) -> void {
  switch (segment.kind) {
  case ImportedPathSegmentKind::line:
    samples.push_back(segment.end);
    return;
  case ImportedPathSegmentKind::cubic_bezier:
    append_cubic_bezier_points(segment.start, segment.control1,
                               segment.control2, segment.end, tolerance,
                               samples);
    return;
  }
}

[[nodiscard]] auto ring_from_shape(const ImportedShape &shape,
                                   const ImportPreprocessOptions &options,
                                   const bool normalize_piece_origins)
    -> std::expected<geom::PolygonWithHoles, util::Status> {
  auto outer_or = flatten_ring(shape.outer, options.flatten_tolerance);
  if (!outer_or.has_value()) {
    return std::unexpected(outer_or.error());
  }

  geom::PolygonWithHoles polygon;
  polygon.outer() = std::move(outer_or.value());
  for (const auto &hole : shape.holes) {
    auto hole_or = flatten_ring(hole, options.flatten_tolerance);
    if (!hole_or.has_value()) {
      return std::unexpected(hole_or.error());
    }
    polygon.holes().push_back(std::move(hole_or.value()));
  }

  polygon = geom::normalize_polygon(polygon);
  if (options.simplify_epsilon > 0.0) {
    polygon = geom::simplify_polygon_douglas_peucker(polygon,
                                                     options.simplify_epsilon);
  } else {
    polygon = geom::simplify_polygon(polygon);
  }

  if (normalize_piece_origins) {
    const auto bounds = geom::compute_bounds(polygon);
    polygon = geom::translate(polygon,
                              geom::Vector2{-bounds.min.x(), -bounds.min.y()});
  }
  return geom::normalize_polygon(polygon);
}

} // namespace detail

auto flatten_ring(const ImportedRing &ring, const double tolerance)
    -> std::expected<geom::Ring, util::Status> {
  if (!ring.closed || ring.segments.empty() || !std::isfinite(tolerance) ||
      tolerance <= 0.0) {
    return std::unexpected(util::Status::invalid_input);
  }

  std::vector<geom::Point2> samples;
  samples.reserve(ring.segments.size() * 2U);
  samples.push_back(ring.segments.front().start);
  for (const auto &segment : ring.segments) {
    detail::append_segment_samples(segment, tolerance, samples);
  }

  geom::Ring flattened;
  flattened.reserve(samples.size());
  for (const auto &point : samples) {
    if (!flattened.empty() && flattened.back() == point) {
      continue;
    }
    flattened.push_back(point);
  }
  if (flattened.size() > 1U && flattened.front() == flattened.back()) {
    flattened.pop_back();
  }
  if (flattened.size() < 3U) {
    return std::unexpected(util::Status::invalid_input);
  }
  return flattened;
}

auto preprocess_import_request(const ImportPreprocessRequest &request)
    -> std::expected<NormalizedRequest, util::Status> {
  if (!std::isfinite(request.options.flatten_tolerance) ||
      request.options.flatten_tolerance <= 0.0 ||
      !std::isfinite(request.options.simplify_epsilon) ||
      request.options.simplify_epsilon < 0.0) {
    return std::unexpected(util::Status::invalid_input);
  }

  NestingRequest normalized_request = request.base_request;
  normalized_request.preprocess.simplify_epsilon =
      request.options.simplify_epsilon;
  normalized_request.preprocess.normalize_piece_origins =
      request.options.normalize_piece_origins;
  normalized_request.preprocess.discard_empty_bins =
      request.options.discard_empty_bins;

  for (const auto &bin : request.bins) {
    if (request.options.discard_empty_bins && bin.stock == 0U) {
      continue;
    }

    auto polygon_or =
        detail::ring_from_shape(bin.shape, request.options, false);
    if (!polygon_or.has_value()) {
      return std::unexpected(polygon_or.error());
    }

    normalized_request.bins.push_back({
        .bin_id = bin.bin_id,
        .polygon = std::move(polygon_or.value()),
        .stock = bin.stock,
        .start_corner = bin.start_corner,
        .exclusion_zones = bin.exclusion_zones,
    });
  }

  for (const auto &piece : request.pieces) {
    auto polygon_or = detail::ring_from_shape(
        piece.shape, request.options, request.options.normalize_piece_origins);
    if (!polygon_or.has_value()) {
      return std::unexpected(polygon_or.error());
    }

    normalized_request.pieces.push_back({
        .piece_id = piece.piece_id,
        .polygon = std::move(polygon_or.value()),
        .quantity = piece.quantity,
        .allowed_rotations = piece.allowed_rotations,
        .grain_compatibility = piece.grain_compatibility,
        .allowed_bin_ids = piece.allowed_bin_ids,
    });
  }

  return normalize_request(normalized_request);
}

} // namespace shiny::nesting::io
