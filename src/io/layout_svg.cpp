#include "io/layout_svg.hpp"

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>

#include "geometry/polygon.hpp"

namespace shiny::nesting::io {
namespace {

namespace fs = std::filesystem;

[[nodiscard]] auto path_is_safe_svg(const fs::path &path) -> bool {
  if (path.extension() != ".svg") {
    return false;
  }
  for (const auto &component : path) {
    if (component == "..") {
      return false;
    }
  }
  return true;
}

[[nodiscard]] auto ensure_output_directory(const fs::path &path) -> bool {
  const auto parent = path.parent_path();
  if (parent.empty()) {
    return true;
  }
  std::error_code error;
  fs::create_directories(parent, error);
  return !error;
}

[[nodiscard]] auto ring_to_path_data(const geom::Ring &ring) -> std::string {
  if (ring.empty()) {
    return {};
  }
  std::ostringstream stream;
  stream << "M " << ring.front().x << ' ' << ring.front().y;
  for (std::size_t index = 1; index < ring.size(); ++index) {
    stream << " L " << ring[index].x << ' ' << ring[index].y;
  }
  stream << " Z";
  return stream.str();
}

[[nodiscard]] auto polygon_to_path_data(const geom::PolygonWithHoles &polygon)
    -> std::string {
  auto path_data = ring_to_path_data(polygon.outer);
  for (const auto &hole : polygon.holes) {
    path_data += " ";
    path_data += ring_to_path_data(hole);
  }
  return path_data;
}

[[nodiscard]] auto color_for_index(const std::size_t index)
    -> std::string_view {
  constexpr std::string_view kPalette[]{
      "#ffd166", "#06d6a0", "#118ab2", "#ef476f",
      "#8ecae6", "#b5179e", "#fb8500", "#90be6d",
  };
  return kPalette[index % std::size(kPalette)];
}

} // namespace

auto save_layout_svg(const fs::path &path, const pack::Layout &layout)
    -> util::Status {
  if (!path_is_safe_svg(path)) {
    return util::Status::invalid_input;
  }
  if (!ensure_output_directory(path)) {
    return util::Status::computation_failed;
  }

  geom::Box2 bounds{};
  bool initialized = false;
  const auto extend_bounds = [&](const geom::PolygonWithHoles &polygon) {
    const auto polygon_bounds = geom::compute_bounds(polygon);
    if (!initialized) {
      bounds = polygon_bounds;
      initialized = true;
      return;
    }
    bounds.min.x = std::min(bounds.min.x, polygon_bounds.min.x);
    bounds.min.y = std::min(bounds.min.y, polygon_bounds.min.y);
    bounds.max.x = std::max(bounds.max.x, polygon_bounds.max.x);
    bounds.max.y = std::max(bounds.max.y, polygon_bounds.max.y);
  };

  for (const auto &bin : layout.bins) {
    extend_bounds(bin.container);
    for (const auto &placement : bin.placements) {
      extend_bounds(placement.polygon);
    }
  }
  if (!initialized) {
    bounds = {.min = {.x = 0.0, .y = 0.0}, .max = {.x = 1.0, .y = 1.0}};
  }

  constexpr double kMargin = 5.0;
  const auto width = std::max(1.0, geom::box_width(bounds) + (2.0 * kMargin));
  const auto height = std::max(1.0, geom::box_height(bounds) + (2.0 * kMargin));

  std::ofstream output(path);
  if (!output.is_open()) {
    return util::Status::computation_failed;
  }

  output << std::format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{} {} {} {}\">\n",
      bounds.min.x - kMargin, bounds.min.y - kMargin, width, height);

  for (const auto &bin : layout.bins) {
    output << std::format("  <g id=\"bin-{}\">\n", bin.bin_id);
    output << std::format(
        "    <path id=\"bin-{}-container\" fill=\"none\" stroke=\"#111827\" "
        "stroke-width=\"1\" fill-rule=\"evenodd\" d=\"{}\" />\n",
        bin.bin_id, polygon_to_path_data(bin.container));
    for (std::size_t index = 0; index < bin.placements.size(); ++index) {
      const auto &placement = bin.placements[index];
      output << std::format(
          "    <path id=\"piece-{}\" data-bin-id=\"{}\" fill=\"{}\" "
          "fill-opacity=\"0.7\" stroke=\"#111827\" stroke-width=\"0.5\" "
          "fill-rule=\"evenodd\" d=\"{}\" />\n",
          placement.placement.piece_id, bin.bin_id, color_for_index(index),
          polygon_to_path_data(placement.polygon));
    }
    output << "  </g>\n";
  }
  output << "</svg>\n";

  return output.good() ? util::Status::ok : util::Status::computation_failed;
}

} // namespace shiny::nesting::io
