#include "io/svg_polygonize.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "geometry/normalize.hpp"

namespace shiny::nesting::io {
namespace {

struct Parser {
  std::string_view input;
  std::size_t index{0};

  auto skip_separators() -> void {
    while (index < input.size()) {
      const auto ch = static_cast<unsigned char>(input[index]);
      if (std::isspace(ch) || input[index] == ',') {
        ++index;
        continue;
      }
      break;
    }
  }

  [[nodiscard]] auto empty() -> bool {
    skip_separators();
    return index >= input.size();
  }

  [[nodiscard]] auto peek_is_command() -> bool {
    skip_separators();
    return index < input.size() &&
           std::isalpha(static_cast<unsigned char>(input[index])) != 0;
  }

  [[nodiscard]] auto read_command() -> char {
    skip_separators();
    return input[index++];
  }

  [[nodiscard]] auto read_number() -> std::optional<double> {
    skip_separators();
    if (index >= input.size()) {
      return std::nullopt;
    }

    const auto start = index;
    if (input[index] == '+' || input[index] == '-') {
      ++index;
    }
    while (index < input.size() &&
           std::isdigit(static_cast<unsigned char>(input[index])) != 0) {
      ++index;
    }
    if (index < input.size() && input[index] == '.') {
      ++index;
      while (index < input.size() &&
             std::isdigit(static_cast<unsigned char>(input[index])) != 0) {
        ++index;
      }
    }
    if (index < input.size() && (input[index] == 'e' || input[index] == 'E')) {
      ++index;
      if (index < input.size() &&
          (input[index] == '+' || input[index] == '-')) {
        ++index;
      }
      while (index < input.size() &&
             std::isdigit(static_cast<unsigned char>(input[index])) != 0) {
        ++index;
      }
    }

    double value = 0.0;
    const auto token = input.substr(start, index - start);
    const auto *begin = token.data();
    const auto *end = token.data() + token.size();
    auto [ptr, error] = std::from_chars(begin, end, value);
    if (error == std::errc{} && ptr == end) {
      return value;
    }

    try {
      return std::stod(std::string(token));
    } catch (...) {
      return std::nullopt;
    }
  }
};

[[nodiscard]] auto midpoint(const geom::Point2 &lhs, const geom::Point2 &rhs)
    -> geom::Point2 {
  return {.x = (lhs.x + rhs.x) * 0.5, .y = (lhs.y + rhs.y) * 0.5};
}

[[nodiscard]] auto point_line_distance(const geom::Point2 &point,
                                       const geom::Point2 &line_start,
                                       const geom::Point2 &line_end) -> double {
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

auto append_quadratic_points(std::vector<geom::Point2> &ring,
                             const geom::Point2 &start,
                             const geom::Point2 &control,
                             const geom::Point2 &end, double tolerance,
                             int depth = 0) -> void {
  if (depth >= 12 || point_line_distance(control, start, end) <= tolerance) {
    ring.push_back(end);
    return;
  }

  const auto start_control = midpoint(start, control);
  const auto control_end = midpoint(control, end);
  const auto split = midpoint(start_control, control_end);
  append_quadratic_points(ring, start, start_control, split, tolerance,
                          depth + 1);
  append_quadratic_points(ring, split, control_end, end, tolerance, depth + 1);
}

auto append_cubic_points(std::vector<geom::Point2> &ring,
                         const geom::Point2 &start,
                         const geom::Point2 &control_a,
                         const geom::Point2 &control_b, const geom::Point2 &end,
                         double tolerance, int depth = 0) -> void {
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

[[nodiscard]] auto make_point(double x, double y) -> geom::Point2 {
  return {.x = x, .y = y};
}

[[nodiscard]] auto resolve_point(const geom::Point2 &origin, bool relative,
                                 double x, double y) -> geom::Point2 {
  if (!relative) {
    return make_point(x, y);
  }
  return {.x = origin.x + x, .y = origin.y + y};
}

} // namespace

auto polygonize_svg_path(std::string_view svg_path_data,
                         const SvgImportConfig &config)
    -> util::StatusOr<geom::PolygonWithHoles> {
  if (config.curve_flattening_tolerance <= 0.0) {
    return util::Status::invalid_input;
  }

  Parser parser{.input = svg_path_data};
  std::vector<geom::Ring> rings;
  geom::Ring current_ring;
  geom::Point2 current{};
  geom::Point2 subpath_start{};
  char command = '\0';

  while (!parser.empty()) {
    if (parser.peek_is_command()) {
      command = parser.read_command();
    }

    if (command == '\0') {
      return util::Status::invalid_input;
    }

    const bool relative =
        std::islower(static_cast<unsigned char>(command)) != 0;
    const auto opcode = static_cast<char>(std::toupper(command));

    if (opcode == 'Z') {
      if (current_ring.size() < 3U) {
        return util::Status::invalid_input;
      }
      rings.push_back(current_ring);
      current_ring.clear();
      current = subpath_start;
      command = '\0';
      continue;
    }

    if (opcode == 'M') {
      const auto x = parser.read_number();
      const auto y = parser.read_number();
      if (!x || !y) {
        return util::Status::invalid_input;
      }

      if (!current_ring.empty()) {
        return util::Status::invalid_input;
      }

      current = resolve_point(current, relative, *x, *y);
      subpath_start = current;
      current_ring.push_back(current);
      command = relative ? 'l' : 'L';
      continue;
    }

    if (current_ring.empty()) {
      return util::Status::invalid_input;
    }

    if (opcode == 'L') {
      const auto x = parser.read_number();
      const auto y = parser.read_number();
      if (!x || !y) {
        return util::Status::invalid_input;
      }
      current = resolve_point(current, relative, *x, *y);
      current_ring.push_back(current);
      continue;
    }

    if (opcode == 'Q') {
      const auto cx = parser.read_number();
      const auto cy = parser.read_number();
      const auto x = parser.read_number();
      const auto y = parser.read_number();
      if (!cx || !cy || !x || !y) {
        return util::Status::invalid_input;
      }
      const auto control = resolve_point(current, relative, *cx, *cy);
      const auto end = resolve_point(current, relative, *x, *y);
      append_quadratic_points(current_ring, current, control, end,
                              config.curve_flattening_tolerance);
      current = end;
      continue;
    }

    if (opcode == 'C') {
      const auto c1x = parser.read_number();
      const auto c1y = parser.read_number();
      const auto c2x = parser.read_number();
      const auto c2y = parser.read_number();
      const auto x = parser.read_number();
      const auto y = parser.read_number();
      if (!c1x || !c1y || !c2x || !c2y || !x || !y) {
        return util::Status::invalid_input;
      }
      const auto control_a = resolve_point(current, relative, *c1x, *c1y);
      const auto control_b = resolve_point(current, relative, *c2x, *c2y);
      const auto end = resolve_point(current, relative, *x, *y);
      append_cubic_points(current_ring, current, control_a, control_b, end,
                          config.curve_flattening_tolerance);
      current = end;
      continue;
    }

    return util::Status::invalid_input;
  }

  if (!current_ring.empty()) {
    return util::Status::invalid_input;
  }
  if (rings.empty()) {
    return util::Status::invalid_input;
  }

  geom::PolygonWithHoles polygon;
  polygon.outer = std::move(rings.front());
  for (std::size_t index = 1; index < rings.size(); ++index) {
    polygon.holes.push_back(std::move(rings[index]));
  }

  polygon = geom::normalize_polygon(polygon);
  if (polygon.outer.size() < 3U) {
    return util::Status::invalid_input;
  }

  return polygon;
}

} // namespace shiny::nesting::io