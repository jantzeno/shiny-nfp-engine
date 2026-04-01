#include "io/json.hpp"

#include <system_error>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace shiny::nfp::io {
namespace {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;

constexpr std::uintmax_t kMaxJsonInputBytes = 4U * 1024U * 1024U;

[[nodiscard]] auto path_is_safe_json(const fs::path &path) -> bool {
  if (path.extension() != ".json") {
    return false;
  }

  for (const auto &component : path) {
    if (component == "..") {
      return false;
    }
  }

  return true;
}

[[nodiscard]] auto make_point_node(const geom::Point2 &point) -> pt::ptree {
  pt::ptree node;
  pt::ptree x;
  x.put_value(point.x);
  pt::ptree y;
  y.put_value(point.y);
  node.push_back({"", x});
  node.push_back({"", y});
  return node;
}

[[nodiscard]] auto parse_point_node(const pt::ptree &node) -> geom::Point2 {
  auto iterator = node.begin();
  const auto x = iterator->second.get_value<double>();
  ++iterator;
  const auto y = iterator->second.get_value<double>();
  return {.x = x, .y = y};
}

[[nodiscard]] auto make_ring_node(const geom::Ring &ring) -> pt::ptree {
  pt::ptree node;
  for (const auto &point : ring) {
    node.push_back({"", make_point_node(point)});
  }
  return node;
}

[[nodiscard]] auto parse_ring_node(const pt::ptree &node) -> geom::Ring {
  geom::Ring ring;
  for (const auto &child : node) {
    ring.push_back(parse_point_node(child.second));
  }
  return ring;
}

[[nodiscard]] auto make_polygon_node(const geom::PolygonWithHoles &polygon)
    -> pt::ptree {
  pt::ptree node;
  node.add_child("outer", make_ring_node(polygon.outer));

  pt::ptree holes;
  for (const auto &hole : polygon.holes) {
    holes.push_back({"", make_ring_node(hole)});
  }
  node.add_child("holes", holes);
  return node;
}

[[nodiscard]] auto parse_polygon_node(const pt::ptree &node)
    -> geom::PolygonWithHoles {
  geom::PolygonWithHoles polygon;
  polygon.outer = parse_ring_node(node.get_child("outer"));
  if (const auto holes = node.get_child_optional("holes")) {
    for (const auto &hole : *holes) {
      polygon.holes.push_back(parse_ring_node(hole.second));
    }
  }
  return polygon;
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

[[nodiscard]] auto read_json_tree(const fs::path &path)
    -> util::StatusOr<pt::ptree> {
  if (!path_is_safe_json(path)) {
    return util::Status::invalid_input;
  }

  std::error_code error;
  if (!fs::exists(path, error) || error) {
    return util::Status::computation_failed;
  }

  const auto bytes = fs::file_size(path, error);
  if (error) {
    return util::Status::computation_failed;
  }
  if (bytes > kMaxJsonInputBytes) {
    return util::Status::invalid_input;
  }

  pt::ptree tree;
  try {
    pt::read_json(path.string(), tree);
  } catch (...) {
    return util::Status::computation_failed;
  }

  return tree;
}

[[nodiscard]] auto write_json_tree(const fs::path &path, const pt::ptree &tree)
    -> util::Status {
  if (!path_is_safe_json(path)) {
    return util::Status::invalid_input;
  }
  if (!ensure_output_directory(path)) {
    return util::Status::computation_failed;
  }

  try {
    pt::write_json(path.string(), tree);
  } catch (...) {
    return util::Status::computation_failed;
  }

  return util::Status::ok;
}

} // namespace

auto load_polygon_set(const fs::path &path)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>> {
  auto tree_or = read_json_tree(path);
  if (!tree_or.ok()) {
    return tree_or.status();
  }

  std::vector<geom::PolygonWithHoles> polygons;
  try {
    const auto &tree = tree_or.value();
    if (const auto polygon_nodes = tree.get_child_optional("polygons")) {
      for (const auto &polygon_node : *polygon_nodes) {
        polygons.push_back(parse_polygon_node(polygon_node.second));
      }
    } else if (const auto outer = tree.get_child_optional("outer")) {
      polygons.push_back(parse_polygon_node(tree));
    } else {
      return util::Status::invalid_input;
    }
  } catch (...) {
    return util::Status::computation_failed;
  }

  return polygons;
}

auto save_polygon_set(const fs::path &path,
                      std::span<const geom::PolygonWithHoles> polygons)
    -> util::Status {
  pt::ptree root;
  root.put("kind", "polygon_set");

  pt::ptree polygon_nodes;
  for (const auto &polygon : polygons) {
    polygon_nodes.push_back({"", make_polygon_node(polygon)});
  }
  root.add_child("polygons", polygon_nodes);

  return write_json_tree(path, root);
}

auto save_layout(const fs::path &path, const pack::Layout &layout)
    -> util::Status {
  pt::ptree root;
  root.put("kind", "layout");

  pt::ptree bins;
  for (const auto &bin : layout.bins) {
    pt::ptree bin_node;
    bin_node.put("bin_id", bin.bin_id);
    bin_node.add_child("container", make_polygon_node(bin.container));
    bin_node.put("placement_count", bin.placements.size());

    pt::ptree placements;
    for (const auto &placement : bin.placements) {
      pt::ptree placement_node;
      placement_node.put("piece_id", placement.placement.piece_id);
      placement_node.put("rotation_index",
                         placement.placement.rotation_index.value);
      placement_node.add_child(
          "translation", make_point_node(placement.placement.translation));
      placement_node.add_child("polygon", make_polygon_node(placement.polygon));
      placements.push_back({"", placement_node});
    }
    bin_node.add_child("placements", placements);
    bins.push_back({"", bin_node});
  }
  root.add_child("bins", bins);

  pt::ptree unplaced;
  for (const auto piece_id : layout.unplaced_piece_ids) {
    pt::ptree value;
    value.put_value(piece_id);
    unplaced.push_back({"", value});
  }
  root.add_child("unplaced_piece_ids", unplaced);

  return write_json_tree(path, root);
}

auto save_cut_plan(const fs::path &path, const pack::CutPlan &cut_plan)
    -> util::Status {
  pt::ptree root;
  root.put("kind", "cut_plan");
  root.put("raw_cut_length", cut_plan.raw_cut_length);
  root.put("total_cut_length", cut_plan.total_cut_length);
  root.put("removed_cut_length", cut_plan.removed_cut_length);

  pt::ptree segments;
  for (const auto &segment : cut_plan.segments) {
    pt::ptree segment_node;
    segment_node.put("bin_id", segment.bin_id);
    segment_node.put("piece_id", segment.piece_id);
    segment_node.put("from_hole", segment.from_hole);
    segment_node.add_child("start", make_point_node(segment.segment.start));
    segment_node.add_child("end", make_point_node(segment.segment.end));
    segments.push_back({"", segment_node});
  }
  root.add_child("segments", segments);

  return write_json_tree(path, root);
}

} // namespace shiny::nfp::io