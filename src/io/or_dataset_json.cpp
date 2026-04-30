#include "io/or_dataset_json.hpp"

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

#include "geometry/normalize.hpp"
#include "geometry/validity.hpp"

namespace shiny::nesting::io {
namespace detail {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;

constexpr std::uintmax_t kMaxJsonInputBytes = 8U * 1024U * 1024U;

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
  if (error || bytes > kMaxJsonInputBytes) {
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

[[nodiscard]] auto parse_point(const pt::ptree &node) -> geom::Point2 {
  auto iterator = node.begin();
  const auto x = iterator->second.get_value<double>();
  ++iterator;
  const auto y = iterator->second.get_value<double>();
  return {x, y};
}

[[nodiscard]] auto parse_ring(const pt::ptree &node) -> geom::Ring {
  geom::Ring ring;
  for (const auto &child : node) {
    ring.push_back(parse_point(child.second));
  }
  return ring;
}

[[nodiscard]] auto parse_shape(const pt::ptree &node)
    -> geom::PolygonWithHoles {
  const auto type = node.get<std::string>("type");
  if (type == "simple_polygon") {
    return geom::normalize_polygon(
        geom::PolygonWithHoles(parse_ring(node.get_child("data"))));
  }
  if (type == "polygon") {
    geom::PolygonWithHoles polygon;
    const auto &data = node.get_child("data");
    polygon.outer() = parse_ring(data.get_child("outer"));
    if (const auto holes = data.get_child_optional("holes")) {
      for (const auto &hole : *holes) {
        polygon.holes().push_back(parse_ring(hole.second));
      }
    }
    return geom::normalize_polygon(polygon);
  }

  throw std::runtime_error("unsupported OR-Datasets shape type");
}

[[nodiscard]] auto parse_zones(const pt::ptree &node)
    -> std::vector<OrDatasetZone> {
  std::vector<OrDatasetZone> zones;
  for (const auto &child : node) {
    OrDatasetZone zone;
    zone.quality = child.second.get<int>("quality", 0);
    zone.polygon = parse_shape(child.second.get_child("shape"));
    zones.push_back(std::move(zone));
  }
  return zones;
}

} // namespace detail

auto load_or_dataset(const std::filesystem::path &path)
    -> util::StatusOr<OrDataset> {
  auto tree_or = detail::read_json_tree(path);
  if (!tree_or.ok()) {
    return tree_or.status();
  }

  try {
    const auto &tree = tree_or.value();
    OrDataset dataset;
    dataset.name = tree.get<std::string>("name", "");
    if (dataset.name.empty()) {
      return util::Status::invalid_input;
    }

    for (const auto &item_node : tree.get_child("items")) {
      const auto &item_tree = item_node.second;
      OrDatasetItem item;
      item.item_id = item_tree.get<std::uint32_t>("id");
      item.demand = item_tree.get<std::uint32_t>("demand", 1U);
      item.dxf_path = item_tree.get<std::string>("dxf", "");
      if (const auto minimum_quality =
              item_tree.get_optional<int>("min_quality")) {
        item.minimum_quality = *minimum_quality;
      }
      if (const auto orientations =
              item_tree.get_child_optional("allowed_orientations")) {
        for (const auto &orientation : *orientations) {
          item.allowed_orientations.push_back(
              orientation.second.get_value<double>());
        }
      }
      if (const auto zones = item_tree.get_child_optional("zones")) {
        item.zones = detail::parse_zones(*zones);
      }
      item.polygon = detail::parse_shape(item_tree.get_child("shape"));
      if (!geom::validate_polygon(item.polygon).is_valid()) {
        return util::Status::invalid_input;
      }
      dataset.items.push_back(std::move(item));
    }

    if (const auto strip_height = tree.get_optional<double>("strip_height")) {
      dataset.strip_height = *strip_height;
    }

    if (const auto bins = tree.get_child_optional("bins")) {
      for (const auto &bin_node : *bins) {
        const auto &bin_tree = bin_node.second;
        OrDatasetBin bin;
        bin.bin_id = bin_tree.get<std::uint32_t>("id");
        bin.cost = bin_tree.get<int>("cost", 0);
        bin.stock = bin_tree.get<std::uint32_t>("stock", 1U);
        bin.dxf_path = bin_tree.get<std::string>("dxf", "");
        if (const auto zones = bin_tree.get_child_optional("zones")) {
          bin.zones = detail::parse_zones(*zones);
        }
        bin.polygon = detail::parse_shape(bin_tree.get_child("shape"));
        if (!geom::validate_polygon(bin.polygon).is_valid()) {
          return util::Status::invalid_input;
        }
        dataset.bins.push_back(std::move(bin));
      }
    }

    if (dataset.items.empty() ||
        (!dataset.strip_height.has_value() && dataset.bins.empty())) {
      return util::Status::invalid_input;
    }

    return dataset;
  } catch (...) {
    return util::Status::computation_failed;
  }
}

} // namespace shiny::nesting::io
