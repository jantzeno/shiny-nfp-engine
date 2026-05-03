#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::io {

struct OrDatasetZone {
  int quality{0};
  geom::PolygonWithHoles polygon{};
};

struct OrDatasetItem {
  std::uint32_t item_id{0};
  std::uint32_t demand{1};
  std::string dxf_path{};
  std::optional<int> minimum_quality{};
  std::vector<double> allowed_orientations{};
  std::vector<OrDatasetZone> zones{};
  geom::PolygonWithHoles polygon{};
};

struct OrDatasetBin {
  std::uint32_t bin_id{0};
  int cost{0};
  std::uint32_t stock{1};
  std::string dxf_path{};
  std::vector<OrDatasetZone> zones{};
  geom::PolygonWithHoles polygon{};
};

struct OrDataset {
  std::string name{};
  std::vector<OrDatasetItem> items{};
  std::vector<OrDatasetBin> bins{};
  std::optional<double> strip_height{};

  [[nodiscard]] auto uses_strip_container() const -> bool {
    return strip_height.has_value();
  }

  [[nodiscard]] auto uses_explicit_bins() const -> bool {
    return !bins.empty();
  }
};

[[nodiscard]] auto load_or_dataset(const std::filesystem::path &path)
    -> std::expected<OrDataset, util::Status>;

} // namespace shiny::nesting::io
