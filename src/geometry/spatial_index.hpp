#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

struct SpatialIndexEntry {
  std::uint32_t item_id{0};
  Box2 bounds{};
};

class SpatialIndex {
public:
  explicit SpatialIndex(double cell_size = 1.0);

  void clear();

  void insert(std::uint32_t item_id, const Box2 &bounds);

  [[nodiscard]] auto query(const Box2 &bounds) const -> std::vector<std::uint32_t>;

  [[nodiscard]] auto size() const -> std::size_t;

private:
  double cell_size_{1.0};
  std::vector<SpatialIndexEntry> entries_{};
  std::vector<std::vector<std::size_t>> buckets_{};
  std::vector<std::int64_t> bucket_keys_{};
};

} // namespace shiny::nesting::geom
