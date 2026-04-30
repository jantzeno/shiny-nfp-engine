#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

// Spatial bounding-box index backed by Boost.Geometry's R-tree.
//
// Inserts buffer entries into a flat vector; the underlying rtree
// is built lazily on the first query() after a mutation and kept
// alive across subsequent queries. Mutations (`insert`, `clear`)
// invalidate the cached rtree.
class RTreeIndex {
public:
  RTreeIndex();
  ~RTreeIndex();
  RTreeIndex(const RTreeIndex &) = delete;
  auto operator=(const RTreeIndex &) -> RTreeIndex & = delete;
  RTreeIndex(RTreeIndex &&) noexcept;
  auto operator=(RTreeIndex &&) noexcept -> RTreeIndex &;

  void clear();

  void insert(std::uint32_t item_id, const Box2 &bounds);

  [[nodiscard]] auto query(const Box2 &bounds) const
      -> std::vector<std::uint32_t>;

  [[nodiscard]] auto size() const -> std::size_t;

private:
  struct Impl;
  std::vector<std::pair<Box2, std::uint32_t>> entries_{};
  mutable std::unique_ptr<Impl> impl_;
  mutable bool dirty_{true};
};

} // namespace shiny::nesting::geom
