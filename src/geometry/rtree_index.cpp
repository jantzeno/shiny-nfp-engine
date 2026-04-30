#include "geometry/rtree_index.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "geometry/types.hpp"

namespace shiny::nesting::geom {
namespace {

namespace bgi = boost::geometry::index;

using BoostValue = std::pair<Box2, std::uint32_t>;

} // namespace

// PIMPL hides Boost.Geometry from the public header; the rtree is
// built lazily on the first query after a mutation and reused across
// subsequent queries until clear()/insert() marks it dirty again.
struct RTreeIndex::Impl {
  bgi::rtree<BoostValue, bgi::quadratic<16>> rtree;
};

RTreeIndex::RTreeIndex() = default;
RTreeIndex::~RTreeIndex() = default;
RTreeIndex::RTreeIndex(RTreeIndex &&) noexcept = default;
auto RTreeIndex::operator=(RTreeIndex &&) noexcept -> RTreeIndex & = default;

void RTreeIndex::clear() {
  entries_.clear();
  impl_.reset();
  dirty_ = true;
}

void RTreeIndex::insert(const std::uint32_t item_id, const Box2 &bounds) {
  entries_.push_back({bounds, item_id});
  dirty_ = true;
}

auto RTreeIndex::query(const Box2 &bounds) const -> std::vector<std::uint32_t> {
  if (dirty_) {
    auto fresh = std::make_unique<Impl>();
    std::vector<BoostValue> packed;
    packed.reserve(entries_.size());
    for (const auto &[entry_bounds, item_id] : entries_) {
      packed.emplace_back(entry_bounds, item_id);
    }
    fresh->rtree = decltype(fresh->rtree)(packed.begin(), packed.end());
    impl_ = std::move(fresh);
    dirty_ = false;
  }

  std::vector<BoostValue> hits;
  impl_->rtree.query(bgi::intersects(bounds), std::back_inserter(hits));

  std::vector<std::uint32_t> matches;
  matches.reserve(hits.size());
  for (const auto &[unused_box, item_id] : hits) {
    matches.push_back(item_id);
  }
  std::sort(matches.begin(), matches.end());
  matches.erase(std::unique(matches.begin(), matches.end()), matches.end());
  return matches;
}

auto RTreeIndex::size() const -> std::size_t { return entries_.size(); }

} // namespace shiny::nesting::geom
