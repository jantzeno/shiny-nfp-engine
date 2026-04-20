#include "geometry/spatial_index.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "geometry/polygon.hpp"

namespace shiny::nesting::geom {
namespace detail {

[[nodiscard]] auto cell_key(const std::int32_t x, const std::int32_t y)
    -> std::int64_t {
  return (static_cast<std::int64_t>(x) << 32) ^
         static_cast<std::uint32_t>(y);
}

[[nodiscard]] auto bucket_coord(const double value, const double cell_size)
    -> std::int32_t {
  return static_cast<std::int32_t>(std::floor(value / cell_size));
}

} // namespace detail

SpatialIndex::SpatialIndex(const double cell_size)
    : cell_size_(cell_size > 0.0 ? cell_size : 1.0) {}

void SpatialIndex::clear() {
  entries_.clear();
  buckets_.clear();
  bucket_keys_.clear();
}

void SpatialIndex::insert(const std::uint32_t item_id, const Box2 &bounds) {
  const auto entry_index = entries_.size();
  entries_.push_back({.item_id = item_id, .bounds = bounds});

  std::unordered_map<std::int64_t, std::vector<std::size_t>> scratch;
  for (std::size_t index = 0; index < bucket_keys_.size(); ++index) {
    scratch.emplace(bucket_keys_[index], buckets_[index]);
  }

  const auto min_x = detail::bucket_coord(bounds.min.x, cell_size_);
  const auto max_x = detail::bucket_coord(bounds.max.x, cell_size_);
  const auto min_y = detail::bucket_coord(bounds.min.y, cell_size_);
  const auto max_y = detail::bucket_coord(bounds.max.y, cell_size_);

  for (std::int32_t x = min_x; x <= max_x; ++x) {
    for (std::int32_t y = min_y; y <= max_y; ++y) {
      scratch[detail::cell_key(x, y)].push_back(entry_index);
    }
  }

  bucket_keys_.clear();
  buckets_.clear();
  bucket_keys_.reserve(scratch.size());
  buckets_.reserve(scratch.size());
  for (auto &[key, bucket] : scratch) {
    bucket_keys_.push_back(key);
    buckets_.push_back(std::move(bucket));
  }
}

auto SpatialIndex::query(const Box2 &bounds) const -> std::vector<std::uint32_t> {
  std::vector<std::uint32_t> matches;
  if (entries_.empty()) {
    return matches;
  }

  const auto min_x = detail::bucket_coord(bounds.min.x, cell_size_);
  const auto max_x = detail::bucket_coord(bounds.max.x, cell_size_);
  const auto min_y = detail::bucket_coord(bounds.min.y, cell_size_);
  const auto max_y = detail::bucket_coord(bounds.max.y, cell_size_);

  std::vector<std::size_t> candidate_indexes;
  for (std::int32_t x = min_x; x <= max_x; ++x) {
    for (std::int32_t y = min_y; y <= max_y; ++y) {
      const auto key = detail::cell_key(x, y);
      const auto key_it =
          std::find(bucket_keys_.begin(), bucket_keys_.end(), key);
      if (key_it == bucket_keys_.end()) {
        continue;
      }

      const auto bucket_index =
          static_cast<std::size_t>(std::distance(bucket_keys_.begin(), key_it));
      candidate_indexes.insert(candidate_indexes.end(), buckets_[bucket_index].begin(),
                               buckets_[bucket_index].end());
    }
  }

  std::sort(candidate_indexes.begin(), candidate_indexes.end());
  candidate_indexes.erase(
      std::unique(candidate_indexes.begin(), candidate_indexes.end()),
      candidate_indexes.end());

  for (const auto index : candidate_indexes) {
    if (boxes_overlap(entries_[index].bounds, bounds)) {
      matches.push_back(entries_[index].item_id);
    }
  }

  std::sort(matches.begin(), matches.end());
  matches.erase(std::unique(matches.begin(), matches.end()), matches.end());
  return matches;
}

auto SpatialIndex::size() const -> std::size_t { return entries_.size(); }

} // namespace shiny::nesting::geom
