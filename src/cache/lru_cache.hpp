#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "cache/cache_policy.hpp"

namespace shiny::nesting::cache {

// Generic LRU cache (Plan §11.1).
//
// Implementation pattern: doubly-linked list of keys (`order_`) +
// hash map keyed by Key holding (shared_ptr<const Value>, list
// iterator). On hit, the key is spliced to the front of the list
// (recency-update). On insert when over `max_entries`, evict from
// the BACK of the list (LRU eviction). All operations take a single
// mutex.
//
// Reads return `std::shared_ptr<const Value>`: callers get a stable,
// reference-counted handle that survives concurrent evictions. This
// is safe even if another thread races to clear() or insert() and
// triggers eviction: the evicted entry's storage is kept alive by
// outstanding shared_ptr references, then released when the last one
// drops.
template <typename Key, typename Value, typename Hash = std::hash<Key>>
class LruCache {
public:
  explicit LruCache(CacheStoreConfig config = {}) : config_(config) {}

  auto clear() -> void {
    const std::lock_guard lock(mutex_);
    entries_.clear();
    order_.clear();
  }

  [[nodiscard]] auto size() const -> std::size_t {
    const std::lock_guard lock(mutex_);
    return entries_.size();
  }

  auto put(const Key &key, const Value &value) -> void {
    const std::lock_guard lock(mutex_);
    insert_locked(key, std::make_shared<const Value>(value));
  }

  auto put(const Key &key, Value &&value) -> void {
    const std::lock_guard lock(mutex_);
    insert_locked(key, std::make_shared<const Value>(std::move(value)));
  }

  auto put_shared(const Key &key, std::shared_ptr<const Value> value) -> void {
    const std::lock_guard lock(mutex_);
    insert_locked(key, std::move(value));
  }

  // Returns a shared handle to the cached value, or null if absent.
  // Safe for concurrent use; returned handles outlive eviction.
  [[nodiscard]] auto get(const Key &key) const -> std::shared_ptr<const Value> {
    const std::lock_guard lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      return nullptr;
    }
    touch_locked(it);
    return it->second.value;
  }

private:
  struct Entry {
    std::shared_ptr<const Value> value{};
    typename std::list<Key>::iterator order_it{};
  };

  using EntryMap = std::unordered_map<Key, Entry, Hash>;

  auto insert_locked(const Key &key, std::shared_ptr<const Value> value)
      -> void {
    const auto existing = entries_.find(key);
    if (existing != entries_.end()) {
      existing->second.value = std::move(value);
      touch_locked(existing);
      return;
    }

    order_.push_front(key);
    entries_.emplace(
        key, Entry{.value = std::move(value), .order_it = order_.begin()});
    evict_locked();
  }

  auto touch_locked(typename EntryMap::iterator it) const -> void {
    order_.splice(order_.begin(), order_, it->second.order_it);
    it->second.order_it = order_.begin();
  }

  auto evict_locked() -> void {
    if (config_.policy != CachePolicy::lru_bounded ||
        config_.max_entries == 0U) {
      return;
    }
    while (entries_.size() > config_.max_entries) {
      const auto &key = order_.back();
      entries_.erase(key);
      order_.pop_back();
    }
  }

  CacheStoreConfig config_{};
  mutable std::mutex mutex_{};
  mutable std::list<Key> order_{};
  mutable EntryMap entries_{};
};

} // namespace shiny::nesting::cache
