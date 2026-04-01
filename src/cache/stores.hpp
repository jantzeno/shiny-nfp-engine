#pragma once

#include <cstddef>
#include <list>
#include <unordered_map>
#include <utility>

#include "cache/cache_policy.hpp"
#include "cache/key_hash.hpp" // IWYU pragma: keep

namespace shiny::nfp::cache {

/**
 * @brief Generic keyed cache with optional LRU eviction.
 *
 * Wraps a hash map and, when requested, an access-order list so the higher
 * layers can reuse expensive geometry and search evaluations without exposing a
 * backend-specific cache implementation.
 *
 * @par Thread Safety
 * - Not thread-safe; callers must externally synchronize concurrent mutation or
 *   mixed read/write access.
 *
 * @par Invariants
 * - When LRU is enabled, every stored key has a matching node in the access
 *   order list.
 * - When LRU is disabled, the access order list remains empty.
 *
 * @par Performance Notes
 * - `get` and `put` are hash-table operations with bounded LRU list updates.
 */
template <class Key, class Value, class Hasher = std::hash<Key>>
class CacheStore {
public:
  CacheStore() = default;

  explicit CacheStore(CachePolicyConfig policy) : policy_(policy) {}

  /**
   * @brief Looks up one cached value by key.
   *
   * @par Algorithm Detail
   * - **Strategy**: Hash lookup with optional LRU promotion.
   * - **Steps**:
   *   1. Probe the underlying hash table.
   *   2. Return `nullptr` on miss.
   *   3. Promote the entry to most-recently-used position when LRU is enabled.
   *
   * @param key Cache key to probe.
   * @return Pointer to the cached value, or `nullptr` when absent.
   */
  [[nodiscard]] auto get(const Key &key) const -> const Value * {
    const auto it = storage_.find(key);
    if (it == storage_.end()) {
      return nullptr;
    }
    touch(it);
    return &it->second.value;
  }

  /**
   * @brief Reports whether a key is currently cached.
   *
   * @param key Cache key to probe.
   * @return `true` when the store contains `key`.
   */
  [[nodiscard]] auto contains(const Key &key) const -> bool {
    return storage_.contains(key);
  }

  /**
   * @brief Inserts or replaces a cached value.
   *
   * @par Algorithm Detail
   * - **Strategy**: Upsert with optional LRU eviction.
   * - **Steps**:
   *   1. Update and promote an existing entry when the key is already cached.
   *   2. Evict the least-recently-used entry if a bounded cache is full.
   *   3. Insert the new value and register it as most recently used.
   *
   * @param key Cache key to insert or update.
   * @param value Cached payload stored under `key`.
   */
  auto put(Key key, Value value) -> void {
    if (const auto it = storage_.find(key); it != storage_.end()) {
      it->second.value = std::move(value);
      touch(it);
      return;
    }

    evict_if_needed();

    auto inserted = storage_.emplace(std::move(key), StoredValue{});
    auto it = inserted.first;
    it->second.value = std::move(value);
    if (lru_enabled()) {
      lru_order_.push_front(it->first);
      it->second.lru_position = lru_order_.begin();
    }
  }

  /**
   * @brief Removes one entry from the cache.
   *
   * @param key Cache key to erase.
   * @return `true` when an entry was removed.
   */
  auto erase(const Key &key) -> bool {
    const auto it = storage_.find(key);
    if (it == storage_.end()) {
      return false;
    }

    if (lru_enabled()) {
      lru_order_.erase(it->second.lru_position);
    }

    storage_.erase(it);
    return true;
  }

  /**
   * @brief Clears all cached entries and LRU bookkeeping state.
   */
  auto clear() -> void {
    storage_.clear();
    lru_order_.clear();
  }

  /**
   * @brief Returns the number of cached entries.
   *
   * @return Current cache cardinality.
   */
  [[nodiscard]] auto size() const -> std::size_t { return storage_.size(); }

private:
  struct StoredValue {
    Value value{};
    typename std::list<Key>::iterator lru_position{};
  };

  using StorageMap = std::unordered_map<Key, StoredValue, Hasher>;

  [[nodiscard]] auto lru_enabled() const -> bool {
    return policy_.policy == CachePolicy::lru_bounded &&
           policy_.max_entries > 0U;
  }

  auto evict_if_needed() -> void {
    if (!lru_enabled() || storage_.size() < policy_.max_entries) {
      return;
    }

    const auto &lru_key = lru_order_.back();
    storage_.erase(lru_key);
    lru_order_.pop_back();
  }

  auto touch(typename StorageMap::iterator it) const -> void {
    if (!lru_enabled()) {
      return;
    }

    lru_order_.splice(lru_order_.begin(), lru_order_, it->second.lru_position);
    it->second.lru_position = lru_order_.begin();
  }

  CachePolicyConfig policy_{};
  mutable StorageMap storage_{};
  mutable std::list<Key> lru_order_{};
};

} // namespace shiny::nfp::cache