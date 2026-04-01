#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "nfp/cache_keys.hpp"

namespace shiny::nfp::cache::hash {

/**
 * @brief Mixes one hash contribution into an aggregate seed.
 *
 * @param seed Aggregate hash seed to update.
 * @param value Next hash contribution.
 */
inline void combine(std::size_t &seed, std::size_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
}

} // namespace shiny::nfp::cache::hash

namespace std {

/**
 * @brief Hashes a discrete rotation index for cache-key use.
 */
template <> struct hash<shiny::nfp::geom::RotationIndex> {
  auto operator()(const shiny::nfp::geom::RotationIndex &key) const noexcept
      -> std::size_t {
    return std::hash<std::uint16_t>{}(key.value);
  }
};

/**
 * @brief Hashes a resolved rotation angle for cache-key use.
 */
template <> struct hash<shiny::nfp::geom::ResolvedRotation> {
  auto operator()(const shiny::nfp::geom::ResolvedRotation &key) const noexcept
      -> std::size_t {
    return std::hash<double>{}(key.degrees);
  }
};

/**
 * @brief Hashes a geometry revision wrapper.
 */
template <> struct hash<shiny::nfp::cache::GeometryRevision> {
  auto operator()(const shiny::nfp::cache::GeometryRevision &key) const noexcept
      -> std::size_t {
    return std::hash<std::uint64_t>{}(key.value);
  }
};

/**
 * @brief Hashes an algorithm revision wrapper.
 */
template <> struct hash<shiny::nfp::cache::AlgorithmRevision> {
  auto
  operator()(const shiny::nfp::cache::AlgorithmRevision &key) const noexcept
      -> std::size_t {
    return std::hash<std::uint32_t>{}(key.value);
  }
};

/**
 * @brief Hashes a piece/rotation identity used by cache stores.
 */
template <> struct hash<shiny::nfp::cache::PieceRotationKey> {
  auto operator()(const shiny::nfp::cache::PieceRotationKey &key) const noexcept
      -> std::size_t {
    std::size_t seed = 0;
    shiny::nfp::cache::hash::combine(seed,
                                     std::hash<std::uint32_t>{}(key.piece_id));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::geom::ResolvedRotation>{}(key.rotation));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::cache::GeometryRevision>{}(
                  key.geometry_revision));
    return seed;
  }
};

/**
 * @brief Hashes a pair/rotation identity used by convex NFP and IFP caches.
 */
template <> struct hash<shiny::nfp::cache::PairRotationKey> {
  auto operator()(const shiny::nfp::cache::PairRotationKey &key) const noexcept
      -> std::size_t {
    std::size_t seed = 0;
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint32_t>{}(key.piece_a_id));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::geom::ResolvedRotation>{}(key.rotation_a));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint32_t>{}(key.piece_b_id));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::geom::ResolvedRotation>{}(key.rotation_b));
    shiny::nfp::cache::hash::combine(seed, std::hash<bool>{}(key.inside));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::cache::GeometryRevision>{}(
                  key.geometry_a_revision));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::cache::GeometryRevision>{}(
                  key.geometry_b_revision));
    return seed;
  }
};

/**
 * @brief Hashes a nonconvex NFP cache key including algorithm revision.
 */
template <> struct hash<shiny::nfp::cache::NonconvexNfpCacheKey> {
  auto
  operator()(const shiny::nfp::cache::NonconvexNfpCacheKey &key) const noexcept
      -> std::size_t {
    std::size_t seed =
        std::hash<shiny::nfp::cache::PairRotationKey>{}(key.pair);
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::cache::AlgorithmRevision>{}(
                  key.algorithm_revision));
    return seed;
  }
};

/**
 * @brief Hashes a placement query key for placement-engine caches.
 */
template <> struct hash<shiny::nfp::cache::PlacementQueryKey> {
  auto
  operator()(const shiny::nfp::cache::PlacementQueryKey &key) const noexcept
      -> std::size_t {
    std::size_t seed = 0;
    shiny::nfp::cache::hash::combine(seed,
                                     std::hash<std::uint32_t>{}(key.bin_id));
    shiny::nfp::cache::hash::combine(seed,
                                     std::hash<std::uint32_t>{}(key.piece_id));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<shiny::nfp::geom::RotationIndex>{}(key.rotation_index));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.piece_geometry_revision));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.bin_geometry_revision));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.merged_region_revision));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.hole_set_revision));
    shiny::nfp::cache::hash::combine(seed,
                                     std::hash<std::uint32_t>{}(key.policy_id));
    return seed;
  }
};

/**
 * @brief Hashes a layout-evaluation key for local-search reevaluation caches.
 */
template <> struct hash<shiny::nfp::cache::LayoutEvalKey> {
  auto operator()(const shiny::nfp::cache::LayoutEvalKey &key) const noexcept
      -> std::size_t {
    std::size_t seed = 0;
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.piece_order_hash));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.rotation_assignment_hash));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint64_t>{}(key.bin_policy_hash));
    shiny::nfp::cache::hash::combine(
        seed, std::hash<std::uint32_t>{}(key.search_revision));
    return seed;
  }
};

} // namespace std