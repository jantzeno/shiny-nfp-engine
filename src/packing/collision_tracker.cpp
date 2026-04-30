#include "packing/collision_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "geometry/operations/boolean_ops.hpp"
#include "geometry/polygon.hpp"
#include "geometry/queries/normalize.hpp"
#include "packing/overlap_proxy.hpp"
#include "packing/shape_penalty.hpp"

namespace shiny::nesting::pack {
namespace {

[[nodiscard]] auto lerp(const double lhs, const double rhs, const double t)
    -> double {
  return lhs + (rhs - lhs) * t;
}

} // namespace

CollisionTracker::CollisionTracker(geom::PolygonWithHoles container,
                                   std::vector<CollisionTrackerItem> items)
    : container_(geom::normalize_polygon(std::move(container))),
      items_(std::move(items)), polygon_revisions_(items_.size(), 0U),
      // Triangle-number storage of strictly-upper pair entries:
      //   slots = N(N-1)/2 for N >= 2, else 0.
      // Explicit branch avoids relying on `0 * SIZE_MAX == 0` for N=0.
      pair_entries_(items_.size() < 2U
                        ? 0U
                        : (items_.size() * (items_.size() - 1U)) / 2U),
      container_losses_(items_.size(), 0.0),
      container_weights_(items_.size(), 1.0) {
  for (std::size_t index = 0; index < items_.size(); ++index) {
    items_[index].polygon = geom::normalize_polygon(items_[index].polygon);
    polygon_revisions_[index] = geom::polygon_revision(items_[index].polygon);
  }

  for (std::size_t lhs = 0; lhs < items_.size(); ++lhs) {
    for (std::size_t rhs = lhs + 1U; rhs < items_.size(); ++rhs) {
      auto &slot = pair_entries_[pair_index(lhs, rhs)];
      const auto computed = compute_pair_loss(lhs, rhs);
      slot.loss = computed.loss;
      slot.exact = computed.exact;
    }
    container_losses_[lhs] = compute_container_loss(lhs);
  }
}

auto CollisionTracker::item_count() const -> std::size_t {
  return items_.size();
}

auto CollisionTracker::item(const std::size_t index) const
    -> const CollisionTrackerItem & {
  return items_[index];
}

auto CollisionTracker::item_polygon_revision(const std::size_t index) const
    -> std::uint64_t {
  return polygon_revisions_[index];
}

auto CollisionTracker::container() const -> const geom::PolygonWithHoles & {
  return container_;
}

auto CollisionTracker::pole_cache() const -> cache::PoleCache * {
  return const_cast<cache::PoleCache *>(&pole_cache_);
}

auto CollisionTracker::penetration_depth_cache() const
    -> cache::PenetrationDepthCache * {
  return const_cast<cache::PenetrationDepthCache *>(&penetration_depth_cache_);
}

auto CollisionTracker::pair_loss_cache_size() const -> std::size_t {
  return pair_loss_cache_.size();
}

auto CollisionTracker::pair_loss_cache_stats() const
    -> cache::CollisionPairLossCacheStats {
  return cache::CollisionPairLossCacheStats{
      .hits = pair_loss_cache_hits_,
      .misses = pair_loss_cache_misses_,
  };
}

// Triangular upper-matrix index for unordered pair (lhs, rhs).
//   For an n-item set, pairs are stored row-major over the strict upper
//   triangle: row a = (a, a+1), (a, a+2), ..., (a, n-1).
//   index(a, b) = a*n - a*(a+1)/2 + (b - a - 1)         for a < b
// Self-pairs (a == b) MUST NOT be queried; callers ensure this. The
// triangular layout halves storage vs an n×n matrix.
auto CollisionTracker::pair_index(const std::size_t lhs,
                                  const std::size_t rhs) const -> std::size_t {
  const auto a = std::min(lhs, rhs);
  const auto b = std::max(lhs, rhs);
  return a * items_.size() - (a * (a + 1U)) / 2U + (b - a - 1U);
}

// Pair loss = exact overlap area × shape penalty, OR (when no actual
// overlap) the smooth pole-of-inaccessibility proxy. The dual mode
// matters for the separator:
//   * actual overlap > 0: scaled exact area gives a meaningful gradient
//     toward smaller overlap.
//   * actual overlap == 0 but poles intersect: returns a small positive
//     "soft" pressure so coordinate-descent feels a slope even before
//     formal contact.
// CAUTION: because this can be > 0 when polygons don't actually overlap,
// `total_loss() == 0` is NOT a reliable "fully separated" signal — see
// review-report finding "CollisionTracker proxy contaminates termination
// test".
// Soft-overlap proxy + actual overlap area, returned together. The
// distinction matters because callers (separator/strip optimiser) need
// `exact == 0` to detect feasibility; mixing the two would let the
// proxy-pressure tail keep them iterating after collisions are gone.
auto compute_polygon_pair_loss(const geom::PolygonWithHoles &lhs,
                               const std::uint64_t lhs_revision,
                               const geom::PolygonWithHoles &rhs,
                               const std::uint64_t rhs_revision,
                               cache::PoleCache *pole_cache,
                               cache::PenetrationDepthCache *pd_cache)
    -> std::pair<double, double> {
  const auto overlap =
      geom::polygon_area_sum(geom::intersection_polygons(lhs, rhs));
  if (overlap <= 0.0) {
    return {overlap_proxy_loss(lhs, lhs_revision, rhs, rhs_revision, pole_cache,
                               pd_cache),
            0.0};
  }
  const auto exact_loss = overlap * shape_penalty(lhs, rhs);
  return {exact_loss, exact_loss};
}

auto compute_polygon_pair_loss(const geom::PolygonWithHoles &lhs,
                               const geom::PolygonWithHoles &rhs)
    -> std::pair<double, double> {
  return compute_polygon_pair_loss(lhs, geom::polygon_revision(lhs), rhs,
                                   geom::polygon_revision(rhs));
}

auto CollisionTracker::compute_pair_loss(const std::size_t lhs,
                                         const std::size_t rhs) const
    -> PairEntry {
  const auto cache_key = cache::make_collision_pair_loss_cache_key(
      polygon_revisions_[lhs], polygon_revisions_[rhs]);
  if (const auto cached = pair_loss_cache_.get(cache_key); cached != nullptr) {
    ++pair_loss_cache_hits_;
    return PairEntry{
        .loss = cached->loss, .exact = cached->exact, .weight = 0.0};
  }

  ++pair_loss_cache_misses_;
  const auto [loss, exact] = compute_polygon_pair_loss(
      items_[lhs].polygon, polygon_revisions_[lhs], items_[rhs].polygon,
      polygon_revisions_[rhs], pole_cache(), penetration_depth_cache());
  // Cache only the exact-overlap branch. The proxy branch
  // (`exact == 0` but `loss > 0` from inscribed-circle pressure)
  // changes value as items are perturbed even when the polygon
  // revisions are stable, so caching it would thrash the LRU and
  // serve stale soft pressures back to callers. Exact intersection
  // area, by contrast, is purely a function of the two revisions.
  if (exact > 0.0) {
    pair_loss_cache_.put(cache_key, cache::CollisionPairLossCacheValue{
                                        .loss = loss,
                                        .exact = exact,
                                    });
  }
  return PairEntry{.loss = loss, .exact = exact, .weight = 0.0};
}

auto CollisionTracker::compute_container_loss(const std::size_t index) const
    -> double {
  return geom::polygon_area_sum(
      geom::difference_polygons(items_[index].polygon, container_));
}

auto CollisionTracker::register_item_move(const std::size_t index,
                                          const geom::PolygonWithHoles &polygon)
    -> void {
  items_[index].polygon = geom::normalize_polygon(polygon);
  polygon_revisions_[index] = geom::polygon_revision(items_[index].polygon);
  container_losses_[index] = compute_container_loss(index);
  for (std::size_t other = 0; other < items_.size(); ++other) {
    if (other == index) {
      continue;
    }
    auto &slot = pair_entries_[pair_index(index, other)];
    const auto computed = compute_pair_loss(index, other);
    slot.loss = computed.loss;
    slot.exact = computed.exact;
  }
}

auto CollisionTracker::pair_loss(const std::size_t lhs,
                                 const std::size_t rhs) const -> double {
  return pair_entries_[pair_index(lhs, rhs)].loss;
}

auto CollisionTracker::pair_weight(const std::size_t lhs,
                                   const std::size_t rhs) const -> double {
  return pair_entries_[pair_index(lhs, rhs)].weight;
}

auto CollisionTracker::container_loss(const std::size_t index) const -> double {
  return container_losses_[index];
}

auto CollisionTracker::container_weight(const std::size_t index) const
    -> double {
  return container_weights_[index];
}

auto CollisionTracker::total_loss() const -> double {
  double total = 0.0;
  for (const auto &entry : pair_entries_) {
    total += entry.loss;
  }
  for (const auto loss : container_losses_) {
    total += loss;
  }
  return total;
}

auto CollisionTracker::pair_exact_loss(const std::size_t lhs,
                                       const std::size_t rhs) const -> double {
  return pair_entries_[pair_index(lhs, rhs)].exact;
}

auto CollisionTracker::exact_total_loss() const -> double {
  double total = 0.0;
  for (const auto &entry : pair_entries_) {
    total += entry.exact;
  }
  for (const auto loss : container_losses_) {
    total += loss;
  }
  return total;
}

auto CollisionTracker::weighted_total_loss() const -> double {
  double total = 0.0;
  for (const auto &entry : pair_entries_) {
    total += entry.weight * entry.loss;
  }
  for (std::size_t index = 0; index < container_losses_.size(); ++index) {
    total += container_weights_[index] * container_losses_[index];
  }
  return total;
}

// Guided Local Search weight update (Sparrow Algorithm 9 step).
//
//   Let max_loss = max over all pair/container losses.
//   For each constraint c (pair or item-vs-container):
//     if loss(c) > 0:  weight(c) *= lerp(M_l, M_u, loss(c) / max_loss)
//                       where M_l=1.2 amplifies any positive loss and
//                       M_u=2.0 amplifies maximally-violated constraints.
//     else:            weight(c) *= M_d  (=0.95 decay), floored at 1.0.
//
// Effect: persistent collisions accumulate weight, raising their cost in
// `weighted_total_loss()` so subsequent moves preferentially address
// them. Decayed weights of resolved constraints recover toward 1 so
// future fresh collisions start with neutral pressure.
//
// IMPORTANT (correctness): for pair entries the "fully resolved" signal
// is `exact == 0`, NOT `loss == 0`. A pair can have zero exact overlap
// but a non-zero proxy `loss` (inscribed circles still intersect even
// though polygons don't). Treating that as a violation keeps the soft
// pressure alive in `weighted_total_loss()`; treating it as resolved
// would decay the weight to 1 even though the separator still wants a
// gradient toward separation. The decay branch therefore gates on
// `entry.exact <= 0.0` for pair entries.
//
// CAUTION: weight is bounded above by `weight_cap` (default 1e6) —
// without this cap, a constraint that stays violated for many
// iterations grows ×2 per iteration in the worst case and eventually
// saturates doubles, destroying the relative ordering used by
// `weighted_total_loss()`. Sparrow's reference picks 1e6 as the
// practical ceiling.
auto CollisionTracker::update_gls_weights(const double weight_cap) -> void {
  const auto apply_decay = [](double weight) {
    return std::max(1.0, weight * 0.95);
  };

  double max_loss = 0.0;
  for (const auto &entry : pair_entries_) {
    max_loss = std::max(max_loss, entry.loss);
  }
  for (const auto loss : container_losses_) {
    max_loss = std::max(max_loss, loss);
  }
  if (max_loss <= 0.0) {
    // All constraints have zero loss (no proxy pressure either): every
    // constraint is genuinely resolved, so a uniform decay is correct.
    for (auto &entry : pair_entries_) {
      entry.weight = apply_decay(entry.weight);
    }
    for (auto &weight : container_weights_) {
      weight = apply_decay(weight);
    }
    return;
  }

  for (auto &entry : pair_entries_) {
    // Three-way classification (gates on `exact`, not `loss`, so the
    // smooth proxy gradient is preserved when polygons stop actually
    // overlapping but their inscribed circles still touch):
    //   * exact > 0      -> active overlap; amplify proportional to
    //                       loss / max_loss.
    //   * exact == 0 &&
    //     loss  > 0      -> proxy-only pressure; RETAIN weight (no
    //                       decay) so soft pressure persists across
    //                       iterations even after the exact overlap
    //                       has just cleared.
    //   * loss  == 0     -> fully separated; decay toward 1.
    if (entry.exact > 0.0) {
      entry.weight = std::min(
          weight_cap, entry.weight * lerp(1.2, 2.0, entry.loss / max_loss));
    } else if (entry.loss <= 0.0) {
      entry.weight = apply_decay(entry.weight);
    }
  }

  for (std::size_t index = 0; index < container_losses_.size(); ++index) {
    if (container_losses_[index] > 0.0) {
      container_weights_[index] = std::min(
          weight_cap, container_weights_[index] *
                          lerp(1.2, 2.0, container_losses_[index] / max_loss));
    } else {
      container_weights_[index] = apply_decay(container_weights_[index]);
    }
  }
}

} // namespace shiny::nesting::pack
