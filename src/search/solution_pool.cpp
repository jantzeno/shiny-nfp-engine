#include "search/solution_pool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

#include "geometry/polygon.hpp"
#include "runtime/hash.hpp"
#include "validation/layout_validation.hpp"

namespace shiny::nesting::search {
namespace {

[[nodiscard]] auto order_signature(std::span<const std::size_t> order)
    -> std::uint64_t {
  return runtime::hash::fnv1a_of<std::size_t>(order);
}

[[nodiscard]] auto rotation_signature(
    std::span<const std::optional<geom::RotationIndex>> forced_rotations)
    -> std::uint64_t {
  std::uint64_t hash = runtime::hash::kFnv1aOffsetBasis;
  for (const auto &rotation : forced_rotations) {
    const std::uint64_t encoded =
        rotation.has_value()
            ? (static_cast<std::uint64_t>(rotation->value) + 1U)
            : 0ULL;
    runtime::hash::fnv1a_mix_value(hash, encoded);
  }
  return hash;
}

[[nodiscard]] auto gaussian_rank_index(const std::size_t size,
                                       runtime::DeterministicRng &rng)
    -> std::size_t {
  if (size <= 1U) {
    return 0U;
  }

  const double u1 =
      std::max(rng.uniform_real(), std::numeric_limits<double>::min());
  const double u2 = rng.uniform_real();
  const double gaussian =
      std::sqrt(-2.0 * std::log(u1)) *
      std::cos(2.0 * std::numbers::pi_v<double> * u2);
  const double sigma = 0.25;
  const double normalized = std::clamp(std::abs(gaussian) * sigma, 0.0, 1.0);
  return std::min(size - 1U,
                  static_cast<std::size_t>(normalized * static_cast<double>(size)));
}

} // namespace

auto metrics_for_layout(const pack::Layout &layout) -> LayoutMetrics {
  LayoutMetrics metrics;
  metrics.placed_parts = layout.placement_trace.size();
  metrics.bin_count = layout.bins.size();

  double occupied_area = 0.0;
  double container_area = 0.0;
  double strip_length = 0.0;
  for (const auto &bin : layout.bins) {
    occupied_area += bin.utilization.occupied_area;
    container_area += bin.utilization.container_area;
    if (bin.placements.empty()) {
      continue;
    }

    geom::Box2 envelope{};
    bool initialized = false;
    for (const auto &placement : bin.placements) {
      const auto bounds = geom::compute_bounds(placement.polygon);
      if (!initialized) {
        envelope = bounds;
        initialized = true;
        continue;
      }
      envelope.min.x = std::min(envelope.min.x, bounds.min.x);
      envelope.min.y = std::min(envelope.min.y, bounds.min.y);
      envelope.max.x = std::max(envelope.max.x, bounds.max.x);
      envelope.max.y = std::max(envelope.max.y, bounds.max.y);
    }

    strip_length += geom::box_width(envelope);
  }

  metrics.strip_length = strip_length;
  metrics.utilization =
      container_area > 0.0 ? occupied_area / container_area : 0.0;
  return metrics;
}

auto better_metrics(const LayoutMetrics &lhs, const LayoutMetrics &rhs) -> bool {
  if (lhs.placed_parts != rhs.placed_parts) {
    return lhs.placed_parts > rhs.placed_parts;
  }
  if (lhs.bin_count != rhs.bin_count) {
    return lhs.bin_count < rhs.bin_count;
  }
  if (lhs.strip_length != rhs.strip_length) {
    return lhs.strip_length < rhs.strip_length;
  }
  if (lhs.utilization != rhs.utilization) {
    return lhs.utilization > rhs.utilization;
  }
  return false;
}

SolutionPool::SolutionPool(const std::size_t capacity,
                           const NormalizedRequest *validation_request)
    : capacity_(capacity), validation_request_(validation_request) {}

// Bounded LRU-by-quality pool of layout decisions (order + per-piece
// forced rotation). Used by GLS-driven strip search to maintain elites
// for re-seeding disruption operators.
//
// Insertion: O(N) signature linear-search for dedup, then O(N log N)
// stable_sort to keep entries best-first; pool is small (<= 16 by
// design) so this stays cheap.
//
// Selection (`select`): biased gaussian rank — folds half-normal
// |N(0,σ²)| with σ=0.25 to a [0,size) bucket, so picks heavily favour
// pool position 0 (the best). σ=0.25 is a magic constant; smaller σ
// concentrates more on elites, larger σ spreads to mutants.
auto SolutionPool::insert(SolutionPoolEntry entry) -> void {
  if (capacity_ == 0U) {
    return;
  }
  if (entry.result.total_parts > 0U) {
    if (validation_request_ != nullptr &&
        !validation::validate_layout(*validation_request_, entry.result).valid) {
      return;
    }
    if (validation_request_ == nullptr && !entry.result.layout_valid()) {
      return;
    }
  }

  const std::uint64_t signature =
      order_signature(entry.order) ^
      (rotation_signature(entry.piece_indexed_forced_rotations) << 1U);
  // Dedup against the parallel signatures_ vector first; full
  // order/forced_rotations equality is only checked on signature
  // collision (rare). This avoids the O(N²) FNV-1a-per-pair scan that
  // the previous implementation performed on every insert.
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    if (signatures_[i] != signature) {
      continue;
    }
    if (entries_[i].order != entry.order ||
        entries_[i].piece_indexed_forced_rotations !=
            entry.piece_indexed_forced_rotations) {
      continue;
    }
    if (better_metrics(entry.metrics, entries_[i].metrics)) {
      // Replace and re-position: the pool is best-first; keeping the
      // entry at its old slot could break the ordering invariant when
      // the new entry beats earlier elites.
      entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(i));
      signatures_.erase(signatures_.begin() + static_cast<std::ptrdiff_t>(i));
      break;
    }
    return;
  }

  // Maintain best-first order with a binary-search insert (O(log N)
  // for the lookup + O(N) shift). The pool is small (<= 16), so this
  // beats a fresh stable_sort on every call mainly for clarity: the
  // ordering invariant is enforced at the insertion site.
  const auto pos =
      std::upper_bound(entries_.begin(), entries_.end(), entry,
                       [](const auto &lhs, const auto &rhs) {
                         return better_metrics(lhs.metrics, rhs.metrics);
                       });
  const auto offset = pos - entries_.begin();
  entries_.insert(pos, std::move(entry));
  signatures_.insert(signatures_.begin() + offset, signature);

  if (entries_.size() > capacity_) {
    entries_.resize(capacity_);
    signatures_.resize(capacity_);
  }
}

auto SolutionPool::empty() const -> bool { return entries_.empty(); }

auto SolutionPool::size() const -> std::size_t { return entries_.size(); }

auto SolutionPool::best() const -> const SolutionPoolEntry * {
  return entries_.empty() ? nullptr : &entries_.front();
}

auto SolutionPool::select(runtime::DeterministicRng &rng) const
    -> const SolutionPoolEntry * {
  if (entries_.empty()) {
    return nullptr;
  }
  return &entries_[gaussian_rank_index(entries_.size(), rng)];
}

auto SolutionPool::entries() const -> std::span<const SolutionPoolEntry> {
  return entries_;
}

} // namespace shiny::nesting::search
