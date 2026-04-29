#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "geometry/transform.hpp"
#include "result.hpp"
#include "runtime/deterministic_rng.hpp"

namespace shiny::nesting::search {

struct LayoutMetrics {
  std::size_t placed_parts{0};
  std::size_t bin_count{0};
  double strip_length{0.0};
  double utilization{0.0};
};

[[nodiscard]] auto metrics_for_layout(const pack::Layout &layout) -> LayoutMetrics;

[[nodiscard]] auto better_metrics(const LayoutMetrics &lhs,
                                  const LayoutMetrics &rhs) -> bool;

struct SolutionPoolEntry {
  std::vector<std::size_t> order{};
  // Piece-indexed: piece_indexed_forced_rotations[piece_index] describes the expanded piece
  // with the same index in the original normalized request.
  std::vector<std::optional<geom::RotationIndex>> piece_indexed_forced_rotations{};
  LayoutMetrics metrics{};
  NestingResult result{};
};

class SolutionPool {
public:
  explicit SolutionPool(std::size_t capacity = 8U,
                        const NormalizedRequest *validation_request = nullptr);

  auto insert(SolutionPoolEntry entry) -> void;

  [[nodiscard]] auto empty() const -> bool;

  [[nodiscard]] auto size() const -> std::size_t;

  [[nodiscard]] auto best() const -> const SolutionPoolEntry *;

  [[nodiscard]] auto select(runtime::DeterministicRng &rng) const
      -> const SolutionPoolEntry *;

  [[nodiscard]] auto entries() const -> std::span<const SolutionPoolEntry>;

private:
  std::size_t capacity_{0};
  const NormalizedRequest *validation_request_{nullptr};
  std::vector<SolutionPoolEntry> entries_{};
  // Parallel vector of pre-computed (order ⊕ rotation) signatures for
  // O(1) prefilter during dedup. Kept strictly in sync with `entries_`
  // (insert/erase happen pairwise). Eliminates the per-call FNV-1a
  // recompute over every existing entry on every insert (finding #38).
  std::vector<std::uint64_t> signatures_{};
};

} // namespace shiny::nesting::search
