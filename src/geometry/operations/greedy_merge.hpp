#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace shiny::nesting::geom {

// Greedy pairwise merge with restart-on-success.
//
// Repeatedly scans all O(N^2) ordered pairs of items in `items`. For each
// pair `(lhs, rhs)`, invokes `merger(lhs, rhs)`. If it returns a value,
// item `lhs` is replaced by the merged result, item `rhs` is erased, and
// the scan restarts from the beginning (index validity after erase). The
// loop terminates when a full pass produces no successful merge.
//
// Worst-case O(N^3) calls to `merger`; in practice merges drop the count
// quickly so the bound is loose. Used by:
//   - decomposition/convex_decomposition.cpp (convex sub-polygon fusion)
//   - nfp/nfp.cpp + nfp/orbiting_nfp.cpp (NFP fragment unification)
//
// `Merger` must be callable as `std::optional<T>(const T &, const T &) `.
// Returning `std::nullopt` rejects the pair (no merge). The returned T
// replaces the lhs operand.
template <typename T, typename Merger>
[[nodiscard]] auto greedy_pairwise_merge(std::vector<T> items, Merger merger)
    -> std::vector<T> {
  bool merged_any = true;
  while (merged_any) {
    merged_any = false;
    for (std::size_t lhs = 0; lhs < items.size() && !merged_any; ++lhs) {
      for (std::size_t rhs = lhs + 1U; rhs < items.size(); ++rhs) {
        auto merged = merger(items[lhs], items[rhs]);
        if (!merged.has_value()) {
          continue;
        }
        items[lhs] = std::move(*merged);
        items.erase(items.begin() + static_cast<std::ptrdiff_t>(rhs));
        merged_any = true;
        break;
      }
    }
  }
  return items;
}

} // namespace shiny::nesting::geom