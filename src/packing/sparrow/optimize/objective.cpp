#include "packing/sparrow/optimize/objective.hpp"

#include <optional>
#include <span>

#include "runtime/hash.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

namespace {

constexpr double kValueTolerance = 1e-9;

[[nodiscard]] auto maximize_value(const ObjectiveValue &value) -> bool {
  return value.objective_mode == ObjectiveMode::maximize_value;
}

} // namespace

namespace detail {

[[nodiscard]] auto order_signature(std::span<const std::size_t> order)
    -> std::uint64_t {
  return ::shiny::nesting::runtime::hash::fnv1a_of<std::size_t>(order);
}

[[nodiscard]] auto rotation_signature(
    std::span<const std::optional<geom::RotationIndex>> forced_rotations)
    -> std::uint64_t {
  std::uint64_t hash = ::shiny::nesting::runtime::hash::kFnv1aOffsetBasis;
  for (const auto &rotation : forced_rotations) {
    const std::uint64_t encoded =
        rotation.has_value()
            ? (static_cast<std::uint64_t>(rotation->value) + 1U)
            : 0ULL;
    ::shiny::nesting::runtime::hash::fnv1a_mix_value(hash, encoded);
  }
  return hash;
}

} // namespace detail

auto evaluate_objective(const search::SolutionPoolEntry &entry)
    -> ObjectiveValue {
  return {
      .metrics = entry.metrics,
      .multi_bin =
          {
              .active_bin_count = entry.metrics.bin_count,
              .active_bin_compaction = entry.metrics.strip_length,
              .active_bin_utilization = entry.metrics.utilization,
          },
      .placed_value = entry.metrics.placed_value,
      .objective_mode = entry.metrics.objective_mode,
      .order_signature = detail::order_signature(entry.order),
      .rotation_signature =
          detail::rotation_signature(entry.piece_indexed_forced_rotations),
  };
}

auto compare_objective(const ObjectiveValue &lhs, const ObjectiveValue &rhs)
    -> ObjectiveOrdering {
  if (maximize_value(lhs) || maximize_value(rhs)) {
    if (std::abs(lhs.placed_value - rhs.placed_value) > kValueTolerance) {
      return lhs.placed_value > rhs.placed_value ? ObjectiveOrdering::better
                                                 : ObjectiveOrdering::worse;
    }
    if (lhs.metrics.placed_parts != rhs.metrics.placed_parts) {
      return lhs.metrics.placed_parts > rhs.metrics.placed_parts
                 ? ObjectiveOrdering::better
                 : ObjectiveOrdering::worse;
    }
  } else if (lhs.metrics.placed_parts != rhs.metrics.placed_parts) {
    return lhs.metrics.placed_parts > rhs.metrics.placed_parts
               ? ObjectiveOrdering::better
               : ObjectiveOrdering::worse;
  }
  if (lhs.multi_bin.active_bin_count != rhs.multi_bin.active_bin_count) {
    return lhs.multi_bin.active_bin_count < rhs.multi_bin.active_bin_count
               ? ObjectiveOrdering::better
               : ObjectiveOrdering::worse;
  }
  if (lhs.multi_bin.active_bin_compaction !=
      rhs.multi_bin.active_bin_compaction) {
    return lhs.multi_bin.active_bin_compaction <
                   rhs.multi_bin.active_bin_compaction
               ? ObjectiveOrdering::better
               : ObjectiveOrdering::worse;
  }
  if (lhs.multi_bin.active_bin_utilization !=
      rhs.multi_bin.active_bin_utilization) {
    return lhs.multi_bin.active_bin_utilization >
                   rhs.multi_bin.active_bin_utilization
               ? ObjectiveOrdering::better
               : ObjectiveOrdering::worse;
  }
  if (lhs.order_signature != rhs.order_signature) {
    return lhs.order_signature < rhs.order_signature ? ObjectiveOrdering::better
                                                     : ObjectiveOrdering::worse;
  }
  if (lhs.rotation_signature != rhs.rotation_signature) {
    return lhs.rotation_signature < rhs.rotation_signature
               ? ObjectiveOrdering::better
               : ObjectiveOrdering::worse;
  }
  return ObjectiveOrdering::equivalent;
}

auto compare_objective(const search::SolutionPoolEntry &lhs,
                       const search::SolutionPoolEntry &rhs)
    -> ObjectiveOrdering {
  return compare_objective(evaluate_objective(lhs), evaluate_objective(rhs));
}

auto objective_better(const ObjectiveValue &lhs, const ObjectiveValue &rhs)
    -> bool {
  return compare_objective(lhs, rhs) == ObjectiveOrdering::better;
}

} // namespace shiny::nesting::pack::sparrow::optimize