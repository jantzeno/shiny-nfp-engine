#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "geometry/transform.hpp"
#include "packing/irregular/workspace.hpp"
#include "request.hpp"
#include "runtime/deterministic_rng.hpp"
#include "runtime/timing.hpp"
#include "search/solution_pool.hpp"
#include "solve.hpp"

namespace shiny::nesting::search::detail {

enum class NeighborhoodSearch : std::uint8_t {
  adjacent_swap = 0,
  random_swap = 1,
  relocate = 2,
  inversion = 3,
  large_item_swap = 4,
  rotation_change = 5,
  random_destroy_repair = 6,
  area_destroy_repair = 7,
  related_destroy_repair = 8,
  cluster_destroy_repair = 9,
  regret_destroy_repair = 10,
};

struct NeighborhoodMove {
  NeighborhoodSearch op{NeighborhoodSearch::random_swap};
  std::vector<std::size_t> order{};
  std::vector<std::optional<geom::RotationIndex>> forced_rotations{};
  std::size_t primary_index{0};
  std::size_t secondary_index{0};
  std::size_t destroy_count{0};
  bool changed{false};
};

class OrderEvaluator {
public:
  OrderEvaluator(const NormalizedRequest &request, const SolveControl &control,
                 const runtime::TimeBudget &time_budget,
                 const runtime::Stopwatch &stopwatch);

  [[nodiscard]] auto evaluate(
      std::span<const std::size_t> order,
      std::span<const std::optional<geom::RotationIndex>> forced_rotations = {},
      std::uint64_t seed_bias = 0) const -> SolutionPoolEntry;

  [[nodiscard]] auto interrupted() const -> bool;

  [[nodiscard]] auto make_budget(std::size_t operations_completed) const
      -> BudgetState;

  [[nodiscard]] auto piece_areas() const -> std::span<const double>;
  [[nodiscard]] auto piece_rotation_counts() const
      -> std::span<const std::size_t>;

private:
  const NormalizedRequest &request_;
  const SolveControl &control_;
  const runtime::TimeBudget &time_budget_;
  const runtime::Stopwatch &stopwatch_;
  mutable pack::PackerWorkspace local_workspace_{};
  pack::PackerWorkspace *workspace_{nullptr};
  std::vector<double> piece_areas_{};
  std::vector<std::size_t> piece_rotation_counts_{};
};

// Compute the per-expanded-piece area vector for `request`.  Used by
// OrderEvaluator internally and by BRKGA which manages its own evaluation.
[[nodiscard]] auto piece_areas_for(const NormalizedRequest &request)
    -> std::vector<double>;

[[nodiscard]] auto original_order(const NormalizedRequest &request)
    -> std::vector<std::size_t>;

[[nodiscard]] auto descending_area_order(std::span<const double> piece_areas)
    -> std::vector<std::size_t>;

[[nodiscard]] auto reverse_order(const NormalizedRequest &request)
    -> std::vector<std::size_t>;

[[nodiscard]] auto random_order(std::size_t piece_count,
                                runtime::DeterministicRng &rng)
    -> std::vector<std::size_t>;

[[nodiscard]] auto original_forced_rotations(const NormalizedRequest &request)
    -> std::vector<std::optional<geom::RotationIndex>>;

[[nodiscard]] auto propose_move(
    std::span<const std::size_t> order,
    std::span<const std::optional<geom::RotationIndex>> forced_rotations,
    const NormalizedRequest &request, std::span<const double> piece_areas,
    std::span<const std::size_t> piece_rotation_counts,
    runtime::DeterministicRng &rng, NeighborhoodSearch op,
    std::size_t intensity = 1U) -> NeighborhoodMove;

[[nodiscard]] auto random_operator(runtime::DeterministicRng &rng,
                                   bool include_destroy_repair = true)
    -> NeighborhoodSearch;

// Canonical operator set used by ALNS-style adaptive selection drivers.
// Centralised here so adding a new operator only requires touching this
// list (plus any operator-specific dispatch in `propose_move`). Keep
// the list ordered destroy-repair → swaps → rotation/relocate so the
// adaptive weight table semantics are stable across drivers.
[[nodiscard]] auto all_alns_operators() -> std::vector<NeighborhoodSearch>;

[[nodiscard]] auto objective_score(const LayoutMetrics &metrics) -> double;

[[nodiscard]] auto primary_metrics_preserved(const LayoutMetrics &candidate,
                                             const LayoutMetrics &reference)
    -> bool;

[[nodiscard]] auto within_record_window(const LayoutMetrics &candidate,
                                        const LayoutMetrics &best,
                                        double tolerance_ratio) -> bool;

[[nodiscard]] auto operator_label(NeighborhoodSearch op) -> std::string_view;

} // namespace shiny::nesting::search::detail
