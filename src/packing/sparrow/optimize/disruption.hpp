#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "internal/request_normalization.hpp"
#include "packing/sparrow/runtime/trace.hpp"
#include "runtime/deterministic_rng.hpp"
#include "packing/sparrow/search/solution_pool.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct DisruptionResult {
  std::vector<std::size_t> order{};
  std::size_t first_position{0};
  std::size_t second_position{0};
  bool applied{false};
};

struct RankedRollbackCandidate {
  search::SolutionPoolEntry entry{};
  std::size_t rank{0};
};

struct RollbackDecision {
  search::SolutionPoolEntry next_incumbent{};
  bool rolled_back{false};
  bool restored_previous_incumbent{false};
  std::size_t rank{0};
};

[[nodiscard]] auto disrupt_large_items(
    std::span<const std::size_t> order, const NormalizedRequest &request,
    std::span<const double> piece_areas,
    ::shiny::nesting::runtime::DeterministicRng &rng) -> DisruptionResult;

[[nodiscard]] auto
select_rollback_candidate(std::span<const search::SolutionPoolEntry> entries,
                          const search::SolutionPoolEntry &incumbent,
                          std::uint64_t seed = 0,
                          runtime::TraceCapture *trace = nullptr)
    -> std::optional<RankedRollbackCandidate>;

[[nodiscard]] auto
resolve_rollback_candidate(const search::SolutionPoolEntry &incumbent,
                           std::optional<RankedRollbackCandidate> rollback)
    -> RollbackDecision;

} // namespace shiny::nesting::pack::sparrow::optimize