#pragma once

#include <cstdint>

namespace shiny::nfp::search {

/**
 * @brief Configures the local-search refinement loop.
 *
 * @par Invariants
 * - `plateau_budget` bounds consecutive non-improving iterations before
 *   termination.
 *
 * @par Performance Notes
 * - Shared by fixture-driven tests and production search requests.
 */
struct LocalSearchConfig {
  std::uint32_t max_iterations{250};
  std::uint32_t deterministic_seed{1};
  std::uint32_t plateau_budget{3};

  /**
   * @brief Reports whether the local-search settings are supported.
   *
   * @return `true` when iteration and plateau budgets are in range.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

/**
 * @brief Configures the production genetic-search planner.
 *
 * Defines the deterministic generation budget, population size, selection
 * pressure, mutation rate, elitism budget, and optional nested local-search
 * polishing used by `GeneticSearch`.
 *
 * @par Invariants
 * - Mutation rate is interpreted as a percentage in whole numbers.
 * - `elite_count` and `tournament_size` must not exceed
 *   `population_size`.
 *
 * @par Performance Notes
 * - Larger populations and enabled polishing increase decode volume roughly in
 *   proportion to generations times population size.
 */
struct GeneticSearchConfig {
  std::uint32_t max_generations{40};
  std::uint32_t population_size{30};
  std::uint32_t deterministic_seed{1};
  std::uint8_t mutation_rate_percent{10};
  std::uint32_t elite_count{2};
  std::uint32_t tournament_size{3};
  std::uint32_t plateau_generations{6};
  bool enable_local_search_polish{true};
  bool enabled{false};

  /**
   * @brief Reports whether the genetic-search settings are supported.
   *
   * @return `true` when the configured GA surface is internally consistent.
   */
  [[nodiscard]] auto is_valid() const -> bool;
};

} // namespace shiny::nfp::search