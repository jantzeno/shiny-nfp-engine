#include "search/config.hpp"

namespace shiny::nfp::search {

auto LocalSearchConfig::is_valid() const -> bool {
  return max_iterations <= 1000000U && plateau_budget >= 1U;
}

auto GeneticSearchConfig::is_valid() const -> bool {
  return max_generations <= 1000000U && population_size >= 2U &&
         population_size <= 512U && mutation_rate_percent <= 50U &&
         elite_count >= 1U && elite_count <= population_size &&
         tournament_size >= 2U && tournament_size <= population_size &&
         plateau_generations >= 1U;
}

} // namespace shiny::nfp::search