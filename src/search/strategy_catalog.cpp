#include "search/strategy_catalog.hpp"

namespace shiny::nesting::search {

void register_alns_strategy();
void register_simulated_annealing_strategy();
void register_gdrr_strategy();
void register_lahc_strategy();
void register_brkga_strategy();

void ensure_builtin_strategy_registrations() {
  static const bool registered = [] {
    register_alns_strategy();
    register_simulated_annealing_strategy();
    register_gdrr_strategy();
    register_lahc_strategy();
    register_brkga_strategy();
    return true;
  }();
  (void)registered;
}

} // namespace shiny::nesting::search
