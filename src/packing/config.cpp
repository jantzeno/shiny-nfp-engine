#include "packing/config.hpp"

namespace shiny::nfp::pack {

auto LaserCutOptimizationConfig::is_valid() const -> bool {
  switch (mode) {
  case SharedCutOptimizationMode::off:
  case SharedCutOptimizationMode::remove_fully_covered_coincident_segments:
    return true;
  }

  return false;
}

auto PackingConfig::is_valid() const -> bool {
  return placement.is_valid() && laser_cut_optimization.is_valid();
}

} // namespace shiny::nfp::pack