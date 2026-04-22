#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "request.hpp"

namespace shiny::nesting::search::detail {

// Multi-mode cooling schedule used by SA and (geometric only) ALNS.
//
// Schedules:
//   * geometric: T ← T × α, where α = (T_final/T_initial)^(1/(N-1))
//     so after N iterations T reaches T_final.
//   * linear: T = T_initial + (T_final - T_initial) × progress.
//   * adaptive: T ← T × 0.90 on accept, × 0.97 on reject. Faster
//     cooling when many candidates are accepting (likely too hot).
//   * lundy-mees: T ← T / (1 + β × T). Slower as T shrinks; good for
//     long horizons.
//
// All schedules clamp at T_final to prevent collapse to 0.
class CoolingSchedule {
public:
  explicit CoolingSchedule(const SAConfig &config)
      : kind_{config.cooling_schedule},
        max_refinements_{std::max<std::size_t>(config.max_refinements, 1U)},
        initial_temperature_{std::max(config.initial_temperature, 1e-9)},
        final_temperature_{std::clamp(config.final_temperature, 1e-9,
                                      std::max(config.initial_temperature, 1e-9))},
        lundy_beta_{config.lundy_beta},
        geometric_alpha_{max_refinements_ <= 1U
                             ? 1.0
                             : std::pow(final_temperature_ / initial_temperature_,
                                        1.0 / static_cast<double>(max_refinements_ - 1U))} {}

  [[nodiscard]] auto initial_temperature() const -> double {
    return initial_temperature_;
  }

  [[nodiscard]] auto next_temperature(const double current_temperature,
                                      const std::size_t iteration,
                                      const bool accepted) const -> double {
    const double current =
        std::max(final_temperature_, std::max(current_temperature, 1e-9));
    double cooled = current;
    switch (kind_) {
    case CoolingScheduleKind::geometric:
      cooled = current * geometric_alpha_;
      break;
    case CoolingScheduleKind::linear: {
      const double progress =
          std::clamp(static_cast<double>(iteration + 1U) /
                         static_cast<double>(max_refinements_),
                     0.0, 1.0);
      cooled = initial_temperature_ +
               ((final_temperature_ - initial_temperature_) * progress);
      break;
    }
    case CoolingScheduleKind::adaptive:
      cooled = current * (accepted ? 0.90 : 0.97);
      break;
    case CoolingScheduleKind::lundy_mees:
      cooled = current / (1.0 + (lundy_beta_ * current));
      break;
    }
    return std::max(final_temperature_, cooled);
  }

private:
  CoolingScheduleKind kind_{};
  std::size_t max_refinements_{1U};
  double initial_temperature_{1.0};
  double final_temperature_{0.01};
  double lundy_beta_{0.025};
  double geometric_alpha_{1.0};
};

} // namespace shiny::nesting::search::detail
