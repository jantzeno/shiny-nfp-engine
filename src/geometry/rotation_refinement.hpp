#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry/transform.hpp"

namespace shiny::nesting::geom {

using RefinementRotationRange = RotationRange;

// This module intentionally exposes discrete-grid sampling plus a logarithmic
// local refinement schedule. It does not perform continuous optimization over
// an overlap objective; callers evaluate the returned candidate angles.
[[nodiscard]] inline auto sample_refinement_range(const RotationRange &range)
    -> std::vector<double> {
  return materialize_rotations(DiscreteRotationSet{.range_degrees = range});
}

// Two-level local refinement around `seed_degrees`:
//   * Start: emit the seed angle.
//   * Repeat for `levels` rounds (default 4): emit seed±step within
//     range, then halve the step. Stops when step < 1e-6° or levels
//     exhausted. Generates a logarithmic schedule of ±step samples.
//
// Output is sorted+deduplicated (1e-9° tolerance) and then ROTATED so
// the seed angle appears first. Caller can take the first N as the
// "most local" first-pass candidates.
//
// IMPORTANT: range membership is checked AFTER `normalize_angle_degrees`
// so that wrap-around (e.g. seed near 0° with negative step landing at
// -1°, normalized to 359°) does not silently emit angles outside the
// configured [min_degrees, max_degrees] window. Previously, the bounds
// check was applied to the un-normalized value which let wrapped
// candidates slip through.
[[nodiscard]] inline auto local_refinement_angles(const RotationRange &range,
                                                  const double seed_degrees,
                                                  const std::size_t levels = 4U)
    -> std::vector<double> {
  std::vector<double> angles{normalize_angle_degrees(seed_degrees)};
  if (!std::isfinite(range.min_degrees) || !std::isfinite(range.max_degrees) ||
      !std::isfinite(range.step_degrees) || range.step_degrees <= 0.0 ||
      range.max_degrees < range.min_degrees) {
    return angles;
  }

  const auto in_range = [&](const double angle) {
    const double normalized = normalize_angle_degrees(angle);
    return normalized >= range.min_degrees - 1e-9 &&
           normalized <= range.max_degrees + 1e-9;
  };

  double step = range.step_degrees * 0.5;
  for (std::size_t level = 0; level < levels && step > 1e-6; ++level) {
    const double lower = seed_degrees - step;
    const double upper = seed_degrees + step;
    if (in_range(lower)) {
      angles.push_back(normalize_angle_degrees(lower));
    }
    if (in_range(upper)) {
      angles.push_back(normalize_angle_degrees(upper));
    }
    step *= 0.5;
  }

  std::sort(angles.begin(), angles.end());
  angles.erase(std::unique(angles.begin(), angles.end(),
                           [](const double lhs, const double rhs) {
                             return std::fabs(lhs - rhs) <= 1e-9;
                           }),
               angles.end());
  if (!angles.empty()) {
    std::rotate(angles.begin(),
                std::find_if(angles.begin(), angles.end(), [&](const double angle) {
                  return std::fabs(angle - normalize_angle_degrees(seed_degrees)) <= 1e-9;
                }),
                angles.end());
  }
  return angles;
}

} // namespace shiny::nesting::geom
