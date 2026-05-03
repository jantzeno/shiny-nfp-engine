#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "geometry/types.hpp"
#include "packing/sparrow/runtime/rng.hpp"

namespace shiny::nesting::pack::sparrow::sample {

struct UniformSample {
  geom::Point2 translation{};
  double rotation_degrees{0.0};
};

[[nodiscard]] auto sample_uniform_placements(
    runtime::SplitMix64Rng &rng, const geom::Box2 &bounds,
    std::size_t sample_count,
    std::span<const double> allowed_rotations_degrees = {})
    -> std::vector<UniformSample>;

} // namespace shiny::nesting::pack::sparrow::sample