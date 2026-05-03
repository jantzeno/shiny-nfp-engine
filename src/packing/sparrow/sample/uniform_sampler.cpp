#include "packing/sparrow/sample/uniform_sampler.hpp"

namespace shiny::nesting::pack::sparrow::sample {

auto sample_uniform_placements(
    runtime::SplitMix64Rng &rng, const geom::Box2 &bounds,
    const std::size_t sample_count,
    std::span<const double> allowed_rotations_degrees)
    -> std::vector<UniformSample> {
  std::vector<UniformSample> samples;
  samples.reserve(sample_count);

  const double width = bounds.max.x() - bounds.min.x();
  const double height = bounds.max.y() - bounds.min.y();
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double rotation = allowed_rotations_degrees.empty()
                                ? 0.0
                                : allowed_rotations_degrees[rng.uniform_index(
                                      allowed_rotations_degrees.size())];
    samples.push_back({
        .translation =
            geom::Point2{
                bounds.min.x() + width * rng.uniform_real(),
                bounds.min.y() + height * rng.uniform_real(),
            },
        .rotation_degrees = rotation,
    });
  }

  return samples;
}

} // namespace shiny::nesting::pack::sparrow::sample