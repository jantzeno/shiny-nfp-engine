#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "internal/legacy_solve.hpp"
#include "packing/layout.hpp"
#include "packing/sparrow/config.hpp"
#include "solve.hpp"

namespace shiny::nesting::test::sparrow {

struct FixtureDescriptor {
  std::string_view name{};
  pack::sparrow::ParityTestDomain domain{
      pack::sparrow::ParityTestDomain::quantify};
  pack::sparrow::ReferenceAdaptation adaptation{
      pack::sparrow::ReferenceAdaptation::direct_translation};
  bool convex{false};
  bool concave{false};
  bool holes{false};
  bool near_touching{false};
  bool exclusion_pressure{false};
  bool multi_bin_carryover{false};
};

struct NormalizedPlacement {
  std::uint32_t piece_id{0};
  std::uint32_t bin_id{0};
  geom::Point2 translation{};
  pack::ConstructivePlacementPhase phase{
      pack::ConstructivePlacementPhase::primary_order};
};

struct NormalizedLayout {
  std::size_t bin_count{0};
  std::vector<NormalizedPlacement> placements{};
  std::vector<std::uint32_t> unplaced_piece_ids{};
};

inline constexpr std::array<FixtureDescriptor, 6> kFixtureManifest{{
    {
        .name = "convex_baseline",
        .domain = pack::sparrow::ParityTestDomain::quantify,
        .adaptation = pack::sparrow::ReferenceAdaptation::direct_translation,
        .convex = true,
        .near_touching = true,
    },
    {
        .name = "concave_tracker_pressure",
        .domain = pack::sparrow::ParityTestDomain::tracker,
        .adaptation = pack::sparrow::ReferenceAdaptation::fixture_adaptation,
        .concave = true,
        .exclusion_pressure = true,
    },
    {
        .name = "holes_sampling_window",
        .domain = pack::sparrow::ParityTestDomain::sampler,
        .adaptation = pack::sparrow::ReferenceAdaptation::fixture_adaptation,
        .holes = true,
    },
    {
        .name = "separator_near_touching",
        .domain = pack::sparrow::ParityTestDomain::separator,
        .adaptation = pack::sparrow::ReferenceAdaptation::behavior_invariant,
        .near_touching = true,
    },
    {
        .name = "compression_exclusion_pressure",
        .domain = pack::sparrow::ParityTestDomain::compression,
        .adaptation = pack::sparrow::ReferenceAdaptation::behavior_invariant,
        .exclusion_pressure = true,
    },
    {
        .name = "multi_bin_carryover",
        .domain = pack::sparrow::ParityTestDomain::profile_maximum_search,
        .adaptation = pack::sparrow::ReferenceAdaptation::fixture_adaptation,
        .concave = true,
        .multi_bin_carryover = true,
    },
}};

[[nodiscard]] inline auto fixture_manifest()
    -> std::span<const FixtureDescriptor> {
  return kFixtureManifest;
}

[[nodiscard]] inline auto make_profile_control(const std::uint64_t seed)
    -> ProfileSolveControl {
  return {
      .random_seed = seed,
      .seed_mode = SeedProgressionMode::increment,
  };
}

[[nodiscard]] inline auto normalize_layout(const pack::Layout &layout)
    -> NormalizedLayout {
  NormalizedLayout normalized{
      .bin_count = layout.bins.size(),
      .unplaced_piece_ids = layout.unplaced_piece_ids,
  };
  normalized.placements.reserve(layout.placement_trace.size());
  for (const auto &entry : layout.placement_trace) {
    normalized.placements.push_back({
        .piece_id = entry.piece_id,
        .bin_id = entry.bin_id,
        .translation = entry.translation,
        .phase = entry.phase,
    });
  }
  return normalized;
}

[[nodiscard]] inline auto nearly_equal(const double lhs, const double rhs,
                                       const double tolerance = 1e-6) -> bool {
  return std::fabs(lhs - rhs) <= tolerance;
}

[[nodiscard]] inline auto same_layout(const pack::Layout &lhs,
                                      const pack::Layout &rhs,
                                      const double tolerance = 1e-6) -> bool {
  const auto normalized_lhs = normalize_layout(lhs);
  const auto normalized_rhs = normalize_layout(rhs);
  if (normalized_lhs.bin_count != normalized_rhs.bin_count ||
      normalized_lhs.unplaced_piece_ids != normalized_rhs.unplaced_piece_ids ||
      normalized_lhs.placements.size() != normalized_rhs.placements.size()) {
    return false;
  }

  for (std::size_t index = 0; index < normalized_lhs.placements.size();
       ++index) {
    const auto &lhs_entry = normalized_lhs.placements[index];
    const auto &rhs_entry = normalized_rhs.placements[index];
    if (lhs_entry.piece_id != rhs_entry.piece_id ||
        lhs_entry.bin_id != rhs_entry.bin_id ||
        lhs_entry.phase != rhs_entry.phase ||
        !nearly_equal(lhs_entry.translation.x(), rhs_entry.translation.x(),
                      tolerance) ||
        !nearly_equal(lhs_entry.translation.y(), rhs_entry.translation.y(),
                      tolerance)) {
      return false;
    }
  }
  return true;
}

} // namespace shiny::nesting::test::sparrow