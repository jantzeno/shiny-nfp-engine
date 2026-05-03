#pragma once

#include <cstddef>
#include <span>
#include <stop_token>
#include <vector>

#include "packing/sparrow/quantify/collision_tracker.hpp"
#include "packing/sparrow/runtime/trace.hpp"
#include "packing/sparrow/sample/coordinate_descent.hpp"
#include "packing/sparrow/sample/search_placement.hpp"

namespace shiny::nesting::pack::sparrow::optimize {

struct SeparatorConfig {
  std::size_t strike_limit{4};
  std::size_t iter_no_improvement_limit{4};
  std::size_t max_iterations{16};
  double gls_weight_cap{1e6};
  sample::SearchPlacementPolicy search_policy{
      sample::SearchPlacementPolicy::for_profile(SolveProfile::balanced)};
  sample::CoordinateDescentConfig descent{};
  std::span<const double> allowed_rotations_degrees{};
};

struct SeparatorItem {
  std::uint32_t piece_id{0};
  adapters::PortPolygon polygon{};
};

struct SeparatorResult {
  std::vector<adapters::PortPolygon> polygons{};
  double total_loss{0.0};
  bool converged{false};
  std::size_t iterations{0};
  std::size_t accepted_moves{0};
};

[[nodiscard]] auto run_separator(const adapters::PortPolygon &container,
                                 std::span<const SeparatorItem> items,
                                 const SeparatorConfig &config,
                                 runtime::SplitMix64Rng &rng,
                                 runtime::TraceCapture *trace = nullptr,
                                 std::stop_token stoken = {})
    -> SeparatorResult;

} // namespace shiny::nesting::pack::sparrow::optimize