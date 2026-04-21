#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "request.hpp"
#include "runtime/deterministic_rng.hpp"

namespace shiny::nesting::search {

struct DisruptionResult {
  std::vector<std::size_t> order{};
  std::size_t first_position{0};
  std::size_t second_position{0};
  bool applied{false};
};

[[nodiscard]] auto disrupt_large_items(std::span<const std::size_t> order,
                                       const NormalizedRequest &request,
                                       std::span<const double> piece_areas,
                                       runtime::DeterministicRng &rng)
    -> DisruptionResult;

} // namespace shiny::nesting::search
