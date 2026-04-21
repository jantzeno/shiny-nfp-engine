#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/types.hpp"
#include "packing/layout.hpp"

namespace shiny::nesting::pack {

struct CutContour {
  std::uint32_t bin_id{0};
  std::uint32_t piece_id{0};
  bool from_hole{false};
  geom::Ring ring{};
};

[[nodiscard]] auto build_cutting_sequence(const Layout &layout)
    -> std::vector<CutContour>;

} // namespace shiny::nesting::pack
