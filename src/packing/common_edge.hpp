#pragma once

#include <vector>

#include "packing/layout.hpp"

namespace shiny::nesting::pack {

[[nodiscard]] auto detect_common_edges(std::vector<CutSegment> segments)
    -> std::vector<CutSegment>;

} // namespace shiny::nesting::pack
