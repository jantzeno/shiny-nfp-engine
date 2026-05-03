#pragma once

#include "packing/sparrow/adapters/geometry_adapter.hpp"
#include "util/status.hpp"

namespace shiny::nesting::pack::sparrow::quantify {

struct OverlapProxyResult {
  double overlap_area{0.0};
  bool has_overlap{false};
};

[[nodiscard]] auto quantify_overlap(const adapters::PortPolygon &lhs,
                                    const adapters::PortPolygon &rhs)
    -> std::expected<OverlapProxyResult, util::Status>;

} // namespace shiny::nesting::pack::sparrow::quantify