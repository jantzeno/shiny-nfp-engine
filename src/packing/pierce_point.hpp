#pragma once

#include "geometry/types.hpp"
#include "packing/cutting_sequence.hpp"

namespace shiny::nesting::pack {

struct PiercePlan {
  geom::Point2 pierce_point{};
  CutContourOrder::LeadArc lead_in{};
  CutContourOrder::LeadArc lead_out{};
  geom::Point2 exit_point{};
};

[[nodiscard]] auto select_pierce_plan(const CutContour &contour,
                                      const geom::Point2 &previous_exit)
    -> PiercePlan;

[[nodiscard]] auto select_pierce_point(const CutContour &contour,
                                       const geom::Point2 &previous_exit)
    -> geom::Point2;

} // namespace shiny::nesting::pack
