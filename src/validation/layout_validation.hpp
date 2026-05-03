#pragma once

#include "internal/request_normalization.hpp"
#include "result.hpp"

namespace shiny::nesting::validation {

struct LayoutValidationOptions {
  double overlap_area_tolerance{1e-6};
  double containment_area_tolerance{1e-6};
  double spacing_tolerance{1e-6};
  double rotation_angle_tolerance{1e-9};
};

[[nodiscard]] auto validate_layout(const NormalizedRequest &request,
                                   const NestingResult &result,
                                   LayoutValidationOptions options = {})
    -> LayoutValidationReport;

[[nodiscard]] auto
layout_has_geometry_violation(const NormalizedRequest &request,
                              const NestingResult &result,
                              LayoutValidationOptions options = {}) -> bool;

auto finalize_layout_conservation(const NormalizedRequest &request,
                                  NestingResult &result) -> void;

auto finalize_result(const NormalizedRequest &request, NestingResult &result,
                     LayoutValidationOptions options = {}) -> void;

} // namespace shiny::nesting::validation
