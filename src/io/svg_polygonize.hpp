#pragma once

#include <string_view>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nfp::io {

/**
 * @brief Controls SVG path flattening during polygon import.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct SvgImportConfig {
  double curve_flattening_tolerance{0.25};
};

/**
 * @brief Converts one SVG path string into a polygon-with-holes approximation.
 *
 * @param svg_path_data Raw SVG path data.
 * @param config Curve flattening settings.
 * @return Polygonized path or an error status.
 */
[[nodiscard]] auto polygonize_svg_path(std::string_view svg_path_data,
                                       const SvgImportConfig &config = {})
    -> util::StatusOr<geom::PolygonWithHoles>;

} // namespace shiny::nfp::io