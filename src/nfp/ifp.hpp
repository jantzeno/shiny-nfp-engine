#pragma once

#include <vector>

#include "geometry/types.hpp"
#include "util/status.hpp"

namespace shiny::nesting::nfp {

// Closed-form Inner-Fit Rectangle (IFR) bounding box for an
// axis-aligned container and an axis-aligned piece bbox. The returned
// `Box2` describes the legal range of TRANSLATION for the piece (i.e.
// where the piece's bbox.min/max is shifted relative to the container's
// bbox.min/max). Used by:
//   * `nfp::compute_ifp` to materialise the IFR as a polygon.
//   * `pack::container_move_bounds` to clamp separation moves.
// If the piece does not fit in either axis, max < min on that axis;
// callers must check before consuming.
[[nodiscard]] auto inner_fit_rectangle_bounds(const geom::Box2 &container_bounds,
                                              const geom::Box2 &piece_bounds)
    -> geom::Box2;

// Inner-Fit Polygon: the locus of translation deltas that keep `piece`
// inside `container`.
//
// Fast path: axis-aligned rectangular containers return the closed-form
// Inner-Fit Rectangle.
//
// General path: start from the rectangular translation bounds induced by
// the container bbox, then subtract the NFP of every obstacle in
// `bbox(container) - container`. The result may be disconnected.
[[nodiscard]] auto
compute_inner_fit_polygon(const geom::PolygonWithHoles &container,
                          const geom::PolygonWithHoles &piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>>;

// Backward-compatible alias for existing callers. Returns the same
// translation-region set as `compute_inner_fit_polygon`.
[[nodiscard]] auto compute_ifp(const geom::PolygonWithHoles &container,
                               const geom::PolygonWithHoles &piece)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>>;

} // namespace shiny::nesting::nfp
