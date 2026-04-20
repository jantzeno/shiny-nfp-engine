#pragma once

#include <array>
#include <cstdint>

#include "geometry/types.hpp"
#include "predicates/classify.hpp"

namespace shiny::nesting::pred {

/**
 * @brief Classified contact between two segments.
 *
 * Stores both the contact topology and up to two contact points for overlap and
 * point-contact cases.
 *
 * @par Invariants
 * - `point_count` specifies how many entries in `points` are valid.
 * - Vertex-contact flags report whether either segment endpoint participates in
 *   the classified contact.
 *
 * @par Performance Notes
 * - Fixed-size point storage avoids dynamic allocation during intersection
 *   classification.
 */
struct SegmentContact {
  SegmentContactKind kind{SegmentContactKind::disjoint};
  std::array<geom::Point2, 2> points{};
  std::uint8_t point_count{0};
  bool a_vertex_contact{false};
  bool b_vertex_contact{false};
};

/**
 * @brief Classifies how two segments intersect or overlap.
 *
 * @par Algorithm Detail
 * - **Strategy**: Exact-predicate segment intersection with canonicalized
 *   output points.
 * - **Steps**:
 *   1. Evaluate the segment pair through the predicate kernel.
 *   2. Convert intersection geometry back into canonical model coordinates.
 *   3. Classify the contact kind and record vertex participation flags.
 *
 * @par Mathematical Basis
 * - Uses orientation-sign tests and interval overlap on collinear projections
 *   to classify all segment-contact topologies.
 * - Canonicalized intersection coordinates collapse numerically equivalent
 *   contacts before deduplication.
 *
 * @par Invariants & Preconditions
 * - @pre Both segments must be expressed in the same coordinate frame.
 *
 * @par Edge Cases & Degeneracies
 * - Handles endpoint-only touches, proper crossings, and collinear overlaps.
 * - Canonicalizes numerically close intersection points before deduplication.
 *
 * @param lhs First segment.
 * @param rhs Second segment.
 * @return Classified contact result including up to two contact points.
 */
[[nodiscard]] auto classify_segment_contact(const geom::Segment2 &lhs,
                                            const geom::Segment2 &rhs)
    -> SegmentContact;

} // namespace shiny::nesting::pred