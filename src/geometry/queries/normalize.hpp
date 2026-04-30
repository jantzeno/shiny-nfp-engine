#pragma once

#include "geometry/types.hpp"

namespace shiny::nesting::geom {

/**
 * @brief Canonicalizes a simple polygon ring.
 *
 * @par Algorithm Detail
 * - **Strategy**: Ring normalization by duplicate removal, winding repair, and
 *   lexicographic rotation.
 * - **Steps**:
 *   1. Drop redundant closure and consecutive duplicate vertices.
 *   2. Enforce the repository's canonical winding for outer boundaries.
 *   3. Rotate the ring so the lexicographically minimal vertex appears first.
 *
 * @par Mathematical Basis
 * - Canonicalization treats a ring as an equivalence class under cyclic vertex
 *   shifts and duplicate-closure variants, selecting one deterministic
 *   representative.
 * - Winding normalization enforces a consistent signed-area orientation so
 *   downstream geometric predicates observe stable boundary direction.
 *
 * @par Complexity
 * - **Time**: O(n) where `n` is the ring vertex count.
 * - **Space**: O(n).
 *
 * @par Invariants & Preconditions
 * - @pre The input ring should represent a simple polygon boundary.
 *
 * @par Edge Cases & Degeneracies
 * - Removes duplicated closing vertices and adjacent duplicates.
 * - Leaves short or degenerate rings in canonical but still degenerate form.
 *
 * @param polygon Polygon to canonicalize.
 * @return Canonicalized polygon with normalized outer ring ordering.
 */
[[nodiscard]] auto normalize_polygon(const Polygon &polygon) -> Polygon;

/**
 * @brief Canonicalizes a polygon with holes.
 *
 * @par Algorithm Detail
 * - **Strategy**: Outer-plus-hole normalization with independent winding
 * repair.
 * - **Steps**:
 *   1. Normalize the outer ring.
 *   2. Normalize each hole ring with the hole winding convention.
 *   3. Return a polygon whose ring ordering is stable for caching and equality
 *      checks.
 *
 * @par Mathematical Basis
 * - Applies the same ring canonicalization independently to outer and hole
 *   boundaries, preserving the topology while removing representation
 *   ambiguity.
 * - Enforces opposite signed-area orientation between outer and hole rings so
 *   inclusion tests remain consistent.
 *
 * @par Complexity
 * - **Time**: O(n) where `n` is the total vertex count across outer and hole
 *   rings.
 * - **Space**: O(n).
 *
 * @par Invariants & Preconditions
 * - @pre Outer and hole rings should each be simple.
 *
 * @par Edge Cases & Degeneracies
 * - Canonicalizes hole ordering and duplicate removal ring-by-ring.
 * - Preserves empty hole lists without special handling.
 *
 * @param polygon Polygon with holes to canonicalize.
 * @return Canonicalized polygon with normalized outer and hole rings.
 */
[[nodiscard]] auto normalize_polygon(const PolygonWithHoles &polygon)
    -> PolygonWithHoles;

} // namespace shiny::nesting::geom