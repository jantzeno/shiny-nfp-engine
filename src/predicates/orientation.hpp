#pragma once

#include "geometry/types.hpp"
#include "predicates/classify.hpp"

namespace shiny::nesting::pred {

/**
 * @brief Triple of points for orientation testing.
 *
 * Encapsulates the directed turn query used across convexity checks, ring
 * simplification, and contact classification.
 *
 * @par Invariants
 * - Points are interpreted as the ordered sequence `a -> b -> c`.
 *
 * @par Performance Notes
 * - Stored by value because the query is tiny and short-lived.
 */
struct OrientationQuery {
  geom::Point2 a{};
  geom::Point2 b{};
  geom::Point2 c{};
};

/**
 * @brief Classifies the turn direction of three ordered points.
 *
 * @par Algorithm Detail
 * - **Strategy**: Exact-predicate orientation test.
 * - **Steps**:
 *   1. Interpret the query as two directed edges, `a -> b` and `b -> c`.
 *   2. Evaluate the signed turn using the predicate kernel.
 *   3. Return clockwise, counterclockwise, or collinear classification.
 *
 * @par Mathematical Basis
 * - Computes the sign of the 2D cross product
 *   `(b - a) x (c - b)` to distinguish left turn, right turn, and collinearity.
 * - Exact-predicate evaluation preserves the sign classification under nearly
 *   parallel or very small-area triples.
 *
 * @par Invariants & Preconditions
 * - @pre Query points must be expressed in a common coordinate frame.
 *
 * @par Edge Cases & Degeneracies
 * - Returns the collinear classification for zero-area triples.
 * - Handles nearly parallel inputs through the exact-predicate backend.
 *
 * @param query Ordered triple of points to classify.
 * @return Turn classification for the directed point triple.
 */
[[nodiscard]] auto orient(const OrientationQuery &query) -> Orientation;

} // namespace shiny::nesting::pred