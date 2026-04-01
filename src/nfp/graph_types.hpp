#pragma once

#include <cstdint>
#include <vector>

#include "geometry/types.hpp"

namespace shiny::nfp {

/**
 * @brief Labels how a graph vertex entered the arrangement graph.
 */
enum class GraphVertexKind : std::int8_t {
  original_vertex = 0,
  intersection_vertex = 1,
  midpoint_vertex = 2,
};

/**
 * @brief Labels how an arrangement edge participates in extraction.
 */
enum class GraphEdgeKind : std::int8_t {
  boundary_edge = 0,
  duplicate_edge = 1,
  pruned_edge = 2,
};

/**
 * @brief One vertex in the translational arrangement graph.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct GraphVertex {
  std::uint32_t id{0};
  geom::Point2 point{};
  GraphVertexKind kind{GraphVertexKind::original_vertex};
};

/**
 * @brief One directed edge in the translational arrangement graph.
 *
 * @par Thread Safety
 * - Plain value type with no shared state.
 */
struct GraphEdge {
  std::uint32_t from{0};
  std::uint32_t to{0};
  GraphEdgeKind kind{GraphEdgeKind::boundary_edge};
};

/**
 * @brief Graph representation of the nonconvex NFP arrangement.
 *
 * @par Invariants
 * - `pruned` reports whether unusable edges have already been removed.
 */
struct ArrangementGraph {
  std::vector<GraphVertex> vertices{};
  std::vector<GraphEdge> edges{};
  std::vector<geom::Point2> perfect_fit_points{};
  std::vector<geom::Segment2> perfect_sliding_segments{};
  bool pruned{false};
};

} // namespace shiny::nfp