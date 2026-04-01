#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace shiny::nfp {

/**
 * @brief Identifies one outward-facing algorithm family.
 *
 * Short paragraph describing the stable vocabulary shared by API surfaces,
 * fixtures, examples, tools, and benchmarks.
 *
 * @par Invariants
 * - Every enumerator maps to exactly one canonical snake_case label.
 *
 * @par Performance Notes
 * - Canonical string conversion uses constant-time switch dispatch.
 */
enum class AlgorithmKind : std::uint8_t {
  constructive_decoder = 0,
  jostle_search = 1,
  genetic_search = 2,
  masonry_builder = 3,
  convex_nfp = 4,
  convex_ifp = 5,
  nonconvex_graph_nfp = 6,
  orbital_verifier = 7,
  convex_decomposition = 8,
};

/**
 * @brief Returns the canonical serialization label for one algorithm family.
 *
 * Short paragraph describing the repository-wide string vocabulary used in
 * emitted artifacts.
 *
 * @param kind Algorithm family to serialize.
 * @return Canonical snake_case label for `kind`.
 *
 * @pre `kind` must be one of the supported enumerators.
 * @post Returns the same label for the same enumerator on every call.
 * @par Determinism
 * - Deterministic for a fixed `kind`.
 */
[[nodiscard]] constexpr auto to_string(AlgorithmKind kind) -> std::string_view {
  switch (kind) {
  case AlgorithmKind::constructive_decoder:
    return "constructive_decoder";
  case AlgorithmKind::jostle_search:
    return "jostle_search";
  case AlgorithmKind::genetic_search:
    return "genetic_search";
  case AlgorithmKind::masonry_builder:
    return "masonry_builder";
  case AlgorithmKind::convex_nfp:
    return "convex_nfp";
  case AlgorithmKind::convex_ifp:
    return "convex_ifp";
  case AlgorithmKind::nonconvex_graph_nfp:
    return "nonconvex_graph_nfp";
  case AlgorithmKind::orbital_verifier:
    return "orbital_verifier";
  case AlgorithmKind::convex_decomposition:
    return "convex_decomposition";
  }

  return "unknown";
}

/**
 * @brief Parses one canonical algorithm label.
 *
 * Short paragraph describing conversion from artifact strings back into the
 * shared outward-facing algorithm vocabulary.
 *
 * @param value Canonical snake_case algorithm label.
 * @return Matching algorithm family when `value` is recognized.
 *
 * @pre `value` should come from trusted repository artifacts or validated user
 *   input.
 * @post Returns an empty optional when `value` is not part of the canonical
 *   vocabulary.
 * @par Determinism
 * - Deterministic for a fixed input string.
 */
[[nodiscard]] constexpr auto parse_algorithm_kind(std::string_view value)
    -> std::optional<AlgorithmKind> {
  if (value == "constructive_decoder") {
    return AlgorithmKind::constructive_decoder;
  }
  if (value == "jostle_search") {
    return AlgorithmKind::jostle_search;
  }
  if (value == "genetic_search") {
    return AlgorithmKind::genetic_search;
  }
  if (value == "masonry_builder") {
    return AlgorithmKind::masonry_builder;
  }
  if (value == "convex_nfp") {
    return AlgorithmKind::convex_nfp;
  }
  if (value == "convex_ifp") {
    return AlgorithmKind::convex_ifp;
  }
  if (value == "nonconvex_graph_nfp") {
    return AlgorithmKind::nonconvex_graph_nfp;
  }
  if (value == "orbital_verifier") {
    return AlgorithmKind::orbital_verifier;
  }
  if (value == "convex_decomposition") {
    return AlgorithmKind::convex_decomposition;
  }

  return std::nullopt;
}

} // namespace shiny::nfp