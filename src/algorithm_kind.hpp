#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace shiny::nesting {

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
  bounding_box = 0,
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
  case AlgorithmKind::bounding_box:
    return "bounding_box";
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
  if (value == "bounding_box") {
    return AlgorithmKind::bounding_box;
  }

  return std::nullopt;
}

} // namespace shiny::nesting
