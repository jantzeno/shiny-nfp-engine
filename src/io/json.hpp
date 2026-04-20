#pragma once

#include <filesystem>
#include <span>
#include <vector>

#include "geometry/types.hpp"
#include "packing/layout.hpp"
#include "util/status.hpp"

namespace shiny::nesting::io {

/**
 * @brief Loads a polygon set from the repo JSON interchange format.
 *
 * @param path Input JSON file.
 * @return Parsed polygon set or an error status.
 */
[[nodiscard]] auto load_polygon_set(const std::filesystem::path &path)
    -> util::StatusOr<std::vector<geom::PolygonWithHoles>>;

/**
 * @brief Saves a polygon set to the repo JSON interchange format.
 *
 * @param path Output JSON file.
 * @param polygons Polygon set to serialize.
 * @return Operation status.
 */
[[nodiscard]] auto
save_polygon_set(const std::filesystem::path &path,
                 std::span<const geom::PolygonWithHoles> polygons)
    -> util::Status;

/**
 * @brief Serializes a packing layout to JSON.
 *
 * @param path Output JSON file.
 * @param layout Layout to serialize.
 * @return Operation status.
 */
[[nodiscard]] auto save_layout(const std::filesystem::path &path,
                               const pack::Layout &layout) -> util::Status;

/**
 * @brief Serializes a cut plan to JSON.
 *
 * @param path Output JSON file.
 * @param cut_plan Cut plan to serialize.
 * @return Operation status.
 */
[[nodiscard]] auto save_cut_plan(const std::filesystem::path &path,
                                 const pack::CutPlan &cut_plan) -> util::Status;

} // namespace shiny::nesting::io