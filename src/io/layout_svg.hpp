#pragma once

#include <filesystem>

#include "packing/layout.hpp"
#include "util/status.hpp"

namespace shiny::nesting::io {

[[nodiscard]] auto save_layout_svg(const std::filesystem::path &path,
                                   const pack::Layout &layout) -> util::Status;

} // namespace shiny::nesting::io
