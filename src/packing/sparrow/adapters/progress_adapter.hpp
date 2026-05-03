#pragma once

#include "packing/sparrow/runtime/progress.hpp"
#include "runtime/progress.hpp"

namespace shiny::nesting::pack::sparrow::adapters {

[[nodiscard]] auto
to_profile_progress_snapshot(const runtime::PortProgressSnapshot &snapshot)
    -> ProfileProgressSnapshot;

} // namespace shiny::nesting::pack::sparrow::adapters