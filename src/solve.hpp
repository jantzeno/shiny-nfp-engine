#pragma once

#include <cstdint>

#include "api/solve_control.hpp"
#include "request.hpp"
#include "result.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

[[nodiscard]] auto solve(const NestingRequest &request,
                         const SolveControl &control = {})
    -> std::expected<NestingResult, util::Status>;

[[nodiscard]] auto solve(const ProfileRequest &request,
                         const ProfileSolveControl &control = {})
    -> std::expected<NestingResult, util::Status>;

} // namespace shiny::nesting
