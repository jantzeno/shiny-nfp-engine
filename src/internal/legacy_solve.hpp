#pragma once

// Internal header: not part of the public exported surface.
//
// Declares the legacy solve(NestingRequest, SolveControl) overload that
// dispatches through the strategy-era ExecutionPolicy path. This overload is
// retained for internal engine use (e.g. the profile-request translation path
// inside solve.cpp) but is not exposed as a primary downstream API.
//
// Downstream consumers should use solve(ProfileRequest, ProfileSolveControl)
// declared in src/solve.hpp.

#include "api/solve_control.hpp"
#include "request.hpp"
#include "result.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

[[nodiscard]] auto solve(const NestingRequest &request,
                         const SolveControl &control = {})
    -> std::expected<NestingResult, util::Status>;

} // namespace shiny::nesting
