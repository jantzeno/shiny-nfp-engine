#pragma once

#include <cstdint>

#include "api/solve_control.hpp"
#include "request.hpp"
#include "result.hpp"
#include "util/status.hpp"

namespace shiny::nesting {

// The legacy NestingRequest overload is declared in
// src/internal/legacy_solve.hpp for internal engine use only.

[[nodiscard]] auto solve(const ProfileRequest &request,
                         const ProfileSolveControl &control = {})
    -> util::StatusOr<NestingResult>;

} // namespace shiny::nesting
