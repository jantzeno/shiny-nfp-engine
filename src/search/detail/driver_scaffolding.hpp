#pragma once

// search/detail/driver_scaffolding.hpp
//
// Design: free helper functions only — no base class, no CRTP.
//
// Rationale: the five search drivers (SA, ALNS, GDRR, LAHC, BRKGA)
// share a handful of *stateless* utilities (stop-reason classification,
// interruption check, budget construction, empty-request short-circuit)
// that have nothing structurally in common beyond those utilities.  A
// base class or CRTP would add inheritance hierarchy indirection with
// zero benefit; free functions keep call sites obvious and allow each
// driver to remain a standalone value type.
//
// Extracted patterns (finding #79):
//   • driver_interrupted   — verbatim in brkga_search.cpp; identical
//                            logic lives inside OrderEvaluator::interrupted
//                            used by the other four drivers.
//   • driver_stop_reason   — 14-line function duplicated identically in
//                            all five drivers.
//   • driver_make_budget   — 12-line function duplicated in brkga_search.cpp;
//                            equivalent logic is OrderEvaluator::make_budget
//                            used by the other four drivers.
//   • driver_empty_result  — ~14-line early-return block duplicated in all
//                            five drivers, differing only in StrategyKind.

#include <cstddef>
#include <cstdint>

#include "observer.hpp"
#include "request.hpp"
#include "result.hpp"
#include "runtime/timing.hpp"
#include "solve.hpp"

namespace shiny::nesting::search::detail {

// True iff any stop condition is currently active (cancellation or time
// limit expired).  Mirrors OrderEvaluator::interrupted() for drivers that
// manage their own timing state without an OrderEvaluator (e.g. BRKGA).
[[nodiscard]] auto
driver_interrupted(const SolveControl &control,
                   const runtime::TimeBudget &time_budget,
                   const runtime::Stopwatch &stopwatch) noexcept -> bool;

// Classify the terminal stop reason after a search loop exits.
[[nodiscard]] auto driver_stop_reason(const SolveControl &control,
                                      const runtime::TimeBudget &time_budget,
                                      const runtime::Stopwatch &stopwatch,
                                      bool hit_operation_limit) noexcept
    -> StopReason;

// Build a BudgetState snapshot from the current timing and control state.
[[nodiscard]] auto driver_make_budget(const SolveControl &control,
                                      const runtime::TimeBudget &time_budget,
                                      const runtime::Stopwatch &stopwatch,
                                      std::size_t operations_completed) noexcept
    -> BudgetState;

// Construct the zero-iteration NestingResult returned when a request
// carries no expanded pieces.  Takes ownership of `replay`.
[[nodiscard]] auto driver_empty_result(
    StrategyKind strategy, SearchReplay replay, const SolveControl &control,
    const runtime::TimeBudget &time_budget,
    const runtime::Stopwatch &stopwatch) noexcept -> NestingResult;

} // namespace shiny::nesting::search::detail
