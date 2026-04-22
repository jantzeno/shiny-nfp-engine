# Milestone 5 review — Claude Sonnet 4.6

Source of truth: `plan.md`, especially **Milestone 5 — Quality, completeness, robustness review** (`plan.md:95-103`).

## Baseline

- Configure/build: `xmake f -m debug && xmake`
- Run tests: `xmake run shiny_nesting_engine_tests`
- Primary M5 suites:
  - `tests/integration/sequential_backtrack.cpp` (482 lines, 9 test cases)
  - `tests/integration/metaheuristic_search.cpp` (562 lines, 9 test cases)
- Supporting suites touched by this review:
  - `tests/unit/constructive_irregular.cpp` (74 lines, 1 test case)
  - `tests/unit/sequential_backtrack.cpp` (534 lines, 9 test cases)
  - `tests/integration/bounding_box_engine_surface.cpp` (449 lines)

---

## Milestone 5 checklist — line-by-line status

| Checklist item | Status | Evidence | Follow-up |
|---|---|---|---|
| Constructive suite covers full breadth/depth matrix | **Satisfied** | `sequential_backtrack.cpp:175-217` generates over 7 dimensions (bed, spacing, 4 candidate strategies, 4 orderings, overlap-check, backtracking, compaction); targeted depth tests cover all plan items | Keep as-is |
| Production suite covers same surface categories plus optimizer breadth | **Not satisfied — see F3** | Depth tests all hardcode `simulated_annealing`; `brkga/alns/gdrr/lahc` untested for rotations, mirror, quantity, allowed\_bin\_ids, maintain\_bed\_assignment, part-in-part, concave-candidates | GENERATE all 5 kinds in all 7 depth tests |
| Every non-stop/cancel and non-maintain_bed_assignment case asserts all parts placed | **Mostly satisfied — see F6** | Confirmed across all non-conflict depth tests; deliberate conflict section uses `expected_placed_count = 1` | Tighten milestone wording to explicitly exempt admissibility-conflict cases |
| No smoke-only cases remain | **Mostly satisfied — see F4** | M5 suites have real assertions; residual "fast-lane smoke test" comment in `bounding_box_engine_surface.cpp:334` | Update comment wording |
| Grain compatibility still excluded and called out as TBD | **Partially satisfied — see F5** | Grain has unit/packing/masonry coverage elsewhere; M5 integration files contain no exclusion note | Add brief out-of-scope comment to both integration files |
| Naming stays descriptive; deep tests not under section_x files | **Satisfied** | All M5 work lives in `sequential_backtrack.cpp`, `metaheuristic_search.cpp`; `constructive_irregular.cpp` is at unit level; legacy `mtg_section_*` unchanged | No action |
| New suites bring constructive/production up to bounding-box engine-surface parity | **Largely satisfied — see F1, F2** | Both suites cover the same request-surface categories as bounding-box; two correctness gaps need fixing before parity is complete | Fix F1, F2 |

---

## Findings

### F1 — HIGH: Metaheuristic observer test has a vacuous monotonicity assertion

**File:** `tests/integration/metaheuristic_search.cpp:501-515`

```cpp
control.on_progress = [&](const ProgressSnapshot &snapshot) {
    observed.push_back(snapshot);
    if (observed.size() >= 1U) {   // always true on first callback
        cancel_source.request_stop();
    }
};
// ...
for (std::size_t index = 1; index < observed.size(); ++index) {  // never runs
    REQUIRE(observed[index].budget.iterations_completed >=
            observed[index - 1].budget.iterations_completed);
}
```

`observed.size() >= 1U` is unconditionally true after the first callback, so cancellation fires immediately. `observed` ends up with exactly 1 element, so the for-loop body **never executes**. The plan requirement "budget.iterations_completed stays monotonic within each run" passes vacuously for all five optimizer kinds.

**Fix:** Change the cancel condition to `observed.size() >= 3U` (or another value ≥ 2) so at least two snapshots are captured before cancellation. The loop will then execute at least once and the monotonicity assertion becomes meaningful.

---

### F2 — MEDIUM: Mixed placement-count metrics in constructive observer test

**File:** `tests/integration/sequential_backtrack.cpp:412, 435`

```cpp
// Snapshot counting (line 412): uses placement_trace
const std::size_t observed_placed = snapshot.layout.placement_trace.size();

// Final-result counting (line 435): uses bins[*].placements (count_total_placements)
REQUIRE(count_total_placements(solved.value()) == best_observed_layout);
```

`best_observed_layout` is derived from `placement_trace.size()` on snapshots, but the assertion compares it against `count_total_placements()`, which walks `bins[*].placements`. These two paths measure the same logical set of placements via different data structures. They should stay in sync, but using two different accessors for a single "placed count" concept is a latent inconsistency that could silently pass on a corrupt result.

Note: line 437 uses `placement_trace` on both sides (`observed.back().layout.placement_trace.size() < solved.value().layout.placement_trace.size()`), which is consistent. The inconsistency is only between lines 412 and 435.

**Fix:** Use the same accessor throughout the test — either `count_total_placements()` or `layout.placement_trace.size()`, not a mix.

---

### F3 — MEDIUM: Metaheuristic depth tests are locked to simulated_annealing

**Files:** `tests/integration/metaheuristic_search.cpp:307, 337, 367, 456, 522, 543`

All targeted depth tests (per-piece rotations, mirror, quantity > 1, allowed\_bin\_ids, maintain\_bed\_assignment, part-in-part, and concave-candidates) hardcode `ProductionOptimizerKind::simulated_annealing`. The other four optimizer families (`brkga`, `alns`, `gdrr`, `lahc`) are exercised only in:
- The combined observer/cancellation test (which only verifies cancellation fires and stop reason)
- The optimizer-breadth positive matrix (which only varies `kind` × `bed1_only`, not the depth surface)

This means `brkga`, `alns`, `gdrr`, and `lahc` are not tested for mirroring, per-piece rotation enforcement, quantity expansion, admissibility constraint enforcement, part-in-part placement, or concave-candidate placement.

**Fix (per owner feedback):** The 18-part/2-bed MTG fixture places all parts easily on every optimizer. Add `GENERATE(brkga, simulated_annealing, alns, gdrr, lahc)` to all 7 depth tests. The existing small budgets in `baseline_production_options()` + `fast_production_control()` (iteration\_limit=2) keep total run time acceptable. Part-in-part: the existing `make_hole_request(kind)` pattern (square donut + square donut hole) is already the accepted fixture; it just needs to run for all 5 kinds.

---

### F4 — LOW: "Fast-lane smoke test" comment misrepresents the priority test

**File:** `tests/integration/bounding_box_engine_surface.cpp:334`

```cpp
// Here we set the priority field on a bounding-box run as a
// fast-lane smoke test that asserts the high-priority piece is in the placed set.
```

The test that follows verifies real behavioral contracts: the high-priority piece must not appear in `unplaced_piece_ids` and must be found in at least one bin's placements. These are substantive assertions, not smoke. The comment creates unnecessary doubt about the test's quality when reviewing the suite.

**Fix:** Update the comment to name the actual contract being verified: "priority-marked pieces must appear in the placed set regardless of strategy."

---

### F5 — LOW: No grain-compatibility out-of-scope note in M5 integration files

**Files:** `tests/integration/sequential_backtrack.cpp` (header), `tests/integration/metaheuristic_search.cpp` (header)

The root plan (`plan.md:120`) states "exclude grain compatibility for now." Grain compatibility has unit-level coverage in `tests/unit/placement.cpp` and appears in masonry, search, and packing tests. Neither M5 integration file states this boundary; a future reviewer auditing the files cannot determine whether the omission is intentional or a gap.

**Fix:** Add a one-line comment near the top of each integration file, e.g.:
```cpp
// Grain compatibility is intentionally excluded from this suite (see plan.md).
```

---

### F6 — LOW: Milestone 5 wording does not explicitly exempt admissibility-conflict cases

**File:** `plan.md:99`

The checklist item reads: "Verify every non-stop/cancel and non-maintain_bed_assignment case asserts all parts placed." Both integration suites contain a `SECTION("selected beds conflicting with allowed_bin_ids keep the piece unplaced")` that correctly asserts `expected_placed_count = 1`. This section is not covered by the "stop/cancel" or "maintain_bed_assignment" exceptions, so the checklist technically fails — even though the partial-placement assertion is intentional and correct.

**Fix:** Amend the checklist item to add "and non-admissibility-conflict cases" (or equivalent phrasing).

---

### F7 — LOW: No stop_reason assertion in most metaheuristic depth tests

**File:** `tests/integration/metaheuristic_search.cpp` (multiple)

`fast_production_control()` sets `iteration_limit = 2`. Nearly all depth tests (`rotations`, `mirror`, `quantity`, `allowed_bin_ids`, `maintain_bed_assignment`, `part-in-part`, `concave-candidates`) only assert `REQUIRE(solved.has_value())` without checking `stop_reason`. The run may complete as `completed`, `iteration_limit_reached`, or potentially another reason, and the test would pass either way.

Contrast: `sequential_backtrack.cpp:293` does assert `StopReason::completed` in the quantity test.

**Fix (optional):** Assert `stop_reason != StopReason::error` (or a more specific expected value) in depth tests where the fixture is known to complete within the iteration budget.

---

---

## Known engine bugs exposed by this review

These failures were observed after the full M5 test suite was applied (all fixes F1–F6 complete,
observer test using `load_mtg_fixture()` with 18 pieces and `observer_production_options()` budget).
Both failures are in `tests/integration/metaheuristic_search.cpp`, test case
`"mtg metaheuristic-search observer and cancellation work for every optimizer"`.

### BUG-1 — Observer fires only once for at least one optimizer kind

**Assertion:** `metaheuristic_search.cpp:533` — `REQUIRE(observed.size() >= 2U)` — fails with `1 >= 2`

At least one optimizer kind (unidentified from test output alone; check the `GENERATE` sequence)
fires `on_progress` only once during a full 18-piece run with `observer_production_options()`
budget (`max_generations=4` / `max_iterations=8`). The cancel threshold of 3 is never reached,
so only 1 snapshot is captured. Either:
- the optimizer is not emitting a callback per iteration/generation, or
- it completes the entire budget in a single internal step before a second callback can fire.

**Impact:** Cancellation and monotonicity are not exercised for that kind.

**For investigation:** Add per-kind logging around `on_progress` or run the observer test in
isolation with `--reporter console` and `--verbosity high` to identify which `GENERATE` index
fails. Then inspect the optimizer's callback dispatch loop.

---

### BUG-2 — Budget iterations_completed is non-monotonic for at least one optimizer kind

**Assertion:** `metaheuristic_search.cpp:535` — monotonicity loop — fails

For at least one optimizer kind, `observed[index].budget.iterations_completed` decreases between
consecutive callbacks. A decreasing `iterations_completed` suggests either:
- the counter is reset mid-run (e.g., on restart), or
- the `ProgressSnapshot` captures a per-phase counter that resets between phases rather than the
  cumulative total.

**Impact:** Clients relying on `budget.iterations_completed` to track progress (e.g., for
progress-bar display) will see backwards jumps for this optimizer.

**For investigation:** Identify which optimizer kind triggers the failure. Inspect how
`budget.iterations_completed` is populated in its solver loop and whether restarts or
phase boundaries reset the counter.

---

## Mirror-fixture parity note

The constructive mirror test (`sequential_backtrack.cpp:248`) uses `load_mtg_fixture()` with both beds selected (default). The metaheuristic mirror test (`metaheuristic_search.cpp:335`) uses `make_asymmetric_engine_surface_fixture()` with `selected_bin_ids = {kBed1Id}`. Different fixture and bed scope. This is not a bug — the test passes on both sides — but it means mirror behavior with multiple beds is exercised only in the constructive suite.

---

## Recommended execution order

1. **Fix F3 (optimizer depth coverage)** — GENERATE all 5 optimizer kinds in all 7 depth tests; part-in-part already uses the accepted square-donut fixture.
2. **Fix F1 (vacuous monotonicity)** — change cancel condition from `>= 1U` to `>= 3U`.
3. **Fix F2 (mixed metrics)** — align both sides of the constructive observer check to use `placement_trace.size()`.
4. **Fix F5 (grain out-of-scope comment)** — low effort, prevents future confusion.
5. **Fix F4 (smoke comment wording)** — cosmetic, low effort.
6. **Fix F6 (M5 wording)** — update `plan.md` checklist language.
7. **Consider F7 (stop\_reason)** — optional; add assertions if the fixture guarantees completion within budget.
