# Regression investigation: multi-start observer coverage

## Problem

`export_face_console.log` shows an `sequential_backtrack` multi-start run (`random_seed=99`, both beds selected, no iteration/time limit) emitting many observer callbacks with empty layouts during candidate search, then finishing `cancelled` after a later iteration that places fewer parts than the final `NestingResult` still reports. Current engine tests do not cover this path: the fast MTG baseline disables multi-start with `random_seed=0`, Section D checks only final placement counts, and the existing engine-surface coverage is too generic and too bounding-box-biased.

## Milestones

### Milestone 0 — Restructure the test layout and naming

- [x] Move away from `section_x` naming for the touched engine-surface work.
- [x] Define descriptive integration file names before adding new depth:
  - [x] `tests/integration/sequential_backtrack.cpp`
  - [x] `tests/integration/metaheuristic_search.cpp`
  - [x] keep bounding-box surface work in `tests/integration/bounding_box_engine_surface.cpp` rather than a generic section file
- [x] Keep deep constructive and production coverage in separate files.
- [x] Leave shared helpers in `tests/support/mtg_fixture.*` instead of mixing unrelated algorithm detail in one integration file.
- [x] Migrate or supersede the touched `section_x` coverage only as needed to keep ownership clear and naming descriptive.

### Milestone 1 — Lock the solver contract

- [x] Add solver-level coverage in `tests/unit/constructive_irregular.cpp`.
- [x] Add a multi-start test with non-zero seed and observer capture.
- [x] Assert empty-layout search pulses can occur during constructive search.
- [x] Assert the final `solve()` result retains the best full layout even if a later iteration finishes worse before cancellation or stop.
- [x] For the regression path, use `random_seed = 99`, both beds selected, and observer-driven cancellation after multiple multi-start iterations.

### Milestone 2 — Build the sequential-backtrack surface suite

- [x] Create `tests/integration/sequential_backtrack.cpp`.
- [x] Keep the file semi-contained: constructive-only depth, no unrelated production behavior mixed in.
- [x] Add a breadth/depth positive matrix over:
  - [x] selected part beds `{all beds, bed1 only}`
  - [x] part spacing `{0.0 mm, 1.0 mm}`
  - [x] default rotations `{0, 90, 180, 270}`
  - [x] candidate generation `{anchor_vertex, nfp_perfect, nfp_arrangement, nfp_hybrid}`
  - [x] piece ordering `{input, largest_area_first, hull_diameter_first, priority}`
  - [x] overlap check `{direct_overlap=false, direct_overlap=true}`
  - [x] backtracking `{false, true}`
  - [x] compaction `{false, true}`
- [x] Use a fixture/configuration that can genuinely place all parts in both bed-selection cells; if the broad MTG fixture cannot do that for `bed1 only`, use a focused fixture that still exercises the same request surface.
- [x] Require every positive-matrix case to place **all parts**.
- [x] Add targeted constructive depth tests for:
  - [x] per-piece `allowed_rotations`
  - [x] `allow_mirror`
  - [x] `quantity > 1`
  - [x] `allowed_bin_ids` admissibility / conflict behavior
  - [x] `maintain_bed_assignment`
  - [x] `ProgressObserver` multi-start behavior
  - [x] `CancellationToken` across multi-start
  - [x] `enable_part_in_part_placement`
  - [x] `explore_concave_candidates`
- [x] In the observer/cancellation tests, assert:
  - [x] at least one empty-layout search pulse is observed
  - [x] at least one full-layout snapshot is observed
  - [x] the returned result preserves the best full layout rather than the last callback
  - [x] cancellation reports `StopReason::cancelled` and `budget.cancellation_requested`

### Milestone 3 — Build the metaheuristic-search surface suite

- [x] Create `tests/integration/metaheuristic_search.cpp`.
- [x] Keep the file semi-contained: production-only depth, no deep constructive behavior mixed in.
- [x] Add a breadth/depth positive matrix over:
  - [x] selected part beds `{all beds, bed1 only}`
  - [x] part spacing `{0.0 mm, 1.0 mm}`
  - [x] default rotations `{0, 90, 180, 270}`
  - [x] candidate generation `{anchor_vertex, nfp_perfect, nfp_arrangement, nfp_hybrid}`
  - [x] piece ordering `{input, largest_area_first, hull_diameter_first, priority}`
  - [x] overlap check `{direct_overlap=false, direct_overlap=true}`
  - [x] backtracking `{false, true}`
  - [x] compaction `{false, true}`
  - [x] optimizer breadth is covered by a dedicated hidden test over `{brkga, simulated_annealing, alns, gdrr, lahc}` rather than a full cartesian cross-product to keep the suite runnable in debug builds
- [x] Size the budgets/configuration for completeness rather than smoke coverage.
- [x] Require every positive-matrix case to place **all parts**.
- [x] Add targeted production depth tests for:
  - [x] per-piece `allowed_rotations`
  - [x] `allow_mirror`
  - [x] `quantity > 1`
  - [x] `allowed_bin_ids` admissibility / conflict behavior
  - [x] `maintain_bed_assignment`
  - [x] `ProgressObserver` for each optimizer family
  - [x] `CancellationToken` for each optimizer family
  - [x] `enable_part_in_part_placement`
  - [x] `explore_concave_candidates`
- [x] In the observer/cancellation tests, assert:
  - [x] observer callbacks are emitted for every optimizer under test
  - [x] `budget.iterations_completed` stays monotonic within each run
  - [x] cancellation stops cleanly with `StopReason::cancelled`

### Milestone 4 — Add a hidden backstop only if still necessary

- [x] Evaluate whether a hidden repro in `tests/integration/mtg_engine_bug_repros.cpp` is still necessary.
- [x] Keep the contract in the named constructive/production suites and avoid duplicate coverage.

### Milestone 5 — Quality, completeness, robustness review

- [ ] Verify the constructive suite covers the full requested breadth/depth matrix.
- [ ] Verify the production suite covers the same surface categories plus optimizer breadth.
- [ ] Verify every non-`stop/cancel`, non-`maintain_bed_assignment`, and non-admissibility-conflict case asserts **all parts placed**.
- [ ] Verify no smoke-only cases remain.
- [ ] Verify grain compatibility is still excluded and called out as TBD.
- [ ] Verify naming stays descriptive and deep algorithm tests are not added under generic `section_x` files.
- [ ] Verify the new suites genuinely bring sequential backtrack and metaheuristic search up to bounding-box parity for the requested engine-surface contracts.

## Notes

- Relevant existing coverage:
  - `tests/integration/mtg_test_nesting_matrix.cpp`: baseline sequential backtrack uses `random_seed = 0`, so it never enters multi-start.
  - `tests/integration/mtg_section_d_sequential_backtrack.cpp`: broad option matrix validates final layouts only.
  - `tests/integration/mtg_section_k_engine_surface.cpp`: existing engine-surface checks are too generic and too bounding-box-oriented for this regression.
- Naming / containment guidance:
  - Milestone 0 explicitly moves the touched work away from `section_x` filenames
  - keep deep constructive and production contracts in separate, descriptively named files
  - share helpers through `tests/support/mtg_fixture.*`, not by co-locating unrelated algorithm details in one integration file
- Additional scope guidance:
  - include part-bed selections for both `all beds` and `bed1 only`
  - include spacing at both `0.0 mm` and `1.0 mm`
  - use four-rotation coverage (`0/90/180/270`)
  - include candidate generation, piece ordering, overlap-check, backtracking, and compaction coverage
  - exclude grain compatibility for now
  - except for explicit `stop/cancel` and `maintain_bed_assignment` tests, require every case to place all parts
  - prefer breadth/depth matrices over shallow smoke checks
