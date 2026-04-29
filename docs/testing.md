# Testing

Use the grouped runner for the common lanes:

```bash
./scripts/run-tests.sh fast
./scripts/run-tests.sh --group readiness
./scripts/run-tests.sh readiness
./scripts/run-tests.sh regression
./scripts/run-tests.sh topology
./scripts/run-tests.sh sanitizer
./scripts/run-tests.sh benchmark-smoke
./scripts/run-tests.sh api-contract
./scripts/run-tests.sh geometry-core
./scripts/run-tests.sh svg-readiness
./scripts/run-tests.sh mtg-actual
./scripts/run-tests.sh limits
./scripts/run-tests.sh or-datasets-smoke
./scripts/run-tests.sh sparrow-reference
./scripts/run-tests.sh diagnostics-artifacts
./scripts/run-tests.sh ci-required
./scripts/run-tests.sh ci-premerge
./scripts/run-tests.sh all
```

The positional group is a shorthand for `--group <name>`. `fast`, `unit`,
`integration`, `regression`, `topology`, `mtg`, and `all` run in debug mode.
`readiness` and `benchmark-smoke` run in release mode and write persistent artifacts under
`artifacts/readiness/` by default. The readiness lane runs the deterministic
readiness matrix plus SVG fixture parsing checks. The benchmark-smoke lane runs
the baseline-backed readiness envelope only. The sanitizer lane keeps the NFP
and actual-polygon coverage, and adds a representative readiness artifact export
smoke test with:

```bash
./scripts/run-tests.sh --group sanitizer
```

Readiness filtering and artifact options:

```bash
./scripts/run-tests.sh --group readiness \
  --case or-strip-dataset,svg-full-irregular-set \
  --strategy brkga \
  --seed 17 \
  --repeat 2 \
  --output artifacts/readiness \
  --no-baseline-compare
```

The grouped runner fails passing test runs that emit unexpected `ERROR:` lines
or warning logs. Partial-placement warnings from negative/diagnostic tests are
allowed because those tests assert the structured unplaced-piece/result fields.

The readiness output directory contains one JSON and one SVG layout artifact per
`(case, strategy, seed)` plus `results.json`, whose records include case id,
strategy, seed, placed count, utilization, runtime, stop reason, fallback
diagnostics, validation status, validation issue count, full-success status, and
the deterministic relative layout artifact filenames.

Additional readiness-depth groups map production concerns to stable Catch2 tag
sets:

| Group | Purpose |
| --- | --- |
| `unit` | Current-surface unit lane, excluding integration, MTG, readiness, benchmark, manifest, and rotation-range lanes. |
| `integration` | Integration coverage, including MTG, public-surface manifest, and representative-layout cases. |
| `regression` | Extracted bug repros and geometry regressions. |
| `topology` | Polygon union/topology-specific tests. |
| `api-contract` | Public builder, DTO, config, control, and invalid-input contract tests. |
| `geometry-core` | Predicates, normalization, decomposition, NFP, IFP, and topology tests. |
| `svg-readiness` | SVG import readiness. |
| `mtg-actual` | MTG actual-polygon fixture and candidate-generation coverage. |
| `limits` | Focused production/search time-limit, operation-limit, and cancellation tests. |
| `or-datasets-smoke` | OR-Datasets JSON fixture import smoke tests. |
| `sparrow-reference` | Strip optimizer, separator, and disruption behavior inspired by Sparrow. |
| `diagnostics-artifacts` | Readiness artifacts plus progress-diagnostic checks. |
| `ci-required` | Recommended every-change gate: fast, geometry-core, api-contract, and limits filters in one debug build. |
| `ci-premerge` | Recommended pre-merge gate: integration, mtg-actual, readiness, and sanitizer lanes. |

The sanitizer lane covers NFP, IFP, candidate generation, actual-polygon, time,
and cancellation tags.

To refresh the checked-in readiness baselines after an intentional behavior or performance change:

```bash
./scripts/run-tests.sh --group readiness --refresh-baselines
```

The baseline file lives at `tests/readiness/baselines/readiness_matrix.json`. Each case records the minimum placed-part count, minimum utilization, runtime ceiling, fallback ceiling, expected stop reason, full-success requirement, layout-valid requirement, and validation-issue budget for the current readiness fixtures.
