# shiny-nesting-engine

This repository is the renamed C++20 nesting engine library in the
`export_face_workspace`.

Milestone 4 extends the engine-level solve surface with a deterministic
production optimizer for irregular nesting. The production path uses the
constructive irregular solver as its seed decoder, then improves piece order
with a BRKGA-style search loop plus a lightweight Sparrow-style polish pass.

- library target: `shiny_nesting_engine`
- primary namespace: `shiny::nesting`
- preserved baseline: `shiny::nesting::pack::BoundingBoxPacker`
- engine solve API: `shiny::nesting::solve(...)`
- milestone-2 substrate: request normalization, import/preprocess helpers,
  runtime controls, and OR-Datasets ingestion
- milestone-3 strategy: deterministic irregular constructive placement with
  hole placement, exclusion handling, allowed-bin filtering, and live progress
  snapshots
- milestone-4 strategy: BRKGA-backed irregular production optimization with
  replayable best-so-far progress and Sparrow-style polishing
- downstream contract: `export_face` links against prebuilt sibling artifacts

## Build

```bash
cd shiny-nesting-engine
xmake f -m debug
xmake
xmake run shiny_nesting_engine_tests
```

## Example

```bash
cd shiny-nesting-engine
xmake run shiny_nesting_engine_bounding_box_example
```

## Public API helpers

### Simple solve

```cpp
#include "api/request_builder.hpp"
#include "solve.hpp"

using namespace shiny::nesting;

const auto request =
    api::NestingRequestBuilder{}
        .with_strategy(StrategyKind::sequential_backtrack)
        .with_default_rotations(geom::DiscreteRotationSet{{0.0, 90.0}})
        .add_bin(BinRequest{
            .bin_id = 1,
            .polygon = {.outer = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}},
        })
        .add_piece(PieceRequest{
            .piece_id = 7,
            .polygon = {.outer = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 2.0}, {0.0, 2.0}}},
        })
        .build_checked();

if (request.ok()) {
  const auto solved = solve(request.value());
  if (solved.ok() && solved.value().is_full_success()) {
    const auto summary = solved.value().summary();
  }
}
```

### BRKGA solve with typed production config

```cpp
#include "api/request_builder.hpp"

using namespace shiny::nesting;

const auto request =
    api::NestingRequestBuilder{}
        .with_strategy(StrategyKind::metaheuristic_search)
        .with_production_optimizer(ProductionOptimizerKind::brkga)
        .with_production_config(ProductionOptimizerKind::brkga,
                                ProductionSearchConfig{
                                    .population_size = 24,
                                    .elite_count = 6,
                                    .mutant_count = 4,
                                    .max_iterations = 24,
                                })
        .build_checked();
```

### Cancellation and time limits

```cpp
#include "api/request_builder.hpp"
#include "runtime/cancellation.hpp"

using namespace shiny::nesting;

runtime::CancellationSource cancel_source;
const auto timed =
    solve(request.value(),
          api::SolveControlBuilder{}.with_time_limit_ms(250).build());
const auto cancelled = solve(
    request.value(),
    api::SolveControlBuilder{}.with_cancellation(cancel_source.token()).build());
```

### Stable request/response DTOs

```cpp
#include "api/dto.hpp"

using namespace shiny::nesting;

if (request.ok()) {
  const auto request_dto = api::to_dto(request.value());
  const auto solved = solve(api::to_request(request_dto));
  if (solved.ok()) {
    const auto result_dto = api::to_dto(solved.value());
    const bool full_success = result_dto.summary.full_success;
  }
}
```

### Candidate strategy semantics

- `CandidateStrategy::anchor_vertex`: uses constructive anchors, skyline points, and container/exclusion-domain vertices; placement search skips obstacle NFP generation on this path.
- `CandidateStrategy::nfp_perfect`: uses feasible-region vertices from exact obstacle NFPs when available.
- `CandidateStrategy::nfp_arrangement`: extends `nfp_perfect` with blocked-boundary/intersection slide candidates.
- `CandidateStrategy::nfp_hybrid`: combines the constructive anchor set with the NFP-driven candidates above.
- Conservative bbox fallback NFPs are labeled with `cache::NfpCacheAccuracy::conservative_bbox_fallback`; fallback cache entries use separate keys and are never returned as exact NFP geometry.
