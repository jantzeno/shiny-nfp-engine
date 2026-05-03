# shiny-nesting-engine

This repository is the C++23 nesting engine library in the
`export_face_workspace`.

The stable product workflow is profile-first:

- `Quick` runs the fill-first constructive baseline.
- `Balanced` runs the constructive seed plus bounded Sparrow-backed search.
- `Maximum Search` runs the deeper Sparrow-backed search profile.

Low-level `NestingRequest` and `SolveControl` entry points remain available for
engine tests and specialized integrations, but the primary downstream contract
is the compact profile surface built around `ProfileRequestBuilder`,
`SolveProfile`, and `ProfileSolveControlBuilder`.

- library target: `shiny_nesting_engine`
- primary namespace: `shiny::nesting`
- preserved baseline: `shiny::nesting::pack::BoundingBoxPacker`
- engine solve API: `shiny::nesting::solve(...)`
- milestone-2 substrate: request normalization, import/preprocess helpers,
  runtime controls, and OR-Datasets ingestion
- default constructive strategy: deterministic fill-first placement with hole
  placement, exclusion handling, allowed-bin filtering, and monotonic bin
  advancement
- search-backed profiles: bounded `Balanced` search and deeper `Maximum Search`
  over the Sparrow integration path
- downstream contract: `export_face` links against prebuilt sibling artifacts

## Build

Minimum supported toolchain for this repo after the C++23 upgrade:

- GCC 13 or newer
- Clang 17 or newer with a matching standard library that provides `std::expected`

```bash
cd shiny-nesting-engine
xmake f -m debug
xmake
xmake run shiny_nesting_engine_tests

xmake f -m release
xmake
```

Sanitizer builds remain supported:

```bash
cd shiny-nesting-engine
xmake f -m debug --sanitizer=address-undefined
xmake build shiny_nesting_engine_tests
```

The C++23 adoption rules for `std::expected`, `std::span`, ranges, and
`std::jthread` / `std::stop_token` are documented in
`docs/cpp23_usage_rules.md`.

## Example

```bash
cd shiny-nesting-engine
xmake run shiny_nesting_engine_bounding_box_example
```

## Public API helpers

### Quick profile solve

```cpp
#include "api/request_builder.hpp"
#include "solve.hpp"

using namespace shiny::nesting;

const auto request =
  api::ProfileRequestBuilder{}
    .with_profile(SolveProfile::quick)
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

  ### Balanced and Maximum Search

```cpp
#include "api/request_builder.hpp"

using namespace shiny::nesting;

  const auto request =
    api::ProfileRequestBuilder{}
      .with_profile(SolveProfile::balanced)
      .with_time_limit_ms(5 * 60 * 1000)
        .build_checked();

  const auto deeper_search =
    api::ProfileRequestBuilder{}
      .with_profile(SolveProfile::maximum_search)
      .with_time_limit_ms(30 * 60 * 1000)
      .build_checked();
```

### Cancellation and time limits

```cpp
#include "api/request_builder.hpp"
#include "api/solve_control.hpp"
#include "runtime/cancellation.hpp"

using namespace shiny::nesting;

runtime::CancellationSource cancel_source;
const auto timed =
    solve(request.value(),
      api::ProfileSolveControlBuilder{}.with_random_seed(7).build());
const auto cancelled = solve(
    request.value(),
  api::ProfileSolveControlBuilder{}
    .with_cancellation(cancel_source.token())
    .build());
```

### Stable request/response DTOs

```cpp
#include "api/dto.hpp"

using namespace shiny::nesting;

if (request.ok()) {
  const auto request_dto = api::to_dto(request.value());
  const auto solved = solve(api::to_request(request_dto),
                            api::to_solve_control(request_dto.control));
  if (solved.ok()) {
    const auto result_dto = api::to_dto(solved.value());
    const bool full_success = result_dto.summary.full_success;
  }
}
```

### Live visualizer progress snapshots

The profile solve overload reports a stable progress model with the profile id,
phase, phase detail, current layout, best-so-far layout, active bin,
per-bin summaries, placed count, utilization percent, elapsed or remaining
budget, and terminal stop reason. The engine-side progress contract now lives
in `src/runtime/progress.hpp`; `src/api/dto.hpp` remains the stable DTO bridge
for downstream consumers.

```cpp
#include "api/dto.hpp"

using namespace shiny::nesting;

api::ProfileSolveControlBuilder control_builder;
control_builder.with_progress([](const ProfileProgressSnapshot &snapshot) {
  const auto dto = api::to_dto(snapshot);
  const auto profile = dto.profile;
  const auto phase = dto.phase;
  const auto active_bin_id = dto.active_bin_id;
  const auto first_bin = dto.bin_summary.empty()
                             ? std::optional<std::uint32_t>{}
                             : std::optional<std::uint32_t>(
                                   dto.bin_summary.front().bin_id);
  const auto placed_count = dto.placed_count;
});
```

## Export Face migration

The downstream migration checklist for `export_face` lives in
`docs/export_face_migration_checklist.md`.

### Candidate strategy semantics

- `CandidateStrategy::anchor_vertex`: uses constructive anchors, skyline points, and container/exclusion-domain vertices; placement search skips obstacle NFP generation on this path.
- `CandidateStrategy::nfp_perfect`: uses feasible-region vertices from exact obstacle NFPs when available.
- `CandidateStrategy::nfp_arrangement`: extends `nfp_perfect` with blocked-boundary/intersection slide candidates.
- `CandidateStrategy::nfp_hybrid`: combines the constructive anchor set with the NFP-driven candidates above.
- Conservative bbox fallback NFPs are labeled with `cache::NfpCacheAccuracy::conservative_bbox_fallback`; fallback cache entries use separate keys and are never returned as exact NFP geometry.
