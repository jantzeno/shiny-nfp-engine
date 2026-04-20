# Copilot instructions for `shiny-nesting-engine`

This repository is the renamed engine library for the irregular nesting rewrite. Treat the checked-in code and active milestone plan as the source of truth; do not rely on missing `docs/` trees or stale ADR references.

## Milestone-0 contract

- repository name: `shiny-nesting-engine`
- primary static library target: `shiny_nesting_engine`
- primary C++ namespace: `shiny::nesting`
- consuming apps link through public headers plus prebuilt sibling artifacts only
- use MIT-only source ports/adaptations from `u-nesting` and `sparrow`
- treat `2DNesting` as reference-only unless GPL use is explicitly approved

## Execution rules

- treat this as a greenfield C++20 library
- do not preserve provisional APIs with compatibility wrappers
- prefer `namespace detail` over anonymous namespaces
- keep code modular; do not create monolithic algorithm classes or giant single-file implementations
- keep backend-only types out of public headers where practical
- keep changes minimal, coherent, and aligned with the active milestone

## Build guidance

- configure with `xmake f -m debug` or `xmake f -m release`
- build with `xmake`
- run tests with `xmake run shiny_nesting_engine_tests`
- use `SHINY_NESTING_SANITIZER` or `xmake f --sanitizer=...` for sanitizer-enabled builds

## Integration boundary

- `export_face` consumes this repo through public headers and prebuilt library artifacts
- do not move engine implementation sources into `export_face`
- if integration needs new API surface, add it here rather than copying logic downstream
