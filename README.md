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
