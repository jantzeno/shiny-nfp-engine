# Copilot instructions for `shiny-nfp-engine`

This repository is the implementation repo for a greenfield C++20 no-fit-polygon packing library. Planning documents are already migrated under `docs/planning/` and `docs/adr/`. Future Copilot sessions should start from those documents before making code or repository-structure decisions, and should treat them as the source of truth for both the current milestone and the intended end-state of the project.

## Required reading before making changes

- `docs/planning/01-implementation-roadmap.md`
- `docs/planning/02-milestone-checklists.md`
- `docs/planning/03-repository-blueprint.md`
- `docs/planning/04-api-contract.md`
- `docs/planning/05-fixture-contract.md`
- `docs/adr/0001-geometry-model.md`
- `docs/adr/0004-build-and-dependencies.md`
- `docs/adr/0005-api-boundaries-and-cache-design.md`
- `docs/adr/0006-configuration-policy-and-defaults.md`
- `docs/adr/0007-algorithm-identity-and-observer-model.md`
- `docs/adr/0008-manufacturing-constraint-model.md`
- `docs/adr/0009-worker-owned-execution-and-parallel-evaluation.md`

## Execution rules

- treat this as a greenfield C++20 library
- do not preserve provisional APIs with compatibility wrappers
- bootstrap only the repository skeleton and build/test scaffolding first
- keep backend types out of public headers
- do not vendor Boost or CGAL yet unless that step is explicitly requested
- keep changes minimal, coherent, and aligned with the planning documents

## Project deliverables

End-state deliverables for the project:

- a C++20 library aligned with `docs/planning/03-repository-blueprint.md` and the ADR set
- a public geometry kernel for polygon normalization, topology queries, and convex/nonconvex NFP or IFP generation
- a packing stack above the geometry layer covering placement, constructive decoding, and lightweight local search
- an `xmake`-based build, test, and dependency setup aligned with the planning documents
- fixture-driven tests, topology regressions, and benchmark scaffolding added in milestone order
- repository structure, public headers, and module boundaries that follow the planned architecture and keep backend types out of public APIs

Immediate deliverables for the current implementation phase:

- create the repository skeleton described for Milestone 0
- keep the root `xmake.lua` aligned with the active planning set
- add stub source and test files so the library and test targets build
- keep `vendor/README.md` aligned with the active planning set
- add one Catch2 smoke test
- keep `docs/` as-is unless a link or reference must be updated to match the scaffold

## Acceptance target

- each change should advance the repository toward the planned end-state described in `docs/planning/` and `docs/adr/`, not just satisfy a local patch goal
- milestone work should satisfy the acceptance criteria defined for that milestone before advancing to the next layer
- the overall acceptance target is a coherent, buildable, testable C++20 library that matches the planned architecture, module boundaries, and phased deliverables
