#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
group="all"
case_filter=""
strategy_filter=""
seed=""
repeat=""
output=""
no_baseline_compare=0
refresh_baselines=0

cd "$repo_root"

usage() {
  cat >&2 <<'EOF'
usage: scripts/run-tests.sh [group] [options]
       scripts/run-tests.sh --group <fast|unit|integration|regression|topology|mtg|readiness|sanitizer|benchmark-smoke|api-contract|geometry-core|svg-readiness|mtg-actual|limits|or-datasets-smoke|sparrow-reference|diagnostics-artifacts|ci-required|ci-premerge|all> [options]

options:
  --case <id[,id...]>          readiness case filter
  --strategy <name[,name...]>  readiness strategy filter
  --seed <n>                  readiness base seed
  --repeat <n>                readiness repeats, incrementing seed
  --output <dir>              readiness artifact directory
  --no-baseline-compare       skip readiness baseline comparison
  --refresh-baselines         refresh committed readiness baselines
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --group)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      group="$2"
      shift 2
      ;;
    --case)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      case_filter="$2"
      shift 2
      ;;
    --strategy)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      strategy_filter="$2"
      shift 2
      ;;
    --seed)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      seed="$2"
      shift 2
      ;;
    --repeat)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      repeat="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || { usage; exit 2; }
      if [[ "$2" = /* ]]; then
        output="$2"
      else
        output="$repo_root/$2"
      fi
      shift 2
      ;;
    --no-baseline-compare)
      no_baseline_compare=1
      shift
      ;;
    --refresh-baselines)
      refresh_baselines=1
      shift
      ;;
    fast|unit|integration|regression|topology|mtg|readiness|sanitizer|benchmark-smoke|api-contract|geometry-core|svg-readiness|mtg-actual|limits|or-datasets-smoke|sparrow-reference|diagnostics-artifacts|ci-required|ci-premerge|all)
      group="$1"
      shift
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

export_readiness_options() {
  [[ -n "$output" ]] || output="$repo_root/artifacts/readiness"
  [[ -z "$case_filter" ]] || export SHINY_NESTING_ENGINE_READINESS_CASES="$case_filter"
  [[ -z "$strategy_filter" ]] || export SHINY_NESTING_ENGINE_READINESS_STRATEGIES="$strategy_filter"
  [[ -z "$seed" ]] || export SHINY_NESTING_ENGINE_READINESS_SEED="$seed"
  [[ -z "$repeat" ]] || export SHINY_NESTING_ENGINE_READINESS_REPEAT="$repeat"
  [[ -z "$output" ]] || export SHINY_NESTING_ENGINE_READINESS_OUTPUT="$output"
  [[ "$no_baseline_compare" -eq 0 ]] || export SHINY_NESTING_ENGINE_NO_BASELINE_COMPARE=1
  [[ "$refresh_baselines" -eq 0 ]] || export SHINY_NESTING_ENGINE_REFRESH_BASELINES=1
}

configure_mode() {
  local mode="$1"
  xmake f -m "$mode"
}

build_tests() {
  xmake build shiny_nesting_engine_tests
}

run_catch() {
  local output_file
  output_file="$(mktemp)"
  trap 'rm -f "$output_file"' RETURN

  if [[ $# -eq 0 ]]; then
    SHINY_NESTING_LOG_LEVEL=info xmake run shiny_nesting_engine_tests 2>&1 | tee "$output_file"
  else
    SHINY_NESTING_LOG_LEVEL=info xmake run shiny_nesting_engine_tests "$1" 2>&1 | tee "$output_file"
  fi
  local status=${PIPESTATUS[0]}
  if [[ "$status" -ne 0 ]]; then
    return "$status"
  fi
  if grep -q 'ERROR:' "$output_file"; then
    echo "unexpected ERROR: lines in passing test output" >&2
    grep 'ERROR:' "$output_file" >&2
    return 1
  fi
  if grep '\[warning\]' "$output_file" \
      | grep -v 'solve: completed with unplaced parts' \
      | grep -v 'solve: unresolved strategy=' >/dev/null; then
    echo "unexpected warning lines in passing test output" >&2
    grep '\[warning\]' "$output_file" \
      | grep -v 'solve: completed with unplaced parts' \
      | grep -v 'solve: unresolved strategy=' >&2
    return 1
  fi
}

case "$group" in
  fast)
    configure_mode debug
    build_tests
    run_catch "~[mtg]~[manifest]~[slow]~[rotation-range]~[benchmark]~[benchmark-smoke]~[readiness]"
    ;;
  unit)
    configure_mode debug
    build_tests
    run_catch "~[integration]~[mtg]~[manifest]~[rotation-range]~[readiness]~[readiness-sanitizer]~[benchmark]~[benchmark-smoke]"
    ;;
  integration)
    configure_mode debug
    build_tests
    run_catch "[integration],[mtg],[manifest]"
    ;;
  regression)
    configure_mode debug
    build_tests
    run_catch "[regression]"
    ;;
  topology)
    configure_mode debug
    build_tests
    run_catch "[topology]"
    ;;
  mtg)
    configure_mode debug
    build_tests
    run_catch "[mtg]"
    ;;
  readiness)
    export_readiness_options
    configure_mode release
    build_tests
    run_catch "[readiness]"
    ;;
  sanitizer)
    xmake f -m debug --sanitizer=address-undefined
    xmake build shiny_nesting_engine_tests
    run_catch "[readiness-sanitizer],[nfp],[ifp],[packing][candidate-generation],[actual-polygons],[time],[cancellation]"
    ;;
  benchmark-smoke)
    export_readiness_options
    configure_mode release
    build_tests
    run_catch "[benchmark-smoke]"
    ;;
  api-contract)
    configure_mode debug
    build_tests
    run_catch "[api]"
    ;;
  geometry-core)
    configure_mode debug
    build_tests
    run_catch "[geometry],[predicates],[polygon-ops],[decomposition],[nfp],[ifp],[topology]"
    ;;
  svg-readiness)
    configure_mode debug
    build_tests
    run_catch "[io][svg]"
    ;;
  mtg-actual)
    configure_mode debug
    build_tests
    run_catch "[packing][candidate-generation][actual-polygons],[mtg][nesting-matrix][fixture][actual-polygons]"
    ;;
  limits)
    configure_mode debug
    build_tests
    run_catch "[solve][production][limits],[solve][production][time],[solve][production][cancellation],[search][observer][cancellation]"
    ;;
  or-datasets-smoke)
    configure_mode debug
    build_tests
    run_catch "[io][or-datasets]"
    ;;
  sparrow-reference)
    configure_mode debug
    build_tests
    run_catch "[search][strip-optimizer],[search][disruption],[search][separator]"
    ;;
  diagnostics-artifacts)
    export_readiness_options
    configure_mode release
    build_tests
    run_catch "[readiness],[progress-diagnostic]"
    ;;
  ci-required)
    configure_mode debug
    build_tests
    run_catch "~[mtg]~[manifest]~[slow]~[rotation-range]~[benchmark]~[benchmark-smoke]~[readiness]"
    run_catch "[geometry],[predicates],[polygon-ops],[decomposition],[nfp],[ifp],[topology]"
    run_catch "[api]"
    run_catch "[solve][production][limits],[solve][production][time],[solve][production][cancellation],[search][observer][cancellation]"
    ;;
  ci-premerge)
    "$0" integration
    "$0" mtg-actual
    "$0" readiness
    "$0" sanitizer
    ;;
  all)
    configure_mode debug
    build_tests
    run_catch "~[benchmark]~[benchmark-smoke]~[readiness]"
    ;;
  *)
    usage
    exit 2
    ;;
esac
