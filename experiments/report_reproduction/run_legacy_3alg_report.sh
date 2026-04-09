#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

METHODS="pp_fixed kw2 alg1_org"
MAP_FILTER=""
TRIALS=30
RESULTS_ROOT="/tmp/f1tenth_eval/report_main_3alg"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --method) METHODS="$2"; shift 2 ;;
        --map) MAP_FILTER="$2"; shift 2 ;;
        --trials) TRIALS="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

export REPORT_DRY_RUN="${DRY_RUN}"
load_workspace
EVAL_RUNNER="$(resolve_eval_runner)"
TEST_DIR="${WS_ROOT}/src/f1tenth_driver_benchmark_suite/config/tests"

echo "============================================================"
echo " Legacy 3-Algorithm Report Reproduction"
echo " Methods : ${METHODS}"
echo " Map     : ${MAP_FILTER:-all}"
echo " Trials  : ${TRIALS}"
echo " Results : ${RESULTS_ROOT}"
echo " DryRun  : ${DRY_RUN}"
echo "============================================================"

for method in ${METHODS}; do
    shopt -s nullglob
    for yaml_path in "${TEST_DIR}/${method}"*_n30.yaml; do
        base_name="$(basename "${yaml_path}")"
        if ! matches_map_filter "${base_name}" "${MAP_FILTER}"; then
            continue
        fi
        echo ">>> ${base_name}"
        run_patched_eval "${yaml_path}" "${RESULTS_ROOT}" "${TRIALS}" "${EVAL_RUNNER}"
    done
    shopt -u nullglob
done

echo ""
echo "Done. Results are under: ${RESULTS_ROOT}"
