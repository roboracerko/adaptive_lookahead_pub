#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

FILTER=""
TRIALS=30
RESULTS_ROOT="/tmp/f1tenth_eval/report_ablation"
SKIP_RACELINES=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --filter) FILTER="$2"; shift 2 ;;
        --trials) TRIALS="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        --skip-racelines) SKIP_RACELINES=1; shift ;;
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
BENCH_DIR="${WS_ROOT}/src/pp_adaptive/config/benchmarks/ablation"

if [[ "${SKIP_RACELINES}" != "1" ]]; then
    bash "${SCRIPT_DIR}/generate_report_racelines.sh" --family ablation $([[ "${DRY_RUN}" == "1" ]] && printf '%s' '--dry-run')
fi

echo "============================================================"
echo " Ablation Report Reproduction"
echo " Filter  : ${FILTER:-all}"
echo " Trials  : ${TRIALS}"
echo " Results : ${RESULTS_ROOT}"
echo " DryRun  : ${DRY_RUN}"
echo "============================================================"

shopt -s nullglob
for yaml_path in "${BENCH_DIR}"/*_n30.yaml; do
    base_name="$(basename "${yaml_path}")"
    if [[ -n "${FILTER}" && "${base_name}" != *"${FILTER}"* ]]; then
        continue
    fi
    echo ">>> ${base_name}"
    run_patched_eval "${yaml_path}" "${RESULTS_ROOT}" "${TRIALS}" "${EVAL_RUNNER}"
done
shopt -u nullglob

echo ""
echo "Done. Results are under: ${RESULTS_ROOT}"
