#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

METHODS="alg1_org alg1_ext kv curv"
MAPS="budapest spielberg icra skir austin brandshatch"
TRIALS=30
RESULTS_ROOT="/tmp/f1tenth_eval/report_revision"
SKIP_RACELINES=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --method) METHODS="$2"; shift 2 ;;
        --map) MAPS="$2"; shift 2 ;;
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
BENCH_DIR="${WS_ROOT}/src/pp_adaptive/config/benchmarks/main"

if [[ "${SKIP_RACELINES}" != "1" ]]; then
    bash "${SCRIPT_DIR}/generate_report_racelines.sh" --family revision $([[ "${DRY_RUN}" == "1" ]] && printf '%s' '--dry-run')
fi

echo "============================================================"
echo " Revision Report Reproduction"
echo " Methods : ${METHODS}"
echo " Maps    : ${MAPS}"
echo " Trials  : ${TRIALS}"
echo " Results : ${RESULTS_ROOT}"
echo " DryRun  : ${DRY_RUN}"
echo "============================================================"

for method in ${METHODS}; do
    for map_name in ${MAPS}; do
        yaml_path="${BENCH_DIR}/${method}_${map_name}_n30.yaml"
        if [[ ! -f "${yaml_path}" ]]; then
            echo "[SKIP] Missing config: ${yaml_path}"
            continue
        fi
        echo ">>> ${method} / ${map_name}"
        run_patched_eval "${yaml_path}" "${RESULTS_ROOT}" "${TRIALS}" "${EVAL_RUNNER}"
    done
done

echo ""
echo "Done. Results are under: ${RESULTS_ROOT}"
