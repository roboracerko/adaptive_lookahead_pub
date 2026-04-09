#!/usr/bin/env bash
# ==============================================================================
# run_sensitivity.sh
# 파라미터 민감도 스윕: Budapest 맵, N=10 trials
# 논문 Fig. 6 (Sensitivity Analysis) 재현
#
# 파라미터: a_lat_max, alpha, t_margin, w_cte, kappa_window
# 각 파라미터를 5~7 값으로 스윕 (총 ~290 trials)
#
# 사용법:
#   bash experiments/run_sensitivity.sh [--param PARAM]
#
# 옵션:
#   --param PARAM   alat|alpha|tmargin|wcte|kw (기본: 전체)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BENCH_CONFIG_DIR="$WS_ROOT/src/pp_adaptive/config/benchmarks/sensitivity"
TMP_DIR="/tmp/f1tenth_eval_patched"
RESULTS_ROOT="/tmp/f1tenth_eval/sensitivity"
PARAM_FILTER=""
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --param) PARAM_FILTER="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$TMP_DIR"
EVAL_RUNNER="$WS_ROOT/src/f1tenth_driver_benchmark_suite/f1tenth_driver_benchmark_suite/evaluation_runner.py"

echo "============================================================"
echo " F1TENTH Adaptive Lookahead — Sensitivity Sweep (Budapest)"
echo " Param   : ${PARAM_FILTER:-all}"
echo " Results : $RESULTS_ROOT"
echo " DryRun  : $DRY_RUN"
echo "============================================================"

for YAML in "$BENCH_CONFIG_DIR"/bench_sensitivity_*.yaml; do
    YAML_NAME="$(basename $YAML)"

    # 파라미터 필터
    if [ -n "$PARAM_FILTER" ] && [[ "$YAML_NAME" != *"$PARAM_FILTER"* ]]; then
        continue
    fi

    PATCHED="$TMP_DIR/sens_$YAML_NAME"

    python3 "$SCRIPT_DIR/_patch_benchmarks.py" \
        --ws-root "$WS_ROOT" \
        --results-root "$RESULTS_ROOT" \
        --input "$YAML" \
        --output "$PATCHED"

    echo ">>> $YAML_NAME"
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "[DRY] python3 $EVAL_RUNNER --config $PATCHED"
    else
        python3 "$EVAL_RUNNER" --config "$PATCHED"
    fi
done

echo "Done. Results: $RESULTS_ROOT"
echo "To plot: python3 analysis/gen_figures.py --mode sensitivity --results $RESULTS_ROOT"
