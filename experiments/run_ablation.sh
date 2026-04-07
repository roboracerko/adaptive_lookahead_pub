#!/usr/bin/env bash
# ==============================================================================
# run_ablation.sh
# Ablation 실험: kw_sim vs noA/noB/noC/noD variants
# 논문 Table IV (Ablation Study) 재현
#
# 사용법:
#   bash experiments/run_ablation.sh [--trials N]
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BENCH_CONFIG_DIR="$WS_ROOT/src/pp_adaptive/config/benchmarks/ablation"
TMP_DIR="/tmp/f1tenth_eval_patched"
RESULTS_ROOT="/tmp/f1tenth_eval/ablation"
TRIALS=30

while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials) TRIALS="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$TMP_DIR"

EVAL_RUNNER="$WS_ROOT/src/f1tenth_driver_benchmark_suite/f1tenth_driver_benchmark_suite/evaluation_runner.py"

echo "============================================================"
echo " F1TENTH Adaptive Lookahead — Ablation Study"
echo " Trials  : $TRIALS"
echo " Results : $RESULTS_ROOT"
echo "============================================================"

for YAML in "$BENCH_CONFIG_DIR"/*.yaml; do
    YAML_NAME="$(basename $YAML)"
    PATCHED="$TMP_DIR/ablation_$YAML_NAME"

    python3 "$SCRIPT_DIR/_patch_benchmarks.py" \
        --ws-root "$WS_ROOT" \
        --results-root "$RESULTS_ROOT" \
        --input "$YAML" \
        --output "$PATCHED"

    python3 -c "
import yaml
with open('$PATCHED') as f: cfg=yaml.safe_load(f)
cfg['evaluation']['trials_per_driver'] = $TRIALS
with open('$PATCHED','w') as f: yaml.dump(cfg,f,allow_unicode=True)
"
    echo ">>> $YAML_NAME"
    python3 "$EVAL_RUNNER" --config "$PATCHED"
done

echo "Done. Results: $RESULTS_ROOT"
