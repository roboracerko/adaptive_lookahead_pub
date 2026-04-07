#!/usr/bin/env bash
# ==============================================================================
# run_main_benchmark.sh
# 메인 벤치마크: 5 methods × 6 maps, N=30 trials
# 논문 Table II (Success Rate) 및 Table III (Performance) 재현
#
# 사전 조건:
#   source install/setup.bash  (adaptive_lookahead_pub 워크스페이스 루트에서)
#
# 사용법:
#   bash experiments/run_main_benchmark.sh [--trials N] [--method METHOD] [--map MAP]
#
# 옵션:
#   --trials N      trial 수 (기본 30, 논문: 30)
#   --method METHOD kw_sim|alg1_org|alg1_ext|kv|curv (기본: 전체)
#   --map MAP       budapest|spielberg|icra|skir|austin|brandshatch (기본: 전체)
#   --results-root  결과 저장 경로 (기본: /tmp/f1tenth_eval/main)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BENCH_CONFIG_DIR="$WS_ROOT/src/pp_adaptive/config/benchmarks/main"
TMP_DIR="/tmp/f1tenth_eval_patched"
RESULTS_ROOT="/tmp/f1tenth_eval/main"
TRIALS=30
METHODS="kw_sim alg1_org alg1_ext kv curv"
MAPS="budapest spielberg icra skir austin brandshatch"

# --- 인수 파싱 ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --trials)    TRIALS="$2"; shift 2 ;;
        --method)    METHODS="$2"; shift 2 ;;
        --map)       MAPS="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$TMP_DIR"

# evaluation_runner.py 경로
EVAL_RUNNER="$(ros2 pkg prefix f1tenth_driver_benchmark_suite)/share/f1tenth_driver_benchmark_suite/../../../lib/f1tenth_driver_benchmark_suite/evaluation_runner.py"
# fallback: 소스 경로
if [ ! -f "$EVAL_RUNNER" ]; then
    EVAL_RUNNER="$WS_ROOT/src/f1tenth_driver_benchmark_suite/f1tenth_driver_benchmark_suite/evaluation_runner.py"
fi

echo "============================================================"
echo " F1TENTH Adaptive Lookahead — Main Benchmark"
echo " Methods : $METHODS"
echo " Maps    : $MAPS"
echo " Trials  : $TRIALS"
echo " Results : $RESULTS_ROOT"
echo "============================================================"

for METHOD in $METHODS; do
    for MAP in $MAPS; do
        YAML_NAME="${METHOD}_${MAP}_n30.yaml"
        SRC_YAML="$BENCH_CONFIG_DIR/$YAML_NAME"

        if [ ! -f "$SRC_YAML" ]; then
            echo "[SKIP] $YAML_NAME not found"
            continue
        fi

        PATCHED_YAML="$TMP_DIR/$YAML_NAME"

        # 경로 패치
        python3 "$SCRIPT_DIR/_patch_benchmarks.py" \
            --ws-root "$WS_ROOT" \
            --results-root "$RESULTS_ROOT" \
            --input "$SRC_YAML" \
            --output "$PATCHED_YAML"

        # trials_per_driver override
        python3 -c "
import yaml
with open('$PATCHED_YAML') as f: cfg=yaml.safe_load(f)
cfg['evaluation']['trials_per_driver'] = $TRIALS
with open('$PATCHED_YAML','w') as f: yaml.dump(cfg,f,allow_unicode=True)
"

        echo ""
        echo ">>> Running: $METHOD / $MAP ($TRIALS trials)"
        python3 "$EVAL_RUNNER" --config "$PATCHED_YAML"
    done
done

echo ""
echo "All done. Results in: $RESULTS_ROOT"
echo "To compute stats: python3 analysis/compute_stats.py --results $RESULTS_ROOT"
