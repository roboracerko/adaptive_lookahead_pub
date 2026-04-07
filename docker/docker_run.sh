#!/usr/bin/env bash
# Docker 컨테이너 실행 (GPU 없음, headless 시뮬레이션)
IMAGE_NAME="adaptive_lookahead_pub"
RESULTS_DIR="${1:-/tmp/f1tenth_eval}"

docker run -it --rm \
    --network host \
    --ipc host \
    -v "$RESULTS_DIR":/results \
    -e RESULTS_ROOT=/results \
    "$IMAGE_NAME" \
    /bin/bash -c "
        source /workspace/install/setup.bash
        bash /workspace/experiments/run_main_benchmark.sh --results-root /results/main
    "
