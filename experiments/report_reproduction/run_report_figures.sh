#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

OUTPUT_DIR="${WS_ROOT}/results/report_figures"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

mkdir -p "${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == "1" ]]; then
    echo "[DRY] python3 ${WS_ROOT}/analysis/gen_figures.py --raceline-dir ${WS_ROOT}/src/pp_core/racelines --output-dir ${OUTPUT_DIR}"
    exit 0
fi

python3 "${WS_ROOT}/analysis/gen_figures.py" \
    --raceline-dir "${WS_ROOT}/src/pp_core/racelines" \
    --output-dir "${OUTPUT_DIR}"

echo "Done. Figures are under: ${OUTPUT_DIR}"
