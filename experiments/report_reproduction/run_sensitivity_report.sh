#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

PARAM=""
RESULTS_ROOT="/tmp/f1tenth_eval/report_sensitivity"
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --param) PARAM="$2"; shift 2 ;;
        --results-root) RESULTS_ROOT="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

load_workspace

echo "============================================================"
echo " Sensitivity Report Reproduction"
echo " Param   : ${PARAM:-all}"
echo " Results : ${RESULTS_ROOT}"
echo " DryRun  : ${DRY_RUN}"
echo "============================================================"

if [[ -n "${PARAM}" ]]; then
    bash "${WS_ROOT}/experiments/run_sensitivity.sh" --param "${PARAM}" --results-root "${RESULTS_ROOT}" $([[ "${DRY_RUN}" == "1" ]] && printf '%s' '--dry-run')
else
    bash "${WS_ROOT}/experiments/run_sensitivity.sh" --results-root "${RESULTS_ROOT}" $([[ "${DRY_RUN}" == "1" ]] && printf '%s' '--dry-run')
fi

if [[ "${DRY_RUN}" == "1" ]]; then
    echo "[DRY] summary generation skipped"
    echo ""
    echo "Done. Results would be written under: ${RESULTS_ROOT}"
    exit 0
fi

python3 - "${RESULTS_ROOT}" <<'PY'
import csv
import json
import sys
from pathlib import Path

import pandas as pd

results_root = Path(sys.argv[1])
rows = []

for eval_dir in sorted(results_root.iterdir() if results_root.exists() else []):
    if not eval_dir.is_dir() or not eval_dir.name.startswith("sensitivity_"):
        continue

    remainder = eval_dir.name[len("sensitivity_"):]
    param, _, value_token = remainder.partition("_")
    value = value_token.replace("_", ".")

    driver_dir = eval_dir / "pp_adaptive"
    total = 0
    successes = 0
    ctes = []
    jerks = []

    if driver_dir.exists():
        for trial_dir in sorted(driver_dir.iterdir()):
            run_dir = trial_dir / "runs"
            run_files = sorted(run_dir.glob("run_*.json")) if run_dir.exists() else []
            if not run_files:
                continue

            total += 1
            run_data = json.loads(run_files[0].read_text())
            if run_data.get("status") == "success" or run_data.get("laps_completed", 0) >= 3:
                successes += 1

            lap_csv = trial_dir / "all_laps_summary.csv"
            if lap_csv.exists():
                lap_df = pd.read_csv(lap_csv)
                if "mean_cte" in lap_df.columns:
                    ctes.extend(lap_df["mean_cte"].dropna().tolist())
                if "mean_jerk" in lap_df.columns:
                    jerks.extend(lap_df["mean_jerk"].dropna().tolist())

    completion = 100.0 * successes / total if total else 0.0
    rows.append({
        "param": param,
        "value": value,
        "completion_rate": round(completion, 1),
        "lat_err_mean": round(sum(ctes) / len(ctes), 4) if ctes else "",
        "steer_jerk_mean": round(sum(jerks) / len(jerks), 4) if jerks else "",
        "n_trials": total,
    })

summary_path = results_root / "SUMMARY.csv"
with summary_path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=["param", "value", "completion_rate", "lat_err_mean", "steer_jerk_mean", "n_trials"],
    )
    writer.writeheader()
    writer.writerows(rows)

print(f"Saved summary: {summary_path}")
PY

echo ""
echo "Done. Results are under: ${RESULTS_ROOT}"
