#!/usr/bin/env python3
"""Static audit for report-to-script reproducibility coverage."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys


@dataclass
class Check:
    label: str
    ok: bool
    detail: str


def count_files(path: Path, pattern: str) -> int:
    return len(list(path.glob(pattern))) if path.exists() else 0


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    ws_root = script_dir.parent.parent
    report_root = ws_root.parent / "sim_ws" / "src" / "pp_adaptive" / "report"

    checks: list[Check] = []

    revision_dirs = count_files(report_root / "revision", "*/*")
    revision_cfgs = count_files(ws_root / "src" / "pp_adaptive" / "config" / "benchmarks" / "main", "*_n30.yaml")
    checks.append(
        Check(
            "revision",
            (script_dir / "run_revision_report.sh").exists() and revision_dirs == 24 and revision_cfgs >= 24,
            f"report dirs={revision_dirs}, benchmark yamls={revision_cfgs}",
        )
    )

    legacy_cfgs = (
        count_files(ws_root / "src" / "f1tenth_driver_benchmark_suite" / "config" / "tests", "pp_fixed_*_n30.yaml")
        + count_files(ws_root / "src" / "f1tenth_driver_benchmark_suite" / "config" / "tests", "kw2_*_n30.yaml")
        + count_files(ws_root / "src" / "f1tenth_driver_benchmark_suite" / "config" / "tests", "alg1_org_*_n30.yaml")
    )
    checks.append(
        Check(
            "legacy_3alg",
            (script_dir / "run_legacy_3alg_report.sh").exists() and legacy_cfgs == 18,
            f"test yamls={legacy_cfgs}",
        )
    )

    ablation_cfgs = count_files(ws_root / "src" / "pp_adaptive" / "config" / "benchmarks" / "ablation", "*_n30.yaml")
    ablation_ctrl = count_files(
        ws_root / "src" / "pp_adaptive" / "config" / "benchmarks" / "ablation",
        "pure_pursuit_*.yaml",
    )
    checks.append(
        Check(
            "ablation",
            (script_dir / "run_ablation_report.sh").exists() and ablation_cfgs == 14 and ablation_ctrl == 14,
            f"benchmark yamls={ablation_cfgs}, controller yamls={ablation_ctrl}",
        )
    )

    sensitivity_cfgs = count_files(
        ws_root / "src" / "pp_adaptive" / "config" / "benchmarks" / "sensitivity",
        "bench_sensitivity_*.yaml",
    )
    sensitivity_csvs = sum(
        count_files(ws_root / "src" / "pp_adaptive" / "racelines" / "sensitivity" / param, "*.csv")
        for param in ["alat", "alpha", "cdyn", "tmargin", "wcte"]
    )
    checks.append(
        Check(
            "sensitivity",
            (script_dir / "run_sensitivity_report.sh").exists() and sensitivity_cfgs == 29 and sensitivity_csvs == 29,
            f"benchmark yamls={sensitivity_cfgs}, bundled sweep csvs={sensitivity_csvs}",
        )
    )

    checks.append(
        Check(
            "figures",
            (script_dir / "run_report_figures.sh").exists(),
            "figure wrapper present; outputs are regenerated from analysis/gen_figures.py rather than archived file names",
        )
    )

    print("Report reproducibility audit")
    print(f"workspace: {ws_root}")
    print(f"report:     {report_root}")
    print("")

    failed = False
    for check in checks:
        status = "OK" if check.ok else "MISSING"
        print(f"{status:7s} {check.label:14s} {check.detail}")
        failed = failed or not check.ok

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
