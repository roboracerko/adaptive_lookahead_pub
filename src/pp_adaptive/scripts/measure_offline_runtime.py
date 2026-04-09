#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measure_offline_runtime.py  —  R9: 오프라인 런타임 측정

kw2 labeler의 각 맵별 실행 시간과 메모리 사용량을 측정한다.
결과는 논문 App B 및 Discussion §Practical Implications에 기재한다.

사용법:
  cd <workspace>/src/pp_adaptive
  python3 scripts/measure_offline_runtime.py

출력:
  콘솔: 맵별 타이밍 표
  파일: /tmp/revision_runtime_results.csv
"""

import csv
import os
import subprocess
import sys
import time
from pathlib import Path

# ─── 설정 ─────────────────────────────────────────────────────────────────
BASE = Path(__file__).resolve().parent.parent  # pp_adaptive/
PYDIR   = BASE / "pp_adaptive"
RLINES  = BASE / "racelines"
MAPS_DIR = BASE / "maps"

MAPS = [
    ("Budapest_map",    MAPS_DIR / "Budapest_map.yaml"),
    ("Spielberg",       MAPS_DIR / "Spielberg.yaml"),
    ("icra",            MAPS_DIR / "icra.yaml"),
    ("Austin_map",      MAPS_DIR / "Austin_map.yaml"),
    ("skir",            MAPS_DIR / "skir.yaml"),
    ("BrandsHatch_map", MAPS_DIR / "BrandsHatch_map.yaml"),
]

KW2_PARAMS = [
    "--ld_candidates", "0.9", "1.2", "1.6", "2.0",
    "--objective", "ALAT_MAX_CTE",
    "--alpha", "0.30",
    "--a_lat_max", "7.5",
    "--c_dyn", "2.0",
    "--kw_margin_t", "0.3",
    "--w_cte", "0.5",
]

N_REPEATS = 3           # 평균 측정 횟수
OUT_CSV   = Path("/tmp/revision_runtime_results.csv")


# ─── 헬퍼 ─────────────────────────────────────────────────────────────────

def count_waypoints(csv_path: Path) -> int:
    with open(csv_path) as f:
        return sum(1 for _ in f) - 1  # header 제외


def run_labeler(in_csv: Path, map_yaml: Path, out_csv: Path) -> float:
    """kw2 labeler 한 번 실행. 실행 시간(초) 반환."""
    cmd = [
        sys.executable,
        str(PYDIR / "compute_adaptive_lookahead_kw2.py"),
        "--in_csv",  str(in_csv),
        "--map_yaml", str(map_yaml),
        "--out_csv",  str(out_csv),
    ] + KW2_PARAMS

    t0 = time.perf_counter()
    result = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.perf_counter() - t0

    if result.returncode != 0:
        print(f"  ERROR: {result.stderr[-300:]}")
        return float("nan")
    return elapsed


def get_file_size_kb(path: Path) -> float:
    try:
        return path.stat().st_size / 1024.0
    except FileNotFoundError:
        return 0.0


# ─── 메인 ─────────────────────────────────────────────────────────────────

def main():
    print("=" * 66)
    print(f" kw2 Offline Labeler  —  Runtime Measurement  (N_repeats={N_REPEATS})")
    print("=" * 66)

    results = []

    for map_name, map_yaml in MAPS:
        in_csv  = RLINES / f"{map_name}_optimal_rl.csv"
        out_csv = Path(f"/tmp/runtime_test_{map_name}.csv")

        if not in_csv.exists():
            print(f"  [SKIP] {map_name}: raceline not found ({in_csv})")
            continue
        if not map_yaml.exists():
            print(f"  [SKIP] {map_name}: map yaml not found ({map_yaml})")
            continue

        n_wp = count_waypoints(in_csv)
        print(f"\n  {map_name}  ({n_wp} waypoints)")

        times = []
        for rep in range(N_REPEATS):
            t = run_labeler(in_csv, map_yaml, out_csv)
            times.append(t)
            print(f"    rep {rep+1}/{N_REPEATS}: {t:.2f} s")

        valid_times = [t for t in times if not (t != t)]  # NaN 제거
        if not valid_times:
            print(f"    ERROR: all runs failed")
            continue

        avg_t   = sum(valid_times) / len(valid_times)
        min_t   = min(valid_times)
        max_t   = max(valid_times)
        us_per_wp = 1e6 * avg_t / n_wp if n_wp > 0 else 0
        mem_kb  = get_file_size_kb(out_csv)
        k_cands = 4  # fixed

        print(f"    avg={avg_t:.2f}s  min={min_t:.2f}s  max={max_t:.2f}s"
              f"  →  {us_per_wp:.0f} μs/wp  |  output={mem_kb:.1f} KB")

        results.append({
            "map":       map_name,
            "n_wp":      n_wp,
            "k_cands":   k_cands,
            "avg_s":     round(avg_t, 3),
            "min_s":     round(min_t, 3),
            "max_s":     round(max_t, 3),
            "us_per_wp": round(us_per_wp, 1),
            "output_kb": round(mem_kb, 1),
        })

        # 임시 파일 삭제
        out_csv.unlink(missing_ok=True)

    # ─── 콘솔 요약 ────────────────────────────────────────────────────────
    print()
    print("=" * 66)
    print(" Summary for Paper (App B / §Practical Implications)")
    print("=" * 66)
    print(f"  {'Map':<18} {'N_wp':>6} {'K':>3} {'Avg [s]':>8} "
          f"{'μs/wp':>8} {'Output [KB]':>12}")
    print(f"  {'-'*18} {'-'*6} {'-'*3} {'-'*8} {'-'*8} {'-'*12}")
    for r in results:
        print(f"  {r['map']:<18} {r['n_wp']:>6} {r['k_cands']:>3}"
              f" {r['avg_s']:>8.2f} {r['us_per_wp']:>8.0f}"
              f" {r['output_kb']:>12.1f}")

    if results:
        avg_us = sum(r["us_per_wp"] for r in results) / len(results)
        total_s = sum(r["avg_s"] for r in results)
        print(f"\n  {'Mean μs/wp':.<30} {avg_us:.0f}")
        print(f"  {'Total (6 maps, seq.)':.<30} {total_s:.1f} s")

    print()
    print(" Suggested paper text (App B):")
    if results:
        r_ex = max(results, key=lambda r: r["n_wp"])
        print(f"   On a standard laptop CPU (single-threaded), the full offline")
        print(f"   assignment for {r_ex['map']} ({r_ex['n_wp']} waypoints, K=4)")
        print(f"   completes in {r_ex['avg_s']:.1f} s, producing a label array of")
        print(f"   {r_ex['output_kb']:.0f} KB. The per-waypoint cost is"
              f" ~{r_ex['us_per_wp']:.0f} μs.")

    # ─── CSV 저장 ──────────────────────────────────────────────────────────
    if results:
        with open(OUT_CSV, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)
        print(f"\n  Saved → {OUT_CSV}")

    print("=" * 66)


if __name__ == "__main__":
    main()
