#!/usr/bin/env python3
"""
compute_stats.py
벤치마크 결과 디렉토리에서 논문 Table II~III 통계를 계산한다.

- Success rate with Wilson CI (95%)
- Mean speed, CTE, jerk
- Spearman r(|κ|, LD)

Usage:
    python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/main
    python3 analysis/compute_stats.py --results results/main_benchmark --precomputed
"""
import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats


# ── Wilson 신뢰구간 (95%) ────────────────────────────────────────────────────
def wilson_ci(successes: int, n: int, z: float = 1.96):
    if n == 0:
        return 0.0, 0.0, 0.0
    p = successes / n
    denom = 1 + z ** 2 / n
    centre = (p + z ** 2 / (2 * n)) / denom
    margin = z * math.sqrt(p * (1 - p) / n + z ** 2 / (4 * n ** 2)) / denom
    return p, max(0, centre - margin), min(1, centre + margin)


# ── 결과 디렉토리 파싱 ───────────────────────────────────────────────────────
def load_results(results_root: str):
    """
    results_root/
      {method}_{map}/            ← evaluation name
        pp_adaptive/
          */runs/run_*.json       ← 각 trial
          */all_laps_summary.csv
    """
    rows = []
    for eval_dir in sorted(Path(results_root).iterdir()):
        if not eval_dir.is_dir():
            continue
        name = eval_dir.name  # e.g. "kw_sim_budapest"
        parts = name.split('_')
        # method: kw_sim(2), alg1_org(2), alg1_ext(2), kv(1), curv(1)
        # 마지막 파트가 map name의 첫 토큰
        for method in ['kw_sim', 'alg1_org', 'alg1_ext', 'kv', 'curv',
                       'ablation_noA', 'ablation_noB', 'ablation_noC', 'ablation_noD']:
            if name.startswith(method):
                map_name = name[len(method) + 1:]
                break
        else:
            method, map_name = name, 'unknown'

        driver_dir = eval_dir / 'pp_adaptive'
        if not driver_dir.exists():
            continue

        successes = 0
        total = 0
        speeds, ctes, jerks = [], [], []

        for trial_dir in sorted(driver_dir.iterdir()):
            if not trial_dir.is_dir():
                continue
            run_json = trial_dir / 'runs' / f'run_run_{3}laps.json'
            # 파일명 패턴 유연하게 탐색
            run_files = list((trial_dir / 'runs').glob('run_*.json')) if (trial_dir / 'runs').exists() else []
            if not run_files:
                continue
            run_data = json.loads(run_files[0].read_text())
            total += 1
            if run_data.get('status') == 'success' or run_data.get('laps_completed', 0) >= 3:
                successes += 1

            # lap summaries
            lap_csv = trial_dir / 'all_laps_summary.csv'
            if lap_csv.exists():
                ldf = pd.read_csv(lap_csv)
                if 'avg_speed' in ldf.columns:
                    speeds.extend(ldf['avg_speed'].dropna().tolist())
                if 'mean_cte' in ldf.columns:
                    ctes.extend(ldf['mean_cte'].dropna().tolist())
                if 'mean_jerk' in ldf.columns:
                    jerks.extend(ldf['mean_jerk'].dropna().tolist())

        p, lo, hi = wilson_ci(successes, total)
        rows.append({
            'method': method,
            'map': map_name,
            'n': total,
            'successes': successes,
            'success_rate': round(p * 100, 1),
            'wilson_lo': round(lo * 100, 1),
            'wilson_hi': round(hi * 100, 1),
            'avg_speed': round(float(np.mean(speeds)), 2) if speeds else None,
            'mean_cte': round(float(np.mean(ctes)), 4) if ctes else None,
            'mean_jerk': round(float(np.mean(jerks)), 4) if jerks else None,
        })

    return pd.DataFrame(rows)


# ── Spearman r(|κ|, LD) ──────────────────────────────────────────────────────
def spearman_kappa_ld(raceline_dir: str):
    results = []
    for csv_path in sorted(Path(raceline_dir).glob('*.csv')):
        try:
            df = pd.read_csv(csv_path)
            if 'kappa_1pm' not in df.columns or 'ld_m' not in df.columns:
                continue
            kappa = df['kappa_1pm'].abs().dropna()
            ld = df['ld_m'].dropna()
            if len(kappa) < 10:
                continue
            r, p = stats.spearmanr(kappa, ld)
            results.append({
                'file': csv_path.name,
                'spearman_r': round(r, 3),
                'p_value': round(p, 4),
                'n_waypoints': len(kappa),
            })
        except Exception:
            pass
    return pd.DataFrame(results)


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', required=True, help='eval results root dir')
    ap.add_argument('--raceline-dir', default=None,
                    help='racelines/ dir for Spearman analysis')
    ap.add_argument('--output', default=None, help='output CSV path')
    ap.add_argument('--precomputed', action='store_true',
                    help='load from results/ CSV instead of scanning dirs')
    args = ap.parse_args()

    if args.precomputed and os.path.exists(args.results):
        df = pd.read_csv(args.results)
    else:
        df = load_results(args.results)

    if df.empty:
        print('No results found.')
        sys.exit(1)

    print('\n=== Success Rate (Wilson 95% CI) ===')
    pivot = df.pivot_table(
        index='method', columns='map',
        values='success_rate', aggfunc='first'
    )
    print(pivot.to_string())

    print('\n=== Average Speed (m/s) ===')
    pivot_spd = df.pivot_table(
        index='method', columns='map',
        values='avg_speed', aggfunc='first'
    )
    print(pivot_spd.to_string())

    # Spearman r
    if args.raceline_dir:
        print('\n=== Spearman r(|κ|, LD) ===')
        sr = spearman_kappa_ld(args.raceline_dir)
        print(sr.to_string(index=False))

    out = args.output or os.path.join(args.results, 'stats_summary.csv')
    df.to_csv(out, index=False)
    print(f'\nSaved: {out}')


if __name__ == '__main__':
    main()
