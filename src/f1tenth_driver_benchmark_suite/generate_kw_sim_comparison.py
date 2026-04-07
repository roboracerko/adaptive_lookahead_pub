#!/usr/bin/env python3
"""
kw_sim 벤치마크 결과를 기존 pp_fixed / rl 결과와 합쳐 비교 보고서를 생성한다.

Usage:
  python3 generate_kw_sim_comparison.py \
    --kw_sim_dir  /tmp/f1tenth_driver_benchmark_suite/kw_sim_n30 \
    --baseline_dir /tmp/f1tenth_driver_benchmark_suite/results_final_judgment_n30_20260304_070109 \
    --output_dir  /tmp/f1tenth_driver_benchmark_suite/comparison_kw_sim
"""

import argparse
import csv
import json
import os
from glob import glob
from statistics import mean
from typing import Any, Dict, List, Optional


# ──────────────────────────────────────────────
# JSON loading helpers
# ──────────────────────────────────────────────
def _safe_float(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
        return None if f != f else f   # NaN → None
    except Exception:
        return None


def _load_run_json(run_dir: str) -> Optional[Dict]:
    """runs/ 디렉토리에서 최신 run_*.json 을 읽는다."""
    runs_dir = os.path.join(run_dir, 'runs')
    if not os.path.isdir(runs_dir):
        return None
    files = sorted(glob(os.path.join(runs_dir, '*.json')))
    if not files:
        return None
    with open(files[-1], encoding='utf-8') as f:
        return json.load(f)


def _load_error_json(run_dir: str) -> Optional[Dict]:
    """error_*.json 을 읽는다 (없으면 None)."""
    files = sorted(glob(os.path.join(run_dir, 'error_*.json')))
    if not files:
        return None
    with open(files[0], encoding='utf-8') as f:
        return json.load(f)


# ──────────────────────────────────────────────
# Scan one driver's result directory
# ──────────────────────────────────────────────
def scan_driver_dir(driver_root: str, driver_name: str, target_laps: int = 1) -> List[Dict]:
    """
    driver_root/<run_tag>/ 구조를 스캔해 per-trial row list 를 반환한다.
    """
    rows = []
    if not os.path.isdir(driver_root):
        print(f"  [WARN] directory not found: {driver_root}")
        return rows

    run_dirs = sorted(d for d in glob(os.path.join(driver_root, '*')) if os.path.isdir(d))
    for run_dir in run_dirs:
        run_tag = os.path.basename(run_dir)
        run_json = _load_run_json(run_dir)
        error_json = _load_error_json(run_dir)

        total_laps = int(run_json.get('total_laps', 0)) if run_json else 0
        completed = (total_laps >= target_laps) and (error_json is None)

        row = {
            'driver': driver_name,
            'run_tag': run_tag,
            'target_laps': target_laps,
            'total_laps': total_laps,
            'lap_completed': completed,
            'avg_lap_time_s':             _safe_float(run_json.get('avg_lap_time_s'))         if run_json else None,
            'avg_speed_mps':              _safe_float(run_json.get('avg_speed_mps'))           if run_json else None,
            'max_speed_mps':              _safe_float(run_json.get('max_speed_mps'))           if run_json else None,
            'min_speed_mps':              _safe_float(run_json.get('min_speed_mps'))           if run_json else None,
            'avg_rms_lateral_error_m':    _safe_float(run_json.get('avg_rms_lateral_error_m')) if run_json else None,
            'max_lateral_error_m':        _safe_float(run_json.get('max_lateral_error_m'))     if run_json else None,
            'avg_rms_heading_error_rad':  _safe_float(run_json.get('avg_rms_heading_error_rad')) if run_json else None,
            'avg_rms_yaw_rate_radps':     _safe_float(run_json.get('avg_rms_yaw_rate_radps'))  if run_json else None,
            'avg_rms_steering_jerk_radps2': _safe_float(run_json.get('avg_rms_steering_jerk_radps2')) if run_json else None,
            'avg_rms_longitudinal_jerk_mps3': _safe_float(run_json.get('avg_rms_longitudinal_jerk_mps3')) if run_json else None,
            'avg_path_efficiency':        _safe_float(run_json.get('avg_path_efficiency'))     if run_json else None,
            'overall_offtrack_ratio':     _safe_float(run_json.get('overall_offtrack_ratio'))  if run_json else None,
            'estop_count_total':          run_json.get('estop_count_total')                    if run_json else None,
            'error':      error_json.get('error')      if error_json else None,
            'error_code': error_json.get('error_code') if error_json else None,
            'run_dir':    run_dir,
        }
        rows.append(row)

    print(f"  {driver_name}: {len(rows)} trials scanned from {driver_root}")
    return rows


# ──────────────────────────────────────────────
# Load existing comparison_runs.csv (baseline)
# ──────────────────────────────────────────────
def load_baseline_csv(baseline_dir: str, drivers: List[str]) -> List[Dict]:
    """
    기존 comparison_runs.csv 에서 지정 드라이버 행만 추출한다.
    파일이 없으면 디렉토리 스캔으로 fallback.
    """
    csv_path = os.path.join(baseline_dir, 'comparison_runs.csv')
    if os.path.exists(csv_path):
        rows = []
        with open(csv_path, encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get('driver') in drivers:
                    # lap_completed: string → bool
                    lc = row.get('lap_completed', 'False')
                    row['lap_completed'] = lc in ('True', 'true', '1')
                    rows.append(dict(row))
        print(f"  Loaded {len(rows)} rows from {csv_path}")
        return rows
    else:
        print(f"  [WARN] {csv_path} not found — scanning directories")
        rows = []
        for drv in drivers:
            drv_dir = os.path.join(baseline_dir, drv)
            rows.extend(scan_driver_dir(drv_dir, drv))
        return rows


# ──────────────────────────────────────────────
# Aggregate
# ──────────────────────────────────────────────
def aggregate(rows: List[Dict]) -> List[Dict]:
    grouped: Dict[str, List[Dict]] = {}
    for r in rows:
        grouped.setdefault(r['driver'], []).append(r)

    out = []
    for driver, drows in sorted(grouped.items()):
        completed = [r for r in drows if r.get('lap_completed') is True]
        n = len(drows)
        n_ok = len(completed)

        def avg(key):
            vals = [_safe_float(r.get(key)) for r in drows]
            vals = [v for v in vals if v is not None]
            return mean(vals) if vals else None

        def avg_ok(key):
            vals = [_safe_float(r.get(key)) for r in completed]
            vals = [v for v in vals if v is not None]
            return mean(vals) if vals else None

        out.append({
            'driver':               driver,
            'runs':                 n,
            'success_runs':         n_ok,
            'completion_rate_pct':  n_ok / n * 100.0 if n else 0.0,
            'avg_lap_time_s':       avg_ok('avg_lap_time_s'),
            'avg_speed_mps':        avg_ok('avg_speed_mps'),
            'peak_speed_mps':       max(
                (_safe_float(r.get('max_speed_mps')) for r in drows if _safe_float(r.get('max_speed_mps')) is not None),
                default=None,
            ),
            'avg_rms_lateral_error_m':  avg_ok('avg_rms_lateral_error_m'),
            'avg_path_efficiency':       avg_ok('avg_path_efficiency'),
            'failures':             n - n_ok,
        })
    return out


# ──────────────────────────────────────────────
# Write output files
# ──────────────────────────────────────────────
def fmt(v: Any, d: int = 3) -> str:
    if v is None:
        return 'NA'
    if isinstance(v, bool):
        return 'Y' if v else 'N'
    if isinstance(v, float):
        return f'{v:.{d}f}'
    return str(v)


def write_files(output_dir: str, rows: List[Dict], summary: List[Dict]):
    os.makedirs(output_dir, exist_ok=True)

    # ── comparison_runs.csv ──────────────────────────────────────────────
    runs_csv = os.path.join(output_dir, 'comparison_runs.csv')
    fieldnames = [
        'driver', 'run_tag', 'target_laps', 'total_laps', 'lap_completed',
        'avg_lap_time_s', 'avg_speed_mps', 'max_speed_mps', 'min_speed_mps',
        'avg_rms_lateral_error_m', 'max_lateral_error_m',
        'avg_rms_heading_error_rad', 'avg_rms_yaw_rate_radps',
        'avg_rms_steering_jerk_radps2', 'avg_rms_longitudinal_jerk_mps3',
        'avg_path_efficiency', 'overall_offtrack_ratio', 'estop_count_total',
        'error', 'error_code', 'run_dir',
    ]
    with open(runs_csv, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    print(f"[OK] {runs_csv}")

    # ── comparison_summary.csv ───────────────────────────────────────────
    summary_csv = os.path.join(output_dir, 'comparison_summary.csv')
    sfn = [
        'driver', 'runs', 'success_runs', 'completion_rate_pct',
        'avg_lap_time_s', 'avg_speed_mps', 'peak_speed_mps',
        'avg_rms_lateral_error_m', 'avg_path_efficiency', 'failures',
    ]
    with open(summary_csv, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=sfn)
        w.writeheader()
        w.writerows(summary)
    print(f"[OK] {summary_csv}")

    # ── comparison_summary.md ────────────────────────────────────────────
    md_path = os.path.join(output_dir, 'comparison_summary.md')
    lines = [
        '# F1TENTH Driver Comparison: kw_sim vs pp_fixed vs rl',
        '',
        '| driver | runs | success | completion(%) | avg_lap_time(s) | avg_speed(m/s) | peak_speed(m/s) | rms_lat_err(m) | path_eff | failures |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for r in summary:
        lines.append(
            f"| **{r['driver']}** | {r['runs']} | {r['success_runs']} "
            f"| {fmt(r['completion_rate_pct'], 1)} "
            f"| {fmt(r['avg_lap_time_s'])} "
            f"| {fmt(r['avg_speed_mps'])} "
            f"| {fmt(r['peak_speed_mps'])} "
            f"| {fmt(r['avg_rms_lateral_error_m'], 4)} "
            f"| {fmt(r['avg_path_efficiency'], 4)} "
            f"| {r['failures']} |"
        )
    lines += [
        '',
        '## Per-trial details',
        '',
        '| driver | run_tag | completed | laps | avg_speed | lap_time | rms_lat_err | error |',
        '|---|---|:---:|---:|---:|---:|---:|---|',
    ]
    for r in rows:
        lines.append(
            f"| {r['driver']} | {r['run_tag']} "
            f"| {'✅' if r.get('lap_completed') else '❌'} "
            f"| {r.get('total_laps', 'NA')} "
            f"| {fmt(_safe_float(r.get('avg_speed_mps')))} "
            f"| {fmt(_safe_float(r.get('avg_lap_time_s')))} "
            f"| {fmt(_safe_float(r.get('avg_rms_lateral_error_m')), 4)} "
            f"| {r.get('error', '') or ''} |"
        )

    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"[OK] {md_path}")


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='kw_sim vs pp_fixed vs rl 비교 보고서 생성')
    ap.add_argument(
        '--kw_sim_dir',
        default='/tmp/f1tenth_driver_benchmark_suite/kw_sim_n30',
        help='kw_sim N=30 결과 디렉토리 (evaluation_runner output_root)',
    )
    ap.add_argument(
        '--baseline_dir',
        default='/tmp/f1tenth_driver_benchmark_suite/results_final_judgment_n30_20260304_070109',
        help='기존 N=30 결과 디렉토리 (pp_fixed, rl 포함)',
    )
    ap.add_argument(
        '--output_dir',
        default='/tmp/f1tenth_driver_benchmark_suite/comparison_kw_sim',
        help='비교 보고서 출력 디렉토리',
    )
    ap.add_argument(
        '--baseline_drivers',
        nargs='+',
        default=['pp_fixed', 'rl'],
        help='baseline 에서 가져올 드라이버 목록',
    )
    ap.add_argument(
        '--kw_sim_laps',
        type=int,
        default=1,
        help='kw_sim 평가 목표 랩 수 (default: 1)',
    )
    ap.add_argument(
        '--baseline_laps',
        type=int,
        default=3,
        help='baseline (pp_fixed/rl) 목표 랩 수 (default: 3)',
    )
    args = ap.parse_args()

    print('[INFO] 기존 baseline 결과 로드...')
    baseline_rows = load_baseline_csv(args.baseline_dir, args.baseline_drivers)
    # target_laps 보정 (baseline은 3-lap)
    for r in baseline_rows:
        r['target_laps'] = args.baseline_laps

    print('[INFO] kw_sim pp_adaptive 결과 스캔...')
    kw_rows = scan_driver_dir(
        os.path.join(args.kw_sim_dir, 'pp_adaptive'),
        driver_name='pp_adaptive_kw_sim',
        target_laps=args.kw_sim_laps,
    )

    if not kw_rows:
        print('[WARN] kw_sim 결과 없음. 벤치마크를 먼저 실행하세요:')
        print(f'  cd /home/jin/ros2_prj/benchmark_ws')
        print(f'  source /home/jin/ros2_prj/sim_ws/install/setup.sh && source install/setup.bash')
        print(f'  python3 -u src/f1tenth_driver_benchmark_suite/f1tenth_driver_benchmark_suite/evaluation_runner.py \\')
        print(f'    --config src/f1tenth_driver_benchmark_suite/config/tests/kw_sim_n30.yaml')
        return

    all_rows = baseline_rows + kw_rows
    summary = aggregate(all_rows)

    print(f'\n[RESULT] Summary:')
    for s in summary:
        print(
            f"  {s['driver']:25s}  "
            f"완주율={s['completion_rate_pct']:.1f}%  "
            f"({s['success_runs']}/{s['runs']})  "
            f"avg_speed={fmt(s['avg_speed_mps'])} m/s  "
            f"rms_lat={fmt(s['avg_rms_lateral_error_m'], 4)} m"
        )

    write_files(args.output_dir, all_rows, summary)
    print(f'\n[Done] 결과 저장: {args.output_dir}')


if __name__ == '__main__':
    main()
