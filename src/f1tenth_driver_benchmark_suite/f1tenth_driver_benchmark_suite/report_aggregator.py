#!/usr/bin/env python3

import argparse
import csv
import json
import os
from glob import glob
from statistics import mean


def _load_latest_run_summary(run_dir: str):
    files = sorted(glob(os.path.join(run_dir, 'runs', '*.json')))
    if not files:
        return None
    with open(files[-1], 'r', encoding='utf-8') as f:
        return json.load(f)


def _load_first_error(run_dir: str):
    files = sorted(glob(os.path.join(run_dir, 'error_*.json')))
    if not files:
        return None
    with open(files[0], 'r', encoding='utf-8') as f:
        return json.load(f)


def _mean_lap_field(run_dir: str, key: str):
    values = []
    for path in sorted(glob(os.path.join(run_dir, 'laps', '*.json'))):
        with open(path, 'r', encoding='utf-8') as f:
            lap = json.load(f)
        value = lap.get(key)
        if isinstance(value, (int, float)):
            values.append(float(value))
    return mean(values) if values else None


def _collect(output_root: str):
    rows = []
    if not os.path.isdir(output_root):
        return rows
    for driver in sorted([d for d in os.listdir(output_root) if os.path.isdir(os.path.join(output_root, d))]):
        driver_dir = os.path.join(output_root, driver)
        for run_tag in sorted(os.listdir(driver_dir)):
            run_dir = os.path.join(driver_dir, run_tag)
            if not os.path.isdir(run_dir):
                continue
            summary = _load_latest_run_summary(run_dir)
            error = _load_first_error(run_dir)
            total_laps = summary.get('total_laps', 0) if summary else 0
            rows.append({
                'driver': driver,
                'run_tag': run_tag,
                'total_laps': total_laps,
                'avg_lap_time_s': summary.get('avg_lap_time_s') if summary else None,
                'avg_speed_mps': summary.get('avg_speed_mps') if summary else None,
                'max_speed_mps': summary.get('max_speed_mps') if summary else None,
                'min_speed_mps': summary.get('min_speed_mps') if summary else None,
                'avg_rms_lateral_error_m': summary.get('avg_rms_lateral_error_m') if summary else None,
                'avg_rms_heading_error_rad': summary.get('avg_rms_heading_error_rad') if summary else None,
                'avg_rms_yaw_rate_radps': summary.get('avg_rms_yaw_rate_radps') if summary else None,
                'avg_yaw_rate_std_radps': summary.get('avg_yaw_rate_std_radps') if summary else None,
                'avg_rms_steering_rate_radps': summary.get('avg_rms_steering_rate_radps') if summary else None,
                'avg_rms_steering_jerk_radps2': summary.get('avg_rms_steering_jerk_radps2') if summary else None,
                'avg_rms_longitudinal_accel_mps2': summary.get('avg_rms_longitudinal_accel_mps2') if summary else None,
                'avg_rms_longitudinal_jerk_mps3': summary.get('avg_rms_longitudinal_jerk_mps3') if summary else None,
                'avg_lookahead_variance': (
                    summary.get('avg_lookahead_variance')
                    if summary and summary.get('avg_lookahead_variance') is not None
                    else _mean_lap_field(run_dir, 'lookahead_variance')
                ),
                'avg_path_efficiency': summary.get('avg_path_efficiency') if summary else None,
                'avg_path_excess_ratio': summary.get('avg_path_excess_ratio') if summary else None,
                'overall_offtrack_ratio': summary.get('overall_offtrack_ratio') if summary else None,
                'estop_count_total': summary.get('estop_count_total') if summary else None,
                'error': error.get('error') if error else None,
                'error_code': error.get('error_code') if error else None,
                'run_dir': run_dir,
            })
    return rows


def _safe_mean(values):
    vals = [v for v in values if isinstance(v, (int, float))]
    return mean(vals) if vals else None


def _aggregate(rows):
    by_driver = {}
    for row in rows:
        by_driver.setdefault(row['driver'], []).append(row)

    summary = []
    for driver, drows in by_driver.items():
        completed_rows = [r for r in drows if r.get('lap_completed') is True]
        summary.append({
            'driver': driver,
            'runs': len(drows),
            'success_runs': len(completed_rows),
            'completion_rate_pct': (len(completed_rows) / len(drows) * 100.0) if drows else 0.0,
            'avg_total_laps': _safe_mean([r['total_laps'] for r in drows]),
            'avg_lap_time_s': _safe_mean([r['avg_lap_time_s'] for r in drows]),
            'avg_speed_mps': _safe_mean([r['avg_speed_mps'] for r in drows]),
            'peak_speed_mps': max([v for v in [r['max_speed_mps'] for r in drows] if isinstance(v, (int, float))], default=None),
            'avg_rms_lateral_error_m': _safe_mean([r['avg_rms_lateral_error_m'] for r in drows]),
            'avg_rms_heading_error_rad': _safe_mean([r['avg_rms_heading_error_rad'] for r in drows]),
            'avg_rms_yaw_rate_radps': _safe_mean([r['avg_rms_yaw_rate_radps'] for r in drows]),
            'avg_yaw_rate_std_radps': _safe_mean([r['avg_yaw_rate_std_radps'] for r in drows]),
            'avg_rms_steering_rate_radps': _safe_mean([r['avg_rms_steering_rate_radps'] for r in drows]),
            'avg_rms_steering_jerk_radps2': _safe_mean([r['avg_rms_steering_jerk_radps2'] for r in drows]),
            'avg_rms_longitudinal_accel_mps2': _safe_mean([r['avg_rms_longitudinal_accel_mps2'] for r in drows]),
            'avg_rms_longitudinal_jerk_mps3': _safe_mean([r['avg_rms_longitudinal_jerk_mps3'] for r in drows]),
            'avg_lookahead_variance': _safe_mean([r['avg_lookahead_variance'] for r in drows]),
            'avg_path_efficiency': _safe_mean([r['avg_path_efficiency'] for r in drows]),
            'avg_path_excess_ratio': _safe_mean([r['avg_path_excess_ratio'] for r in drows]),
            'avg_offtrack_ratio': _safe_mean([r['overall_offtrack_ratio'] for r in drows]),
            'avg_estop_count': _safe_mean([r['estop_count_total'] for r in drows]),
            'failures': len(drows) - len(completed_rows),
        })
    return sorted(summary, key=lambda x: x['driver'])


def _write_csv(path, rows, fieldnames):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _write_markdown(path, summary_rows):
    lines = [
        '# F1TENTH Driver Benchmark Summary',
        '',
        '| driver | runs | success_runs | completion_rate(%) | avg_total_laps | avg_lap_time_s | avg_speed_mps | peak_speed_mps | avg_rms_lat(m) | avg_rms_head(rad) | yaw_std(rad/s) | steer_jerk(rad/s2) | long_jerk(m/s3) | lookahead_var | path_eff | avg_offtrack_ratio | avg_estop_count | failures |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]

    def fmt(v, ndigits):
        if v is None:
            return 'NA'
        return f'{v:.{ndigits}f}'

    for row in summary_rows:
        lines.append(
            f"| {row['driver']} | {row['runs']} | {row['success_runs']} | {fmt(row['completion_rate_pct'], 1)} | "
            f"{fmt(row['avg_total_laps'], 3)} | "
            f"{fmt(row['avg_lap_time_s'], 3)} | "
            f"{fmt(row['avg_speed_mps'], 3)} | "
            f"{fmt(row['peak_speed_mps'], 3)} | "
            f"{fmt(row['avg_rms_lateral_error_m'], 4)} | "
            f"{fmt(row['avg_rms_heading_error_rad'], 4)} | "
            f"{fmt(row['avg_yaw_rate_std_radps'], 4)} | "
            f"{fmt(row['avg_rms_steering_jerk_radps2'], 4)} | "
            f"{fmt(row['avg_rms_longitudinal_jerk_mps3'], 4)} | "
            f"{fmt(row['avg_lookahead_variance'], 4)} | "
            f"{fmt(row['avg_path_efficiency'], 4)} | "
            f"{fmt(row['avg_offtrack_ratio'], 4)} | "
            f"{fmt(row['avg_estop_count'], 3)} | {row['failures']} |"
        )

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description='Aggregate benchmark run summaries')
    parser.add_argument('--output-root', default='/tmp/f1tenth_driver_benchmark_suite/results')
    parser.add_argument('--target-laps', type=int, default=3)
    args = parser.parse_args()

    rows = _collect(args.output_root)
    for row in rows:
        row['target_laps'] = args.target_laps
        row['lap_completed'] = bool(
            isinstance(row.get('total_laps'), (int, float))
            and row['total_laps'] >= args.target_laps
            and not row.get('error')
        )
    summary = _aggregate(rows)

    raw_csv = os.path.join(args.output_root, 'comparison_runs.csv')
    agg_csv = os.path.join(args.output_root, 'comparison_summary.csv')
    md_file = os.path.join(args.output_root, 'comparison_summary.md')

    if rows:
        _write_csv(
            raw_csv,
            rows,
            [
                'driver', 'run_tag', 'target_laps', 'total_laps', 'lap_completed',
                'avg_lap_time_s', 'avg_speed_mps', 'max_speed_mps', 'min_speed_mps',
                'avg_rms_lateral_error_m', 'avg_rms_heading_error_rad',
                'avg_rms_yaw_rate_radps', 'avg_yaw_rate_std_radps',
                'avg_rms_steering_rate_radps', 'avg_rms_steering_jerk_radps2',
                'avg_rms_longitudinal_accel_mps2', 'avg_rms_longitudinal_jerk_mps3',
                'avg_lookahead_variance',
                'avg_path_efficiency', 'avg_path_excess_ratio',
                'overall_offtrack_ratio', 'estop_count_total', 'error', 'error_code', 'run_dir',
            ],
        )

    if summary:
        _write_csv(
            agg_csv,
            summary,
            [
                'driver', 'runs', 'success_runs', 'completion_rate_pct',
                'avg_total_laps', 'avg_lap_time_s', 'avg_speed_mps', 'peak_speed_mps',
                'avg_rms_lateral_error_m', 'avg_rms_heading_error_rad',
                'avg_rms_yaw_rate_radps', 'avg_yaw_rate_std_radps',
                'avg_rms_steering_rate_radps', 'avg_rms_steering_jerk_radps2',
                'avg_rms_longitudinal_accel_mps2', 'avg_rms_longitudinal_jerk_mps3',
                'avg_lookahead_variance',
                'avg_path_efficiency', 'avg_path_excess_ratio',
                'avg_offtrack_ratio', 'avg_estop_count', 'failures',
            ],
        )
        _write_markdown(md_file, summary)

    print(f'[aggregator] rows={len(rows)} summary={len(summary)}')
    if rows:
        print(f'[aggregator] wrote: {raw_csv}')
    if summary:
        print(f'[aggregator] wrote: {agg_csv}')
        print(f'[aggregator] wrote: {md_file}')


if __name__ == '__main__':
    main()
