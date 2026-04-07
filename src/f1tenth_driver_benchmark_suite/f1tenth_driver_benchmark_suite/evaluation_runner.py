#!/usr/bin/env python3

import argparse
import csv
import glob as _glob_mod
import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from glob import glob
from statistics import mean
from typing import Any, Dict, List, Optional

import yaml

POSE_RE = re.compile(
    r'Updated ego_pose: x=([-+0-9.eE]+), y=([-+0-9.eE]+), '
    r'theta=([-+0-9.eE]+) v=([-+0-9.eE]+).*cmd_v=([-+0-9.eE]+)'
)


def _log(*args, **kwargs) -> None:
    """Force line delivery even when stdout is piped through tee."""
    kwargs.setdefault('flush', True)
    print(*args, **kwargs)


def _cleanup_between_trials(verbose: bool = False) -> None:
    """trial 전 DDS 공유 메모리 및 잔여 프로세스를 정리한다."""
    for pattern in ['gym_bridge', 'metrics_collector_node', 'map_server',
                    'lifecycle_manager', 'robot_state_publisher']:
        subprocess.run(['pkill', '-f', pattern], capture_output=True)
    time.sleep(1.5)

    removed = 0
    for path in _glob_mod.glob('/dev/shm/fastrtps_*'):
        try:
            os.remove(path)
            removed += 1
        except OSError:
            pass
    if verbose and removed:
        _log(f'[cleanup] removed {removed} fastrtps shm segments')

    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    time.sleep(0.5)
    subprocess.run(['ros2', 'daemon', 'start'], capture_output=True)
    time.sleep(0.5)


def _safe_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        v = float(value)
    except Exception:
        return None
    if v != v:  # NaN
        return None
    return v


def _normalize_text(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bytes):
        return value.decode('utf-8', errors='replace')
    return str(value)


def _write_status_file(path: str, payload: Dict[str, Any]):
    try:
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
    except Exception:
        pass


def _terminate_process_tree(proc: subprocess.Popen, term_timeout_s: float = 5.0, kill_timeout_s: float = 3.0):
    if proc.poll() is not None:
        return
    used_group_signal = False
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        used_group_signal = True
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    try:
        proc.wait(timeout=term_timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass

    if used_group_signal:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass
    else:
        try:
            proc.kill()
        except Exception:
            pass
    try:
        proc.wait(timeout=kill_timeout_s)
    except Exception:
        pass


def _read_stream_lines(stream, sink: deque, runtime_state: Optional[Dict[str, Any]] = None):
    try:
        for line in iter(stream.readline, ''):
            text = line.rstrip('\n')
            sink.append(text)
            if runtime_state is not None:
                if 'SAFETY_OFFTRACK' in text:
                    runtime_state['safety_offtrack_count'] = int(runtime_state.get('safety_offtrack_count', 0)) + 1
                    runtime_state['last_safety'] = 'offtrack'
                    runtime_state['last_safety_line'] = text
                    runtime_state['last_safety_time'] = time.time()
                if 'SAFETY_COLLISION_SCAN' in text:
                    runtime_state['safety_collision_count'] = int(runtime_state.get('safety_collision_count', 0)) + 1
                    runtime_state['last_safety'] = 'collision'
                    runtime_state['last_safety_line'] = text
                    runtime_state['last_safety_time'] = time.time()
                m = POSE_RE.search(text)
                if m:
                    runtime_state['pose_x'] = float(m.group(1))
                    runtime_state['pose_y'] = float(m.group(2))
                    runtime_state['theta'] = float(m.group(3))
                    runtime_state['speed'] = float(m.group(4))
                    runtime_state['cmd_speed'] = float(m.group(5))
                    runtime_state['last_pose_time'] = time.time()
    except Exception:
        pass
    finally:
        try:
            stream.close()
        except Exception:
            pass


def _load_latest_json(dir_path: str) -> Optional[Dict[str, Any]]:
    files = sorted(glob(os.path.join(dir_path, '*.json')))
    if not files:
        return None
    with open(files[-1], 'r', encoding='utf-8') as f:
        return json.load(f)


def _parse_report_scalar(text: Optional[str]) -> Any:
    if text is None:
        return None
    if text in ('None', 'NA'):
        return None
    if text == 'True':
        return True
    if text == 'False':
        return False
    if text.endswith('s'):
        try:
            return float(text[:-1])
        except ValueError:
            return text
    try:
        if any(ch in text for ch in ('.', 'e', 'E')):
            return float(text)
        return int(text)
    except ValueError:
        return text


def _extract_report_value(lines: List[str], label: str) -> Optional[str]:
    prefix = f"- {label}: `"
    for line in lines:
        if line.startswith(prefix) and line.endswith('`'):
            return line[len(prefix):-1]
    return None


def _load_existing_row_from_run_dir(driver: str, run_dir: str, target_laps: int) -> Optional[Dict[str, Any]]:
    report_path = os.path.join(run_dir, 'run_report.md')
    run_summary = _load_latest_json(os.path.join(run_dir, 'runs')) if os.path.isdir(os.path.join(run_dir, 'runs')) else None
    run_tag: Optional[str] = None
    lines: List[str] = []

    if os.path.isfile(report_path):
        try:
            with open(report_path, 'r', encoding='utf-8') as f:
                lines = [line.rstrip('\n') for line in f]
        except Exception:
            return None
        run_tag = _extract_report_value(lines, 'Run Tag')

    if not run_tag:
        run_tag = os.path.basename(run_dir)

    match = re.search(r'_t(\d+)_', run_tag)
    if not match:
        return None
    total_laps = run_summary.get('total_laps', 0) if run_summary else _count_current_laps(run_dir)
    completed = int(total_laps) >= int(target_laps)

    row = {
        'driver': driver,
        'trial': int(match.group(1)),
        'run_tag': run_tag,
        'target_laps': _parse_report_scalar(_extract_report_value(lines, 'Target Laps')) if lines else target_laps,
        'total_laps': _parse_report_scalar(_extract_report_value(lines, 'Completed Laps')) if lines else total_laps,
        'lap_completed': _parse_report_scalar(_extract_report_value(lines, 'Lap Completion')) if lines else completed,
        'avg_lap_time_s': run_summary.get('avg_lap_time_s') if run_summary else _parse_report_scalar(_extract_report_value(lines, 'Avg Lap Time (s)')),
        'avg_speed_mps': run_summary.get('avg_speed_mps') if run_summary else _parse_report_scalar(_extract_report_value(lines, 'Avg Speed (m/s)')),
        'max_speed_mps': run_summary.get('max_speed_mps') if run_summary else _parse_report_scalar(_extract_report_value(lines, 'Max Speed (m/s)')),
        'min_speed_mps': run_summary.get('min_speed_mps') if run_summary else None,
        'avg_rms_lateral_error_m': run_summary.get('avg_rms_lateral_error_m') if run_summary else None,
        'max_lateral_error_m': run_summary.get('max_lateral_error_m') if run_summary else None,
        'avg_rms_heading_error_rad': run_summary.get('avg_rms_heading_error_rad') if run_summary else None,
        'avg_rms_yaw_rate_radps': run_summary.get('avg_rms_yaw_rate_radps') if run_summary else None,
        'avg_yaw_rate_std_radps': run_summary.get('avg_yaw_rate_std_radps') if run_summary else None,
        'avg_rms_yaw_accel_radps2': run_summary.get('avg_rms_yaw_accel_radps2') if run_summary else None,
        'avg_rms_steering_rate_radps': run_summary.get('avg_rms_steering_rate_radps') if run_summary else None,
        'avg_rms_steering_jerk_radps2': run_summary.get('avg_rms_steering_jerk_radps2') if run_summary else None,
        'avg_rms_longitudinal_accel_mps2': run_summary.get('avg_rms_longitudinal_accel_mps2') if run_summary else None,
        'avg_rms_longitudinal_jerk_mps3': run_summary.get('avg_rms_longitudinal_jerk_mps3') if run_summary else None,
        'avg_lookahead_variance': run_summary.get('avg_lookahead_variance') if run_summary else None,
        'avg_path_efficiency': run_summary.get('avg_path_efficiency') if run_summary else None,
        'avg_path_excess_ratio': run_summary.get('avg_path_excess_ratio') if run_summary else None,
        'overall_offtrack_ratio': run_summary.get('overall_offtrack_ratio') if run_summary else None,
        'estop_count_total': run_summary.get('estop_count_total') if run_summary else None,
        'returncode': _parse_report_scalar(_extract_report_value(lines, 'Return Code')) if lines else (0 if completed else None),
        'duration_s': _parse_report_scalar(_extract_report_value(lines, 'Duration')) if lines else None,
        'error': _extract_report_value(lines, 'Error') if lines else None,
        'error_code': _extract_report_value(lines, 'Error Code') if lines else None,
        'run_dir': run_dir,
        'run_report': report_path if os.path.isfile(report_path) else None,
        'forced_error_file': None,
        '_mtime': os.path.getmtime(report_path) if os.path.isfile(report_path) else os.path.getmtime(run_dir),
    }
    return row


def _load_existing_rows(output_root: str, drivers: List[str], target_laps: int) -> Dict[tuple[str, int], Dict[str, Any]]:
    latest_rows: Dict[tuple[str, int], Dict[str, Any]] = {}
    for driver in drivers:
        driver_dir = os.path.join(output_root, driver)
        if not os.path.isdir(driver_dir):
            continue
        for entry in sorted(glob(os.path.join(driver_dir, '*'))):
            if not os.path.isdir(entry):
                continue
            row = _load_existing_row_from_run_dir(driver, entry, target_laps)
            if row is None:
                continue
            key = (driver, int(row['trial']))
            prev = latest_rows.get(key)
            if prev is None or float(row.get('_mtime', 0.0)) >= float(prev.get('_mtime', 0.0)):
                latest_rows[key] = row
    for row in latest_rows.values():
        row.pop('_mtime', None)
    return latest_rows


def _load_first_error(run_dir: str) -> Optional[Dict[str, Any]]:
    files = sorted(glob(os.path.join(run_dir, 'error_*.json')))
    if not files:
        return None
    with open(files[0], 'r', encoding='utf-8') as f:
        return json.load(f)


def _count_current_laps(run_dir: str) -> int:
    laps_dir = os.path.join(run_dir, 'laps')
    if not os.path.isdir(laps_dir):
        return 0
    try:
        return len(glob(os.path.join(laps_dir, 'lap_*.json')))
    except Exception:
        return 0


def _forced_error_filename(reason: str) -> str:
    mapping = {
        'SAFETY_COLLISION_IMMEDIATE': 'error_safety_collision.json',
        'SAFETY_OFFTRACK_IMMEDIATE': 'error_safety_offtrack.json',
        'SAFETY_HALT_STALL': 'error_safety_halt_stall.json',
    }
    return mapping.get(reason, f"error_{reason.lower()}.json")


def _write_forced_error_json(run_dir: str, payload: Dict[str, Any]) -> Optional[str]:
    try:
        os.makedirs(run_dir, exist_ok=True)
        reason = str(payload.get('error', 'forced_error'))
        path = os.path.join(run_dir, _forced_error_filename(reason))
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        return path
    except Exception:
        return None


def _write_run_report(run_dir: str, row: Dict[str, Any], command: List[str], stdout_tail: str, stderr_tail: str):
    report_path = os.path.join(run_dir, 'run_report.md')
    os.makedirs(run_dir, exist_ok=True)

    lines = [
        '# Run Report',
        '',
        f"- Driver: `{row['driver']}`",
        f"- Run Tag: `{row['run_tag']}`",
        f"- Return Code: `{row['returncode']}`",
        f"- Duration: `{row['duration_s']:.1f}s`",
        f"- Target Laps: `{row['target_laps']}`",
        f"- Completed Laps: `{row.get('total_laps')}`",
        f"- Lap Completion: `{row.get('lap_completed')}`",
        f"- Avg Speed (m/s): `{row.get('avg_speed_mps')}`",
        f"- Max Speed (m/s): `{row.get('max_speed_mps')}`",
        f"- Avg Lap Time (s): `{row.get('avg_lap_time_s')}`",
        '',
        '## Command',
        '```bash',
        ' '.join(command),
        '```',
        '',
    ]

    if row.get('error'):
        lines.extend([
            '## Error',
            f"- Error: `{row.get('error')}`",
            f"- Error Code: `{row.get('error_code')}`",
            '',
        ])

    perf_report = os.path.join(run_dir, 'performance_report.txt')
    if os.path.exists(perf_report):
        lines.extend([
            '## Generated Files',
            f"- `performance_report.txt`",
            '',
        ])

    lines.extend([
        '## stdout tail',
        '```text',
        stdout_tail,
        '```',
        '',
        '## stderr tail',
        '```text',
        stderr_tail,
        '```',
    ])

    with open(report_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def _aggregate_rows(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(row['driver'], []).append(row)

    mean_fields = [
        'avg_total_laps',
        'avg_lap_time_s',
        'avg_speed_mps',
        'avg_rms_lateral_error_m',
        'avg_rms_heading_error_rad',
        'avg_rms_yaw_rate_radps',
        'avg_yaw_rate_std_radps',
        'avg_rms_yaw_accel_radps2',
        'avg_rms_steering_rate_radps',
        'avg_rms_steering_jerk_radps2',
        'avg_rms_longitudinal_accel_mps2',
        'avg_rms_longitudinal_jerk_mps3',
        'avg_lookahead_variance',
        'avg_path_efficiency',
        'avg_path_excess_ratio',
        'avg_offtrack_ratio',
        'avg_estop_count',
    ]
    peak_fields = {
        'peak_speed_mps': 'max_speed_mps',
    }

    out = []
    for driver, drows in sorted(grouped.items(), key=lambda x: x[0]):
        completed = [r for r in drows if r.get('lap_completed') is True]
        completion_rate = (len(completed) / len(drows) * 100.0) if drows else 0.0

        row = {
            'driver': driver,
            'runs': len(drows),
            'completion_rate_pct': completion_rate,
            'success_runs': len(completed),
            'failures': len(drows) - len(completed),
        }
        for field in mean_fields:
            source_field = 'total_laps' if field == 'avg_total_laps' else field
            values = [_safe_float(r.get(source_field)) for r in drows]
            row[field] = mean(v for v in values if v is not None) if any(v is not None for v in values) else None
        for field, source_field in peak_fields.items():
            values = [_safe_float(r.get(source_field)) for r in drows]
            row[field] = max(v for v in values if v is not None) if any(v is not None for v in values) else None
        out.append(row)
    return out


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return 'NA'
    if isinstance(value, bool):
        return 'Y' if value else 'N'
    if isinstance(value, (int, float)):
        return f'{value:.{digits}f}'
    return str(value)


def _write_table_files(output_root: str, rows: List[Dict[str, Any]], summary: List[Dict[str, Any]]):
    run_csv = os.path.join(output_root, 'comparison_runs.csv')
    summary_csv = os.path.join(output_root, 'comparison_summary.csv')
    summary_md = os.path.join(output_root, 'comparison_summary.md')

    run_fieldnames = [
        'driver', 'trial', 'run_tag', 'target_laps', 'total_laps', 'lap_completed',
        'avg_lap_time_s', 'avg_speed_mps', 'max_speed_mps', 'min_speed_mps',
        'avg_rms_lateral_error_m', 'max_lateral_error_m',
        'avg_rms_heading_error_rad',
        'avg_rms_yaw_rate_radps', 'avg_yaw_rate_std_radps', 'avg_rms_yaw_accel_radps2',
        'avg_rms_steering_rate_radps', 'avg_rms_steering_jerk_radps2',
        'avg_rms_longitudinal_accel_mps2', 'avg_rms_longitudinal_jerk_mps3',
        'avg_lookahead_variance',
        'avg_path_efficiency', 'avg_path_excess_ratio',
        'overall_offtrack_ratio', 'estop_count_total', 'returncode', 'duration_s',
        'error', 'error_code', 'run_dir', 'run_report',
    ]
    summary_fieldnames = [
        'driver', 'runs', 'success_runs', 'completion_rate_pct',
        'avg_total_laps', 'avg_lap_time_s', 'avg_speed_mps', 'peak_speed_mps',
        'avg_rms_lateral_error_m', 'avg_rms_heading_error_rad',
        'avg_rms_yaw_rate_radps', 'avg_yaw_rate_std_radps', 'avg_rms_yaw_accel_radps2',
        'avg_rms_steering_rate_radps', 'avg_rms_steering_jerk_radps2',
        'avg_rms_longitudinal_accel_mps2', 'avg_rms_longitudinal_jerk_mps3',
        'avg_lookahead_variance',
        'avg_path_efficiency', 'avg_path_excess_ratio',
        'avg_offtrack_ratio', 'avg_estop_count', 'failures',
    ]

    with open(run_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=run_fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in run_fieldnames})

    with open(summary_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
        writer.writeheader()
        for row in summary:
            writer.writerow({k: row.get(k) for k in summary_fieldnames})

    md_lines = [
        '# Driver Comparison Summary',
        '',
        '| driver | runs | success_runs | completion_rate(%) | avg_total_laps | avg_lap_time_s | avg_speed_mps | peak_speed_mps | avg_rms_lat(m) | avg_rms_head(rad) | avg_yaw_rate(rad/s) | yaw_std(rad/s) | avg_yaw_accel(rad/s2) | avg_steer_rate(rad/s) | steer_jerk(rad/s2) | long_accel(m/s2) | long_jerk(m/s3) | lookahead_var | path_eff | path_excess | avg_offtrack_ratio | avg_estop_count | failures |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for row in summary:
        md_lines.append(
            f"| {row['driver']} | {row['runs']} | {row['success_runs']} | {_fmt(row['completion_rate_pct'], 1)} | "
            f"{_fmt(row['avg_total_laps'])} | {_fmt(row['avg_lap_time_s'])} | {_fmt(row['avg_speed_mps'])} | "
            f"{_fmt(row['peak_speed_mps'])} | {_fmt(row['avg_rms_lateral_error_m'], 4)} | "
            f"{_fmt(row['avg_rms_heading_error_rad'], 4)} | {_fmt(row['avg_rms_yaw_rate_radps'], 4)} | "
            f"{_fmt(row['avg_yaw_rate_std_radps'], 4)} | {_fmt(row['avg_rms_yaw_accel_radps2'], 4)} | "
            f"{_fmt(row['avg_rms_steering_rate_radps'], 4)} | {_fmt(row['avg_rms_steering_jerk_radps2'], 4)} | "
            f"{_fmt(row['avg_rms_longitudinal_accel_mps2'], 4)} | {_fmt(row['avg_rms_longitudinal_jerk_mps3'], 4)} | "
            f"{_fmt(row['avg_lookahead_variance'], 4)} | "
            f"{_fmt(row['avg_path_efficiency'], 4)} | {_fmt(row['avg_path_excess_ratio'], 4)} | "
            f"{_fmt(row['avg_offtrack_ratio'], 4)} | {_fmt(row['avg_estop_count'], 3)} | {row['failures']} |"
        )

    md_lines.extend([
        '',
        '## Run Details',
        '',
        '| driver | trial | run_tag | target_laps | total_laps | completed | avg_speed | max_speed | avg_lap_time | rms_lat | rms_head | steer_rate | steer_jerk | yaw_rate | lookahead_var | path_eff | error |',
        '|---|---:|---|---:|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|',
    ])
    for row in rows:
        md_lines.append(
            f"| {row['driver']} | {row['trial']} | {row['run_tag']} | {row['target_laps']} | "
            f"{row.get('total_laps', 'NA')} | {_fmt(row.get('lap_completed'), 0)} | "
            f"{_fmt(_safe_float(row.get('avg_speed_mps')))} | {_fmt(_safe_float(row.get('max_speed_mps')))} | "
            f"{_fmt(_safe_float(row.get('avg_lap_time_s')))} | {_fmt(_safe_float(row.get('avg_rms_lateral_error_m')), 4)} | "
            f"{_fmt(_safe_float(row.get('avg_rms_heading_error_rad')), 4)} | {_fmt(_safe_float(row.get('avg_rms_steering_rate_radps')), 4)} | "
            f"{_fmt(_safe_float(row.get('avg_rms_steering_jerk_radps2')), 4)} | {_fmt(_safe_float(row.get('avg_rms_yaw_rate_radps')), 4)} | "
            f"{_fmt(_safe_float(row.get('avg_lookahead_variance')), 4)} | "
            f"{_fmt(_safe_float(row.get('avg_path_efficiency')), 4)} | {row.get('error', '') or ''} |"
        )

    with open(summary_md, 'w', encoding='utf-8') as f:
        f.write('\n'.join(md_lines) + '\n')

    return run_csv, summary_csv, summary_md


def _build_launch_command(config: Dict[str, Any], driver: str, run_tag: str) -> List[str]:
    eval_cfg = config['evaluation']
    rl_cfg = eval_cfg.get('rl', {})
    metrics_cfg = eval_cfg.get('metrics', {})
    pp_fixed_config_file = eval_cfg.get('pp_fixed_config_file', '')
    pp_adaptive_config_file = eval_cfg.get('pp_adaptive_config_file', '')
    kmpc_cfg = eval_cfg.get('kmpc', {})

    cmd = [
        'ros2', 'launch', 'f1tenth_driver_benchmark_suite', 'benchmark.launch.py',
        f'driver:={driver}',
        f'mode:={eval_cfg.get("mode", "headless")}',
        f'run_tag:={run_tag}',
        f'output_root:={eval_cfg["output_root"]}',
        f'scenario_file:={eval_cfg["scenario_file"]}',
        f'raceline_path:={eval_cfg["raceline_path"]}',
        f'min_laps:={eval_cfg["laps_per_run"]}',
        f'max_laps:={eval_cfg["laps_per_run"]}',
        f'auto_shutdown:={str(eval_cfg.get("auto_shutdown", True)).lower()}',
        f'use_realtime_visualizer:={str(eval_cfg.get("use_realtime_visualizer", False)).lower()}',
        f'metrics_package:={metrics_cfg.get("package", "f1tenth_driver_benchmark_suite")}',
        f'metrics_executable:={metrics_cfg.get("executable", "metrics_collector_node")}',
        f'metrics_visualizer_package:={metrics_cfg.get("visualizer_package", "f1tenth_driver_benchmark_suite")}',
        f'metrics_visualizer_executable:={metrics_cfg.get("visualizer_executable", "realtime_visualizer")}',
        f'metrics_common_file:={eval_cfg["metrics_common_file"]}',
        f'pp_fixed_config_file:={pp_fixed_config_file}',
        f'pp_adaptive_config_file:={pp_adaptive_config_file}',
        f'kmpc_config_file:={kmpc_cfg.get("config_file", "")}',
        f'kmpc_raceline_path:={kmpc_cfg.get("raceline_path", "")}',
        f'rl_checkpoint_dir:={rl_cfg.get("checkpoint_dir", "")}',
        f'rl_mode:={rl_cfg.get("mode", "lookahead")}',
        f'rl_rate_hz:={rl_cfg.get("rate_hz", 100.0)}',
        f'rl_vgain:={rl_cfg.get("vgain", 1.2)}',
        f'rl_min_speed:={rl_cfg.get("min_speed", 0.0)}',
        f'rl_max_speed:={rl_cfg.get("max_speed", 99.0)}',
        f'rl_lookahead_distance:={rl_cfg.get("lookahead_distance", 1.0)}',
        f'rl_local_index_search:={str(rl_cfg.get("local_index_search", False)).lower()}',
        f'rl_recovery_track_error:={rl_cfg.get("recovery_track_error", 0.6)}',
        f'rl_recovery_speed_max:={rl_cfg.get("recovery_speed_max", 1.8)}',
        f'rl_use_common_speed_constraints:={str(rl_cfg.get("use_common_speed_constraints", True)).lower()}',
        f'rl_pp_adv_a_lat_max:={rl_cfg.get("pp_adv_a_lat_max", 3.8)}',
        f'rl_pp_adv_slowdown_gain:={rl_cfg.get("pp_adv_slowdown_gain", 1.4)}',
        f'rl_offtrack_threshold:={rl_cfg.get("offtrack_threshold", 1.8)}',
        f'rl_offtrack_hold_steps:={rl_cfg.get("offtrack_hold_steps", 12)}',
        f'rl_collision_scan_threshold:={rl_cfg.get("collision_scan_threshold", 0.16)}',
        f'rl_collision_hold_steps:={rl_cfg.get("collision_hold_steps", 5)}',
    ]
    return cmd


def _resolve_defaults(config: Dict[str, Any]):
    eval_cfg = config.setdefault('evaluation', {})

    eval_cfg.setdefault('name', 'headless_eval')
    eval_cfg.setdefault('mode', 'headless')
    eval_cfg.setdefault('drivers', ['rl', 'pp_fixed', 'pp_adaptive'])
    eval_cfg.setdefault('trials_per_driver', 3)
    eval_cfg.setdefault('laps_per_run', 3)
    eval_cfg.setdefault('timeout_s', 1200)
    eval_cfg.setdefault('completion_shutdown_grace_s', 10.0)
    eval_cfg.setdefault('heartbeat_s', 10)
    eval_cfg.setdefault('fail_fast_on_collision', True)
    eval_cfg.setdefault('fail_fast_on_offtrack', False)
    eval_cfg.setdefault('fail_fast_on_safety_halt', True)
    eval_cfg.setdefault('safety_halt_grace_s', 8.0)
    eval_cfg.setdefault('safety_halt_speed_epsilon', 0.05)
    eval_cfg.setdefault('cooldown_s', 3.0)
    eval_cfg.setdefault('cleanup_between_trials', True)
    eval_cfg.setdefault('auto_shutdown', True)
    eval_cfg.setdefault('run_tag_prefix', eval_cfg['name'])
    eval_cfg.setdefault('use_realtime_visualizer', False)
    eval_cfg.setdefault('output_root', '/tmp/f1tenth_driver_benchmark_suite/results')
    eval_cfg.setdefault(
        'scenario_file',
        '/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/config/scenarios/budapest.yaml',
    )
    eval_cfg.setdefault(
        'raceline_path',
        '/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/assets/racelines/Budapest_optimal_rl.csv',
    )
    eval_cfg.setdefault(
        'metrics_common_file',
        '/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/config/benchmark_common.yaml',
    )
    eval_cfg.setdefault(
        'pp_fixed_config_file',
        '/home/jin/ros2_prj/sim_ws/src/pp_core/config/pure_pursuit.yaml',
    )
    eval_cfg.setdefault(
        'pp_adaptive_config_file',
        '/home/jin/ros2_prj/sim_ws/src/pp_adaptive/config/pure_pursuit.yaml',
    )

    eval_cfg.setdefault('metrics', {})
    eval_cfg['metrics'].setdefault('package', 'f1tenth_driver_benchmark_suite')
    eval_cfg['metrics'].setdefault('executable', 'metrics_collector_node')
    eval_cfg['metrics'].setdefault('visualizer_package', 'f1tenth_driver_benchmark_suite')
    eval_cfg['metrics'].setdefault('visualizer_executable', 'realtime_visualizer')

    eval_cfg.setdefault('kmpc', {})
    eval_cfg['kmpc'].setdefault(
        'raceline_path',
        '/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/assets/racelines/Budapest_optimal_rl_kmpc.csv',
    )
    eval_cfg['kmpc'].setdefault(
        'config_file',
        '/home/jin/ros2_prj/sim_ws/src/kmpc_driver/config/kmpc_budapest_benchmark_stable.yaml',
    )

    eval_cfg.setdefault('rl', {})
    eval_cfg['rl'].setdefault(
        'checkpoint_dir',
        '/home/jin/ros2_prj/rl_f1tenth/checkpoints/lookahead/vgain_curriculum_asafe_3lap/s6/best',
    )
    eval_cfg['rl'].setdefault('mode', 'lookahead')
    eval_cfg['rl'].setdefault('rate_hz', 100.0)
    eval_cfg['rl'].setdefault('vgain', 1.2)
    eval_cfg['rl'].setdefault('min_speed', 0.0)
    eval_cfg['rl'].setdefault('max_speed', 99.0)
    eval_cfg['rl'].setdefault('lookahead_distance', 1.0)
    eval_cfg['rl'].setdefault('local_index_search', False)
    eval_cfg['rl'].setdefault('recovery_track_error', 0.6)
    eval_cfg['rl'].setdefault('recovery_speed_max', 1.8)
    eval_cfg['rl'].setdefault('use_common_speed_constraints', True)
    eval_cfg['rl'].setdefault('pp_adv_a_lat_max', 3.8)
    eval_cfg['rl'].setdefault('pp_adv_slowdown_gain', 1.4)
    eval_cfg['rl'].setdefault('offtrack_threshold', 1.8)
    eval_cfg['rl'].setdefault('offtrack_hold_steps', 12)
    eval_cfg['rl'].setdefault('collision_scan_threshold', 0.16)
    eval_cfg['rl'].setdefault('collision_hold_steps', 5)

    if eval_cfg['mode'] != 'headless':
        raise ValueError("evaluation.mode must be 'headless' for this runner")


def _run_single(config: Dict[str, Any], driver: str, trial: int) -> Dict[str, Any]:
    eval_cfg = config['evaluation']
    run_tag = f"{eval_cfg['run_tag_prefix']}_{driver}_t{trial:02d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    run_dir = os.path.join(eval_cfg['output_root'], driver, run_tag)
    cmd = _build_launch_command(config, driver, run_tag)
    status_file = os.path.join(eval_cfg['output_root'], 'evaluation_status.json')

    started_at = time.time()
    timed_out = False
    timeout_s = int(eval_cfg['timeout_s'])
    completion_shutdown_grace_s = float(eval_cfg.get('completion_shutdown_grace_s', 10.0))
    heartbeat_s = int(eval_cfg.get('heartbeat_s', 10))
    fail_fast_on_collision = bool(eval_cfg.get('fail_fast_on_collision', True))
    fail_fast_on_offtrack = bool(eval_cfg.get('fail_fast_on_offtrack', False))
    fail_fast_on_safety_halt = bool(eval_cfg.get('fail_fast_on_safety_halt', True))
    safety_halt_grace_s = float(eval_cfg.get('safety_halt_grace_s', 8.0))
    safety_halt_speed_eps = float(eval_cfg.get('safety_halt_speed_epsilon', 0.05))

    stdout_lines: deque = deque(maxlen=4000)
    stderr_lines: deque = deque(maxlen=1200)
    runtime_state: Dict[str, Any] = {
        'safety_offtrack_count': 0,
        'safety_collision_count': 0,
        'lap_for_counts': None,
        'lap_base_offtrack_count': 0,
        'lap_base_collision_count': 0,
        'last_safety': None,
        'last_safety_line': None,
        'last_safety_time': None,
        'pose_x': None,
        'pose_y': None,
        'theta': None,
        'speed': None,
        'cmd_speed': None,
        'last_pose_time': None,
    }

    proc = subprocess.Popen(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    t_out = threading.Thread(
        target=_read_stream_lines,
        args=(proc.stdout, stdout_lines, runtime_state),
        daemon=True,
    )
    t_err = threading.Thread(target=_read_stream_lines, args=(proc.stderr, stderr_lines), daemon=True)
    t_out.start()
    t_err.start()

    _log(
        f"[eval][start] {driver} t{trial:02d} run_tag={run_tag} pid={proc.pid} "
        f"timeout={timeout_s}s"
    )
    _write_status_file(status_file, {
        'status': 'running',
        'driver': driver,
        'trial': trial,
        'run_tag': run_tag,
        'pid': proc.pid,
        'started_at': started_at,
        'elapsed_s': 0.0,
    })

    returncode = 1
    forced_error: Optional[str] = None
    forced_error_code: Optional[str] = None
    forced_error_context: Optional[Dict[str, Any]] = None
    safety_halt_started_at: Optional[float] = None
    completion_started_at: Optional[float] = None
    last_heartbeat = -heartbeat_s
    target_laps = int(eval_cfg['laps_per_run'])
    while True:
        now = time.time()
        elapsed = now - started_at
        rc = proc.poll()
        lap_now = _count_current_laps(run_dir)
        if lap_now >= target_laps:
            if completion_started_at is None:
                completion_started_at = now
        else:
            completion_started_at = None
        spd_now = runtime_state.get('speed')
        cmd_spd_now = runtime_state.get('cmd_speed')
        safety_off_now = int(runtime_state.get('safety_offtrack_count', 0))
        safety_col_now = int(runtime_state.get('safety_collision_count', 0))
        safety_triggered = (safety_off_now > 0) or (safety_col_now > 0)
        stalled_now = (
            safety_triggered
            and (lap_now < target_laps)
            and (spd_now is not None and abs(float(spd_now)) <= safety_halt_speed_eps)
            and (cmd_spd_now is not None and abs(float(cmd_spd_now)) <= safety_halt_speed_eps)
        )
        if stalled_now:
            if safety_halt_started_at is None:
                safety_halt_started_at = now
        else:
            safety_halt_started_at = None

        if elapsed - last_heartbeat >= heartbeat_s:
            last_heartbeat = elapsed
            px = runtime_state.get('pose_x')
            py = runtime_state.get('pose_y')
            spd = spd_now
            cmd_spd = cmd_spd_now
            safety_off = safety_off_now
            safety_col = safety_col_now
            last_safety = runtime_state.get('last_safety') or '-'
            lap_for_counts = runtime_state.get('lap_for_counts')
            if lap_for_counts != lap_now:
                runtime_state['lap_for_counts'] = lap_now
                runtime_state['lap_base_offtrack_count'] = safety_off
                runtime_state['lap_base_collision_count'] = safety_col
            lap_off = max(0, safety_off - int(runtime_state.get('lap_base_offtrack_count', 0)))
            lap_col = max(0, safety_col - int(runtime_state.get('lap_base_collision_count', 0)))
            if px is not None and py is not None and spd is not None and cmd_spd is not None:
                state_str = (
                    f"lap={lap_now}/{target_laps} "
                    f"pos=({px:.2f},{py:.2f}) v={spd:.2f} cmd_v={cmd_spd:.2f} "
                    f"safety(total_off={safety_off},total_col={safety_col},"
                    f"lap_off={lap_off},lap_col={lap_col},last={last_safety})"
                )
            else:
                state_str = (
                    f"lap={lap_now}/{target_laps} "
                    f"pos=(n/a,n/a) v=n/a cmd_v=n/a "
                    f"safety(total_off={safety_off},total_col={safety_col},"
                    f"lap_off={lap_off},lap_col={lap_col},last={last_safety})"
                )
            _log(
                f"[eval][running] {driver} t{trial:02d} run_tag={run_tag} "
                f"elapsed={elapsed:.1f}s pid={proc.pid} {state_str}"
            )
            _write_status_file(status_file, {
                'status': 'running',
                'driver': driver,
                'trial': trial,
                'run_tag': run_tag,
                'pid': proc.pid,
                'started_at': started_at,
                'elapsed_s': elapsed,
                'current_laps': lap_now,
                'pose_x': px,
                'pose_y': py,
                'speed_mps': spd,
                'cmd_speed_mps': cmd_spd,
                'safety_offtrack_count': safety_off,
                'safety_collision_count': safety_col,
                'safety_offtrack_count_current_lap': lap_off,
                'safety_collision_count_current_lap': lap_col,
                'last_safety': last_safety,
            })

        if rc is not None:
            returncode = rc
            break

        if (
            fail_fast_on_collision
            and safety_col_now > 0
            and (lap_now < target_laps)
        ):
            forced_error = 'SAFETY_COLLISION_IMMEDIATE'
            forced_error_code = 'SAFETY_COLLISION'
            lap_off_now = max(
                0,
                safety_off_now - int(runtime_state.get('lap_base_offtrack_count', 0))
            )
            lap_col_now = max(
                0,
                safety_col_now - int(runtime_state.get('lap_base_collision_count', 0))
            )
            forced_error_context = {
                'timestamp': now,
                'elapsed_s': elapsed,
                'current_laps': lap_now,
                'target_laps': int(eval_cfg['laps_per_run']),
                'position': [runtime_state.get('pose_x'), runtime_state.get('pose_y')],
                'yaw_rad': runtime_state.get('theta'),
                'speed_mps': runtime_state.get('speed'),
                'cmd_speed_mps': runtime_state.get('cmd_speed'),
                'safety_offtrack_count': safety_off_now,
                'safety_collision_count': safety_col_now,
                'safety_offtrack_count_current_lap': lap_off_now,
                'safety_collision_count_current_lap': lap_col_now,
                'last_safety': runtime_state.get('last_safety'),
                'last_safety_line': runtime_state.get('last_safety_line'),
            }
            _terminate_process_tree(proc)
            returncode = 125
            _log(
                f"[eval][fail-fast] {driver} t{trial:02d} run_tag={run_tag} "
                f"reason={forced_error} lap={lap_now}/{target_laps}"
            )
            break

        if (
            fail_fast_on_offtrack
            and safety_off_now > 0
            and (lap_now < target_laps)
        ):
            forced_error = 'SAFETY_OFFTRACK_IMMEDIATE'
            forced_error_code = 'SAFETY_OFFTRACK'
            lap_off_now = max(
                0,
                safety_off_now - int(runtime_state.get('lap_base_offtrack_count', 0))
            )
            lap_col_now = max(
                0,
                safety_col_now - int(runtime_state.get('lap_base_collision_count', 0))
            )
            forced_error_context = {
                'timestamp': now,
                'elapsed_s': elapsed,
                'current_laps': lap_now,
                'target_laps': int(eval_cfg['laps_per_run']),
                'position': [runtime_state.get('pose_x'), runtime_state.get('pose_y')],
                'yaw_rad': runtime_state.get('theta'),
                'speed_mps': runtime_state.get('speed'),
                'cmd_speed_mps': runtime_state.get('cmd_speed'),
                'safety_offtrack_count': safety_off_now,
                'safety_collision_count': safety_col_now,
                'safety_offtrack_count_current_lap': lap_off_now,
                'safety_collision_count_current_lap': lap_col_now,
                'last_safety': runtime_state.get('last_safety'),
                'last_safety_line': runtime_state.get('last_safety_line'),
            }
            _terminate_process_tree(proc)
            returncode = 125
            _log(
                f"[eval][fail-fast] {driver} t{trial:02d} run_tag={run_tag} "
                f"reason={forced_error} lap={lap_now}/{target_laps}"
            )
            break

        if (
            fail_fast_on_safety_halt
            and (safety_halt_started_at is not None)
            and ((now - safety_halt_started_at) >= safety_halt_grace_s)
        ):
            forced_error = 'SAFETY_HALT_STALL'
            forced_error_code = 'SAFETY_LATCH'
            lap_off_now = max(
                0,
                safety_off_now - int(runtime_state.get('lap_base_offtrack_count', 0))
            )
            lap_col_now = max(
                0,
                safety_col_now - int(runtime_state.get('lap_base_collision_count', 0))
            )
            forced_error_context = {
                'timestamp': now,
                'elapsed_s': elapsed,
                'current_laps': lap_now,
                'target_laps': int(eval_cfg['laps_per_run']),
                'position': [runtime_state.get('pose_x'), runtime_state.get('pose_y')],
                'yaw_rad': runtime_state.get('theta'),
                'speed_mps': runtime_state.get('speed'),
                'cmd_speed_mps': runtime_state.get('cmd_speed'),
                'safety_offtrack_count': safety_off_now,
                'safety_collision_count': safety_col_now,
                'safety_offtrack_count_current_lap': lap_off_now,
                'safety_collision_count_current_lap': lap_col_now,
                'last_safety': runtime_state.get('last_safety'),
                'last_safety_line': runtime_state.get('last_safety_line'),
                'stall_duration_s': now - safety_halt_started_at,
            }
            _terminate_process_tree(proc)
            returncode = 125
            _log(
                f"[eval][fail-fast] {driver} t{trial:02d} run_tag={run_tag} "
                f"reason={forced_error} after={now - safety_halt_started_at:.1f}s "
                f"lap={lap_now}/{target_laps}"
            )
            break

        if elapsed >= timeout_s:
            if (
                completion_started_at is not None
                and (elapsed < (timeout_s + completion_shutdown_grace_s))
            ):
                time.sleep(1.0)
                continue
            timed_out = True
            _terminate_process_tree(proc, term_timeout_s=8.0, kill_timeout_s=3.0)
            returncode = 124
            break

        time.sleep(1.0)

    t_out.join(timeout=1.0)
    t_err.join(timeout=1.0)
    ended_at = time.time()
    stdout_text = '\n'.join(list(stdout_lines))
    stderr_text = '\n'.join(list(stderr_lines))

    run_summary = _load_latest_json(os.path.join(run_dir, 'runs')) if os.path.isdir(run_dir) else None
    error_summary = _load_first_error(run_dir) if os.path.isdir(run_dir) else None
    forced_error_file = None
    if forced_error and (error_summary is None):
        payload = {
            'error': forced_error,
            'error_code': forced_error_code,
            'source': 'evaluation_runner_fail_fast',
            'driver': driver,
            'trial': trial,
            'run_tag': run_tag,
        }
        if forced_error_context:
            payload.update(forced_error_context)
        forced_error_file = _write_forced_error_json(run_dir, payload)
        error_summary = _load_first_error(run_dir) if os.path.isdir(run_dir) else None

    total_laps = int(run_summary.get('total_laps', 0)) if run_summary else 0
    completed = (total_laps >= target_laps) and (error_summary is None) and (forced_error is None)

    row = {
        'driver': driver,
        'trial': trial,
        'run_tag': run_tag,
        'target_laps': target_laps,
        'total_laps': total_laps,
        'lap_completed': completed,
        'avg_lap_time_s': run_summary.get('avg_lap_time_s') if run_summary else None,
        'avg_speed_mps': run_summary.get('avg_speed_mps') if run_summary else None,
        'max_speed_mps': run_summary.get('max_speed_mps') if run_summary else None,
        'min_speed_mps': run_summary.get('min_speed_mps') if run_summary else None,
        'avg_rms_lateral_error_m': run_summary.get('avg_rms_lateral_error_m') if run_summary else None,
        'max_lateral_error_m': run_summary.get('max_lateral_error_m') if run_summary else None,
        'avg_rms_heading_error_rad': run_summary.get('avg_rms_heading_error_rad') if run_summary else None,
        'avg_rms_yaw_rate_radps': run_summary.get('avg_rms_yaw_rate_radps') if run_summary else None,
        'avg_yaw_rate_std_radps': run_summary.get('avg_yaw_rate_std_radps') if run_summary else None,
        'avg_rms_yaw_accel_radps2': run_summary.get('avg_rms_yaw_accel_radps2') if run_summary else None,
        'avg_rms_steering_rate_radps': run_summary.get('avg_rms_steering_rate_radps') if run_summary else None,
        'avg_rms_steering_jerk_radps2': run_summary.get('avg_rms_steering_jerk_radps2') if run_summary else None,
        'avg_rms_longitudinal_accel_mps2': run_summary.get('avg_rms_longitudinal_accel_mps2') if run_summary else None,
        'avg_rms_longitudinal_jerk_mps3': run_summary.get('avg_rms_longitudinal_jerk_mps3') if run_summary else None,
        'avg_lookahead_variance': run_summary.get('avg_lookahead_variance') if run_summary else None,
        'avg_path_efficiency': run_summary.get('avg_path_efficiency') if run_summary else None,
        'avg_path_excess_ratio': run_summary.get('avg_path_excess_ratio') if run_summary else None,
        'overall_offtrack_ratio': run_summary.get('overall_offtrack_ratio') if run_summary else None,
        'estop_count_total': run_summary.get('estop_count_total') if run_summary else None,
        'returncode': returncode,
        'duration_s': ended_at - started_at,
        'error': (
            error_summary.get('error')
            if error_summary
            else (forced_error if forced_error else ('TIMEOUT' if timed_out else None))
        ),
        'error_code': (
            error_summary.get('error_code')
            if error_summary
            else forced_error_code
        ),
        'run_dir': run_dir,
        'run_report': os.path.join(run_dir, 'run_report.md'),
        'forced_error_file': forced_error_file,
    }

    stdout_tail = '\n'.join(stdout_text.splitlines()[-30:])
    stderr_tail = '\n'.join(stderr_text.splitlines()[-30:])
    _write_run_report(run_dir, row, cmd, stdout_tail, stderr_tail)
    _write_status_file(status_file, {
        'status': 'finished',
        'driver': driver,
        'trial': trial,
        'run_tag': run_tag,
        'returncode': returncode,
        'timed_out': timed_out,
        'forced_error': forced_error,
        'elapsed_s': ended_at - started_at,
        'lap_completed': completed,
        'total_laps': total_laps,
    })
    return row


def main():
    parser = argparse.ArgumentParser(description='Config-based headless benchmark runner (m laps x n trials)')
    parser.add_argument(
        '--config',
        default='/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/config/tests/headless_3lap_n3.yaml',
    )
    parser.add_argument('--output-root', default='')
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f) or {}
    _resolve_defaults(config)

    eval_cfg = config['evaluation']
    if args.output_root:
        eval_cfg['output_root'] = args.output_root

    os.makedirs(eval_cfg['output_root'], exist_ok=True)

    rows: List[Dict[str, Any]] = []
    drivers = list(eval_cfg['drivers'])
    trials = int(eval_cfg['trials_per_driver'])
    resume_existing = bool(eval_cfg.get('resume_existing', False))
    existing_rows = _load_existing_rows(eval_cfg['output_root'], drivers, int(eval_cfg['laps_per_run'])) if resume_existing else {}

    _log(f"[eval] name={eval_cfg['name']} mode={eval_cfg['mode']} laps={eval_cfg['laps_per_run']} trials={trials}")
    _log(f"[eval] drivers={drivers}")
    _log(f"[eval] output_root={eval_cfg['output_root']}")
    _log(f"[eval] heartbeat={eval_cfg['heartbeat_s']}s")
    if resume_existing:
        reused = sum(1 for row in existing_rows.values() if row.get('lap_completed') is True)
        _log(f"[eval] resume_existing=True reused_completed_trials={reused}")

    cleanup_enabled = bool(eval_cfg.get('cleanup_between_trials', True))
    for driver in drivers:
        for trial in range(1, trials + 1):
            existing = existing_rows.get((driver, trial))
            if existing is not None and existing.get('lap_completed') is True:
                rows.append(existing)
                _log(
                    f"[eval][resume] reusing {driver} t{trial:02d} "
                    f"run_tag={existing['run_tag']} rc={existing['returncode']}"
                )
                continue
            if cleanup_enabled:
                _cleanup_between_trials(verbose=True)
            row = _run_single(config, driver, trial)
            rows.append(row)
            _log(
                f"[eval] {driver} t{trial:02d} laps={row['total_laps']}/{row['target_laps']} "
                f"completed={row['lap_completed']} rc={row['returncode']} dur={row['duration_s']:.1f}s"
            )
            time.sleep(float(eval_cfg['cooldown_s']))

    summary = _aggregate_rows(rows)
    run_csv, summary_csv, summary_md = _write_table_files(eval_cfg['output_root'], rows, summary)

    _log(f"[eval] wrote: {run_csv}")
    _log(f"[eval] wrote: {summary_csv}")
    _log(f"[eval] wrote: {summary_md}")
    _write_status_file(os.path.join(eval_cfg['output_root'], 'evaluation_status.json'), {
        'status': 'all_finished',
        'name': eval_cfg['name'],
        'drivers': drivers,
        'trials_per_driver': trials,
        'finished_at': time.time(),
        'summary_csv': summary_csv,
        'summary_md': summary_md,
    })


if __name__ == '__main__':
    main()
