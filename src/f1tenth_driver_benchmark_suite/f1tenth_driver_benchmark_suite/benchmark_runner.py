#!/usr/bin/env python3

import argparse
import glob as _glob
import json
import os
import subprocess
import time
from datetime import datetime
from typing import List


def _cleanup_between_trials(verbose: bool = False) -> None:
    """trial 전 DDS 공유 메모리 및 잔여 프로세스를 정리한다."""
    # 1. 잔여 시뮬레이터/드라이버 프로세스 종료
    for pattern in ['gym_bridge', 'metrics_collector_node', 'map_server',
                    'lifecycle_manager', 'robot_state_publisher']:
        subprocess.run(['pkill', '-f', pattern], capture_output=True)
    time.sleep(1.5)

    # 2. Fast-DDS 공유 메모리 세그먼트 제거
    removed = 0
    for path in _glob.glob('/dev/shm/fastrtps_*'):
        try:
            os.remove(path)
            removed += 1
        except OSError:
            pass
    if verbose and removed:
        print(f'[cleanup] removed {removed} fastrtps shm segments')

    # 3. ROS2 데몬 재시작 (stale 그래프 상태 초기화)
    subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True)
    time.sleep(0.5)
    subprocess.run(['ros2', 'daemon', 'start'], capture_output=True)
    time.sleep(0.5)


def _run_once(driver: str, trial: int, args) -> dict:
    run_tag = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_t{trial:02d}"

    cmd = [
        'ros2', 'launch', 'f1tenth_driver_benchmark_suite', 'benchmark.launch.py',
        f'driver:={driver}',
        f'mode:={args.mode}',
        f'run_tag:={run_tag}',
        f'output_root:={args.output_root}',
        f'rl_checkpoint_dir:={args.rl_checkpoint_dir}',
    ]

    if args.scenario_file:
        cmd.append(f'scenario_file:={args.scenario_file}')
    if args.raceline_path:
        cmd.append(f'raceline_path:={args.raceline_path}')

    started_at = time.time()
    ret = subprocess.run(cmd, text=True, capture_output=True, timeout=args.timeout_s)
    ended_at = time.time()

    return {
        'driver': driver,
        'trial': trial,
        'run_tag': run_tag,
        'returncode': ret.returncode,
        'duration_s': ended_at - started_at,
        'stdout_tail': '\n'.join(ret.stdout.splitlines()[-20:]),
        'stderr_tail': '\n'.join(ret.stderr.splitlines()[-20:]),
    }


def _parse_drivers(raw: str) -> List[str]:
    drivers = [x.strip() for x in raw.split(',') if x.strip()]
    allowed = {'rl', 'pp_fixed', 'pp_adaptive'}
    bad = [d for d in drivers if d not in allowed]
    if bad:
        raise ValueError(f'Unsupported driver(s): {bad}. allowed={sorted(allowed)}')
    return drivers


def main():
    parser = argparse.ArgumentParser(description='Batch benchmark runner for F1TENTH drivers')
    parser.add_argument('--drivers', default='rl,pp_fixed,pp_adaptive')
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--mode', default='headless', choices=['headless', 'visual'])
    parser.add_argument('--output-root', default='/tmp/f1tenth_driver_benchmark_suite/results')
    parser.add_argument('--timeout-s', type=int, default=1200)
    parser.add_argument('--cooldown-s', type=float, default=3.0)
    parser.add_argument('--no-cleanup', action='store_true',
                        help='trial 간 DDS/프로세스 클린업을 비활성화')
    parser.add_argument('--scenario-file', default='')
    parser.add_argument('--raceline-path', default='')
    parser.add_argument(
        '--rl-checkpoint-dir',
        default=os.environ.get('RL_CHECKPOINT_DIR', ''),
    )
    args = parser.parse_args()

    os.makedirs(args.output_root, exist_ok=True)
    drivers = _parse_drivers(args.drivers)

    results = []
    for driver in drivers:
        for trial in range(1, args.trials + 1):
            if not args.no_cleanup:
                _cleanup_between_trials(verbose=True)
            result = _run_once(driver, trial, args)
            results.append(result)
            print(f"[runner] {driver} t{trial:02d} rc={result['returncode']} dur={result['duration_s']:.1f}s")
            time.sleep(args.cooldown_s)

    summary_path = os.path.join(args.output_root, 'runner_summary.json')
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump({'runs': results}, f, indent=2)

    print(f'[runner] summary saved: {summary_path}')


if __name__ == '__main__':
    main()
