#!/usr/bin/env python3
"""
성능 보고서 생성 스크립트
f1tenth_driver_benchmark_suite 실행 결과를 분석하여 성능 보고서를 생성합니다.
"""

import json
import csv
import os
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional
from datetime import datetime


def load_lap_summaries(output_dir: str) -> List[Dict[str, Any]]:
    """Load all lap summary JSON files."""
    laps_dir = os.path.join(output_dir, 'laps')
    if not os.path.exists(laps_dir):
        return []
    
    lap_files = sorted([f for f in os.listdir(laps_dir) if f.endswith('.json')])
    laps = []
    
    for lap_file in lap_files:
        filepath = os.path.join(laps_dir, lap_file)
        try:
            with open(filepath, 'r') as f:
                lap_data = json.load(f)
                laps.append(lap_data)
        except Exception as e:
            print(f"Warning: Failed to load {filepath}: {e}")
    
    return laps


def load_run_summary(output_dir: str) -> Optional[Dict[str, Any]]:
    """Load run summary JSON file."""
    runs_dir = os.path.join(output_dir, 'runs')
    if not os.path.exists(runs_dir):
        return None
    
    run_files = sorted([f for f in os.listdir(runs_dir) if f.endswith('.json')])
    if not run_files:
        return None
    
    # Get the most recent run summary
    filepath = os.path.join(runs_dir, run_files[-1])
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Failed to load {filepath}: {e}")
        return None


def load_error_summaries(output_dir: str) -> List[Dict[str, Any]]:
    """Load error summary JSON files."""
    error_files = [f for f in os.listdir(output_dir) if f.startswith('error_') and f.endswith('.json')]
    errors = []
    
    for error_file in error_files:
        filepath = os.path.join(output_dir, error_file)
        try:
            with open(filepath, 'r') as f:
                error_data = json.load(f)
                errors.append(error_data)
        except Exception as e:
            print(f"Warning: Failed to load {filepath}: {e}")
    
    return errors


def calculate_statistics(laps: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Calculate statistics from lap data."""
    if not laps:
        return {}
    
    stats = {}
    
    # Lap times
    lap_times = [lap.get('lap_time_s', 0) for lap in laps if not (isinstance(lap.get('lap_time_s'), float) and (lap.get('lap_time_s') != lap.get('lap_time_s')))]
    if lap_times:
        stats['lap_times'] = {
            'avg': sum(lap_times) / len(lap_times),
            'min': min(lap_times),
            'max': max(lap_times),
            'count': len(lap_times)
        }
    
    # RMS Lateral Errors
    rms_errors = [lap.get('rms_lateral_error_m', 0) for lap in laps if not (isinstance(lap.get('rms_lateral_error_m'), float) and (lap.get('rms_lateral_error_m') != lap.get('rms_lateral_error_m')))]
    if rms_errors:
        stats['rms_lateral_errors'] = {
            'avg': sum(rms_errors) / len(rms_errors),
            'min': min(rms_errors),
            'max': max(rms_errors)
        }
    
    # Max Lateral Errors
    max_errors = [lap.get('max_lateral_error_m', 0) for lap in laps if not (isinstance(lap.get('max_lateral_error_m'), float) and (lap.get('max_lateral_error_m') != lap.get('max_lateral_error_m')))]
    if max_errors:
        stats['max_lateral_errors'] = {
            'avg': sum(max_errors) / len(max_errors),
            'min': min(max_errors),
            'max': max(max_errors)
        }
    
    # Average Speeds
    avg_speeds = [lap.get('avg_speed_mps', 0) for lap in laps if not (isinstance(lap.get('avg_speed_mps'), float) and (lap.get('avg_speed_mps') != lap.get('avg_speed_mps')))]
    if avg_speeds:
        stats['avg_speeds'] = {
            'avg': sum(avg_speeds) / len(avg_speeds),
            'min': min(avg_speeds),
            'max': max(avg_speeds)
        }
    
    # Max Speeds
    max_speeds = [lap.get('max_speed_mps', 0) for lap in laps if not (isinstance(lap.get('max_speed_mps'), float) and (lap.get('max_speed_mps') != lap.get('max_speed_mps')))]
    if max_speeds:
        stats['max_speeds'] = {
            'avg': sum(max_speeds) / len(max_speeds),
            'min': min(max_speeds),
            'max': max(max_speeds)
        }
    
    # Min Speeds
    min_speeds = [lap.get('min_speed_mps', 0) for lap in laps if not (isinstance(lap.get('min_speed_mps'), float) and (lap.get('min_speed_mps') != lap.get('min_speed_mps')))]
    if min_speeds:
        stats['min_speeds'] = {
            'avg': sum(min_speeds) / len(min_speeds),
            'min': min(min_speeds),
            'max': max(min_speeds)
        }
    
    # Off-track Ratios
    offtrack_ratios = [lap.get('offtrack_ratio', 0) for lap in laps]
    if offtrack_ratios:
        stats['offtrack_ratios'] = {
            'avg': sum(offtrack_ratios) / len(offtrack_ratios),
            'min': min(offtrack_ratios),
            'max': max(offtrack_ratios)
        }
    
    # Steering Rates
    steering_rates = [lap.get('rms_steering_rate_radps', 0) for lap in laps if not (isinstance(lap.get('rms_steering_rate_radps'), float) and (lap.get('rms_steering_rate_radps') != lap.get('rms_steering_rate_radps')))]
    if steering_rates:
        stats['steering_rates'] = {
            'avg': sum(steering_rates) / len(steering_rates),
            'min': min(steering_rates),
            'max': max(steering_rates)
        }

    # Heading Errors
    heading_errors = [lap.get('rms_heading_error_rad', 0) for lap in laps if not (isinstance(lap.get('rms_heading_error_rad'), float) and (lap.get('rms_heading_error_rad') != lap.get('rms_heading_error_rad')))]
    if heading_errors:
        stats['heading_errors'] = {
            'avg': sum(heading_errors) / len(heading_errors),
            'min': min(heading_errors),
            'max': max(heading_errors)
        }

    # Yaw Rate Standard Deviation
    yaw_rate_std = [lap.get('yaw_rate_std_radps', 0) for lap in laps if not (isinstance(lap.get('yaw_rate_std_radps'), float) and (lap.get('yaw_rate_std_radps') != lap.get('yaw_rate_std_radps')))]
    if yaw_rate_std:
        stats['yaw_rate_std'] = {
            'avg': sum(yaw_rate_std) / len(yaw_rate_std),
            'min': min(yaw_rate_std),
            'max': max(yaw_rate_std)
        }

    # Steering Jerk
    steering_jerk = [lap.get('rms_steering_jerk_radps2', 0) for lap in laps if not (isinstance(lap.get('rms_steering_jerk_radps2'), float) and (lap.get('rms_steering_jerk_radps2') != lap.get('rms_steering_jerk_radps2')))]
    if steering_jerk:
        stats['steering_jerk'] = {
            'avg': sum(steering_jerk) / len(steering_jerk),
            'min': min(steering_jerk),
            'max': max(steering_jerk)
        }

    # Longitudinal Jerk
    longitudinal_jerk = [lap.get('rms_longitudinal_jerk_mps3', 0) for lap in laps if not (isinstance(lap.get('rms_longitudinal_jerk_mps3'), float) and (lap.get('rms_longitudinal_jerk_mps3') != lap.get('rms_longitudinal_jerk_mps3')))]
    if longitudinal_jerk:
        stats['longitudinal_jerk'] = {
            'avg': sum(longitudinal_jerk) / len(longitudinal_jerk),
            'min': min(longitudinal_jerk),
            'max': max(longitudinal_jerk)
        }

    # Path Efficiency
    path_efficiency = [lap.get('path_efficiency', 0) for lap in laps if not (isinstance(lap.get('path_efficiency'), float) and (lap.get('path_efficiency') != lap.get('path_efficiency')))]
    if path_efficiency:
        stats['path_efficiency'] = {
            'avg': sum(path_efficiency) / len(path_efficiency),
            'min': min(path_efficiency),
            'max': max(path_efficiency)
        }
    
    return stats


def generate_report(output_dir: str, report_path: str):
    """Generate performance report."""
    print(f"Loading data from: {output_dir}")
    
    # Load data
    laps = load_lap_summaries(output_dir)
    run_summary = load_run_summary(output_dir)
    errors = load_error_summaries(output_dir)
    stats = calculate_statistics(laps)
    
    # Generate report
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("F1TENTH Driver Benchmark 성능 평가 보고서")
    report_lines.append("=" * 80)
    report_lines.append("")
    report_lines.append(f"생성 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"출력 디렉토리: {output_dir}")
    report_lines.append("")
    
    # Summary
    report_lines.append("=" * 80)
    report_lines.append("1. 전체 요약")
    report_lines.append("=" * 80)
    report_lines.append("")
    
    if run_summary:
        report_lines.append(f"총 완주 Lap 수: {run_summary.get('total_laps', len(laps))}")
        report_lines.append(f"총 주행 거리: {run_summary.get('total_distance_m', 0):.2f} m")
        report_lines.append(f"전체 Off-track 비율: {run_summary.get('overall_offtrack_ratio', 0)*100:.2f}%")
        report_lines.append(f"E-stop 발생 횟수: {run_summary.get('estop_count_total', 0)}")
        if 'avg_lap_time_s' in run_summary:
            report_lines.append(f"평균 Lap 시간: {run_summary['avg_lap_time_s']:.2f} s")
            report_lines.append(f"최소 Lap 시간: {run_summary.get('min_lap_time_s', 0):.2f} s")
            report_lines.append(f"최대 Lap 시간: {run_summary.get('max_lap_time_s', 0):.2f} s")
        if 'avg_rms_heading_error_rad' in run_summary:
            report_lines.append(f"평균 RMS Heading Error: {run_summary.get('avg_rms_heading_error_rad', float('nan')):.4f} rad")
        if 'avg_yaw_rate_std_radps' in run_summary:
            report_lines.append(f"평균 Yaw Rate Std: {run_summary.get('avg_yaw_rate_std_radps', float('nan')):.4f} rad/s")
        if 'avg_rms_steering_jerk_radps2' in run_summary:
            report_lines.append(f"평균 RMS Steering Jerk: {run_summary.get('avg_rms_steering_jerk_radps2', float('nan')):.4f} rad/s²")
        if 'avg_rms_longitudinal_jerk_mps3' in run_summary:
            report_lines.append(f"평균 RMS Longitudinal Jerk: {run_summary.get('avg_rms_longitudinal_jerk_mps3', float('nan')):.4f} m/s³")
        if 'avg_path_efficiency' in run_summary:
            report_lines.append(f"평균 Path Efficiency: {run_summary.get('avg_path_efficiency', float('nan')):.4f}")
    else:
        report_lines.append(f"완주 Lap 수: {len(laps)}")
    
    report_lines.append("")
    
    # Errors
    if errors:
        report_lines.append("=" * 80)
        report_lines.append("2. 오류 발생 내역")
        report_lines.append("=" * 80)
        report_lines.append("")
        for error in errors:
            report_lines.append(f"오류 유형: {error.get('error', 'UNKNOWN')}")
            report_lines.append(f"오류 코드: {error.get('error_code', 'N/A')}")
            report_lines.append(f"발생 시간: {error.get('timestamp', 'N/A')}")
            if 'final_position' in error and error['final_position']:
                report_lines.append(f"최종 위치: ({error['final_position'][0]:.2f}, {error['final_position'][1]:.2f})")
            report_lines.append("")
    
    # Statistics
    if stats:
        report_lines.append("=" * 80)
        report_lines.append("3. 통계 분석")
        report_lines.append("=" * 80)
        report_lines.append("")
        
        if 'lap_times' in stats:
            lt = stats['lap_times']
            report_lines.append(f"Lap 시간:")
            report_lines.append(f"  평균: {lt['avg']:.2f} s")
            report_lines.append(f"  최소: {lt['min']:.2f} s")
            report_lines.append(f"  최대: {lt['max']:.2f} s")
            report_lines.append("")
        
        if 'rms_lateral_errors' in stats:
            rms = stats['rms_lateral_errors']
            report_lines.append(f"RMS Lateral Error:")
            report_lines.append(f"  평균: {rms['avg']:.4f} m")
            report_lines.append(f"  최소: {rms['min']:.4f} m")
            report_lines.append(f"  최대: {rms['max']:.4f} m")
            report_lines.append("")
        
        if 'max_lateral_errors' in stats:
            max_e = stats['max_lateral_errors']
            report_lines.append(f"Max Lateral Error:")
            report_lines.append(f"  평균: {max_e['avg']:.4f} m")
            report_lines.append(f"  최소: {max_e['min']:.4f} m")
            report_lines.append(f"  최대: {max_e['max']:.4f} m")
            report_lines.append("")
        
        if 'avg_speeds' in stats:
            spd = stats['avg_speeds']
            report_lines.append(f"평균 속도:")
            report_lines.append(f"  평균: {spd['avg']:.2f} m/s")
            report_lines.append(f"  최소: {spd['min']:.2f} m/s")
            report_lines.append(f"  최대: {spd['max']:.2f} m/s")
            report_lines.append("")
        
        if 'max_speeds' in stats:
            max_spd = stats['max_speeds']
            report_lines.append(f"최고 속도:")
            report_lines.append(f"  평균: {max_spd['avg']:.2f} m/s")
            report_lines.append(f"  최소: {max_spd['min']:.2f} m/s")
            report_lines.append(f"  최대: {max_spd['max']:.2f} m/s")
            report_lines.append("")
        
        if 'min_speeds' in stats:
            min_spd = stats['min_speeds']
            report_lines.append(f"최저 속도:")
            report_lines.append(f"  평균: {min_spd['avg']:.2f} m/s")
            report_lines.append(f"  최소: {min_spd['min']:.2f} m/s")
            report_lines.append(f"  최대: {min_spd['max']:.2f} m/s")
            report_lines.append("")
        
        if 'offtrack_ratios' in stats:
            off = stats['offtrack_ratios']
            report_lines.append(f"Off-track 비율:")
            report_lines.append(f"  평균: {off['avg']*100:.2f}%")
            report_lines.append(f"  최소: {off['min']*100:.2f}%")
            report_lines.append(f"  최대: {off['max']*100:.2f}%")
            report_lines.append("")
        
        if 'steering_rates' in stats:
            sr = stats['steering_rates']
            report_lines.append(f"RMS Steering Rate:")
            report_lines.append(f"  평균: {sr['avg']:.4f} rad/s")
            report_lines.append(f"  최소: {sr['min']:.4f} rad/s")
            report_lines.append(f"  최대: {sr['max']:.4f} rad/s")
            report_lines.append("")

        if 'heading_errors' in stats:
            he = stats['heading_errors']
            report_lines.append(f"RMS Heading Error:")
            report_lines.append(f"  평균: {he['avg']:.4f} rad")
            report_lines.append(f"  최소: {he['min']:.4f} rad")
            report_lines.append(f"  최대: {he['max']:.4f} rad")
            report_lines.append("")

        if 'yaw_rate_std' in stats:
            ys = stats['yaw_rate_std']
            report_lines.append(f"Yaw Rate Std:")
            report_lines.append(f"  평균: {ys['avg']:.4f} rad/s")
            report_lines.append(f"  최소: {ys['min']:.4f} rad/s")
            report_lines.append(f"  최대: {ys['max']:.4f} rad/s")
            report_lines.append("")

        if 'steering_jerk' in stats:
            sj = stats['steering_jerk']
            report_lines.append(f"RMS Steering Jerk:")
            report_lines.append(f"  평균: {sj['avg']:.4f} rad/s²")
            report_lines.append(f"  최소: {sj['min']:.4f} rad/s²")
            report_lines.append(f"  최대: {sj['max']:.4f} rad/s²")
            report_lines.append("")

        if 'longitudinal_jerk' in stats:
            lj = stats['longitudinal_jerk']
            report_lines.append(f"RMS Longitudinal Jerk:")
            report_lines.append(f"  평균: {lj['avg']:.4f} m/s³")
            report_lines.append(f"  최소: {lj['min']:.4f} m/s³")
            report_lines.append(f"  최대: {lj['max']:.4f} m/s³")
            report_lines.append("")

        if 'path_efficiency' in stats:
            pe = stats['path_efficiency']
            report_lines.append(f"Path Efficiency:")
            report_lines.append(f"  평균: {pe['avg']:.4f}")
            report_lines.append(f"  최소: {pe['min']:.4f}")
            report_lines.append(f"  최대: {pe['max']:.4f}")
            report_lines.append("")
    
    # Lap Details
    if laps:
        report_lines.append("=" * 80)
        report_lines.append("4. Lap별 상세 메트릭")
        report_lines.append("=" * 80)
        report_lines.append("")
        
        for lap in laps:
            lap_num = lap.get('lap', 'N/A')
            report_lines.append(f"Lap {lap_num}:")
            report_lines.append(f"  Lap 시간: {lap.get('lap_time_s', 0):.2f} s")
            report_lines.append(f"  평균 속도: {lap.get('avg_speed_mps', 0):.2f} m/s")
            report_lines.append(f"  최고 속도: {lap.get('max_speed_mps', 0):.2f} m/s")
            report_lines.append(f"  최저 속도: {lap.get('min_speed_mps', 0):.2f} m/s")
            report_lines.append(f"  RMS Lateral Error: {lap.get('rms_lateral_error_m', 0):.4f} m")
            report_lines.append(f"  Max Lateral Error: {lap.get('max_lateral_error_m', 0):.4f} m")
            report_lines.append(f"  RMS Steering Rate: {lap.get('rms_steering_rate_radps', 0):.4f} rad/s")
            report_lines.append(f"  Off-track 비율: {lap.get('offtrack_ratio', 0)*100:.2f}%")
            report_lines.append(f"  주행 거리: {lap.get('lap_distance_m', 0):.2f} m")
            report_lines.append("")
    
    # Conclusion
    report_lines.append("=" * 80)
    report_lines.append("5. 결론")
    report_lines.append("=" * 80)
    report_lines.append("")
    
    if errors:
        report_lines.append("⚠️  평가 중 오류가 발생했습니다:")
        for error in errors:
            report_lines.append(f"  - {error.get('error', 'UNKNOWN')} (코드: {error.get('error_code', 'N/A')})")
        report_lines.append("")
    
    if len(laps) >= 3:
        report_lines.append("✅ 최소 3 Lap을 완주했습니다.")
    elif len(laps) > 0:
        report_lines.append(f"⚠️  {len(laps)} Lap만 완주했습니다 (목표: 3 Lap).")
    else:
        report_lines.append("❌ 완주한 Lap이 없습니다.")
    
    report_lines.append("")
    
    if stats and 'offtrack_ratios' in stats:
        avg_offtrack = stats['offtrack_ratios']['avg']
        if avg_offtrack < 0.01:
            report_lines.append("✅ 트랙 이탈이 거의 없었습니다 (Off-track 비율 < 1%).")
        elif avg_offtrack < 0.05:
            report_lines.append("⚠️  일부 트랙 이탈이 있었습니다 (Off-track 비율 < 5%).")
        else:
            report_lines.append("❌ 트랙 이탈이 많았습니다 (Off-track 비율 >= 5%).")
        report_lines.append("")
    
    report_lines.append("=" * 80)
    report_lines.append("보고서 끝")
    report_lines.append("=" * 80)
    
    # Write report
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(report_lines))
    
    print(f"✅ 성능 보고서 생성 완료: {report_path}")
    
    # Also print to console
    print('\n'.join(report_lines))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 generate_performance_report.py <output_dir> [report_path]")
        print("Example: python3 generate_performance_report.py /tmp/f1tenth_metrics/pure_pursuit_eval_20260114_200226")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    if len(sys.argv) >= 3:
        report_path = sys.argv[2]
    else:
        report_path = os.path.join(output_dir, 'performance_report.txt')
    
    if not os.path.exists(output_dir):
        print(f"Error: Output directory does not exist: {output_dir}")
        sys.exit(1)
    
    generate_report(output_dir, report_path)


if __name__ == '__main__':
    main()
