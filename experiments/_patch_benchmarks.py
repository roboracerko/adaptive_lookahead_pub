#!/usr/bin/env python3
"""
benchmark yaml 파일들의 절대경로를 현재 workspace 기반 경로로 치환한다.
experiments/run_*.sh 에서 내부적으로 호출됨.

Usage:
    python3 _patch_benchmarks.py --ws-root /path/to/adaptive_lookahead_pub
                                  --results-root /tmp/f1tenth_eval
                                  --input config/benchmarks/main/kw_sim_budapest_n30.yaml
                                  --output /tmp/_patched_kw_sim_budapest_n30.yaml
"""
import argparse
import os
import yaml


def resolve_share(pkg_name: str) -> str:
    """ros2 pkg prefix 로 share 경로를 반환한다."""
    import subprocess
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', pkg_name],
        capture_output=True, text=True
    )
    prefix = result.stdout.strip()
    return os.path.join(prefix, 'share', pkg_name)


def patch(input_yaml: str, output_yaml: str, ws_root: str, results_root: str) -> None:
    pp_share = resolve_share('pp_adaptive')
    bench_share = resolve_share('f1tenth_driver_benchmark_suite')

    with open(input_yaml) as f:
        cfg = yaml.safe_load(f)

    ev = cfg.get('evaluation', cfg)

    # --- scenario_file ---
    sf = ev.get('scenario_file', '')
    if sf:
        # e.g. .../config/scenarios/budapest_map.yaml
        basename = os.path.basename(sf)
        ev['scenario_file'] = os.path.join(bench_share, 'config', 'scenarios', basename)

    # --- metrics_common_file ---
    mf = ev.get('metrics_common_file', '')
    if mf:
        ev['metrics_common_file'] = os.path.join(bench_share, 'config', 'benchmark_common.yaml')

    # --- raceline_path ---
    rp = ev.get('raceline_path', '')
    if rp:
        basename = os.path.basename(rp)
        if '/racelines/' in rp and '/sensitivity/' not in rp:
            # main/ablation raceline: racelines/ 에서 직접
            ev['raceline_path'] = os.path.join(pp_share, 'racelines', basename)
        elif '/sensitivity/' in rp:
            # sensitivity sweep raceline: racelines/sensitivity/{param}/ 에서
            # 경로 패턴: .../report/sensitivity/{param}/sensitivity_{param}_{val}.csv
            path_parts = rp.replace('\\', '/').split('/')
            try:
                sens_idx = path_parts.index('sensitivity')
                param_name = path_parts[sens_idx + 1]  # e.g. 'alat'
                ev['raceline_path'] = os.path.join(
                    pp_share, 'racelines', 'sensitivity', param_name, basename
                )
            except (ValueError, IndexError):
                pass  # 패턴 불일치 시 원본 유지

    # --- pp_adaptive_config_file ---
    cf = ev.get('pp_adaptive_config_file', '')
    if cf:
        basename = os.path.basename(cf)
        # methods/ 에 있는 파일로 매핑
        candidate = os.path.join(pp_share, 'config', 'methods', basename)
        if os.path.exists(candidate):
            ev['pp_adaptive_config_file'] = candidate
        else:
            # sensitivity yaml 등 config/benchmarks/sensitivity/ 에서 찾기
            for subdir in ['sensitivity', 'ablation', '']:
                candidate = os.path.join(pp_share, 'config', 'benchmarks', subdir, basename) if subdir else \
                            os.path.join(pp_share, 'config', basename)
                if os.path.exists(candidate):
                    ev['pp_adaptive_config_file'] = candidate
                    break

    # --- output_root ---
    name = ev.get('name', 'unknown')
    ev['output_root'] = os.path.join(results_root, name)

    os.makedirs(os.path.dirname(output_yaml) or '.', exist_ok=True)
    with open(output_yaml, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)

    print(f'[patch] {input_yaml} → {output_yaml}')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--ws-root', required=True)
    p.add_argument('--results-root', default='/tmp/f1tenth_eval')
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()

    patch(args.input, args.output, args.ws_root, args.results_root)
