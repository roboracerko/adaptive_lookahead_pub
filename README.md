# Adaptive Lookahead Benchmark Workspace

This repository contains an anonymized implementation of adaptive lookahead selection methods for F1TENTH-style simulation.
It is organized for review-time reproducibility: build the workspace, run the benchmark scripts, and compare the outputs against the bundled reference summaries.

## Scope

- 5 lookahead-selection methods
- 6 benchmark maps
- main benchmark, ablation, and sensitivity sweeps
- analysis scripts for summary statistics and figure regeneration

## Quick Start

### 1. Build the workspace

```bash
cd adaptive_lookahead_pub
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r analysis/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

### 2. Run the benchmark

```bash
# Full 6-map benchmark
bash experiments/run_main_benchmark.sh --trials 30

# Fast smoke test
bash experiments/run_main_benchmark.sh --method kw_sim --map budapest --trials 3
```

### 3. Recompute summary statistics

```bash
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/main
```

### 4. Regenerate figures

```bash
python3 analysis/gen_figures.py --output-dir results/figures
```

## Repository Layout

```text
adaptive_lookahead_pub/
├── analysis/      # statistics and figure generation
├── docs/          # setup, experiment, anonymization notes
├── experiments/   # benchmark entrypoints
├── results/       # bundled reference summaries
├── src/           # ROS2 packages and simulator bridge
└── tools/         # audit helpers for review release hygiene
```

## Methods Included

| Code | Role | Summary |
|---|---|---|
| `kw_sim` | proposed variant | curvature-window filtering with simulated lateral-acceleration screening |
| `alg1_org` | baseline | faithful reproduction of a prior adaptive-lookahead procedure |
| `alg1_ext` | baseline | `alg1_org` with an expanded candidate set |
| `kv` | baseline | speed-proportional lookahead |
| `curv` | baseline | curvature-conditioned lookahead |

## Reproducibility Notes

- The benchmark scripts patch packaged configuration files at runtime so the workspace can be run from any checkout path.
- Reference summaries are bundled under `results/`.
- Review-time anonymization guidance is documented in `docs/ANONYMIZATION.md`.

## Report Reproduction

The dated `sim_ws/src/pp_adaptive/report` tree is covered by dedicated wrappers under `experiments/report_reproduction/`.

```bash
# Static coverage audit against the original report tree
python3 experiments/report_reproduction/check_report_reproducibility.py

# Regenerate derived racelines needed by revision and ablation experiments
bash experiments/report_reproduction/generate_report_racelines.sh --family all

# report/revision/{alg1_org,alg1_ext,kv,curv}/*
bash experiments/report_reproduction/run_revision_report.sh

# report/f1tenth_eval_20260322/BENCHMARK_STATISTICAL_ANALYSIS_3ALG_6MAP_N30.md
bash experiments/report_reproduction/run_legacy_3alg_report.sh

# ablation-study report set
bash experiments/report_reproduction/run_ablation_report.sh

# report/sensitivity/* and a regenerated SUMMARY.csv
bash experiments/report_reproduction/run_sensitivity_report.sh

# analysis figure bundle derived from the reproduced results/racelines
bash experiments/report_reproduction/run_report_figures.sh
```

Each wrapper is scoped to one experiment family.

- `run_revision_report.sh`: reruns the 24 revision-baseline evaluations used for `report/revision`.
- `run_legacy_3alg_report.sh`: reruns the legacy 3-algorithm benchmark family (`pp_fixed`, `kw2`, `alg1_org`) referenced by the dated benchmark report.
- `run_ablation_report.sh`: reruns the `kw_sim` + `[A,B,C,D]` ablation study after regenerating the required ablation racelines.
- `run_sensitivity_report.sh`: reruns the Budapest N=10 sensitivity sweep and writes a fresh `SUMMARY.csv`.
- `run_report_figures.sh`: regenerates the provided figure set from `analysis/gen_figures.py`.
- `check_report_reproducibility.py`: verifies that the public repository still has the scripts, configs, and bundled assets needed to cover those report families.

Notes:

- The archived timestamped result folders in `sim_ws/src/pp_adaptive/report` are not copied one-to-one into this repository. The wrappers reproduce the underlying experiment families with new output roots under `/tmp/f1tenth_eval/...`.
- The figure wrapper regenerates the maintained analysis figures, not byte-identical copies of the archived `report/figures/*` filenames.

## Review Release Note

Submission-specific citation metadata is intentionally omitted from this anonymized review package.
