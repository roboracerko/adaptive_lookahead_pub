# Experiment Guide

## Environment

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Main Benchmark

```bash
# 5 methods x 6 maps
bash experiments/run_main_benchmark.sh --trials 30

# Subset run
bash experiments/run_main_benchmark.sh --method kw_sim --map budapest --trials 5

# Statistics
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/main
```

## Ablation

```bash
bash experiments/run_ablation.sh --trials 30
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/ablation
```

## Sensitivity Sweep

```bash
bash experiments/run_sensitivity.sh
bash experiments/run_sensitivity.sh --param alpha
python3 analysis/gen_figures.py --output-dir results/figures
```

## Raceline Regeneration

```bash
bash experiments/generate_racelines.sh --method kw_sim --map budapest
```

## Result Layout

```text
/tmp/f1tenth_eval/
├── main/
├── ablation/
└── sensitivity/
```
