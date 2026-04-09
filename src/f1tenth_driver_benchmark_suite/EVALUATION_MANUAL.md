# Evaluation Manual

`f1tenth_driver_benchmark_suite` is the packaged evaluation layer used by the repository-level scripts.

## Minimal Flow

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run f1tenth_driver_benchmark_suite evaluation_runner --config config/tests/headless_3lap_n3.yaml
```

## Outputs

- per-run reports under `<output_root>/<driver>/<run_tag>/`
- aggregated CSV and Markdown summaries under `<output_root>/`

## Recommendation

Use `experiments/run_main_benchmark.sh`, `experiments/run_ablation.sh`, or `experiments/run_sensitivity.sh` unless a package-level debug session is needed.
