# f1tenth_driver_benchmark_suite

This package provides the repeatable evaluation harness used by the workspace-level benchmark scripts.

## Responsibilities

- launch a selected driver under a shared scenario
- collect run-level metrics and summaries
- aggregate repeated trials into comparison tables

## Typical Entry Points

```bash
ros2 run f1tenth_driver_benchmark_suite evaluation_runner --config config/tests/headless_3lap_n3.yaml
ros2 run f1tenth_driver_benchmark_suite report_aggregator --output-root /tmp/f1tenth_driver_benchmark_suite/results
```

Repository-level users should normally call the wrappers in `experiments/` instead of invoking this package directly.
