# Review Release Checklist

This file tracks the intended structure of the anonymized review package.

## Goals

- keep author-identifying metadata out of the tracked files
- make the benchmark runnable from the repository root
- bundle enough assets and scripts to reproduce the reported comparisons

## Package Roles

| Path | Purpose |
|---|---|
| `src/pp_adaptive` | adaptive-lookahead controller and map-specific method configs |
| `src/pp_core` | fixed-lookahead baseline controller |
| `src/f1tenth_driver_benchmark_suite` | repeatable evaluation runner and aggregation utilities |
| `src/f1tenth_gym_ros` | simulator bridge used by the evaluation scripts |
| `experiments/` | top-level scripts for the main, ablation, and sensitivity studies |
| `analysis/` | statistics and figure generation |
| `results/` | bundled reference summaries |

## Release Checks

- top-level docs use neutral wording
- package metadata uses anonymous maintainer information
- local absolute paths are removed or translated at runtime
- audit helper reports no obvious author-identifying strings in tracked files
