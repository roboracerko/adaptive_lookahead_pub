# Setup Guide

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

## Installation

```bash
cd adaptive_lookahead_pub
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r analysis/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

## Smoke Test

```bash
bash experiments/run_main_benchmark.sh --method kw_sim --map budapest --trials 1
```

## Optional Docker Workflow

```bash
docker build -t adaptive_lookahead_pub docker/
bash docker/docker_run.sh /tmp/adaptive_lookahead_results
```

## Notes

- The benchmark entrypoints patch packaged configs to the current checkout path at runtime.
- Analysis scripts expect result directories under `/tmp/f1tenth_eval/*` by default.
