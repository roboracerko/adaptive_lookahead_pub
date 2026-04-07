# Setup Guide

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- f1tenth_gym: `pip install f1tenth-gym`

## Installation

```bash
# 1. Clone (submodules 포함)
git clone --recurse-submodules https://github.com/[YOUR_REPO]/adaptive_lookahead_pub
cd adaptive_lookahead_pub

# 2. ROS2 의존성 설치
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# 3. Python 분석 의존성
pip3 install -r analysis/requirements.txt

# 4. Build
colcon build --symlink-install
source install/setup.bash
```

## Docker (선택사항)

```bash
cd adaptive_lookahead_pub
docker build -t adaptive_lookahead_pub docker/
bash docker/docker_run.sh /tmp/my_results
```

## 패키지 구성

| 패키지 | 역할 |
|---|---|
| `pp_adaptive` | 핵심 드라이버 + LD labeler |
| `f1tenth_driver_benchmark_suite` | 벤치마크 평가 인프라 |
| `f1tenth_gym_ros` | F1TENTH 시뮬레이터 ROS2 브릿지 |
| `pp_core` | Pure Pursuit 기본 드라이버 (참고용) |
