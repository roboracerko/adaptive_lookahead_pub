# F1TENTH Driver Metrics Evaluation System

> **ROS 2 Distribution: Humble Hawksbill**  
> **Tested on: Ubuntu 22.04**  
> **ROS 2 Version: Humble**

F1TENTH 자율주행 드라이버의 성능을 평가하기 위한 통합 평가 시스템입니다. Pure Pursuit, SVG-MPPI 등 다양한 드라이버의 주행 메트릭을 실시간으로 수집하고 분석합니다.

## 📋 목적

이 시스템은 F1TENTH 시뮬레이션 환경에서 드라이버의 성능을 객관적으로 평가하기 위해 설계되었습니다:

- **Path Tracking Accuracy**: 경로 추적 정확도 (RMS/최대 측면 오차)
- **Performance**: 주행 성능 (Lap Time, 평균 속도)
- **Steering Stability**: 조향 안정성 (조향률, Lookahead 분산)
- **Safety & Robustness**: 안전성 및 견고성 (트랙 이탈 비율, E-stop 횟수)

## 🏗️ 시스템 아키텍처

### 전체 구조 (3계층)

```
┌─────────────────────────────────────────────────────────┐
│ Launch 계층 (launch/)                                    │
│ - evaluation_pure_pursuit.launch.py (통합 평가)         │
│ - metrics.launch.py (메트릭만)                          │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│ 런타임 노드 계층 (f1tenth_driver_metrics/*.py)          │
│ - metrics_collector_node.py (메트릭 수집)               │
│ - progress_lap_manager.py (Lap 감지)                     │
│ - realtime_visualizer.py (실시간 시각화)                │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│ 오프라인 분석 계층                                       │
│ - generate_performance_report.py (성능 보고서 생성)     │
└─────────────────────────────────────────────────────────┘
```

### 핵심 컴포넌트

#### 1. Metrics Collector Node
- **역할**: 주행 토픽을 구독하여 실시간 메트릭 수집
- **입력 토픽**:
  - `/ego_racecar/odom` (Odometry)
  - `/drive` (Ackermann Drive)
  - `/estop` (Emergency Stop, 선택)
- **출력 토픽**:
  - `/metrics/lap_summary` (Lap별 요약)
  - `/metrics/run_summary` (전체 실행 요약)
  - `/metrics/debug/lateral_error` (실시간 측면 오차)

#### 2. Progress-based Lap Detector
- **알고리즘**: Centerline 투영 기반 원형 진행률 계산
- **상태 기계**: INIT → ARMED → IN_START_ZONE → COOLDOWN
- **특징**: Wrap-around 감지, 초기 스폰 오탐 방지, 흔들림 필터링

#### 3. Real-time Visualizer
- **기능**: matplotlib 기반 실시간 성능 그래프
- **표시 내용**: 궤적, 속도, 측면 오차, 분포 히스토그램

## 🚀 설치 및 빌드

### 필수 요구사항

- **ROS 2 Humble** (Ubuntu 22.04)
- **F1TENTH Gym**: [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym)
- **Python 패키지**: `numpy`, `matplotlib`, `pyyaml`, `psutil`

### 설치 단계

1. **워크스페이스 설정**
```bash
cd ~/ros2_prj/sim_ws/src
git clone <repository_url> f1tenth_driver_metrics
```

2. **의존성 설치**
```bash
cd ~/ros2_prj/sim_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y
```

3. **빌드**
```bash
colcon build --packages-select f1tenth_driver_metrics
source install/setup.bash
```

## 📖 사용법

### 기본 평가 시뮬레이션 실행

가장 간단한 방법은 통합 launch 파일을 사용하는 것입니다:

```bash
cd ~/ros2_prj/sim_ws
source install/setup.bash

# RViz2와 Realtime Visualizer 모두 활성화
ros2 launch f1tenth_driver_metrics evaluation_pure_pursuit.launch.py \
    use_rviz:=true \
    use_realtime_visualizer:=true
```

### 시각화 옵션

#### RViz2만 실행 (3D 시각화)
```bash
ros2 launch f1tenth_driver_metrics evaluation_pure_pursuit.launch.py \
    use_rviz:=true \
    use_realtime_visualizer:=false
```

#### Realtime Visualizer만 실행 (성능 그래프)
```bash
ros2 launch f1tenth_driver_metrics evaluation_pure_pursuit.launch.py \
    use_rviz:=false \
    use_realtime_visualizer:=true
```

#### 헤드리스 모드 (GUI 없음)
```bash
ros2 launch f1tenth_driver_metrics evaluation_pure_pursuit.launch.py \
    use_rviz:=false \
    use_realtime_visualizer:=false
```

### 출력 디렉토리 지정

```bash
ros2 launch f1tenth_driver_metrics evaluation_pure_pursuit.launch.py \
    output_dir:=/custom/path/to/results
```

기본값은 `./eval_results/budapest` (workspace root 기준)입니다.

### 평가 완료 후

평가가 완료되면 (max_laps 도달 시) 자동으로:
1. 최종 Run Summary 저장
2. 성능 보고서 생성 (`performance_report.txt`)
3. 모든 프로세스 안전하게 종료

## ⚙️ 설정 파일

### 설정 파일 우선순위

```
Launch Argument > budapest_metrics.yaml > evaluation_config.yaml > metrics.yaml
```

### 주요 설정 파일

#### 1. `budapest_metrics.yaml` (트랙 특화)
- **위치**: `config/budapest_metrics.yaml`
- **용도**: Budapest 트랙에 특화된 설정
- **주요 설정**:
  - `output_dir`: 출력 디렉토리 (상대 경로 지원)
  - `track_half_width_m`: 트랙 반폭
  - `start_line_p1`, `start_line_p2`: 시작선 좌표

#### 2. `evaluation_config.yaml` (평가 설정)
- **위치**: `config/evaluation_config.yaml`
- **용도**: 평가 시뮬레이션 전용 설정
- **주요 설정**:
  - `lap_config.min_laps`: 최소 완주 Lap 수
  - `lap_config.max_laps`: 최대 완주 Lap 수 (도달 시 자동 종료)
  - `safety`: 안전 모니터링 임계값
  - `visualization`: 시각화 옵션

#### 3. `metrics.yaml` (기본 템플릿)
- **위치**: `config/metrics.yaml`
- **용도**: 일반 메트릭 수집용 기본 템플릿

### 설정 파일 예시

```yaml
# evaluation_config.yaml
lap_config:
  min_laps: 2
  max_laps: 2
  auto_shutdown: true
  min_lap_time_s: 5.0

safety:
  offtrack_threshold_ratio: 0.3
  track_boundary_check_enabled: true
  auto_shutdown_on_error: true

visualization:
  enable_rviz: true
  enable_realtime_visualizer: true
  update_rate: 2.0
  max_history: 1000
```

## 📊 출력 파일 구조

평가 완료 후 출력 디렉토리 구조:

```
eval_results/budapest/
├── performance_report.txt      # 성능 평가 보고서 (자동 생성)
├── all_laps_summary.csv        # 전체 Lap 요약 CSV
├── laps/                       # Lap별 상세 데이터
│   ├── lap_001.json           # Lap 1 상세 메트릭 (JSON)
│   ├── lap_001.csv            # Lap 1 상세 메트릭 (CSV)
│   ├── lap_002.json
│   └── lap_002.csv
└── runs/                       # Run 요약 데이터
    └── run_2laps.json         # 전체 실행 요약
```

### Lap Summary JSON 예시

```json
{
  "lap": 1,
  "lap_time_s": 45.23,
  "avg_speed_mps": 4.42,
  "max_speed_mps": 12.5,
  "min_speed_mps": 2.1,
  "rms_lateral_error_m": 0.15,
  "max_lateral_error_m": 0.85,
  "rms_steering_rate_radps": 0.12,
  "lookahead_variance": 0.05,
  "offtrack_ratio": 0.02,
  "lap_distance_m": 200.0,
  "lap_offtrack_distance_m": 4.0,
  "estop_count_total": 0
}
```

## 🔧 주요 기능

### 1. Lap 감지

**Progress-based Lap Detector** (기본, 권장):
- Centerline 투영 기반 원형 진행률 계산
- Wrap-around 감지 (트랙 끝 → 시작)
- 초기 스폰 오탐 방지
- 흔들림 필터링 (Cooldown)

**파라미터 튜닝**:
```yaml
progress_start_zone_s: 5.0      # Start zone 크기 (m)
progress_wrap_margin_s: 3.0      # Wrap-around 감지 마진 (m)
progress_cooldown_s: 1.0         # Lap 직후 쿨다운 시간 (s)
progress_min_progress_s: 0.10    # 최소 전진량 필터 (m)
```

### 2. 안전 모니터링

- **트랙 경계 위반**: 즉시 종료
- **역주행 감지**: 즉시 종료
- **Off-track 비율**: 임계값 초과 시 종료
- **Off-track 지속 시간**: 임계값 초과 시 종료

### 3. 실시간 메트릭 수집

- **Online Statistics**: 샘플 전체 저장 없이 실시간 통계 계산
  - RMS, Mean, Variance, Max, Min
- **메모리 효율적**: 전체 궤적 저장 없이 통계만 누적

### 4. 자동 종료

- `max_laps` 도달 시 자동 종료
- 안전 위반 시 즉시 종료
- 모든 프로세스 안전하게 종료 (psutil 기반)

## 📈 성능 보고서

평가 완료 후 자동으로 생성되는 `performance_report.txt`에는 다음 정보가 포함됩니다:

- **Lap별 요약**: 각 Lap의 시간, 속도, 측면 오차 등
- **전체 통계**: 평균, 최소, 최대, 표준편차
- **성능 비교**: Lap 간 비교 분석

## ⚠️ 유의사항

### 1. 좌표계 일치

⚠️ **중요**: `centerline` CSV 파일과 차량 `odom`이 **동일 좌표계**여야 합니다.

- Centerline CSV가 `map` 기준이면 TF 변환 필요
- 현재는 `odom` 기준으로 동작 (TF 변환 미구현)

### 2. 프로세스 종료

- 평가 완료 시 모든 프로세스가 자동으로 안전하게 종료됩니다
- `psutil`을 사용하여 launch 세션에 속한 프로세스만 종료
- 시스템 프로세스는 영향받지 않음

### 3. 리소스 사용

- **RViz2**: 리소스 사용량이 높음 (필요 시 비활성화)
- **Realtime Visualizer**: `update_rate` 조정으로 리소스 절약 가능
- **헤드리스 모드**: 원격 서버 실행 시 권장

### 4. 설정 파일 우선순위

Launch argument가 가장 높은 우선순위를 가집니다:
```bash
# 이 값이 설정 파일보다 우선
ros2 launch ... output_dir:=/custom/path
```

## 🐛 문제 해결

### 맵이 RViz2에 표시되지 않는 경우

1. **Map Server 상태 확인**:
```bash
ros2 lifecycle get /map_server
# active [3] 이어야 함
```

2. **맵 토픽 확인**:
```bash
ros2 topic echo /map --once
```

3. **Lifecycle Manager 확인**: `pure_pursuit_budapest_launch.py`에서 자동 활성화 확인

### Lap이 감지되지 않는 경우

1. **Centerline 파일 확인**: CSV 파일 경로 및 형식 확인
2. **Start Zone 크기 조정**: `progress_start_zone_s` 증가
3. **Wrap Margin 조정**: `progress_wrap_margin_s` 증가

### GUI가 종료되지 않는 경우

- 최신 버전 사용 (psutil 기반 안전 종료 로직)
- 수동 종료: `pkill -f rviz2` 또는 `pkill -f realtime_visualizer`

### Exit Code 1 에러

- `rclpy.shutdown()` 중복 호출 문제는 최신 버전에서 해결됨
- 프로세스들이 SIGTERM을 받고 정상 종료됨

## 📚 관련 문서

- `ARCHITECTURE.md`: 전체 아키텍처 상세 설명
- `CONFIG_FILES_GUIDE.md`: 설정 파일 가이드
- `VISUALIZATION_OPTIONS.md`: 시각화 옵션 가이드
- `PROGRESS_LAP_DETECTOR.md`: Lap 감지 알고리즘 설명

## 🔗 관련 패키지

- **f1tenth_gym_ros**: F1TENTH Gym 시뮬레이터 ROS2 브리지
- **pure_pursuit_driver**: Pure Pursuit 드라이버
- **nav2_map_server**: 맵 서버 (lifecycle)

## 📝 개발 및 확장

### 새로운 드라이버 평가 추가

1. 드라이버 launch 파일 생성
2. `evaluation_<driver>.launch.py` 생성
3. 드라이버 launch 파일 include
4. Metrics Collector 노드 추가

### 커스텀 메트릭 추가

1. `metrics_collector_node.py`에서 메트릭 계산 로직 추가
2. `online_stats.py`에 필요한 통계 클래스 추가
3. `exporters.py`에서 출력 형식 추가

## 📄 라이선스

MIT License

---

**작성일**: 2026-01-15  
**버전**: 1.0.0  
**ROS 2 Distribution**: Humble Hawksbill
