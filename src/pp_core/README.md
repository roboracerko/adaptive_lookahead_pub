# Pure Pursuit Driver for F1TENTH Gym ROS2

> **ROS 2 Distribution**: Humble Hawksbill (Foxy Fitzroy 호환)  
> **Tested on**: Ubuntu 22.04 (Ubuntu 20.04 호환)  
> **Dependencies**: f1tenth_gym_ros

F1TENTH Gym ROS2 시뮬레이터를 위한 **고정 lookahead 거리 Pure Pursuit 제어 드라이버**입니다. CSV 파일에서 레이스라인 경로를 로드하고, Odometry를 구독하여 `AckermannDriveStamped` 제어 명령을 발행합니다.

---

## 📋 목차

- [개요](#개요)
- [주요 기능](#주요-기능)
- [프로젝트 구조](#프로젝트-구조)
- [설치 및 빌드](#설치-및-빌드)
- [사용법](#사용법)
- [f1tenth_gym_ros와 통합](#f1tenth_gym_ros와-통합)
- [설정 파일](#설정-파일)
- [주요 파일 설명](#주요-파일-설명)
- [ROS2 토픽](#ros2-토픽)
- [알고리즘](#알고리즘)
- [새로운 맵 적용](#새로운-맵-적용)
- [참고 자료](#참고-자료)

---

## 개요

이 패키지는 `adaptive_lookahead` 프로젝트에서 Pure Pursuit 제어 알고리즘을 추출하여 ROS2 노드로 구현한 것입니다. F1TENTH Gym ROS2 시뮬레이터(`f1tenth_gym_ros`)와 통합하여 자율주행 레이싱 차량의 경로 추종 제어를 수행합니다.

### 핵심 특징

- ✅ **고정 lookahead 거리** 기반 Pure Pursuit 제어
- ✅ **CSV 파일 경로 로드**: 레이스라인 경로 데이터 지원
- ✅ **실시간 제어**: Odometry 기반 경로 추종
- ✅ **속도 제약**: 횡가속도 제한 및 조향 감속
- ✅ **F1TENTH Gym 통합**: `f1tenth_gym_ros`와 완전 호환

---

## 주요 기능

### 1. Pure Pursuit 경로 추종

Ackermann 기하학 기반 Pure Pursuit 알고리즘을 사용하여 경로를 추종합니다.

- **고정 lookahead 거리**: 설정 가능한 lookahead 거리 사용
- **조향각 제한**: 최대 조향각 제한 (F1TENTH 기본값: ±24도)
- **경로 검색**: 윈도우 기반 가장 가까운 waypoint 검색

### 2. 속도 제어

- **횡가속도 제한**: 곡률 기반 최대 속도 제한
- **조향 감속**: 조향각에 따른 속도 감소 옵션
- **속도 프로파일**: CSV 파일에서 속도 프로파일 로드

### 3. ROS2 통합

- **토픽 기반 통신**: ROS2 표준 메시지 타입 사용
- **파라미터 설정**: YAML 파일 기반 파라미터 관리
- **런치 파일**: 통합 런치 파일 제공

---

## 프로젝트 구조

```
pp_core/
├── pp_core/           # Python 패키지
│   ├── __init__.py
│   └── pure_pursuit_node.py       # 메인 ROS2 노드
│
├── config/                         # 설정 파일
│   └── pure_pursuit.yaml          # Pure Pursuit 제어 파라미터
│
├── racelines/                      # 레이스라인 경로 데이터
│   └── Budapest_raceline_vehicleaware.csv
│
├── maps/                           # 맵 파일 (참고용)
│   ├── Budapest_map.png           # 맵 이미지
│   └── Budapest_map.yaml          # 맵 메타데이터
│
├── launch/                         # 런치 파일
│   ├── pure_pursuit_launch.py              # 드라이버만 실행
│   ├── pure_pursuit_with_sim_launch.py     # 시뮬레이션과 함께 실행
│   └── pure_pursuit_map_launch.py     # 맵 통합 런치 (권장)
│
├── scripts/                        # 스크립트
│
├── test/                           # 테스트 파일
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── setup.py                        # Python 패키지 설정
├── package.xml                     # ROS2 패키지 매니페스트
├── setup.cfg                       # 테스트 설정
│
└── 문서/
    ├── README.md                   # 이 파일
    ├── MAP_SETUP_GUIDE.md         # 새로운 맵 적용 가이드
    ├── INTEGRATION_CHECK.md       # 통합 확인 보고서
    └── TEST_RESULTS.md            # 테스트 결과
```

---

## 설치 및 빌드

### 사전 요구사항

1. **ROS 2** (Foxy 또는 Humble)
   - Ubuntu 20.04: ROS 2 Foxy
   - Ubuntu 22.04: ROS 2 Humble

2. **F1TENTH Gym**
   ```bash
   git clone https://github.com/f1tenth/f1tenth_gym
   cd f1tenth_gym && pip3 install -e .
   ```

3. **f1tenth_gym_ros** 패키지
   - `sim_ws/src/f1tenth_gym_ros` 패키지가 필요합니다
   - [f1tenth_gym_ros README](../f1tenth_gym_ros/README.md) 참고

### 빌드

```bash
cd sim_ws
source /opt/ros/humble/setup.bash  # 또는 foxy
colcon build --packages-select pp_core
source install/setup.bash
```

### 의존성 설치

```bash
cd sim_ws
rosdep install -i --from-path src --rosdistro humble -y  # 또는 foxy
```

---

## 사용법

### 방법 1: 통합 런치 파일 사용 (권장) ⭐

**런치 파일**: `pure_pursuit_map_launch.py`

**설명**: 맵별 시뮬레이션 설정(`sim_config`)과 Pure Pursuit 드라이버를 통합한 런치 파일입니다. 시뮬레이션 환경, 맵 서버, RViz2 시각화, 그리고 Pure Pursuit 제어 드라이버를 한 번에 실행합니다.

**실행되는 노드**:
- `gym_bridge`: F1TENTH Gym 시뮬레이터 (`sim_config` 사용)
- `map_server`: 맵 서버 (`sim_config`의 `map_path` 사용)
- `rviz2`: 시각화 도구
- `robot_state_publisher`: 로봇 상태 발행 (URDF → TF)
- `nav2_lifecycle_manager`: 맵 서버 생명주기 관리
- `pp_core`: Pure Pursuit 제어 드라이버 (`pure_pursuit.yaml`의 `raceline_csv` 사용)

**특징**:
- ✅ 맵별 `sim_config`로 손쉽게 변경 가능
- ✅ 모든 노드를 한 번에 실행 (통합 런치)
- ✅ 맵 서버와 시뮬레이션이 같은 맵 사용 (일관성 보장)

**실행 방법**:
```bash
cd sim_ws
source install/setup.bash
ros2 launch pp_core pure_pursuit_map_launch.py
```

**맵 변경 예시**:
```bash
ros2 launch pp_core pure_pursuit_map_launch.py sim_config:=/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/config/sim_bexco.yaml
```

### 방법 2: 시뮬레이션과 함께 실행

**런치 파일**: `pure_pursuit_with_sim_launch.py`

**설명**: F1TENTH Gym 시뮬레이터와 Pure Pursuit 드라이버를 함께 실행하는 통합 런치 파일입니다. 시뮬레이션 환경(`f1tenth_gym_ros`), 맵 서버, RViz2 시각화, 그리고 Pure Pursuit 제어 드라이버를 한 번에 시작합니다.

**실행되는 노드**:
- `gym_bridge`: F1TENTH Gym 시뮬레이터 (물리 엔진, LiDAR 시뮬레이션)
- `map_server`: 맵 서버 (RViz 맵 표시용)
- `rviz2`: 시각화 도구
- `robot_state_publisher`: 로봇 상태 발행 (URDF → TF)
- `nav2_lifecycle_manager`: 맵 서버 생명주기 관리
- `pp_core`: Pure Pursuit 제어 드라이버

**특징**:
- ✅ 모든 노드를 한 번에 실행 (통합 런치)
- ✅ `sim.yaml` 설정 파일 사용 (기본값: levine 맵)
- ✅ `sim.yaml`의 `map_path`를 읽어서 Map Server에 동적으로 적용
- ✅ 시뮬레이션 맵과 Map Server 맵이 항상 동일 (일관성 보장)
- ✅ 맵 파일명을 원본 그대로 사용 가능 (파일명 변경 불필요)

**실행 방법**:
```bash
cd sim_ws
source install/setup.bash
ros2 launch pp_core pure_pursuit_with_sim_launch.py
```

**데이터 배치 위치**:
- **시뮬레이션 맵**: `f1tenth_gym_ros/config/sim.yaml`에서 `map_path` 설정
  - 기본값: `levine` 맵 (`$PRJ_HOME/sim_ws/src/f1tenth_gym_ros/maps/levine`)
  - 맵 파일 배치: `f1tenth_gym_ros/maps/<맵이름>.png`, `f1tenth_gym_ros/maps/<맵이름>.yaml`
  - **✅ 개선**: 원본 파일명 그대로 사용 가능 (파일명 변경 불필요)
- **Map Server 맵**: `sim.yaml`의 `map_path`를 동적으로 사용 (RViz 표시용)
  - 시뮬레이션 맵과 동일한 맵 사용 (일관성 보장)
  - 별도 맵 파일 준비 불필요
- **Raceline 경로**: `pp_core/racelines/Budapest_raceline_vehicleaware.csv`
  - 기본 경로로 하드코딩되어 있음
  - 다른 경로 사용 시 launch 파일 수정 필요

**✅ 개선 사항** (v2.0):
- ✅ 맵 파일명을 원본 그대로 사용 가능
- ✅ `sim.yaml`의 `map_path`를 읽어서 Map Server에 동적으로 적용
- ✅ 시뮬레이션 맵과 Map Server 맵이 항상 동일 (일관성 보장)

**⚠️ 주의사항**:
- `sim.yaml`의 `map_path`는 실제 맵 파일 경로와 일치해야 합니다 (확장자 없이)
- Raceline CSV 파일의 좌표는 시뮬레이션 맵(`sim.yaml`의 `map_path`) 좌표계와 일치해야 합니다
- 시작 위치(`sim.yaml`의 `sx`, `sy`, `stheta`)는 raceline 첫 waypoint와 일치해야 합니다

### 방법 3: 드라이버만 실행

이미 실행 중인 시뮬레이션이 있을 때:

```bash
cd sim_ws
source install/setup.bash
ros2 launch pp_core pure_pursuit_launch.py
```

**데이터 배치 위치**:
- **Raceline 경로**: `pp_core/racelines/Budapest_raceline_vehicleaware.csv`
  - 기본 경로로 하드코딩되어 있음
  - 다른 경로 사용 시 launch 파일 수정 필요
- **맵 데이터**: 필요 없음 (이미 실행 중인 시뮬레이션에서 처리)

**설명**:
- 이 launch 파일은 드라이버 노드만 실행합니다
- 시뮬레이션은 별도로 실행되어 있어야 합니다
- 드라이버는 `/ego_racecar/odom`을 구독하고 `/drive`에 제어 명령을 발행합니다

---

## Launch 파일별 데이터 요구사항

### `pure_pursuit_launch.py` (드라이버만)

| 데이터 | 위치 | 파일명 | 필수 여부 |
|--------|------|--------|-----------|
| Raceline CSV | `pp_core/racelines/` | `Budapest_raceline_vehicleaware.csv` | ✅ 필수 |
| 맵 파일 | - | - | ❌ 불필요 (시뮬레이션에서 처리) |

**데이터 준비**:
```bash
# 1. Raceline CSV 파일을 racelines 디렉토리에 배치
cp <your_raceline>.csv sim_ws/src/pp_core/racelines/Budapest_raceline_vehicleaware.csv

# 2. launch 파일에서 경로 확인 (필요시 수정)
# launch/pure_pursuit_launch.py의 23번 라인
```

### `pure_pursuit_with_sim_launch.py` (시뮬레이션 + 드라이버)

| 데이터 | 위치 | 파일명 | 필수 여부 | 용도 |
|--------|------|--------|-----------|------|
| 시뮬레이션 맵 | `f1tenth_gym_ros/maps/` | `<맵이름>.png`, `<맵이름>.yaml` (원본 파일명 사용 가능) | ✅ 필수 | F1TENTH Gym 시뮬레이터 |
| Map Server 맵 | - | 시뮬레이션 맵과 동일 (`sim.yaml`의 `map_path` 사용) | ✅ 자동 | RViz 맵 표시 |
| Raceline CSV | `pp_core/racelines/` | `Budapest_raceline_vehicleaware.csv` (또는 launch 파일에서 지정) | ✅ 필수 | Pure Pursuit 경로 추종 |
| 시뮬레이션 설정 | `f1tenth_gym_ros/config/` | `sim.yaml` | ✅ 필수 | 시뮬레이션 파라미터 |

**데이터 준비**:
```bash
# 1. 시뮬레이션 맵 파일 (원본 이름 그대로 사용)
# 원하는 맵 파일을 maps 디렉토리에 배치 (파일명 변경 불필요)
cp <your_map>.png sim_ws/src/f1tenth_gym_ros/maps/<your_map>.png
cp <your_map>.yaml sim_ws/src/f1tenth_gym_ros/maps/<your_map>.yaml

# your_map.yaml 수정 (이미지 경로 확인)
# image: <your_map>.png (같은 디렉토리 기준 상대 경로)

# 2. sim.yaml 수정 (맵 경로 설정)
# f1tenth_gym_ros/config/sim.yaml 수정:
#   map_path: '$PRJ_HOME/sim_ws/src/f1tenth_gym_ros/maps/<your_map>'  # 확장자 없이
#   sx: <x좌표>  # raceline 첫 waypoint의 x_m 값
#   sy: <y좌표>  # raceline 첫 waypoint의 y_m 값
#   stheta: <방향>  # 첫 waypoint → 두 번째 waypoint 방향 (atan2 계산)

# 3. Raceline CSV 파일 (원본 이름 그대로 사용 가능)
# 방법 A: 기본 파일명 사용
cp <your_raceline>.csv sim_ws/src/pp_core/racelines/Budapest_raceline_vehicleaware.csv

# 방법 B: 다른 파일명 사용 시 launch 파일 수정
cp <your_raceline>.csv sim_ws/src/pp_core/racelines/<your_raceline>.csv
# launch/pure_pursuit_with_sim_launch.py의 24번 라인 수정:
# default_raceline = os.path.join(raceline_dir, '<your_raceline>.csv')
```

**✅ 개선 사항**:
- ✅ 맵 파일명을 원본 그대로 사용 가능 (파일명 변경 불필요)
- ✅ `sim.yaml`의 `map_path`를 읽어서 Map Server에 동적으로 적용
- ✅ 시뮬레이션 맵과 Map Server 맵이 항상 동일 (일관성 보장)

**⚠️ 중요 체크리스트**:
- [ ] `sim.yaml`의 `map_path`가 실제 맵 파일 경로와 일치하는가?
- [ ] `sim.yaml`의 `sx`, `sy`, `stheta`가 raceline 첫 waypoint와 일치하는가?
- [ ] Raceline CSV 좌표계가 시뮬레이션 맵 좌표계와 일치하는가?
- [ ] `Budapest_map.yaml`의 `image` 경로가 올바른가?
- [ ] 빌드 완료 후 실행 (`colcon build`)


## f1tenth_gym_ros와 통합

### 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    f1tenth_gym_ros                          │
│  ┌───────────────────────────────────────────────────┐      |
│  │ gym_bridge (ROS2 Node)                            │      │
│  │  - F1TENTH Gym 시뮬레이터                           │     │
│  │  - 물리 엔진 (Single-track model)                  │     │
│  │  - LiDAR 시뮬레이션                                 │     │
│  │                                                   │     │
│  │  Published Topics:                                │     │
│  │    /ego_racecar/odom  (nav_msgs/Odometry)         │     │
│  │    /scan            (sensor_msgs/LaserScan)       │     │
│  │    /map             (nav_msgs/OccupancyGrid)      │     │
│  │                                                   │     │
│  │  Subscribed Topics:                               │     │
│  │    /drive           (ackermann_msgs/AckermannDrive│     │
│  └───────────────────────────────────────────────────┘     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ ROS2 Topics
                       │
┌──────────────────────▼──────────────────────────────────────┐
│              pp_core                            │
│  ┌───────────────────────────────────────────────────┐      │
│  │ pure_pursuit_node (ROS2 Node)                     │      │
│  │  - Pure Pursuit 제어 알고리즘                        │      │
│  │  - CSV 경로 로드                                    │      │
│  │  - 실시간 경로 추종                                  │       │
│  │                                                   │      │
│  │  Subscribed Topics:                               │      │
│  │    /ego_racecar/odom  (nav_msgs/Odometry)         │      │
│  │                                                   │      │
│  │  Published Topics:                                │      │
│  │    /drive           (ackermann_msgs/AckermannDrive│      │
│  │                      Stamped)                     │      │
│  └───────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 통합 방식

1. **토픽 기반 통신**
   - `pp_core`는 `/ego_racecar/odom`을 구독하여 차량 상태를 받습니다
   - `/drive` 토픽에 제어 명령을 발행합니다
   - `f1tenth_gym_ros`는 `/drive`를 구독하여 차량을 제어합니다

2. **맵 및 경로 데이터**
   - 맵 파일: `f1tenth_gym_ros/maps/` 디렉토리에 저장
   - 경로 파일: `pp_core/racelines/` 디렉토리에 저장
   - 설정 파일: `f1tenth_gym_ros/config/` 디렉토리에 저장

3. **런치 파일 통합**
   - `pure_pursuit_map_launch.py`는 두 패키지를 통합하여 실행
   - 시뮬레이터, 맵 서버, 드라이버, RViz를 한 번에 실행

### 토픽 매핑

| pp_core | f1tenth_gym_ros | 메시지 타입 |
|---------------------|-----------------|-------------|
| **구독** `/ego_racecar/odom` | **발행** `/ego_racecar/odom` | `nav_msgs/Odometry` |
| **발행** `/drive` | **구독** `/drive` | `ackermann_msgs/AckermannDriveStamped` |

---

## 설정 파일

### config/pure_pursuit.yaml

Pure Pursuit 제어 파라미터:

```yaml
pp_core:
  ros__parameters:
    # 경로 설정
    raceline_csv: 'Budapest_raceline_vehicleaware.csv'  # racelines 디렉토리 내 파일명
    
    # 토픽 설정
    odom_topic: '/ego_racecar/odom'
    drive_topic: '/drive'
    
    # 차량 파라미터 (F1TENTH Gym 기본값)
    wheelbase: 0.3302      # m (lf + lr)
    delta_max_deg: 24.0    # deg
    
    # Pure Pursuit 파라미터
    lookahead_distance: 1.0    # 고정 lookahead 거리 (m)
    search_window: 300         # 웨이포인트 검색 윈도우 크기
    control_hz: 50.0           # 제어 주기 (Hz)
    
    # 속도 제약
    a_lat_max: 7.5           # 최대 횡가속도 (m/s²)
    use_steer_slowdown: true # 조향 감속 활성화
    slowdown_gain: 0.5       # 조향 감속 게인
    v_floor: 1.0             # 최저 속도 (m/s)
```

### 파라미터 설명

#### 경로 설정
- `raceline_csv`: 레이스라인 CSV 파일명 (racelines 디렉토리 기준)

#### 차량 파라미터
- `wheelbase`: 차량 휠베이스 (m) - F1TENTH 기본값: 0.3302m
- `delta_max_deg`: 최대 조향각 (도) - F1TENTH 기본값: ±24도

#### Pure Pursuit 파라미터
- `lookahead_distance`: 고정 lookahead 거리 (m) - 일반적으로 0.5~2.0m
- `search_window`: 가장 가까운 waypoint 검색 윈도우 크기
- `control_hz`: 제어 루프 주파수 (Hz)

#### 속도 제약
- `a_lat_max`: 최대 횡가속도 (m/s²) - 곡률 기반 속도 제한
- `use_steer_slowdown`: 조향각에 따른 속도 감소 활성화
- `slowdown_gain`: 조향 감속 게인 (0.0~1.0)
- `v_floor`: 최저 속도 (m/s)

---

## 주요 파일 설명

### pp_core/pure_pursuit_node.py

**메인 ROS2 노드** - Pure Pursuit 제어 알고리즘 구현

**주요 클래스/함수**:

- `PurePursuitDriver(Node)`: 메인 ROS2 노드 클래스
  - `__init__()`: 노드 초기화, 파라미터 선언, 토픽 구독/발행
  - `load_raceline()`: CSV 파일에서 경로 로드
  - `odom_callback()`: Odometry 메시지 처리
  - `control_callback()`: 제어 루프 (Pure Pursuit 알고리즘 실행)

- `pure_pursuit_delta()`: Pure Pursuit 조향각 계산
- `nearest_point_idx_window()`: 가장 가까운 waypoint 검색
- `lookahead_point_from_idx()`: Lookahead 포인트 계산
- `lateral_speed_limit()`: 횡가속도 기반 속도 제한

**입력/출력**:
- **입력**: `/ego_racecar/odom` (Odometry)
- **출력**: `/drive` (AckermannDriveStamped)

### launch/pure_pursuit_map_launch.py

**통합 런치 파일** - 맵/시뮬레이션과 함께 실행

**실행되는 노드**:
1. `gym_bridge`: F1TENTH Gym 시뮬레이터
2. `map_server`: 맵 서버 (`sim_config`의 `map_path`)
3. `rviz2`: 시각화
4. `robot_state_publisher`: 로봇 상태 발행
5. `nav2_lifecycle_manager`: 맵 서버 생명주기 관리
6. `pp_core`: Pure Pursuit 제어 드라이버

**사용법**:
```bash
ros2 launch pp_core pure_pursuit_map_launch.py
```

```bash
ros2 launch pp_core pure_pursuit_map_launch.py sim_config:=/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/config/sim_bexco.yaml
```

### launch/pure_pursuit_with_sim_launch.py

**시뮬레이션과 함께 실행** - 기본 설정 파일 사용

**사용법**:
```bash
ros2 launch pp_core pure_pursuit_with_sim_launch.py
```

### config/pure_pursuit.yaml

**제어 파라미터 설정 파일** - Pure Pursuit 알고리즘 파라미터

### racelines/Budapest_raceline_vehicleaware.csv

**레이스라인 경로 데이터** - CSV 형식의 waypoint 데이터

**필수 컬럼**:
- `x_m`: X 좌표 (m)
- `y_m`: Y 좌표 (m)

**선택 컬럼**:
- `v_mps`: 속도 프로파일 (m/s) - 없으면 기본값 3.0 m/s 사용
- `theta_rad`: 각도 (rad) - 현재 미사용
- `kappa_1pm`: 곡률 (1/m) - 현재 미사용

**예시**:
```csv
x_m,y_m,theta_rad,kappa_1pm,v_mps
-3.1767114354150507,2.556277889421665,-2.4580908231582286,0.034962834565883456,12.0
-3.701227282321227,2.99378007158861,-2.456798540782452,-0.031099628525949753,12.0
...
```

---

## ROS2 토픽

### 구독 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ego_racecar/odom` | `nav_msgs/Odometry` | 차량 현재 상태 (위치, 속도, 방향) |

**메시지 구조**:
```python
nav_msgs/Odometry
  header:
    stamp: Time
    frame_id: "map"
  pose:
    pose:
      position: Point (x, y, z)
      orientation: Quaternion (x, y, z, w)
  twist:
    twist:
      linear: Vector3 (x, y, z)  # 속도
      angular: Vector3 (x, y, z)  # 각속도
```

### 발행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | 제어 명령 (속도, 조향각) |

**메시지 구조**:
```python
ackermann_msgs/AckermannDriveStamped
  header:
    stamp: Time
    frame_id: "base_link"
  drive:
    speed: float              # 속도 (m/s)
    steering_angle: float     # 조향각 (rad)
    steering_angle_velocity: float
    acceleration: float
    jerk: float
```

---

## 알고리즘

### Pure Pursuit 제어 알고리즘

Pure Pursuit은 차량이 목표 경로를 추종하기 위한 기하학 기반 제어 알고리즘입니다.

#### 1. 가장 가까운 Waypoint 검색

```python
def nearest_point_idx_window(pos, traj, i0, win):
    """윈도우 내에서 가장 가까운 waypoint 인덱스 찾기"""
    idxs = (np.arange(i0, i0 + win) % N)
    distances = np.linalg.norm(traj[idxs] - pos, axis=1)
    return idxs[np.argmin(distances)]
```

#### 2. Lookahead 포인트 계산

```python
def lookahead_point_from_idx(traj, i0, Ld):
    """현재 위치에서 lookahead 거리만큼 앞의 포인트 찾기"""
    dist_acc = 0.0
    i = i0
    while dist_acc < Ld:
        j = (i + 1) % N
        dist_acc += np.linalg.norm(traj[j] - traj[i])
        i = j
    return traj[i], i
```

#### 3. 조향각 계산

```python
def pure_pursuit_delta(x, y, theta, lookahead_pt, wheelbase, Ld, delta_max):
    """Pure Pursuit 조향각 계산 (Ackermann 기하학)"""
    dx = lookahead_pt[0] - x
    dy = lookahead_pt[1] - y
    alpha = atan2(dy, dx) - theta  # Lookahead 각도
    delta = atan2(2.0 * wheelbase * sin(alpha), Ld)  # 조향각
    delta = clip(delta, -delta_max, delta_max)  # 제한
    return delta, alpha
```

**파라미터**:
- `x, y, theta`: 현재 차량 위치 및 방향
- `lookahead_pt`: Lookahead 포인트 [x, y]
- `wheelbase`: 차량 휠베이스 (m)
- `Ld`: Lookahead 거리 (m)
- `delta_max`: 최대 조향각 (rad)

#### 4. 속도 제어

**횡가속도 제한**:
```python
kappa = abs(tan(delta)) / wheelbase  # 곡률
v_max = sqrt(a_lat_max / kappa)      # 최대 속도
v_target = min(v_ref, v_max)
```

**조향 감속** (옵션):
```python
if use_steer_slowdown:
    v_target = v_ref * (1 - slowdown_gain * abs(delta) / delta_max)
```

---

## 새로운 맵 적용

새로운 맵을 적용하는 방법은 사용하는 launch 파일에 따라 다릅니다:

### 통합 런치 파일 사용 시 (권장)

`pure_pursuit_map_launch.py`와 같이 맵별 통합 런치 파일을 사용하는 경우:

1. **맵 파일 준비** (`f1tenth_gym_ros/maps/`)
   - `{맵이름}.png`: 맵 이미지
   - `{맵이름}.yaml`: 맵 메타데이터

2. **시뮬레이션 설정 파일 생성** (`f1tenth_gym_ros/config/`)
   - `sim_{맵이름}.yaml`: 시뮬레이션 설정 (시작 위치 포함)

3. **경로 CSV 파일 준비** (`pp_core/racelines/`)
   - `{맵이름}_raceline_vehicleaware.csv`: 레이스라인 경로

4. **런치 파일 생성** (`pp_core/launch/`)
   - `pure_pursuit_{맵이름}_launch.py`: 통합 런치 파일

### `pure_pursuit_with_sim_launch.py` 사용 시

기존 런치 파일을 수정하여 사용하려면:

1. **시뮬레이션 맵**: `f1tenth_gym_ros/config/sim.yaml` 수정
   ```yaml
   map_path: '/path/to/f1tenth_gym_ros/maps/{맵이름}'
   sx: <x좌표>  # raceline 첫 waypoint의 x_m 값
   sy: <y좌표>  # raceline 첫 waypoint의 y_m 값
   stheta: <방향>  # 첫 waypoint → 두 번째 waypoint 방향 (atan2 계산)
   ```
   맵 파일은 `f1tenth_gym_ros/maps/{맵이름}.png`, `{맵이름}.yaml`에 배치

2. **Map Server 맵**: `pp_core/maps/` 디렉토리에 파일 추가
   - `Budapest_map.png` → `{맵이름}_map.png`로 변경하거나
   - launch 파일의 56번 라인에서 경로 수정
   - `{맵이름}_map.yaml`에서 `image: {맵이름}_map.png`로 설정

3. **Raceline**: `pp_core/racelines/` 디렉토리에 파일 추가
   - launch 파일의 24번 라인에서 파일명 수정

### `pure_pursuit_launch.py` 사용 시

드라이버만 사용하는 경우:

1. **Raceline**: `pp_core/racelines/` 디렉토리에 파일 추가
   - launch 파일의 23번 라인에서 파일명 수정

**⚠️ 중요**: 
- `stheta`는 경로 진행 방향과 일치해야 합니다 (U-turn 방지)
- Raceline 좌표계는 시뮬레이션 맵 좌표계와 일치해야 합니다

**상세 가이드**: [MAP_SETUP_GUIDE.md](MAP_SETUP_GUIDE.md) 참고

---

## 참고 자료

### 관련 프로젝트

- **F1TENTH Gym**: [https://github.com/f1tenth/f1tenth_gym](https://github.com/f1tenth/f1tenth_gym)
- **f1tenth_gym_ros**: `sim_ws/src/f1tenth_gym_ros`
- **adaptive_lookahead**: 원본 Pure Pursuit 알고리즘 소스

### 문서

- [MAP_SETUP_GUIDE.md](MAP_SETUP_GUIDE.md): 새로운 맵 적용 가이드
- [INTEGRATION_CHECK.md](INTEGRATION_CHECK.md): 통합 확인 보고서
- [TEST_RESULTS.md](TEST_RESULTS.md): 테스트 결과

### 알고리즘 참고

- **Pure Pursuit**: "Implementation of the Pure Pursuit Path Tracking Algorithm" (R. Craig Coulter, 1992)
- **Ackermann Geometry**: Ackermann steering geometry

---

## 라이선스

MIT License

---

## 작성자

- **프로젝트**: Pure Pursuit Driver for F1TENTH Gym ROS2
- **기반**: adaptive_lookahead 프로젝트의 Pure Pursuit 알고리즘
- **통합**: f1tenth_gym_ros 시뮬레이터
