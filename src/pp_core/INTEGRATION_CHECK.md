# Pure Pursuit Driver 통합 확인 보고서

## ✅ 시뮬레이터 실행 가능성: **가능**

`pp_core`는 `f1tenth_gym_ros` 시뮬레이터와 완전히 호환됩니다.

## 🔍 통합 확인 사항

### 1. 토픽 호환성 ✅

| 항목 | pp_core | f1tenth_gym_ros | 상태 |
|------|-------------------|-----------------|------|
| **구독 토픽** | `/ego_racecar/odom` | `/ego_racecar/odom` 발행 | ✅ 일치 |
| **발행 토픽** | `/drive` | `/drive` 구독 | ✅ 일치 |
| **메시지 타입** | `AckermannDriveStamped` | `AckermannDriveStamped` | ✅ 일치 |

### 2. 맵 파일 통합 ✅

**이동된 파일**:
- ✅ `Budapest_map.png` → `f1tenth_gym_ros/maps/Budapest_map.png`
- ✅ `Budapest_map.yaml` → `f1tenth_gym_ros/maps/Budapest_map.yaml`

**설정 파일**:
- ✅ `sim_budapest.yaml` 생성 (Budapest 맵용 설정)
  - `map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/Budapest'`
  - 시작 위치: 레이스라인 첫 번째 웨이포인트로 설정

### 3. 런치 파일 ✅

**생성된 런치 파일**:
- ✅ `pure_pursuit_map_launch.py`: 맵 통합 Pure Pursuit 드라이버 런치

**포함된 노드**:
1. `gym_bridge`: F1TENTH Gym 시뮬레이션
2. `map_server`: 맵 서버 (`sim_config`의 `map_path`)
3. `pp_core`: Pure Pursuit 제어 드라이버
4. `rviz2`: 시각화
5. `robot_state_publisher`: 로봇 상태 발행
6. `nav2_lifecycle_manager`: 맵 서버 생명주기 관리

### 4. 경로 데이터 ✅

**경로 파일**:
- ✅ `Budapest_raceline_vehicleaware.csv` (pp_core/racelines/)
- 형식: `x_m, y_m, theta_rad, kappa_1pm, v_mps`
- 총 602 웨이포인트

## 🚀 실행 방법

### 방법 1: 통합 런치 파일 사용 (권장)

```bash
cd sim_ws
source install/setup.bash
ros2 launch pp_core pure_pursuit_map_launch.py
```

### 방법 2: 개별 실행

```bash
# 터미널 1: 시뮬레이션
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 터미널 2: Pure Pursuit 드라이버
ros2 run pp_core pp_core \
  --ros-args \
  -p raceline_csv:=$(ros2 pkg prefix pp_core)/share/pp_core/racelines/Budapest_raceline_vehicleaware.csv
```

## ⚙️ 설정 확인

### sim_budapest.yaml 주요 설정

```yaml
bridge:
  ros__parameters:
    # 맵 설정
    map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/Budapest'
    
    # 시작 위치 (레이스라인 첫 웨이포인트)
    sx: -3.1767114354150507
    sy: 2.556277889421665
    stheta: -2.4580908231582286
    
    # 단일 에이전트
    num_agent: 1
    
    # 키보드 텔레옵 비활성화
    kb_teleop: False
```

### pure_pursuit.yaml 주요 설정

```yaml
pp_core:
  ros__parameters:
    # 고정 lookahead 거리
    lookahead_distance: 1.0  # m
    
    # 차량 파라미터 (F1TENTH Gym 기본값)
    wheelbase: 0.3302  # m
    delta_max_deg: 24.0  # deg
    
    # 제어 주기
    control_hz: 50.0  # Hz
```

## 📊 데이터 흐름

```
[F1TENTH Gym 시뮬레이션]
    ↓
/ego_racecar/odom (차량 상태)
    ↓
[Pure Pursuit Driver]
    ↓ (경로 추종 계산)
/drive (제어 명령: speed, steering_angle)
    ↓
[F1TENTH Gym 시뮬레이션]
    ↓
차량 이동
```

## ✅ 통합 체크리스트

- [x] 맵 파일 이동 (Budapest_map.png, Budapest_map.yaml)
- [x] 시뮬레이션 설정 파일 생성 (sim_budapest.yaml)
- [x] 통합 런치 파일 생성 (pure_pursuit_map_launch.py)
- [x] 토픽 호환성 확인
- [x] 메시지 타입 확인
- [x] 경로 데이터 확인
- [x] 시작 위치 설정

## 🎯 다음 단계

1. **실행 테스트**: `ros2 launch pp_core pure_pursuit_map_launch.py`
2. **파라미터 튜닝**: `lookahead_distance` 조정으로 성능 최적화
3. **성능 모니터링**: RViz에서 경로 추종 상태 확인

## 📝 참고

- 맵 파일 위치: `f1tenth_gym_ros/maps/`
- 경로 파일 위치: `pp_core/racelines/`
- 설정 파일: `f1tenth_gym_ros/config/sim_budapest.yaml`
- 런치 파일: `pp_core/launch/pure_pursuit_map_launch.py`
