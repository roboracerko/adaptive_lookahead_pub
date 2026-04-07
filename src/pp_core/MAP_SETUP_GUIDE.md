# 새로운 맵 적용 가이드

새로운 맵을 적용하기 위해 수정이 필요한 파일과 위치를 안내합니다.

## 📋 체크리스트

### 1. 맵 파일 추가 (필수)

**위치**: `f1tenth_gym_ros/maps/`

**필요한 파일**:
- `{맵이름}.png` - 맵 이미지 파일
- `{맵이름}.yaml` - 맵 메타데이터 파일

**예시** (Austin 맵을 추가하는 경우):
```bash
sim_ws/src/f1tenth_gym_ros/maps/
├── Austin.png          # 새로 추가
├── Austin.yaml         # 새로 추가
├── Budapest.png
├── Budapest.yaml
└── ...
```

**YAML 파일 형식** (`Austin.yaml`):
```yaml
image: Austin.png
resolution: 0.05        # 맵 해상도 (m/pixel)
origin: [-50.0, -50.0, 0.0]  # 원점 [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**⚠️ 주의사항**:
- 파일명은 확장자 없이 동일해야 함 (`Austin.png`, `Austin.yaml`)
- YAML 파일의 `image` 필드는 PNG 파일명만 지정 (예: `Austin.png`)

---

### 2. 시뮬레이션 설정 파일 생성/수정 (필수)

**위치**: `f1tenth_gym_ros/config/`

**파일**: `sim_{맵이름}.yaml` (예: `sim_austin.yaml`)

**기존 파일 참고**: `sim_budapest.yaml`을 복사하여 수정

**수정 항목**:
```yaml
bridge:
  ros__parameters:
    # map parameters - {맵이름} map
    map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/{맵이름}'
    map_img_ext: '.png'

    # ego starting pose on map ({맵이름} track starting point)
    # ⚠️ 중요: stheta는 경로 진행 방향과 일치해야 합니다 (U-turn 방지)
    sx: <x 좌표>      # 시작 X 좌표 (m) - 레이스라인 첫 waypoint의 x_m 값
    sy: <y 좌표>      # 시작 Y 좌표 (m) - 레이스라인 첫 waypoint의 y_m 값
    stheta: <각도>    # 시작 각도 (rad) - 첫 waypoint에서 두 번째 waypoint로의 방향 (atan2 계산)
```

**예시** (`sim_austin.yaml`):
```yaml
bridge:
  ros__parameters:
    # ... (다른 설정은 동일) ...
    
    # map parameters - Austin map
    map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/Austin'
    map_img_ext: '.png'

    # ego starting pose on map (Austin track starting point)
    # 주의: stheta는 경로 진행 방향과 일치해야 합니다 (U-turn 방지)
    sx: 0.0
    sy: 0.0
    stheta: 0.0  # 경로 시작 방향 (첫 waypoint → 두 번째 waypoint 방향)
```

**💡 시작 위치 찾기**:
- **sx, sy**: 레이스라인 CSV 파일의 첫 번째 웨이포인트 좌표 사용 (`x_m`, `y_m`)
- **stheta**: 첫 번째 waypoint에서 두 번째 waypoint로의 방향 계산
  ```python
  import math
  # CSV에서 첫 두 waypoint 읽기
  x1, y1 = 첫_waypoint의_x_m, 첫_waypoint의_y_m
  x2, y2 = 두번째_waypoint의_x_m, 두번째_waypoint의_y_m
  dx = x2 - x1
  dy = y2 - y1
  stheta = math.atan2(dy, dx)  # 경로 진행 방향 (rad)
  ```
- **⚠️ U-turn 방지**: `stheta`가 경로 진행 방향과 일치하지 않으면 차량이 U-turn을 시도합니다

---

### 3. 경로 CSV 파일 준비 (필수)

**위치**: `pp_core/racelines/`

**파일**: `{맵이름}_raceline_vehicleaware.csv`

**필수 컬럼**:
```csv
x_m,y_m,theta_rad,kappa_1pm,v_mps
0.0,0.0,0.0,0.0,3.0
0.5,0.1,0.1,0.05,5.0
...
```

**컬럼 설명**:
- `x_m`: X 좌표 (m) - **필수**
- `y_m`: Y 좌표 (m) - **필수**
- `v_mps`: 속도 프로파일 (m/s) - **선택** (없으면 기본값 3.0 m/s 사용)
- `theta_rad`: 각도 (rad) - 선택
- `kappa_1pm`: 곡률 (1/m) - 선택

**예시**:
```bash
sim_ws/src/pp_core/racelines/
├── Austin_raceline_vehicleaware.csv    # 새로 추가
├── Budapest_raceline_vehicleaware.csv
└── ...
```

---

### 4. 런치 파일 생성/수정 (필수)

**방법 A: 기존 통합 런치 파일 사용 (권장)**

**파일**: `pure_pursuit_map_launch.py`

**맵 변경**: `sim_config` 런치 인자로 맵 설정 파일을 지정합니다.
```bash
ros2 launch pp_core pure_pursuit_map_launch.py sim_config:=/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/config/sim_{맵이름}.yaml
```

**레이스라인 변경**: `pp_core/config/pure_pursuit.yaml`의 `raceline_csv`를 수정합니다.

**방법 B: 맵별 런치 파일이 필요하면**
- `pure_pursuit_map_launch.py`를 복사해서 `sim_config` 기본값만 바꿔 사용하세요.

**방법 B: 런치 파일 파라미터로 변경**

기존 런치 파일 사용 시 런치 인자로 전달:
```bash
ros2 launch pp_core pure_pursuit_launch.py \
  sim_config:=sim_austin.yaml \
  raceline_csv:=Austin_raceline_vehicleaware.csv
```
*(이 방법은 런치 파일 수정이 필요함)*

---

### 5. Pure Pursuit 설정 파일 수정 (선택)

**위치**: `pp_core/config/`

**파일**: `pure_pursuit.yaml`

**수정 항목** (기본값 변경 시):
```yaml
pp_core:
  ros__parameters:
    # 경로 설정 (런치 파일에서 오버라이드되므로 선택적)
    raceline_csv: '{맵이름}_raceline_vehicleaware.csv'
    
    # Pure Pursuit 파라미터 (필요시 조정)
    lookahead_distance: 1.0  # 고정 lookahead 거리 (m)
    # ... 기타 설정 ...
```

**💡 참고**: 런치 파일에서 `raceline_csv` 파라미터를 오버라이드하므로 이 파일 수정은 선택사항입니다.

---

## 📝 단계별 작업 순서

### Step 1: 맵 파일 준비
```bash
# 1. 맵 이미지와 YAML 파일을 maps 디렉토리에 복사
cp <새맵>.png sim_ws/src/f1tenth_gym_ros/maps/
cp <새맵>.yaml sim_ws/src/f1tenth_gym_ros/maps/
```

### Step 2: 시뮬레이션 설정 파일 생성
```bash
# 기존 설정 파일을 복사
cp sim_ws/src/f1tenth_gym_ros/config/sim_budapest.yaml \
   sim_ws/src/f1tenth_gym_ros/config/sim_{맵이름}.yaml

# 설정 파일 수정
# - map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/{맵이름}'
# - sx, sy: 레이스라인 첫 waypoint 좌표
# - stheta: 경로 진행 방향 (첫 waypoint → 두 번째 waypoint 방향, atan2 계산)
```

### Step 3: 경로 CSV 파일 준비
```bash
# 경로 CSV 파일을 racelines 디렉토리에 복사
cp <경로파일>.csv sim_ws/src/pp_core/racelines/{맵이름}_raceline_vehicleaware.csv
```

### Step 4: 런치 파일 생성
```bash
# 기존 런치 파일을 복사
cp sim_ws/src/pp_core/launch/pure_pursuit_map_launch.py \
   sim_ws/src/pp_core/launch/pure_pursuit_{맵이름}_launch.py

# 런치 파일 수정:
# - sim_config 경로
# - default_raceline 경로
```

### Step 5: 빌드 및 테스트
```bash
cd sim_ws
colcon build --packages-select pp_core f1tenth_gym_ros
source install/setup.bash
ros2 launch pp_core pure_pursuit_{맵이름}_launch.py
```

---

## 📂 파일 구조 요약

새로운 맵(Austin 예시) 적용 후 파일 구조:

```
sim_ws/src/
├── f1tenth_gym_ros/
│   ├── maps/
│   │   ├── Austin.png          # ✅ 새로 추가
│   │   ├── Austin.yaml         # ✅ 새로 추가
│   │   └── ...
│   └── config/
│       ├── sim_austin.yaml     # ✅ 새로 생성
│       └── ...
│
└── pp_core/
    ├── racelines/
    │   ├── Austin_raceline_vehicleaware.csv  # ✅ 새로 추가
    │   └── ...
    └── launch/
        ├── pure_pursuit_austin_launch.py     # ✅ 새로 생성
        └── ...
```

---

## 🔧 실제 예시: Austin 맵 추가

### 1. 맵 파일 확인
```bash
ls sim_ws/src/f1tenth_gym_ros/maps/ | grep -i austin
# adaptive_lookahead 프로젝트에 Austin 맵이 있다면 복사
cp adaptive_lookahead/data/maps/Austin_map.png sim_ws/src/f1tenth_gym_ros/maps/Austin.png
cp adaptive_lookahead/config/maps/Austin_map.yaml sim_ws/src/f1tenth_gym_ros/maps/Austin.yaml
# Austin.yaml 파일의 image 경로 수정 필요: "image: Austin.png"
```

### 2. 설정 파일 생성
```bash
# sim_austin.yaml 생성 및 수정
# map_path: '/home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros/maps/Austin'
# sx, sy: 레이스라인 첫 waypoint 좌표 (x_m, y_m)
# stheta: 경로 진행 방향 계산 (첫 waypoint → 두 번째 waypoint, atan2 사용)
#   예: python3 -c "import math; print(math.atan2(y2-y1, x2-x1))"
```

### 3. 경로 파일 확인
```bash
# adaptive_lookahead 프로젝트에서 경로 파일 복사
cp adaptive_lookahead/data/racelines/Austin_raceline_vehicleaware.csv \
   sim_ws/src/pp_core/racelines/
```

### 4. 런치 파일 생성
```bash
# pure_pursuit_austin_launch.py 생성
# - sim_config: 'sim_austin.yaml'
# - default_raceline: 'Austin_raceline_vehicleaware.csv'
```

---

## ⚠️ 주의사항

1. **파일명 일치**: 
   - 맵 파일명은 확장자 없이 동일해야 함 (`Austin.png`, `Austin.yaml`)
   - 런치 파일에서 `{맵이름}.yaml`로 참조

2. **경로 설정**:
   - `map_path`는 **절대 경로**를 사용 (확장자 없음)
   - 런치 파일에서 자동으로 `.yaml` 추가

3. **시작 위치**:
   - 레이스라인 CSV의 첫 웨이포인트와 일치하도록 설정
   - 맵 좌표계 기준
   - **⚠️ 중요**: `stheta`는 경로 진행 방향(첫 waypoint → 두 번째 waypoint)과 일치해야 합니다
   - `stheta`가 잘못 설정되면 차량이 U-turn을 시도하여 경로 추종이 실패할 수 있습니다

4. **경로 CSV 형식**:
   - `x_m`, `y_m` 컬럼은 필수
   - `v_mps` 컬럼이 없으면 기본값 3.0 m/s 사용

5. **빌드 필요**:
   - 새로운 파일 추가 후 `colcon build` 필요
   - `setup.py`에 자동으로 포함됨 (glob 패턴 사용)

---

## 📚 참고

- 예시(Budapest): `sim_budapest.yaml`, `pure_pursuit_map_launch.py`
- 맵 파일 위치: `f1tenth_gym_ros/maps/`
- 경로 파일 위치: `pp_core/racelines/`
- 설정 파일 위치: `f1tenth_gym_ros/config/`
- 런치 파일 위치: `pp_core/launch/`
