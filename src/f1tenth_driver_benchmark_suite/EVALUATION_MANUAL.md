# F1TENTH Driver Benchmark 평가 실행 매뉴얼

이 문서는 `f1tenth_driver_benchmark_suite` 패키지로 드라이버 평가를 재현하는 절차를 정리합니다.

## 1) 평가 목적

- 동일 맵, 동일 레이스라인, 동일 메트릭 기준으로 드라이버 비교
- 비교 대상
- `rl` (`rl_f1tenth`)
- `pp_fixed` (`pp_core`)
- `pp_adaptive` (`pp_adaptive`)

## 2) 워크스페이스 구성

- Underlay(의존 패키지): `/home/jin/ros2_prj/sim_ws`
- Benchmark overlay: `/home/jin/ros2_prj/benchmark_ws`
- 평가 패키지: `/home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite`

## 3) 최초 1회 빌드

```bash
source /home/jin/ros2_prj/sim_ws/install/setup.bash
cd /home/jin/ros2_prj/benchmark_ws
colcon build --packages-select f1tenth_driver_benchmark_suite
```

## 4) 실행 전 공통 환경

매 터미널마다 아래를 먼저 실행합니다.

```bash
source /home/jin/ros2_prj/sim_ws/install/setup.bash
source /home/jin/ros2_prj/benchmark_ws/install/setup.bash
```

## 5) 빠른 시각화 테스트 (드라이버별)

### 5-1) RL (기본: tuned script 사용)

```bash
ros2 launch f1tenth_driver_benchmark_suite test_rl_visual.launch.py
```

옵션
- benchmark RL 노드로 실행: `use_tuned_script:=false`
- Matplotlib 실시간 평가창: `use_realtime_visualizer:=true`

예시:

```bash
ros2 launch f1tenth_driver_benchmark_suite test_rl_visual.launch.py \
  use_realtime_visualizer:=true
```

### 5-2) Pure Pursuit Fixed

```bash
ros2 launch f1tenth_driver_benchmark_suite test_pp_fixed_visual.launch.py
```

Matplotlib 실시간 평가창 포함:

```bash
ros2 launch f1tenth_driver_benchmark_suite test_pp_fixed_visual.launch.py \
  use_realtime_visualizer:=true
```

### 5-3) Pure Pursuit Adaptive

```bash
ros2 launch f1tenth_driver_benchmark_suite test_pp_adaptive_visual.launch.py
```

Matplotlib 실시간 평가창 포함:

```bash
ros2 launch f1tenth_driver_benchmark_suite test_pp_adaptive_visual.launch.py \
  use_realtime_visualizer:=true
```

## 6) 공식 비교 실행 (headless 권장)

단일 드라이버:

```bash
ros2 launch f1tenth_driver_benchmark_suite benchmark.launch.py \
  driver:=rl mode:=headless run_tag:=rl_headless_01
```

3개 드라이버 반복 평가(기본 runner):

```bash
ros2 run f1tenth_driver_benchmark_suite benchmark_runner \
  --drivers rl,pp_fixed,pp_adaptive \
  --trials 3 \
  --mode headless \
  --output-root /tmp/f1tenth_driver_benchmark_suite/results
```

설정파일 기반 `m laps x n trials` 평가(`f1tenth_driver_metrics` 사용):

```bash
ros2 run f1tenth_driver_benchmark_suite evaluation_runner \
  --config /home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/config/tests/headless_3lap_n3.yaml
```

## 7) 결과 경로와 파일

기본 결과 루트:
- `/tmp/f1tenth_driver_benchmark_suite/results`

실행별 디렉토리:
- `/tmp/f1tenth_driver_benchmark_suite/results/<driver>/<run_tag>/`

주요 파일:
- `laps/lap_*.json`, `laps/lap_*.csv`
- `runs/run_*.json`
- `run_report.md` (실행별 요약 리포트)
- 에러 발생 시 `error_*.json`

## 8) 결과 집계

```bash
ros2 run f1tenth_driver_benchmark_suite report_aggregator \
  --output-root /tmp/f1tenth_driver_benchmark_suite/results \
  --target-laps 3
```

집계 결과:
- `comparison_runs.csv`
- `comparison_summary.csv`
- `comparison_summary.md`

## 9) 주요 런치 파라미터

- `run_tag:=...` 결과 폴더 이름 구분
- `output_root:=...` 결과 저장 루트 변경
- `mode:=headless|visual`
- `use_realtime_visualizer:=true|false`
- `visualizer_update_rate:=2.0`
- `visualizer_max_history:=1000`
- `raceline_path:=...` 레이스라인 CSV 지정

## 10) 트러블슈팅

### 10-1) 실행 직후 종료됨

- 결과 폴더의 `error_*.json` 확인
- 예시:

```bash
ls -la /tmp/f1tenth_driver_benchmark_suite/results/pp_adaptive/<run_tag>/
cat /tmp/f1tenth_driver_benchmark_suite/results/pp_adaptive/<run_tag>/error_reverse_driving.json
```

### 10-2) RL이 움직이지 않음

- `test_rl_visual.launch.py` 기본은 tuned script 실행 모드
- benchmark RL 노드 테스트를 원하면:

```bash
ros2 launch f1tenth_driver_benchmark_suite test_rl_visual.launch.py use_tuned_script:=false
```

### 10-3) Matplotlib 창이 안 뜸

- `use_realtime_visualizer:=true` 확인
- X11/GUI 환경(`DISPLAY`) 확인
- headless 서버에서는 Matplotlib GUI 대신 결과 파일 기반 분석 권장

## 11) 권장 평가 순서

1. `test_*_visual`로 각 드라이버 주행 안정성 확인
2. 동일 조건으로 `benchmark_runner --mode headless` 수행
3. `report_aggregator`로 최종 비교표 생성
