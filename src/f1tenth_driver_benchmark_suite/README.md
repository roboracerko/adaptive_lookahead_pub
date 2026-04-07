# f1tenth_driver_benchmark_suite

F1TENTH 드라이버 비교용 단일 패키지입니다.

상세 실행 절차는 `EVALUATION_MANUAL.md`를 참고하세요.

- 동일 맵: `assets/maps/Budapest.*`
- 동일 레이스라인: `assets/racelines/Budapest_raceline_vehicleaware.csv`
- 동일 평가 메트릭: 패키지 내부 `metrics_collector_node`
- RViz2 모드 지원: `mode:=visual`

## 비교 대상 드라이버

- `rl` (`rl_f1tenth`)
- `pp_fixed` (`pp_core`)
- `pp_adaptive` (`pp_adaptive`)

## 빌드

```bash
cd /home/jin/ros2_prj/sim_ws
colcon build --packages-select f1tenth_driver_benchmark_suite
source install/setup.bash
```

## 단일 실행

```bash
# Headless (공식 비교 권장)
ros2 launch f1tenth_driver_benchmark_suite benchmark.launch.py driver:=rl mode:=headless

# RViz2 시각화 포함
ros2 launch f1tenth_driver_benchmark_suite benchmark.launch.py driver:=pp_adaptive mode:=visual

# RViz2 + Matplotlib 실시간 평가 플롯 동시 실행
ros2 launch f1tenth_driver_benchmark_suite benchmark.launch.py \
  driver:=pp_adaptive mode:=visual use_realtime_visualizer:=true
```

## 반복 실행

```bash
ros2 run f1tenth_driver_benchmark_suite benchmark_runner \
  --drivers rl,pp_fixed,pp_adaptive \
  --trials 3 \
  --mode headless
```

## 설정파일 기반 평가 (m랩 x n회)

`f1tenth_driver_metrics` 메트릭을 사용한 headless 평가를 실행합니다.

```bash
ros2 run f1tenth_driver_benchmark_suite evaluation_runner \
  --config /home/jin/ros2_prj/benchmark_ws/src/f1tenth_driver_benchmark_suite/config/tests/headless_3lap_n3.yaml
```

기본 결과:
- 실행별 리포트: `<output_root>/<driver>/<run_tag>/run_report.md`
- 종합 테이블: `<output_root>/comparison_runs.csv`, `<output_root>/comparison_summary.csv`, `<output_root>/comparison_summary.md`

## 결과 집계

```bash
ros2 run f1tenth_driver_benchmark_suite report_aggregator \
  --output-root /tmp/f1tenth_driver_benchmark_suite/results
```

집계 파일:

- `comparison_runs.csv`
- `comparison_summary.csv`
- `comparison_summary.md`

## 주요 파라미터

- 공통 메트릭: `config/benchmark_common.yaml`
- 드라이버별 메트릭 오버라이드: `config/drivers/*.yaml`
- 시나리오(맵/시뮬): `config/scenarios/budapest.yaml`
