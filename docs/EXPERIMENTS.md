# Experiment Reproduction Guide

## 사전 준비

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # adaptive_lookahead_pub 루트에서 build 후
```

---

## 1. 메인 벤치마크 (논문 Table II, III)

5 methods × 6 maps, N=30 trials.  
소요 시간: 약 6~8시간 (전체)

```bash
# 전체 실행
bash experiments/run_main_benchmark.sh --trials 30

# 특정 method/map만
bash experiments/run_main_benchmark.sh --method kw_sim --map budapest --trials 5

# 결과 통계 계산
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/main
```

## 2. Ablation Study (논문 Table IV)

kw_sim의 4가지 구성 요소(A~D) 제거 실험.  
소요 시간: 약 2~3시간

```bash
bash experiments/run_ablation.sh --trials 30
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/ablation
```

## 3. 파라미터 민감도 스윕 (논문 Fig. 6)

Budapest 맵, 5 파라미터 × 5~7 값 × N=10 trials.  
소요 시간: 약 3~4시간

```bash
# 전체
bash experiments/run_sensitivity.sh

# 특정 파라미터만
bash experiments/run_sensitivity.sh --param alat
bash experiments/run_sensitivity.sh --param alpha
bash experiments/run_sensitivity.sh --param tmargin
bash experiments/run_sensitivity.sh --param wcte
bash experiments/run_sensitivity.sh --param kw

# Figure 생성
python3 analysis/gen_figures.py --output-dir results/figures
```

## 4. Raceline 재생성 (선택사항)

포함된 `racelines/*.csv`를 직접 재생성하려면:

```bash
bash experiments/generate_racelines.sh --method kw_sim --map budapest
bash experiments/generate_racelines.sh  # 전체 (6 maps × 5 methods)
```

## 5. 단일 시뮬레이션 (시각화)

```bash
# simulation.yaml 편집하여 원하는 맵/raceline 설정 후:
ros2 launch pp_adaptive pp_adaptive_sim_launch.py
```

---

## 결과 디렉토리 구조

```
/tmp/f1tenth_eval/
└── main/
    └── kw_sim_budapest/
        └── pp_adaptive/
            ├── trial_001_*/
            │   ├── all_laps_summary.csv
            │   └── runs/run_*.json
            └── ...
```
