# Adaptive Lookahead for High-Speed F1TENTH Racing

**Paper**: "Failure Analysis and Curvature-Aware Redesign of Adaptive Lookahead for High-Speed Driving"  
**Venue**: IEEE Robotics and Automation Letters (RA-L), 2026  
**Authors**: Jin Hyun Kim

> Official code release for all experiments in the paper.  
> `git clone` → `colcon build` → `bash experiments/run_main_benchmark.sh` 로 논문 수치 완전 재현.

---

## Key Results (논문 Table II — Success Rate, N=30)

| Method | Budapest | Spielberg | ICRA | Skir | Austin | BrandsHatch | **Overall** |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **kw_sim** (Ours) | **100%** | **100%** | **100%** | **100%** | **100%** | **93.3%** | **99.4%** |
| alg1_org (Sukhil & Behl, 2021) | 0% | 16.7% | 13.3% | 33.3% | 13.3% | 0% | 12.8% |
| alg1_ext | — | — | — | — | — | — | ~13% |
| pp_kv | — | — | — | — | — | — | — |
| pp_curv | — | — | — | — | — | — | — |

Wilson 95% CI: kw_sim **99.4% [96.9, 99.9]** vs alg1_org 33.3% [26.9, 40.5]

---

## Quick Start

### Requirements
- Ubuntu 22.04 + ROS2 Humble
- Python 3.10+
- `pip install f1tenth-gym numpy scipy matplotlib pandas pyyaml pillow`

### Installation

```bash
git clone https://github.com/[YOUR_REPO]/adaptive_lookahead_pub
cd adaptive_lookahead_pub

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Run Experiments

```bash
# 메인 벤치마크 전체 (5 methods × 6 maps, N=30) — 약 6~8시간
bash experiments/run_main_benchmark.sh --trials 30

# 빠른 확인 (kw_sim, Budapest, N=3)
bash experiments/run_main_benchmark.sh --method kw_sim --map budapest --trials 3

# Ablation study
bash experiments/run_ablation.sh --trials 30

# 파라미터 민감도 스윕 (Budapest)
bash experiments/run_sensitivity.sh

# 통계 계산 (Wilson CI, Spearman r)
python3 analysis/compute_stats.py --results /tmp/f1tenth_eval/main \
    --raceline-dir src/pp_adaptive/racelines

# 논문 Figure 재생성
python3 analysis/gen_figures.py --output-dir results/figures
```

### Docker

```bash
docker build -t adaptive_lookahead_pub docker/
bash docker/docker_run.sh /tmp/my_results
```

---

## Package Structure

```
adaptive_lookahead_pub/
├── src/
│   ├── pp_adaptive/                     ← 핵심 패키지
│   │   ├── pp_adaptive/
│   │   │   ├── pure_pursuit_node.py     ← ROS2 드라이버 노드
│   │   │   ├── compute_adaptive_lookahead_kw2.py   ← kw_sim (Proposed)
│   │   │   ├── compute_adaptive_lookahead_org.py   ← alg1_org / alg1_ext
│   │   │   ├── compute_adaptive_lookahead_kv.py    ← pp_kv
│   │   │   └── compute_adaptive_lookahead_curv.py  ← pp_curv
│   │   ├── maps/           ← 6 maps (Budapest, Spielberg, ICRA, Skir, Austin, BrandsHatch)
│   │   └── racelines/      ← 30 CSVs (6 maps × 5 methods)
│   ├── f1tenth_driver_benchmark_suite/  ← 벤치마크 평가 인프라
│   ├── f1tenth_gym_ros/                 ← F1TENTH ROS2 시뮬레이터
│   └── pp_core/                         ← 기본 Pure Pursuit 드라이버
├── experiments/            ← 실험 실행 스크립트
├── analysis/               ← 통계 + Figure 생성
├── results/                ← 논문 기준값 CSV
└── docs/                   ← 상세 가이드
```

---

## Methods

| Code | Paper Name | Description |
|---|---|---|
| `kw_sim` | **Proposed** | κ-window + a_lat simulation scoring |
| `alg1_org` | Sukhil & Behl (2021) | Original Algorithm 1 reproduction |
| `alg1_ext` | Alg1-Ext | alg1_org with L={0.6,0.9,1.2,1.6,2.0}m |
| `kv` | PP-kv | Ld = k·v (k=0.15) |
| `curv` | PP-curv | Ld ∝ 1/√\|κ\|, α_h=0.35 |

자세한 알고리즘 설명: [docs/METHODS.md](docs/METHODS.md)

---

## Reproduce Paper Numbers

사전 계산된 논문 기준값: [results/main_benchmark/paper_reference_values.csv](results/main_benchmark/paper_reference_values.csv)

상세 재현 방법: [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md)  
설치 가이드: [docs/SETUP.md](docs/SETUP.md)

---

## Citation

```bibtex
@article{kim2026adaptive,
  title     = {Failure Analysis and Curvature-Aware Redesign of Adaptive Lookahead
               for High-Speed Driving},
  author    = {Kim, Jin Hyun},
  journal   = {IEEE Robotics and Automation Letters},
  year      = {2026},
  note      = {To appear}
}
```

---

## License

MIT License. See [LICENSE](LICENSE).
