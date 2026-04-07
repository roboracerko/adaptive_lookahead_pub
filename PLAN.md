# 공개 패키지 구성 계획서

**논문**: "Failure Analysis and Curvature-Aware Redesign of Adaptive Lookahead for High-Speed Driving"  
**대상 저널**: IEEE Robotics and Automation Letters (RA-L) 2026  
**작성일**: 2026-04-05  
**목적**: 논문의 모든 실험을 재현 가능한 독립적 ROS2 패키지 공개

---

## 1. 목표 및 원칙

### 1.1 목표
- `git clone` + `colcon build` + `bash run_all.sh` 로 논문의 모든 실험 수치 재현 가능
- 외부 의존성 최소화 (표준 ROS2 Humble + f1tenth_gym_ros 공개 패키지만 의존)
- 내부 개발 중간물(report/, raw 실험 로그, 개인 스크립트) 완전 제외
- Docker 이미지 제공으로 환경 독립성 보장

### 1.2 포함 범위 (논문 기준)
| 실험 | 구성 | 트라이얼 수 |
|---|---|---|
| 메인 벤치마크 | 5 methods × 6 maps | N=30/map |
| Ablation | 4 variants × 2~3 maps | N=30/map |
| 파라미터 민감도 | 5 params × 5-7 값 × Budapest | N=10/값 |

### 1.3 포함 method 목록
| 코드명 | 논문 명칭 | 설명 |
|---|---|---|
| `kw_sim` | Proposed (kw_sim) | κ-window + a_lat simulation scoring |
| `alg1_org` | Sukhil & Behl (2021) | 원본 Algorithm 1 재현 |
| `alg1_ext` | Alg1-Ext | alg1_org + 확장 L set {0.6,0.9,1.2,1.6,2.0} |
| `pp_kv` | PP-kv | Ld = k·v (k=0.15) |
| `pp_curv` | PP-curv | Ld ∝ 1/√|κ|, α_h=0.35 |

### 1.4 포함 맵 (6개)
Budapest, Spielberg, icra, skir, Austin_map, BrandsHatch_map

---

## 2. 디렉토리 구조 설계

```
adaptive_lookahead_pub/          ← git 루트 (이 폴더)
├── README.md                    ← 설치·실행 가이드 (최종 작성)
├── CITATION.md                  ← 논문 인용 정보
├── LICENSE                      ← MIT
├── .gitignore
│
├── docker/
│   ├── Dockerfile               ← ROS2 Humble + gym 환경
│   └── docker_run.sh
│
├── docs/
│   ├── SETUP.md                 ← 상세 설치 가이드
│   ├── EXPERIMENTS.md           ← 실험별 실행 방법
│   ├── METHODS.md               ← 각 method 알고리즘 설명
│   └── RESULTS.md               ← 논문 수치 요약 (재현 기준값)
│
├── src/                         ← ROS2 workspace src/
│   ├── pp_adaptive/             ← 핵심 패키지 (아래 3.1 참조)
│   ├── f1tenth_driver_metrics/  ← 평가 메트릭 패키지 (복사)
│   ├── pp_core/                 ← Pure Pursuit 기본 드라이버
│   └── f1tenth_gym_ros/         ← git submodule (공개 repo)
│
├── experiments/                 ← 실험 실행 스크립트
│   ├── run_main_benchmark.sh    ← 메인 벤치마크 전체 실행
│   ├── run_ablation.sh          ← Ablation 실험 실행
│   ├── run_sensitivity.sh       ← 파라미터 민감도 스윕
│   ├── generate_racelines.sh    ← raceline labeling (전처리)
│   └── generate_figures.sh      ← 논문 figure 생성
│
├── results/                     ← 사전 계산된 결과 (요약 CSV만)
│   ├── main_benchmark/
│   │   └── summary_all_methods_6maps.csv
│   ├── ablation/
│   │   └── summary_ablation.csv
│   └── sensitivity/
│       └── summary_sensitivity_budapest.csv
│
└── analysis/                    ← 분석 스크립트
    ├── compute_stats.py         ← Wilson CI, Spearman r 계산
    ├── gen_figures.py           ← 논문 Figure 생성 (matplotlib)
    └── requirements.txt         ← numpy, scipy, matplotlib, pandas
```

---

## 3. 핵심 패키지 구성: `src/pp_adaptive/`

### 3.1 포함 파일 (sim_ws/src/pp_adaptive/ 에서 선별)

```
src/pp_adaptive/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│
├── pp_adaptive/                 ← Python source (핵심)
│   ├── __init__.py
│   ├── pure_pursuit_node.py     ← 메인 드라이버 노드
│   ├── compute_adaptive_lookahead_kw2.py    ← 제안방법 (kw_sim)
│   ├── compute_adaptive_lookahead_org.py    ← alg1_org baseline
│   ├── compute_adaptive_lookahead_kv.py     ← pp_kv baseline
│   ├── compute_adaptive_lookahead_curv.py   ← pp_curv baseline
│   └── utils/
│       ├── __init__.py
│       ├── map_loader.py
│       └── math_utils.py
│
├── config/
│   ├── simulation.yaml          ← 시뮬레이션 공통 설정
│   ├── pure_pursuit.yaml        ← kw_sim 기본 파라미터
│   │
│   ├── methods/                 ← method별 파라미터 (6 maps × 5 methods)
│   │   ├── kw_sim_{map}.yaml    ← (revision/ → methods/ 로 재구성)
│   │   ├── alg1_org_{map}.yaml
│   │   ├── alg1_ext_{map}.yaml
│   │   ├── kv_{map}.yaml
│   │   └── curv_{map}.yaml
│   │
│   ├── benchmarks/              ← 벤치마크 실행 설정
│   │   ├── main/                ← 6 maps × 5 methods 벤치마크 yaml
│   │   ├── ablation/            ← ablation 벤치마크 yaml
│   │   └── sensitivity/         ← sensitivity sweep yaml
│   │
│   └── scenarios/               ← 시뮬레이터 시나리오 (맵별)
│       ├── budapest.yaml
│       ├── spielberg.yaml
│       ├── icra.yaml
│       ├── skir.yaml
│       ├── austin.yaml
│       └── brandshatch.yaml
│
├── launch/
│   ├── pp_adaptive_sim_launch.py       ← 메인 런치
│   └── pure_pursuit_adaptive_launch.py ← 드라이버만 런치
│
├── maps/                        ← 6개 맵 파일 (yaml + png/pgm)
│   ├── Budapest_map.{yaml,png}
│   ├── Spielberg.{yaml,png}
│   ├── icra.{yaml,png}
│   ├── skir.{yaml,pgm}
│   ├── Austin_map.{yaml,png}
│   └── BrandsHatch_map.{yaml,png}
│
├── racelines/                   ← 6 maps × 5 methods = 30개 CSV
│   ├── Budapest_map_optimal_rl_kw_sim.csv
│   ├── Budapest_map_optimal_rl_alg1_org.csv
│   ├── Budapest_map_optimal_rl_alg1_ext.csv
│   ├── Budapest_map_optimal_rl_kv.csv
│   ├── Budapest_map_optimal_rl_curv.csv
│   └── ... (나머지 5 maps × 5 methods)
│
└── scripts/
    ├── run_pipeline.sh          ← raceline labeling 파이프라인
    ├── generate_revision_racelines.sh
    └── measure_offline_runtime.py
```

### 3.2 제외 파일 (공개 버전에서 삭제)
- `report/` 전체 (개발 중간 결과물, ~수십 GB)
- `config/ablation/` 내 noA/noB/noC/noD 파일 → `config/benchmarks/ablation/` 로 재구성
- `2111.08873v1.pdf` (저작권 문제)
- `run_kw2_from_ld_compare.*`, `run_raceline_15_compare.*` (내부 비교 스크립트)
- `tune_kw2_candidates.py` (튜닝 중간물)
- `compute_adaptive_lookahead.py`, `compute_adaptive_lookahead_hs.py`, `compute_adaptive_lookahead_kw.py` (deprecated)
- `launch/__pycache__/`, `pp_adaptive/__pycache__/` 등 캐시
- `config/pipeline_input.yaml` (내부 개발용)

---

## 4. 의존성 처리 계획

### 4.1 포함 의존 패키지 (src/ 에 직접 포함)
| 패키지 | 출처 | 처리 방식 |
|---|---|---|
| `f1tenth_gym_ros` | 공개 GitHub (f1tenth-org) | git submodule (humble 브랜치) |
| `f1tenth_driver_metrics` | 우리 개발 | src/에 복사, 정리 |
| `pp_core` | 우리 개발 | src/에 복사 |

> **Note**: `f1tenth_gym_ros` 공개 URL: `https://github.com/f1tenth/f1tenth_gym_ros`  
> humble 브랜치 최신 커밋을 기준으로 테스트 후 submodule 커밋 고정

### 4.2 제외 의존 패키지 (pp_adaptive에서 미사용 확인 후)
- `reference_waypoint_loader` (pp_adaptive가 직접 사용하지 않음 - 확인 필요)
- `f1tenth_frenet_msgs`, `kmpc_driver`, `svg_mppi_driver` (완전 무관)
- `rl_f1tenth` (무관)

### 4.3 시스템 의존성 (README에 명시)
```
ROS2 Humble (Ubuntu 22.04)
Python 3.10+
pip: numpy, pyyaml, pillow, scipy, matplotlib, pandas
f1tenth_gym: pip install f1tenth-gym  (or from gym submodule)
```

---

## 5. 실험 재현 절차 (최종 사용자 관점)

```bash
# 1. Clone
git clone --recurse-submodules https://github.com/[user]/adaptive_lookahead_pub
cd adaptive_lookahead_pub

# 2. Build
colcon build --symlink-install
source install/setup.bash

# 3. (Option A) 메인 벤치마크 전체 실행 (~6-8시간)
bash experiments/run_main_benchmark.sh --trials 30

# 3. (Option B) 결과 확인만 (사전 계산된 결과 사용)
python3 analysis/compute_stats.py --results results/main_benchmark/

# 4. Figure 생성
python3 analysis/gen_figures.py --output figs/
```

---

## 6. 작업 체크리스트

### Phase 1: 파일 선별 및 구조 생성 (이 단계)
- [ ] `src/pp_adaptive/` 폴더 구조 생성
- [ ] 핵심 Python 소스 복사 (5개 labeler + node + utils)
- [ ] maps/ 복사 (6 maps)
- [ ] racelines/ 복사 (30개 CSV: 6maps × 5methods)
- [ ] config/ 재구성 (methods/, benchmarks/, scenarios/ 계층 정리)
- [ ] launch/ 복사 및 경로 정리

### Phase 2: 의존 패키지 정리
- [ ] `f1tenth_gym_ros` git submodule 설정
- [ ] `f1tenth_driver_metrics` 복사 및 정리 (내부 경로 참조 제거)
- [ ] `pp_core` 복사
- [ ] `colcon build` 성공 확인

### Phase 3: 실험 스크립트 작성
- [ ] `experiments/run_main_benchmark.sh` (6 maps × 5 methods)
- [ ] `experiments/run_ablation.sh` (4 ablation variants)
- [ ] `experiments/run_sensitivity.sh` (5 params, Budapest)
- [ ] `experiments/generate_racelines.sh` (raceline 전처리 재현)

### Phase 4: 분석 및 Figure 스크립트
- [ ] `analysis/compute_stats.py` (Wilson CI, Spearman r, LD 분포 등)
- [ ] `analysis/gen_figures.py` (논문 Figure 1~8 재생성)
- [ ] `results/` 사전 계산 결과 CSV 생성 및 배치

### Phase 5: 문서화
- [ ] `README.md` (설치, 빠른 시작, 논문 수치 대조표)
- [ ] `docs/SETUP.md` (상세 설치 가이드)
- [ ] `docs/EXPERIMENTS.md` (실험별 실행 방법)
- [ ] `docs/METHODS.md` (알고리즘 설명, 논문 수식 참조)
- [ ] `docs/RESULTS.md` (재현 기준값: 논문 Table 내 수치)
- [ ] `CITATION.md`

### Phase 6: Docker 및 최종 검증
- [ ] `docker/Dockerfile` 작성
- [ ] 클린 환경에서 end-to-end 재현 테스트
- [ ] `.gitignore` 작성 (build/, install/, log/, report/, __pycache__ 등)
- [ ] git init + 초기 커밋

---

## 7. 주요 설계 결정사항 및 근거

### 7.1 config 계층 재구성
- 현재: `config/revision/benchmarks/kw_sim_budapest_n30.yaml` (개발 맥락 이름)
- 공개: `config/benchmarks/main/kw_sim_budapest_n30.yaml` (직관적)
- 이유: "revision"은 내부 개발 용어, 외부 사용자에게 혼란

### 7.2 raceline 파일명 유지
- 현재 이름 `Budapest_map_optimal_rl_kw_sim.csv` 유지 (변경 불필요)
- kw2 → kw_sim으로 이미 논문에서 통일됨 (`kw2_tuned_auto` → 논문명 `kw_sim`)
- 단, `kw2_tuned_auto` 파일을 `kw_sim`으로 심볼릭 또는 리네임 필요

### 7.3 report/ 완전 제외
- 용량 수십 GB, 개인 분석 중간물
- 논문에 필요한 수치는 `results/` 요약 CSV로 대체

### 7.4 alg1_ext 포함
- 논문 Table에 포함된 방법 (구조적 실패 증명 역할)
- `compute_adaptive_lookahead_org.py` + 설정 파일로 재현 가능

### 7.6 f1tenth_gym_ros git 이력 없음
- 현재 sim_ws/src/f1tenth_gym_ros는 git repo가 아님 (단순 복사본)
- 공개 버전에서는 공식 f1tenth-org GitHub repo의 humble 브랜치를 submodule로 사용
- 현재 사용 중인 파일과 공개 repo 간 diff 확인 후 패치 여부 결정

### 7.7 kw2_tuned_auto vs kw_sim raceline
- 두 파일이 모두 존재: `_kw2_tuned_auto.csv`, `_kw_sim.csv`
- 논문 기준: `_kw_sim.csv` 만 포함, `_kw2_tuned_auto.csv` 제외

### 7.5 pp_adaptive 내부 코드명 vs 논문 명칭 매핑
| 내부 코드명 | 파일명 접미사 | 논문 명칭 |
|---|---|---|
| kw2 / kw_sim | `_kw_sim` | Proposed (kw_sim) |
| alg1_org | `_alg1_org` | Sukhil & Behl (2021) |
| alg1_ext | `_alg1_ext` | Alg1-Ext |
| kv | `_kv` | PP-kv |
| curv | `_curv` | PP-curv |

---

## 8. 리스크 및 주의사항

### 8.1 f1tenth_gym_ros 버전 고정
- 공개 repo의 특정 커밋을 submodule로 고정해야 재현성 보장
- 현재 사용 중인 버전 확인 후 커밋 해시 기록 필요

### 8.2 `tracksplit_processed` 맵 파일
- 일부 맵은 `_tracksplit_processed.yaml/png` 파일이 별도 존재
- 런치 파일에서 어느 것을 사용하는지 확인 후 포함 여부 결정

### 8.3 raceline `kw2_tuned_auto` 파일명
- 현재 `Budapest_map_optimal_rl_kw2_tuned_auto.csv`로 저장됨
- 논문 명칭 `kw_sim`과 일치하는 `_kw_sim.csv` 파일이 별도 존재 → 후자 사용
- `kw2_tuned_auto` 파일은 제외

### 8.4 f1tenth_driver_metrics 내부 경로 참조
- benchmark 실행 스크립트에서 `sim_ws` 절대 경로 참조 가능성
- 상대 경로 또는 ROS2 패키지 경로로 교체 필요

### 8.5 sensitivity sweep 설정
- 현재 sensitivity sweep yaml이 `config/revision/` 내에 있음
- 이를 `config/benchmarks/sensitivity/`로 재구성 필요
- sweep 파라미터 범위를 README에 명시

---

## 9. 파일 크기 및 git 관리

### 9.1 대용량 파일 처리
- maps/ 파일: .png/.pgm 파일, 개별 <5MB → git 직접 포함 가능
- racelines/ CSV: 파일당 수십~수백 KB × 30개 → git 직접 포함 가능
- results/ 요약 CSV: 소용량 → git 직접 포함

### 9.2 .gitignore 대상
```
build/
install/
log/
__pycache__/
*.pyc
report/
*.bag
*.db3
```

---

## 10. 다음 단계 (즉시 실행 가능)

1. **Phase 1 시작**: `src/pp_adaptive/` 구조 생성 및 파일 복사
2. **검증 우선**: `colcon build` 성공이 모든 것의 전제
3. **f1tenth_gym_ros**: submodule 설정 시 현재 사용 중인 커밋 해시 확인

```bash
# 현재 sim_ws에서 f1tenth_gym_ros 커밋 확인
cd /home/jin/ros2_prj/sim_ws/src/f1tenth_gym_ros && git log --oneline -3
```
