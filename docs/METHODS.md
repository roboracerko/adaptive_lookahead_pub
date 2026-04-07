# Methods Description

논문에 포함된 5가지 lookahead distance (LD) 할당 방법.

## Proposed: kw_sim (κ-window + a_lat Simulation)

**코드**: `pp_adaptive/compute_adaptive_lookahead_kw2.py`  
**Raceline 접미사**: `_kw_sim`

각 waypoint에서 κ-window(전방 k개 waypoint의 최대 곡률)를 계산하고,
각 LD 후보값에 대해 단순 kinematic 시뮬레이션으로 lateral acceleration을 예측하여
`a_lat_max` 제약 내에서 가장 긴 LD를 선택한다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `kappa_window` | 15 | κ 탐색 전방 waypoint 수 |
| `a_lat_max` | 8.5 m/s² | 허용 최대 횡가속도 |
| `lookahead_set` | [0.6, 1.0, 1.5, 2.0] | LD 후보 집합 (m) |
| `alpha` | 0.3 | CTE 가중치 (score = v_exit - α·CTE) |
| `t_margin` | 0.3 | 시뮬레이션 시간 마진 (s) |
| `w_cte` | 1.5 | CTE 불이익 가중치 |

## Baseline: alg1_org (Sukhil & Behl, 2021)

**코드**: `pp_adaptive/compute_adaptive_lookahead_org.py`  
**Raceline 접미사**: `_alg1_org`  
**참고 논문**: Sukhil & Behl, "Safe Adaptive Cruise Control on Curved Roads," 2021

원본 Algorithm 1의 충실한 재현. 각 waypoint에서 kinematic bicycle 시뮬레이션으로
L={1.0, 1.5, 2.0}m 중 가장 빠른 exit velocity를 내는 LD를 선택.

## Baseline: alg1_ext (Extended LD Set)

**코드**: `pp_adaptive/compute_adaptive_lookahead_org.py` (--lookahead-set 옵션)  
**Raceline 접미사**: `_alg1_ext`

alg1_org와 동일한 알고리즘, L={0.6, 0.9, 1.2, 1.6, 2.0}m로 확장.
구조적 실패(score 퇴화)가 LD set 확장으로 해결되지 않음을 증명.

## Baseline: pp_kv (Speed-proportional)

**코드**: `pp_adaptive/compute_adaptive_lookahead_kv.py`  
**Raceline 접미사**: `_kv`

Ld = k · v, k=0.15 (단순 속도 비례)

## Baseline: pp_curv (Curvature-inverse)

**코드**: `pp_adaptive/compute_adaptive_lookahead_curv.py`  
**Raceline 접미사**: `_curv`

Ld ∝ 1/√|κ|, α_h=0.35 (곡률의 역수 비례)
