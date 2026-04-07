# Paper Reference Results

IEEE RA-L 2026 논문의 재현 기준값.  
실험 결과가 이 수치와 일치하면 재현 성공.

## Table II: Success Rate (%, N=30)

| Method | Budapest | Spielberg | ICRA | Skir | Austin | BrandsHatch | **Overall** |
|---|---|---|---|---|---|---|---|
| **kw_sim** (Proposed) | 100 | 100 | 100 | 100 | 100 | 93.3 | **99.4** |
| alg1_org | 0 | 16.7 | 13.3 | 33.3 | 13.3 | 0 | 12.8 |
| alg1_ext | — | — | — | — | — | — | ~13 |
| pp_kv | — | — | — | — | — | — | — |
| pp_curv | — | — | — | — | — | — | — |

**Wilson 95% CI (aggregate)**:
- kw_sim: 99.4% [96.9%, 99.9%]
- alg1_org: 33.3% [26.9%, 40.5%]

## Key Statistical Findings

| Metric | alg1_org | kw_sim |
|---|---|---|
| Spearman r(|κ|, LD) | +0.094 (무상관) | −0.757 (강한 음상관) |
| Budapest avg speed | — | 7.22 m/s |
| alg1_org crash cause | LD=2.0m at κ>0.1 corners | — |

## Failure Analysis (Budapest, alg1_org)

- 고위험 waypoint 15개: alg1_org assigns LD=2.0m
- 해당 waypoints의 예측 a_lat: 9.7~11.6 m/s² (한계 8.5 초과)
- Score 퇴화 원인: v_exit 차이 0.040 m/s (score +0.020) > CTE 불이익 (+0.008)
  → LD=2.0 선택 시 CTE 패널티보다 속도 이득이 더 크게 계산됨

## Figure Reference

| Figure | 내용 | 재현 명령 |
|---|---|---|
| Fig. 3 | LD vs κ scatter | `python3 analysis/gen_figures.py` |
| Fig. 4 | Success rate heatmap | 위 동일 |
| Fig. 5 | Lateral error bar chart | 위 동일 |
| Fig. 6 | Sensitivity grid | `run_sensitivity.sh` 후 위 동일 |
