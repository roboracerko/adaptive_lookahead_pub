#!/bin/bash
# =============================================================================
# generate_revision_racelines.sh
#
# 리비전 베이스라인 raceline 일괄 생성
# R1a: pp_kv   — Ld = clip(k * v_ref,  0.6, 2.0)
# R1b: pp_curv — Ld = clip(alpha_h / sqrt(|kappa|),  0.6, 2.0)
# R7:  alg1_ext — alg1_org + 확장 candidates {0.6, 0.9, 1.2, 1.6, 2.0}
#
# 사용법:
#   cd <workspace>/src/pp_adaptive
#   bash scripts/generate_revision_racelines.sh
#
# 출력:
#   racelines/{MAP}_optimal_rl_kv.csv
#   racelines/{MAP}_optimal_rl_curv.csv
#   racelines/{MAP}_optimal_rl_alg1_ext.csv
# =============================================================================

set -e

BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYDIR="${BASE}/pp_adaptive"
RLINES="${BASE}/racelines"
MAPS_DIR="${BASE}/maps"

# ─── 맵 이름 배열 (raceline prefix = map yaml basename) ───────────────────
MAPS=(
    Budapest_map
    Spielberg
    icra
    Austin_map
    skir
    BrandsHatch_map
)

# ─── 파라미터 ─────────────────────────────────────────────────────────────
KV_K="${KV_K:-0.15}"
KV_LD_MIN="${KV_LD_MIN:-0.6}"
KV_LD_MAX="${KV_LD_MAX:-2.0}"

CURV_ALPHA_H="${CURV_ALPHA_H:-0.35}"
CURV_KAPPA_MIN="${CURV_KAPPA_MIN:-0.01}"
CURV_LD_MIN="${CURV_LD_MIN:-0.6}"
CURV_LD_MAX="${CURV_LD_MAX:-2.0}"

ALG1EXT_CANDIDATES="${ALG1EXT_CANDIDATES:-0.6 0.9 1.2 1.6 2.0}"

# ─── 색깔 출력 ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}  ⚠ $*${NC}"; }
fail() { echo -e "${RED}  ✗ $*${NC}"; exit 1; }

echo "============================================================"
echo " Revision Raceline Generator"
echo " BASE: ${BASE}"
echo "============================================================"

# ─── [1/3] pp_kv ─────────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[1/3] pp_kv  (k=${KV_K}, ld=[${KV_LD_MIN},${KV_LD_MAX}]m)${NC}"

for MAP in "${MAPS[@]}"; do
    IN="${RLINES}/${MAP}_optimal_rl.csv"
    OUT="${RLINES}/${MAP}_optimal_rl_kv.csv"

    if [[ ! -f "$IN" ]]; then
        warn "Input not found: ${IN} — skipping"
        continue
    fi

    echo "  ${MAP}..."
    python3 "${PYDIR}/compute_adaptive_lookahead_kv.py" \
        --in_csv  "${IN}" \
        --out_csv "${OUT}" \
        --k       "${KV_K}" \
        --ld_min  "${KV_LD_MIN}" \
        --ld_max  "${KV_LD_MAX}" \
        2>&1 | grep -E "^\[|Saved|⚠|mean=" || true
    ok "${OUT}"
done

# ─── [2/3] pp_curv ───────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[2/3] pp_curv  (alpha_h=${CURV_ALPHA_H}, kappa_min=${CURV_KAPPA_MIN})${NC}"

for MAP in "${MAPS[@]}"; do
    IN="${RLINES}/${MAP}_optimal_rl.csv"
    OUT="${RLINES}/${MAP}_optimal_rl_curv.csv"

    if [[ ! -f "$IN" ]]; then
        warn "Input not found: ${IN} — skipping"
        continue
    fi

    echo "  ${MAP}..."
    python3 "${PYDIR}/compute_adaptive_lookahead_curv.py" \
        --in_csv    "${IN}" \
        --out_csv   "${OUT}" \
        --alpha_h   "${CURV_ALPHA_H}" \
        --kappa_min "${CURV_KAPPA_MIN}" \
        --ld_min    "${CURV_LD_MIN}" \
        --ld_max    "${CURV_LD_MAX}" \
        2>&1 | grep -E "^\[|Saved|⚠|mean=" || true
    ok "${OUT}"
done

# ─── [3/3] alg1_ext ──────────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}[3/3] alg1_ext  (candidates: ${ALG1EXT_CANDIDATES})${NC}"
echo "  (kinematic simulation — takes ~3 min per map)"

for MAP in "${MAPS[@]}"; do
    IN="${RLINES}/${MAP}_optimal_rl.csv"
    OUT="${RLINES}/${MAP}_optimal_rl_alg1_ext.csv"
    MAP_YAML="${MAPS_DIR}/${MAP}.yaml"

    if [[ ! -f "$IN" ]]; then
        warn "Input not found: ${IN} — skipping"
        continue
    fi
    if [[ ! -f "$MAP_YAML" ]]; then
        warn "Map YAML not found: ${MAP_YAML} — skipping"
        continue
    fi

    echo "  ${MAP}..."
    # shellcheck disable=SC2086
    python3 "${PYDIR}/compute_adaptive_lookahead_org.py" \
        --in_csv        "${IN}" \
        --map_yaml      "${MAP_YAML}" \
        --out_csv       "${OUT}" \
        --ld_candidates ${ALG1EXT_CANDIDATES} \
        --beta          0.5 \
        --objective     CONVEX \
        --delta_mode    integral \
        --steps_cap     80 \
        --goal_tol      0.5 \
        --min_sim_time  0.2 \
        2>&1 | tail -5
    ok "${OUT}"
done

# ─── 결과 확인 ────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo " Generated racelines:"
echo "============================================================"
for MAP in "${MAPS[@]}"; do
    for SUFFIX in kv curv alg1_ext; do
        F="${RLINES}/${MAP}_optimal_rl_${SUFFIX}.csv"
        if [[ -f "$F" ]]; then
            NWP=$(( $(wc -l < "$F") - 1 ))
            printf "  %-45s  %5d wp\n" "${MAP}_optimal_rl_${SUFFIX}.csv" "${NWP}"
        else
            printf "  %-45s  MISSING\n" "${MAP}_optimal_rl_${SUFFIX}.csv"
        fi
    done
done
echo ""
echo -e "${GREEN}Done.${NC}"
