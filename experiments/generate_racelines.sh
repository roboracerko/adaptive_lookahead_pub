#!/usr/bin/env bash
# ==============================================================================
# generate_racelines.sh
# 모든 method별 raceline을 재생성한다 (LD labeling 재현)
# 이미 racelines/*.csv 가 포함되어 있으므로 이 스크립트는 선택사항
#
# 사용법:
#   bash experiments/generate_racelines.sh [--method METHOD] [--map MAP]
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PP_ADAPTIVE_SRC="$WS_ROOT/src/pp_adaptive"
RACELINES_DIR="$PP_ADAPTIVE_SRC/racelines"
MAPS_DIR="$PP_ADAPTIVE_SRC/maps"

METHOD="all"
MAP_FILTER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --method) METHOD="$2"; shift 2 ;;
        --map) MAP_FILTER="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# 맵 → 파일명 매핑
declare -A MAP_FILES=(
    ["budapest"]="Budapest_map"
    ["spielberg"]="Spielberg"
    ["icra"]="icra"
    ["skir"]="skir"
    ["austin"]="Austin_map"
    ["brandshatch"]="BrandsHatch_map"
)

MAPS="${MAP_FILTER:-budapest spielberg icra skir austin brandshatch}"

for MAP_KEY in $MAPS; do
    MAP_FILE="${MAP_FILES[$MAP_KEY]}"
    MAP_YAML="$MAPS_DIR/${MAP_FILE}.yaml"
    BASE_CSV="$RACELINES_DIR/${MAP_FILE}_optimal_rl.csv"

    if [ ! -f "$BASE_CSV" ]; then
        echo "[SKIP] Base raceline not found: $BASE_CSV"
        continue
    fi

    echo "=== Map: $MAP_KEY ($MAP_FILE) ==="

    if [[ "$METHOD" == "all" || "$METHOD" == "kw_sim" ]]; then
        echo "  Generating kw_sim..."
        python3 -m pp_adaptive.compute_adaptive_lookahead_kw2 \
            --input "$BASE_CSV" \
            --output "$RACELINES_DIR/${MAP_FILE}_optimal_rl_kw_sim.csv"
    fi

    if [[ "$METHOD" == "all" || "$METHOD" == "alg1_org" ]]; then
        echo "  Generating alg1_org..."
        python3 -m pp_adaptive.compute_adaptive_lookahead_org \
            --input "$BASE_CSV" \
            --output "$RACELINES_DIR/${MAP_FILE}_optimal_rl_alg1_org.csv"
    fi

    if [[ "$METHOD" == "all" || "$METHOD" == "alg1_ext" ]]; then
        echo "  Generating alg1_ext (L={0.6,0.9,1.2,1.6,2.0})..."
        python3 -m pp_adaptive.compute_adaptive_lookahead_org \
            --input "$BASE_CSV" \
            --lookahead-set 0.6 0.9 1.2 1.6 2.0 \
            --output "$RACELINES_DIR/${MAP_FILE}_optimal_rl_alg1_ext.csv"
    fi

    if [[ "$METHOD" == "all" || "$METHOD" == "kv" ]]; then
        echo "  Generating kv (Ld=k*v, k=0.15)..."
        python3 -m pp_adaptive.compute_adaptive_lookahead_kv \
            --input "$BASE_CSV" \
            --k 0.15 \
            --output "$RACELINES_DIR/${MAP_FILE}_optimal_rl_kv.csv"
    fi

    if [[ "$METHOD" == "all" || "$METHOD" == "curv" ]]; then
        echo "  Generating curv (Ld∝1/√|κ|, α_h=0.35)..."
        python3 -m pp_adaptive.compute_adaptive_lookahead_curv \
            --input "$BASE_CSV" \
            --alpha-h 0.35 \
            --output "$RACELINES_DIR/${MAP_FILE}_optimal_rl_curv.csv"
    fi
done

echo ""
echo "Raceline generation complete."
echo "Generated CSVs in: $RACELINES_DIR"
