#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/_common.sh"

FAMILY="all"
FORCE=0
DRY_RUN=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --family) FAMILY="$2"; shift 2 ;;
        --force) FORCE=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

PP_PY_DIR="${WS_ROOT}/src/pp_adaptive/pp_adaptive"
PP_RACELINES="${WS_ROOT}/src/pp_adaptive/racelines"
CORE_RACELINES="${WS_ROOT}/src/pp_core/racelines"
MAPS_DIR="${WS_ROOT}/src/pp_adaptive/maps"
MAPS=(Budapest_map Spielberg icra Austin_map skir BrandsHatch_map)

run_if_needed() {
    local output_csv="$1"
    shift

    if [[ -f "${output_csv}" && "${FORCE}" != "1" ]]; then
        echo "[SKIP] $(basename "${output_csv}")"
        return 0
    fi

    if [[ "${DRY_RUN}" == "1" ]]; then
        echo "[DRY] python3 $*"
        return 0
    fi

    echo "[GEN ] $(basename "${output_csv}")"
    python3 "$@"
}

generate_kw_sim() {
    local map_name="$1"
    run_if_needed "${PP_RACELINES}/${map_name}_optimal_rl_kw_sim.csv" \
        "${PP_PY_DIR}/compute_adaptive_lookahead_kw.py" \
        --in_csv "${CORE_RACELINES}/${map_name}_optimal_rl.csv" \
        --map_yaml "${MAPS_DIR}/${map_name}.yaml" \
        --out_csv "${PP_RACELINES}/${map_name}_optimal_rl_kw_sim.csv" \
        --ld_candidates 1.0 1.5 2.0 2.5 3.0 \
        --auto_alpha \
        --objective ALAT
}

generate_kv() {
    local map_name="$1"
    run_if_needed "${PP_RACELINES}/${map_name}_optimal_rl_kv.csv" \
        "${PP_PY_DIR}/compute_adaptive_lookahead_kv.py" \
        --in_csv "${CORE_RACELINES}/${map_name}_optimal_rl.csv" \
        --out_csv "${PP_RACELINES}/${map_name}_optimal_rl_kv.csv" \
        --k 0.15 \
        --ld_min 0.6 \
        --ld_max 2.0
}

generate_curv() {
    local map_name="$1"
    run_if_needed "${PP_RACELINES}/${map_name}_optimal_rl_curv.csv" \
        "${PP_PY_DIR}/compute_adaptive_lookahead_curv.py" \
        --in_csv "${CORE_RACELINES}/${map_name}_optimal_rl.csv" \
        --out_csv "${PP_RACELINES}/${map_name}_optimal_rl_curv.csv" \
        --alpha_h 0.35 \
        --kappa_min 0.01 \
        --ld_min 0.6 \
        --ld_max 2.0
}

generate_alg1_ext() {
    local map_name="$1"
    run_if_needed "${PP_RACELINES}/${map_name}_optimal_rl_alg1_ext.csv" \
        "${PP_PY_DIR}/compute_adaptive_lookahead_org.py" \
        --in_csv "${CORE_RACELINES}/${map_name}_optimal_rl.csv" \
        --map_yaml "${MAPS_DIR}/${map_name}.yaml" \
        --out_csv "${PP_RACELINES}/${map_name}_optimal_rl_alg1_ext.csv" \
        --ld_candidates 0.6 0.9 1.2 1.6 2.0 \
        --beta 0.5 \
        --objective CONVEX \
        --delta_mode integral \
        --steps_cap 80 \
        --goal_tol 0.5 \
        --min_sim_time 0.2
}

generate_ablation_variant() {
    local output_name="$1"
    local map_name="$2"
    shift 2

    run_if_needed "${PP_RACELINES}/${output_name}.csv" \
        "${PP_PY_DIR}/compute_adaptive_lookahead_kw2.py" \
        --in_csv "${CORE_RACELINES}/${map_name}_optimal_rl.csv" \
        --map_yaml "${MAPS_DIR}/${map_name}.yaml" \
        --out_csv "${PP_RACELINES}/${output_name}.csv" \
        --ld_candidates 0.9 1.2 1.6 2.0 \
        --auto_alpha \
        --a_lat_max 7.5 \
        --c_dyn 2.0 \
        --kw_margin_t 0.3 \
        --w_cte 0.5 \
        --objective ALAT_MAX_CTE \
        "$@"
}

run_revision_family() {
    for map_name in "${MAPS[@]}"; do
        generate_kv "${map_name}"
        generate_curv "${map_name}"
        generate_alg1_ext "${map_name}"
    done
}

run_ablation_family() {
    for map_name in "${MAPS[@]}"; do
        generate_kw_sim "${map_name}"
    done

    generate_ablation_variant "Budapest_map_optimal_rl_ablation_noA" "Budapest_map" \
        --a_lat_max 9999
    generate_ablation_variant "Austin_map_optimal_rl_ablation_noA" "Austin_map" \
        --a_lat_max 9999

    generate_ablation_variant "Budapest_map_optimal_rl_ablation_noB" "Budapest_map" \
        --objective ALAT
    generate_ablation_variant "Spielberg_optimal_rl_ablation_noB" "Spielberg" \
        --objective ALAT

    generate_ablation_variant "Budapest_map_optimal_rl_ablation_noC" "Budapest_map" \
        --no_dynamic_v_target
    generate_ablation_variant "icra_optimal_rl_ablation_noC" "icra" \
        --no_dynamic_v_target

    generate_ablation_variant "Budapest_map_optimal_rl_ablation_noD" "Budapest_map" \
        --kw_margin_t 0.0
    generate_ablation_variant "skir_optimal_rl_ablation_noD" "skir" \
        --kw_margin_t 0.0
}

echo "============================================================"
echo " Report Raceline Generator"
echo " Family : ${FAMILY}"
echo " Force  : ${FORCE}"
echo " DryRun : ${DRY_RUN}"
echo "============================================================"

case "${FAMILY}" in
    revision)
        run_revision_family
        ;;
    ablation)
        run_ablation_family
        ;;
    legacy)
        echo "Legacy 3-alg benchmark uses bundled pp_core racelines directly; no derived CSV generation is required."
        ;;
    all)
        run_revision_family
        run_ablation_family
        ;;
    *)
        echo "Unknown family: ${FAMILY}"
        echo "Use one of: revision, ablation, legacy, all"
        exit 1
        ;;
esac

echo ""
echo "Done. Derived racelines are under: ${PP_RACELINES}"
