#!/usr/bin/env bash
set -euo pipefail

REPORT_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${REPORT_SCRIPT_DIR}/../.." && pwd)"
PATCH_SCRIPT="${WS_ROOT}/experiments/_patch_benchmarks.py"
TMP_CONFIG_DIR="${TMP_CONFIG_DIR:-/tmp/f1tenth_eval_patched}"
REPORT_DRY_RUN="${REPORT_DRY_RUN:-0}"

load_workspace() {
    set +u
    source /opt/ros/humble/setup.bash
    if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
        set -u
        echo "Missing ${WS_ROOT}/install/setup.bash"
        echo "Build the workspace first: source /opt/ros/humble/setup.bash && colcon build --symlink-install"
        exit 1
    fi
    source "${WS_ROOT}/install/setup.bash"
    set -u
}

resolve_eval_runner() {
    local candidate="${WS_ROOT}/src/f1tenth_driver_benchmark_suite/f1tenth_driver_benchmark_suite/evaluation_runner.py"
    if [[ -f "${candidate}" ]]; then
        printf '%s\n' "${candidate}"
        return 0
    fi

    local prefix=""
    prefix="$(ros2 pkg prefix f1tenth_driver_benchmark_suite 2>/dev/null || true)"
    if [[ -n "${prefix}" ]]; then
        candidate="${prefix}/lib/f1tenth_driver_benchmark_suite/evaluation_runner.py"
        if [[ -f "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    fi

    echo "Unable to locate evaluation_runner.py" >&2
    return 1
}

patch_config() {
    local input_yaml="$1"
    local output_yaml="$2"
    local results_root="$3"
    local trials="$4"

    mkdir -p "${TMP_CONFIG_DIR}"

    python3 "${PATCH_SCRIPT}" \
        --ws-root "${WS_ROOT}" \
        --results-root "${results_root}" \
        --input "${input_yaml}" \
        --output "${output_yaml}"

    python3 - "${output_yaml}" "${trials}" <<'PY'
import sys
import yaml

path = sys.argv[1]
trials = int(sys.argv[2])
with open(path, encoding='utf-8') as f:
    cfg = yaml.safe_load(f)
cfg.setdefault('evaluation', {})
cfg['evaluation']['trials_per_driver'] = trials
with open(path, 'w', encoding='utf-8') as f:
    yaml.dump(cfg, f, allow_unicode=True, sort_keys=False)
PY
}

run_patched_eval() {
    local input_yaml="$1"
    local results_root="$2"
    local trials="$3"
    local eval_runner="$4"

    local patched_yaml="${TMP_CONFIG_DIR}/$(basename "${input_yaml}")"
    patch_config "${input_yaml}" "${patched_yaml}" "${results_root}" "${trials}"
    if [[ "${REPORT_DRY_RUN}" == "1" ]]; then
        echo "[DRY] python3 ${eval_runner} --config ${patched_yaml}"
        return 0
    fi
    python3 "${eval_runner}" --config "${patched_yaml}"
}

matches_map_filter() {
    local name="$1"
    local filter="${2:-}"

    if [[ -z "${filter}" ]]; then
        return 0
    fi

    case "${filter}" in
        austin)
            [[ "${name}" == *austin* ]]
            ;;
        brandshatch)
            [[ "${name}" == *brandshatch* ]]
            ;;
        budapest)
            [[ "${name}" == *budapest* ]]
            ;;
        icra)
            [[ "${name}" == *icra* ]]
            ;;
        skir)
            [[ "${name}" == *skir* ]]
            ;;
        spielberg)
            [[ "${name}" == *spielberg* ]]
            ;;
        *)
            [[ "${name}" == *"${filter}"* ]]
            ;;
    esac
}
