#!/bin/bash
# =============================================================
# F1TENTH 시뮬레이션 파이프라인 (Step 2 + Step 3)
# =============================================================
# 사용법:
#   bash scripts/run_pipeline.sh                       # Step 2+3 (기본)
#   bash scripts/run_pipeline.sh --step ld             # Step 2만 (LD 생성)
#   bash scripts/run_pipeline.sh --step setup          # Step 3만 (시뮬 설정)
#   bash scripts/run_pipeline.sh config/my_input.yaml  # 다른 config 사용
#
# 실행 후 Step 4:
#   ros2 launch pp_adaptive pp_adaptive_sim_launch.py
#   ros2 launch ... rviz:=false
# =============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"

# 인수 파싱
STEP="all"
INPUT_YAML=""
for arg in "$@"; do
    case "$arg" in
        --step) NEXT_IS_STEP=1 ;;
        ld|setup|all)
            if [ -n "$NEXT_IS_STEP" ]; then
                STEP="$arg"
                NEXT_IS_STEP=""
            fi
            ;;
        *.yaml|*.yml) INPUT_YAML="$arg" ;;
        *) ;;
    esac
done
[ -z "$INPUT_YAML" ] && INPUT_YAML="$PKG_ROOT/config/pipeline_input.yaml"

if [ ! -f "$INPUT_YAML" ]; then
    echo "[ERROR] 설정 파일 없음: $INPUT_YAML"
    exit 1
fi

echo "============================================================"
echo " F1TENTH 시뮬레이션 파이프라인"
echo "  설정 파일: $INPUT_YAML"
echo "  실행 단계: $STEP"
echo "============================================================"
echo ""

python3 - "$INPUT_YAML" "$SCRIPT_DIR" "$PKG_ROOT" "$STEP" <<'PYEOF'
import sys, os, yaml, subprocess, shutil

yaml_path   = sys.argv[1]
scripts_dir = sys.argv[2]
pkg_root    = sys.argv[3]
step        = sys.argv[4]   # all | ld | setup

racelines_dir = os.path.join(pkg_root, "racelines")
maps_dir      = os.path.join(pkg_root, "maps")
os.makedirs(racelines_dir, exist_ok=True)
os.makedirs(maps_dir, exist_ok=True)

# ── YAML 파싱 ────────────────────────────────────────────────
with open(yaml_path) as f:
    cfg = yaml.safe_load(f)

map_cfg = cfg.get("map", {})
rl_cfg  = cfg.get("raceline", {})
sim_cfg = cfg.get("simulation", {})

def abspath(p):
    return os.path.abspath(p) if p else ""

def find_map_image(yaml_path):
    """map yaml 옆의 이미지(.png/.pgm/.jpg) 탐색"""
    base = os.path.splitext(yaml_path)[0]
    for ext in [".png", ".pgm", ".jpg", ".jpeg"]:
        if os.path.exists(base + ext):
            return base + ext
    try:
        with open(yaml_path) as f:
            d = yaml.safe_load(f)
        img = d.get("image", "")
        if img:
            candidate = os.path.join(os.path.dirname(yaml_path), img)
            if os.path.exists(candidate):
                return candidate
    except Exception:
        pass
    return ""

def copy_if_different(src, dst):
    if os.path.abspath(src) != os.path.abspath(dst):
        shutil.copy2(src, dst)
        return True
    return False

# ── 맵 설정 파싱 ─────────────────────────────────────────────
map_yaml = abspath(map_cfg.get("yaml", ""))
map_name = map_cfg.get("name") or os.path.splitext(os.path.basename(map_yaml))[0]
if not map_yaml or not os.path.exists(map_yaml):
    print(f"[ERROR] 맵 YAML 없음: {map_yaml}")
    sys.exit(1)

# ============================================================
# STEP 1: 맵 파일 → maps/ 복사 (항상 수행)
# ============================================================
print("── Step 1: 맵 파일 복사 ────────────────────────────────")
map_yaml_dst = os.path.join(maps_dir, os.path.basename(map_yaml))
if copy_if_different(map_yaml, map_yaml_dst):
    print(f"  [복사] {os.path.basename(map_yaml_dst)}")
else:
    print(f"  [스킵] {os.path.basename(map_yaml_dst)} (이미 존재)")

map_img_src = find_map_image(map_yaml)
if map_img_src:
    map_img_dst = os.path.join(maps_dir, os.path.basename(map_img_src))
    if copy_if_different(map_img_src, map_img_dst):
        print(f"  [복사] {os.path.basename(map_img_dst)}")
    else:
        print(f"  [스킵] {os.path.basename(map_img_dst)} (이미 존재)")
else:
    print(f"  [WARN] 맵 이미지 없음 (yaml 옆에 .png/.pgm 필요)")
# ld_map_yaml도 maps/ 안의 복사본 기준으로 갱신
map_yaml = map_yaml_dst

# 레이스라인 설정
base_raceline  = abspath(rl_cfg.get("base", ""))
traj_format    = bool(rl_cfg.get("traj_format", False))
ld_algorithm   = rl_cfg.get("ld_algorithm", "none")   # none | alg1 | alg1_org | kw_sim | kw2
ld_candidates  = rl_cfg.get("ld_candidates", [0.6, 0.9, 1.2, 1.6, 2.0])
ld_map_yaml    = abspath(rl_cfg.get("ld_map_yaml")) or map_yaml  # map_yaml은 이미 maps/ 복사본
active_setting = rl_cfg.get("active", "auto")
kw2_cfg        = rl_cfg.get("kw2", {}) or {}

# 시뮬 설정
max_laps    = int(sim_cfg.get("max_laps", 3))
rviz        = bool(sim_cfg.get("rviz", True))
spawn_x     = sim_cfg.get("spawn_x")
spawn_y     = sim_cfg.get("spawn_y")
spawn_theta = sim_cfg.get("spawn_theta")

# ============================================================
# STEP 2: LD 생성
# ============================================================
active_raceline = None   # Step 3에서 사용

if step in ("all", "ld"):
    print("── Step 2: LD 생성 ─────────────────────────────────────")

    # 2-0: traj_format이면 먼저 convert
    if traj_format:
        if not base_raceline or not os.path.exists(base_raceline):
            print(f"[ERROR] base raceline 없음: {base_raceline}")
            sys.exit(1)
        converted_name = f"{map_name}_optimal_rl.csv"
        converted_dst  = os.path.join(racelines_dir, converted_name)
        print(f"  [traj→rl] {os.path.basename(base_raceline)} → {converted_name}")
        convert_script = os.path.join(scripts_dir, "convert_traj_to_raceline.py")
        res = subprocess.run(
            [sys.executable, convert_script,
             "--input", base_raceline, "--name", map_name, "--out_dir", racelines_dir],
            capture_output=True, text=True
        )
        if res.returncode != 0:
            print("[ERROR] 변환 실패:", res.stderr)
            sys.exit(1)
        print(f"  {res.stdout.strip()}")
        base_raceline = converted_dst

    if not base_raceline or not os.path.exists(base_raceline):
        print(f"[ERROR] base raceline 없음: {base_raceline}")
        sys.exit(1)

    # base가 racelines/ 밖에 있으면 복사
    base_dst = os.path.join(racelines_dir, os.path.basename(base_raceline))
    if os.path.abspath(base_raceline) != os.path.abspath(base_dst):
        shutil.copy2(base_raceline, base_dst)
        print(f"  [복사] {os.path.basename(base_dst)}")

    base_stem    = os.path.splitext(os.path.basename(base_dst))[0]
    ld_cands_str = " ".join(str(x) for x in ld_candidates)

    # LD 알고리즘별 출력 파일명 결정 및 스크립트 실행
    algo_scripts = {
        "alg1":     "compute_adaptive_lookahead_hs.py",
        "alg1_org": "compute_adaptive_lookahead_org.py",
        "kw_sim":   "compute_adaptive_lookahead_kw.py",
        "kw2":      "compute_adaptive_lookahead_kw2.py",
    }

    if ld_algorithm == "none":
        print(f"  ld_algorithm=none → base 그대로 사용: {os.path.basename(base_dst)}")
        active_raceline = base_dst

    elif ld_algorithm in algo_scripts:
        suffix  = {"alg1": "alg1", "alg1_org": "alg1_org", "kw_sim": "kw_sim", "kw2": "kw2"}[ld_algorithm]
        out_name = f"{base_stem}_{suffix}.csv"
        out_path = os.path.join(racelines_dir, out_name)
        script   = os.path.join(os.path.dirname(scripts_dir),
                                "pp_adaptive",
                                algo_scripts[ld_algorithm])

        print(f"  [{ld_algorithm}] {os.path.basename(base_dst)} → {out_name}")

        cmd = [sys.executable, script,
               "--in_csv",   base_dst,
               "--out_csv",  out_path,
               "--map_yaml", ld_map_yaml,
               "--ld_candidates"] + [str(x) for x in ld_candidates]

        # kw_sim은 --mode kw_sim 추가
        if ld_algorithm == "kw_sim":
            cmd += ["--mode", "kw_sim"]
        elif ld_algorithm == "kw2":
            # kw2 전용 옵션 (생략 시 kw2 스크립트 기본값 사용)
            kw2_auto_alpha = bool(kw2_cfg.get("auto_alpha", False))
            kw2_alpha = kw2_cfg.get("alpha", 0.30)
            if kw2_auto_alpha:
                cmd += ["--auto_alpha"]
            elif kw2_alpha is not None:
                cmd += ["--alpha", str(kw2_alpha)]

            kw2_arg_map = {
                "kappa_min": "--kappa_min",
                "a_lat_max": "--a_lat_max",
                "c_dyn": "--c_dyn",
                "kw_margin_t": "--kw_margin_t",
                "w_cte": "--w_cte",
                "objective": "--objective",
                "beta": "--beta",
                "wheelbase": "--wheelbase",
                "steps_cap": "--steps_cap",
                "goal_tol": "--goal_tol",
                "min_sim_time": "--min_sim_time",
                "track_half_width": "--track_half_width",
                "offtrack_margin": "--offtrack_margin",
                "boundary_buffer": "--boundary_buffer",
                "ld_smooth_window": "--ld_smooth_window",
                "ld_jump_limit": "--ld_jump_limit",
            }
            for key, flag in kw2_arg_map.items():
                val = kw2_cfg.get(key)
                if val is not None:
                    cmd += [flag, str(val)]

        res = subprocess.run(cmd, text=True)
        if res.returncode != 0:
            print(f"[ERROR] LD 생성 실패 (exit={res.returncode})")
            sys.exit(1)
        print(f"  완료: {out_path}")
        active_raceline = out_path

    else:
        print(f"[ERROR] 알 수 없는 ld_algorithm: {ld_algorithm}")
        print("  유효값: none | alg1 | alg1_org | kw_sim | kw2")
        sys.exit(1)

# ============================================================
# STEP 3: 시뮬레이션 설정
# ============================================================
if step in ("all", "setup"):
    print("── Step 3: 시뮬레이션 설정 ──────────────────────────────")

    # active raceline 결정
    if active_setting == "auto":
        if active_raceline is None:
            # step=setup 단독 실행 시 LD 생성 없었으므로 base 사용
            base_dst = os.path.join(racelines_dir, os.path.basename(
                abspath(rl_cfg.get("base", ""))
            ))
            if ld_algorithm != "none":
                # traj_format=True 이면 변환 출력은 {map_name}_optimal_rl.csv
                if traj_format:
                    base_stem = f"{map_name}_optimal_rl"
                else:
                    base_stem = os.path.splitext(os.path.basename(base_dst))[0]
                suffix = {"alg1": "alg1", "alg1_org": "alg1_org", "kw_sim": "kw_sim", "kw2": "kw2"}.get(
                    ld_algorithm, ld_algorithm)
                candidate = os.path.join(racelines_dir, f"{base_stem}_{suffix}.csv")
                if os.path.exists(candidate):
                    active_raceline = candidate
                    print(f"  active (기존 LD 파일): {os.path.basename(active_raceline)}")
                else:
                    # base 파일 자체도 탐색 (변환된 파일)
                    converted = os.path.join(racelines_dir, f"{base_stem}.csv")
                    fallback = converted if os.path.exists(converted) else base_dst
                    print(f"  [WARN] LD 파일 없음 → --step ld 먼저 실행하세요")
                    print(f"  base 사용: {os.path.basename(fallback)}")
                    active_raceline = fallback
            else:
                active_raceline = base_dst
    else:
        # 파일명 직접 지정
        active_raceline = os.path.join(racelines_dir, active_setting)
        if not os.path.exists(active_raceline):
            print(f"[ERROR] active raceline 없음: {active_raceline}")
            sys.exit(1)
        print(f"  active (직접 지정): {active_setting}")

    # setup_simulation.py 호출
    setup_script = os.path.join(scripts_dir, "setup_simulation.py")
    cmd = [
        sys.executable, setup_script,
        "--map_yaml",  map_yaml,
        "--raceline",  active_raceline,
        "--max_laps",  str(max_laps),
    ]
    if not rviz:
        cmd.append("--no_rviz")
    if spawn_x is not None:
        cmd += ["--spawn_x", str(spawn_x)]
    if spawn_y is not None:
        cmd += ["--spawn_y", str(spawn_y)]
    if spawn_theta is not None:
        cmd += ["--spawn_theta", str(spawn_theta)]

    print(f"  raceline: {os.path.basename(active_raceline)}", flush=True)
    subprocess.run(cmd, check=True)

# ============================================================
# Step 4 안내
# ============================================================
if step in ("all", "setup"):
    print("")
    print("============================================================")
    print(" Step 4: 시뮬레이션 실행")
    print("  ros2 launch pp_adaptive pp_adaptive_sim_launch.py")
    print("  ros2 launch ... rviz:=false")
    print("============================================================")
PYEOF
