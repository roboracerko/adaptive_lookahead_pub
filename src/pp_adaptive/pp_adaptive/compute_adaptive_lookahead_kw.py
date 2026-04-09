#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
κ-Window + Kinematic Sim Lookahead Assignment (kw_sim, offline LD labeler)

──────────────────────────────────────────────────────
Mode: kw_sim  (κ-Window + Kinematic Sim hybrid)
──────────────────────────────────────────────────────
Algorithm:
  For each waypoint i:
    1. [κ-window pre-filter]  valid_ld = { ld | ld ≤ α / κ_window(i, ld) }
    2. [Sim scoring]          For each ld in valid_ld:
         Run kinematic bicycle sim with per-waypoint speed init [Mod D from Alg 1-HS]
         Score via objective (ALAT recommended)
    3. Select π[i] = argmax(score) over valid_ld

Why kw_sim beats Algorithm 1-HS (ALAT+D):
  - Alg 1-HS: ALAT prefers large LD (large LD → small steering angle → low a_lat)
    → structural bias: large LD picked even in corners → crashes
  - kw_sim: κ-window pre-filter removes large LD from corners FIRST,
    then sim scores only among physically safe candidates
  - Combines physical safety guarantee (κ-window) with sim-based fine-tuning

Comparison with prior approaches:
  Algorithm 1 (CONVEX)  : sim-based, v_exit convergence → degenerates at high speed
  Algorithm 1-HS (ALAT) : sim-based, ALAT structural bias → large LD in corners
  kw_sim                : κ-window safety + sim scoring (hybrid)

Usage:
  # kw_sim (needs map YAML)
  python compute_adaptive_lookahead_kw.py \\
    --in_csv  racelines/Budapest_optimal_rl_ld.csv \\
    --map_yaml /path/to/budapest_map.yaml \\
    --out_csv racelines/Budapest_optimal_rl_kw_sim.csv \\
    --alpha 0.30 --objective ALAT
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter
from dataclasses import dataclass
from typing import List, Literal, Optional

import numpy as np


# ─────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────
@dataclass
class Waypoint:
    x: float
    y: float
    heading: float
    kappa: float   # signed curvature (1/m); absolute value used for constraint
    v_ref: float


@dataclass
class State:
    x: float
    y: float
    yaw: float
    v: float


ObjectiveType = Literal["VEL", "DEV", "CONVEX", "ALAT"]


def kappa_window_max(W: List[Waypoint], i: int, ld: float) -> float:
    """
    Return the maximum |κ| over all waypoints within arc-distance `ld` ahead of W[i].

    Window size = ld metres (LD-adaptive):
      LD=1.0m → checks ~1 m ahead  (~15–20 waypoints on Budapest raceline)
      LD=3.0m → checks ~3 m ahead  (~50–60 waypoints on Budapest raceline)
    Actual waypoint count depends on raceline spacing, not a fixed number.
    """
    n = len(W)
    acc = 0.0
    j = i
    kappa_max = abs(W[i].kappa)

    while acc < ld:
        j_next = (j + 1) % n
        seg = math.hypot(W[j_next].x - W[j].x, W[j_next].y - W[j].y)
        acc += seg
        kw = abs(W[j_next].kappa)
        if kw > kappa_max:
            kappa_max = kw
        j = j_next
        if j == i:          # wrapped full lap — should not happen for reasonable ld
            break

    return kappa_max


# ─────────────────────────────────────────────
# Kinematic sim helpers
# ─────────────────────────────────────────────
def _wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _nearest_index(W: List[Waypoint], x: float, y: float) -> int:
    d2 = [(w.x - x) ** 2 + (w.y - y) ** 2 for w in W]
    return int(np.argmin(d2))


def _waypoint_ahead(W: List[Waypoint], i0: int, ld: float) -> int:
    acc = 0.0
    i = i0
    n = len(W)
    while acc < ld:
        j = (i + 1) % n
        acc += math.hypot(W[j].x - W[i].x, W[j].y - W[i].y)
        i = j
        if acc > 1e3:
            break
    return i


def _cross_track_error(W: List[Waypoint], x: float, y: float) -> float:
    i = _nearest_index(W, x, y)
    return math.hypot(W[i].x - x, W[i].y - y)


def _pure_pursuit(state: State, W: List[Waypoint], ld: float, L: float) -> float:
    i_near = _nearest_index(W, state.x, state.y)
    i_goal = _waypoint_ahead(W, i_near, ld)
    gx, gy = W[i_goal].x, W[i_goal].y
    alpha = _wrap_to_pi(math.atan2(gy - state.y, gx - state.x) - state.yaw)
    return math.atan2(2.0 * L * math.sin(alpha), ld)


class _KinematicBicycleSim:
    def __init__(self, L: float, dt: float, a_acc: float, a_brake: float):
        self.L = L
        self.dt = dt
        self.a_acc = a_acc
        self.a_brake = a_brake
        self.state: Optional[State] = None

    def spawn(self, s: State):
        self.state = State(s.x, s.y, s.yaw, s.v)

    def step(self, delta: float, v_target: float):
        s = self.state
        assert s is not None
        if v_target > s.v:
            s.v = min(v_target, s.v + self.a_acc * self.dt)
        else:
            s.v = max(v_target, s.v - self.a_brake * self.dt)
        yaw_rate = (s.v / self.L) * math.tan(delta)
        s.yaw = _wrap_to_pi(s.yaw + yaw_rate * self.dt)
        s.x += s.v * math.cos(s.yaw) * self.dt
        s.y += s.v * math.sin(s.yaw) * self.dt


def _score(
    v_exit: float,
    avg_cte: float,
    max_a_lat: float,
    objective: str,
    beta: float,
) -> float:
    """
    ALAT:   score = -max_a_lat  (minimise peak lateral acceleration)
    VEL:    score = v_exit
    DEV:    score = -avg_cte
    CONVEX: score = β·v_exit − (1−β)·avg_cte
    """
    if objective == "ALAT":
        return -max_a_lat
    if objective == "VEL":
        return v_exit
    if objective == "DEV":
        return -avg_cte
    return beta * v_exit - (1.0 - beta) * avg_cte  # CONVEX


# ─────────────────────────────────────────────
# κ-Window + Sim assignment
# ─────────────────────────────────────────────
def kw_sim_lookahead_assignment(
    W: List[Waypoint],
    occ_map,                        # OccupancyMap | None
    L_candidates: List[float],
    alpha: float,
    objective: str = "ALAT",
    beta: float = 0.6,
    wheelbase: float = 0.3302,
    kappa_min: float = 0.005,
    steps_cap: int = 80,
    goal_tol: float = 0.5,
    min_sim_time: float = 0.2,
) -> List[float]:
    """
    Hybrid κ-Window + Kinematic Sim lookahead assignment.

    Step 1 — κ-window pre-filter:
      Compute valid_ld = { ld | ld ≤ α / κ_window(i, ld) }.
      Eliminates physically unsafe LD candidates before the sim sees them.
      This prevents the ALAT structural bias that causes Alg 1-HS to fail.

    Step 2 — Sim scoring (among safe candidates only):
      Run kinematic bicycle sim with per-waypoint racing speed init [Mod D].
      Score via objective (ALAT recommended).
      π[i] = argmax(score) over valid_ld.

    Step 3 — Shortcuts:
      - No valid candidates → fallback to min(L_candidates)
      - Single valid candidate → skip sim (result is deterministic)
    """
    sim = _KinematicBicycleSim(L=wheelbase, dt=0.02, a_acc=2.0, a_brake=4.0)
    min_steps = int(min_sim_time / sim.dt)
    L_candidates = sorted(L_candidates)
    pi: List[float] = []

    for i in range(len(W)):
        # ── Step 1: κ-window pre-filter ─────────────────────────────────────
        valid_ld = []
        for ld in L_candidates:
            kw = kappa_window_max(W, i, ld)
            ld_safe = alpha / max(kw, kappa_min)
            if ld <= ld_safe:
                valid_ld.append(ld)

        if not valid_ld:
            pi.append(L_candidates[0])
            continue

        # Shortcut: single candidate — no need to run sim
        if len(valid_ld) == 1:
            pi.append(valid_ld[0])
            continue

        # ── Step 2: Sim scoring among valid candidates ───────────────────────
        best_ld = valid_ld[0]
        best_score = -1e9

        for ld in valid_ld:
            # Per-waypoint racing speed initialisation [Mod D from Alg 1-HS]
            T_init = State(W[i].x, W[i].y, W[i].heading, v=W[i].v_ref)
            sim.spawn(T_init)

            delta_acc = 0.0
            max_a_lat = 0.0
            step_count = 0

            i_near = _nearest_index(W, T_init.x, T_init.y)
            i_goal = _waypoint_ahead(W, i_near, ld)
            gx, gy = W[i_goal].x, W[i_goal].y

            crashed = False

            for step in range(steps_cap):
                s = sim.state
                assert s is not None
                step_count = step + 1

                if occ_map is not None and occ_map.is_collision(s.x, s.y):
                    crashed = True
                    break

                if step >= min_steps:
                    if math.hypot(s.x - gx, s.y - gy) < goal_tol:
                        break

                delta = _pure_pursuit(s, W, ld, wheelbase)
                sim.step(delta, W[i].v_ref)

                delta_acc += _cross_track_error(W, s.x, s.y) * sim.dt

                kappa_sim = abs(math.tan(delta)) / max(1e-6, wheelbase)
                a_lat = (s.v ** 2) * kappa_sim
                if a_lat > max_a_lat:
                    max_a_lat = a_lat

            if crashed:
                continue  # occ_map 추가 안전망; κ-window로 이미 필터됐으나 보수적으로 제외

            v_exit = sim.state.v
            avg_cte = delta_acc / (step_count * sim.dt) if step_count > 0 else float("inf")
            s_val = _score(v_exit, avg_cte, max_a_lat, objective, beta)
            if s_val > best_score:
                best_score = s_val
                best_ld = ld

        pi.append(best_ld)

        if (i + 1) % 200 == 0:
            print(f"  [{i+1}/{len(W)}] done...")

    return pi


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description=(
            "κ-Window offline LD assignment (kw_sim only). "
            "κ-Window pre-filter + kinematic sim scoring (hybrid)."
        )
    )
    # Backward compatibility: old commands may still pass --mode kw_sim.
    # Keep parser support but disallow any other mode.
    ap.add_argument("--mode", choices=["kw_sim"], default="kw_sim", help=argparse.SUPPRESS)
    ap.add_argument("--in_csv",  required=True, help="Input raceline CSV (x_m, y_m, kappa_1pm, v_mps, ...)")
    ap.add_argument("--out_csv", required=True, help="Output CSV with ld_m column")
    ap.add_argument("--map_yaml", required=True, help="Map YAML file")
    ap.add_argument(
        "--alpha", type=float, default=None,
        help="Physical scale factor: LD ≤ α / κ_window. "
             "If omitted and --auto_alpha is set, computed automatically from raceline κ_p90. "
             "If both omitted, defaults to 0.30.",
    )
    ap.add_argument(
        "--auto_alpha", action="store_true",
        help="Auto-compute α from raceline curvature: α = κ_p90 × min(ld_candidates) + 0.01. "
             "Ensures LD_min passes the κ-window filter at the 90th-percentile curvature, "
             "reducing blind fallbacks on any map without manual tuning.",
    )
    ap.add_argument(
        "--ld_candidates", nargs="+", type=float,
        default=[1.0, 1.5, 2.0, 2.5, 3.0],
        help="LD candidate values (m)",
    )
    ap.add_argument(
        "--kappa_min", type=float, default=0.005,
        help="Floor curvature to avoid div-by-zero (1/m). "
             "Caps LD_safe at α/κ_min on near-straight sections. (default 0.005)",
    )
    ap.add_argument(
        "--objective", choices=["VEL", "DEV", "CONVEX", "ALAT"], default="ALAT",
        help="Sim scoring objective. ALAT = minimise peak lateral accel (default).",
    )
    ap.add_argument(
        "--beta", type=float, default=0.6,
        help="CONVEX weight β (unused for ALAT)",
    )
    ap.add_argument("--wheelbase", type=float, default=0.3302, help="Vehicle wheelbase (m)")
    ap.add_argument(
        "--steps_cap", type=int, default=80,
        help="Max sim steps per candidate (80 × 0.02s = 1.6s)",
    )
    ap.add_argument(
        "--goal_tol", type=float, default=0.5,
        help="Goal arrival tolerance (m)",
    )
    ap.add_argument(
        "--min_sim_time", type=float, default=0.2,
        help="Min sim time (s) before goal check",
    )
    args = ap.parse_args()

    print("[INFO] κ-Window Lookahead Assignment  [mode=kw_sim]")
    print(f"  in_csv       : {args.in_csv}")
    print(f"  alpha        : {'auto' if args.auto_alpha and args.alpha is None else args.alpha or 0.30} (resolved after raceline load)")
    print(f"  ld_candidates: {args.ld_candidates}")
    print(f"  kappa_min    : {args.kappa_min}")
    print(f"  map_yaml     : {args.map_yaml}")
    print(f"  objective    : {args.objective}")
    print(f"  wheelbase    : {args.wheelbase} m")
    print(f"  steps_cap    : {args.steps_cap}  ({args.steps_cap * 0.02:.1f}s max per candidate)")

    # ── Load CSV ──────────────────────────────────────────────────────────
    rows = []
    fieldnames = None
    with open(args.in_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)

    if fieldnames is None or "x_m" not in fieldnames or "y_m" not in fieldnames:
        raise ValueError(f"CSV must have x_m, y_m columns. Found: {fieldnames}")

    # ── Heading ───────────────────────────────────────────────────────────
    has_heading = "heading" in (fieldnames or [])
    if not has_heading:
        print("[INFO] heading not found — computing from x/y.")
        xs = [float(r["x_m"]) for r in rows]
        ys = [float(r["y_m"]) for r in rows]
        dx = np.roll(xs, -1) - np.array(xs)
        dy = np.roll(ys, -1) - np.array(ys)
        headings = np.arctan2(dy, dx).tolist()
        for i, row in enumerate(rows):
            row["heading"] = str(headings[i])

    # ── Curvature ─────────────────────────────────────────────────────────
    kappa_col = None
    for col in ("kappa_1pm", "kappa", "curvature"):
        if col in (fieldnames or []):
            kappa_col = col
            break
    if kappa_col is None:
        raise ValueError(
            f"CSV must have a curvature column (kappa_1pm / kappa / curvature). "
            f"Found: {fieldnames}"
        )
    print(f"  kappa column : '{kappa_col}'")

    # ── Speed ─────────────────────────────────────────────────────────────
    has_v = "v_mps" in (fieldnames or [])
    if not has_v:
        print("[WARN] v_mps not found — using 3.0 m/s default")
        for row in rows:
            row["v_mps"] = "3.0"

    # ── Build Waypoint list ───────────────────────────────────────────────
    W = [
        Waypoint(
            x=float(r["x_m"]),
            y=float(r["y_m"]),
            heading=float(r.get("heading", 0.0)),
            kappa=float(r[kappa_col]),
            v_ref=float(r.get("v_mps", 3.0)),
        )
        for r in rows
    ]

    kappa_vals = [abs(w.kappa) for w in W]

    # ── Alpha resolution ──────────────────────────────────────────────────
    # Priority: explicit --alpha > --auto_alpha > hardcoded default 0.30
    if args.alpha is not None:
        alpha = args.alpha
        alpha_source = "manual"
    elif args.auto_alpha:
        k_p90 = float(np.percentile(kappa_vals, 90))
        ld_min = min(args.ld_candidates)
        alpha = round(k_p90 * ld_min + 0.01, 4)
        alpha_source = f"auto (κ_p90={k_p90:.4f} × LD_min={ld_min} + 0.01)"
    else:
        alpha = 0.30
        alpha_source = "default"

    print(f"\n  Raceline: {len(W)} waypoints")
    print(f"  |κ| range: [{min(kappa_vals):.4f}, {max(kappa_vals):.4f}] m⁻¹")
    print(f"  α = {alpha:.4f}  [{alpha_source}]")
    print(f"  R_min = {1/max(kappa_vals):.2f} m  →  LD_max at tightest corner = "
          f"{alpha/max(kappa_vals):.2f} m")

    # ── Compute ───────────────────────────────────────────────────────────
    print(f"\n[INFO] Mode kw_sim: κ-Window pre-filter + {args.objective} sim scoring...")
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from pp_adaptive.utils.map_loader import OccupancyMap
    occ_map = OccupancyMap(args.map_yaml)
    pi = kw_sim_lookahead_assignment(
        W=W,
        occ_map=occ_map,
        L_candidates=args.ld_candidates,
        alpha=alpha,
        objective=args.objective,
        beta=args.beta,
        wheelbase=args.wheelbase,
        kappa_min=args.kappa_min,
        steps_cap=args.steps_cap,
        goal_tol=args.goal_tol,
        min_sim_time=args.min_sim_time,
    )

    # ── Distribution summary ──────────────────────────────────────────────
    cnt = Counter(pi)
    total = len(pi)
    print("\n[RESULT] LD distribution:")
    for ld_val in sorted(cnt):
        bar = "█" * int(50 * cnt[ld_val] / total)
        print(f"  LD={ld_val:.1f}m: {cnt[ld_val]:5d} wp ({100*cnt[ld_val]/total:5.1f}%)  {bar}")

    # ── Write output ──────────────────────────────────────────────────────
    for i, row in enumerate(rows):
        row["ld_m"] = str(pi[i])

    out_path = os.path.abspath(args.out_csv)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    output_fieldnames = list(fieldnames) if fieldnames else list(rows[0].keys())
    if "ld_m" not in output_fieldnames:
        output_fieldnames.append("ld_m")
    if not has_heading and "heading" not in output_fieldnames:
        output_fieldnames.insert(output_fieldnames.index("y_m") + 1, "heading")

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=output_fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\n[OK] Saved: {out_path}")
    print(f"  ld_m range: [{min(pi):.2f}, {max(pi):.2f}] m")


if __name__ == "__main__":
    main()
