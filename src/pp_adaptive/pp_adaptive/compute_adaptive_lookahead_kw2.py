#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
κ-Window Lookahead Assignment v2  (offline LD labeler)

Improvements over kw_sim (compute_adaptive_lookahead_kw.py, mode=kw_sim):

  [A] Speed-aware κ filter  — curvature-domain safety condition
      kw_sim:  condition = ld ≤ α / κ_window           (geometry only)
      kw2:     condition = ld ≤ α / κ_window            (geometry, same)
               AND  κ_window ≤ κ_allow(v)              (speed-aware, NEW)
               where κ_allow = a_lat_max / (C_dyn × v_ref²)  [units: 1/m ✓]

      Derivation (pure-pursuit worst case on radius-R arc):
        heading error  α  ≈ ld × κ
        steering       δ  ≈ 2L × κ
        lateral accel  a_lat = v × ω = 2v²κ  ≤  a_lat_max
        ⟹  κ  ≤  a_lat_max / (2v²)          [κ_allow, C_dyn=2]

      Note: the original v1 draft wrote LD_safe_dyn = a_lat_max/(C×v²×κ),
      which is dimensionless, NOT metres.  This revision corrects that error.

  [B] ALAT_MAX_CTE objective  — maximize LD with CTE guard
      kw_sim:  score = −max_a_lat              (large LD bias already)
      kw2:     hard exclude if max_a_lat > a_lat_max
               then  score = ld − w_cte × avg_cte
      → Selects the LARGEST safe LD; CTE penalty prevents path-deviation
        oscillation when a large LD would cause excessive tracking error.
      → w_cte=0 reduces to pure ALAT_MAX; w_cte=∞ reduces to DEV.

  [C] Dynamic v_target in sim  — raceline speed profile per step
      kw_sim:  v_target = W[i].v_ref  (fixed at spawn waypoint)
      kw2:     v_target = W[nearest_sim_wp].v_ref  (updated every step)
      → Captures braking into corners; more realistic a_lat estimate.
      Implemented via O(2·radius) incremental search (radius=12).

  [D] Extended κ-window scan  — look-ahead margin beyond LD
      kw_sim:  scan [0, ld] metres ahead
      kw2:     scan [0, ld + v_ref × kw_margin_t] metres ahead
      → Catches corners that start just past the lookahead point.
      Default kw_margin_t = 0.3 s  →  +2.25 m at 7.5 m/s.

Implementation order (risk-ascending): C → D → E(data) → B → A
Reference: report/KW2_IMPROVEMENT_PLAN.md (v2)

Usage:
  python compute_adaptive_lookahead_kw2.py \\
    --in_csv  racelines/Budapest_optimal_rl_ld.csv \\
    --map_yaml /path/to/budapest_map.yaml \\
    --out_csv racelines/Budapest_optimal_rl_kw2.csv \\
    --auto_alpha --a_lat_max 7.5 --c_dyn 2.0 --kw_margin_t 0.3 --w_cte 0.5
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter
from dataclasses import dataclass
from typing import List, Optional

import numpy as np


# ─────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────

@dataclass
class Waypoint:
    x: float
    y: float
    heading: float
    kappa: float   # signed curvature (1/m)
    v_ref: float   # raceline reference speed (m/s)


@dataclass
class State:
    x: float
    y: float
    yaw: float
    v: float


# ─────────────────────────────────────────────
# κ-Window  [Improvement D: extended scan]
# ─────────────────────────────────────────────

def kappa_window_max(W: List[Waypoint], i: int, scan_dist: float) -> float:
    """
    Return max |κ| over waypoints within arc-distance `scan_dist` ahead of W[i].

    [kw_sim]  scan_dist = ld
    [kw2]     scan_dist = ld + v_ref * kw_margin_t        [Improvement D]

    Includes W[i].kappa itself (the car is already there).
    """
    n = len(W)
    acc = 0.0
    j = i
    kappa_max = abs(W[i].kappa)

    while acc < scan_dist:
        j_next = (j + 1) % n
        seg = math.hypot(W[j_next].x - W[j].x, W[j_next].y - W[j].y)
        acc += seg
        kw = abs(W[j_next].kappa)
        if kw > kappa_max:
            kappa_max = kw
        j = j_next
        if j == i:
            break   # full lap — should not happen for reasonable scan_dist

    return kappa_max


# ─────────────────────────────────────────────
# Sim helpers
# ─────────────────────────────────────────────

def _wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _nearest_index(W: List[Waypoint], x: float, y: float) -> int:
    """Full O(N) search — used once at sim spawn to initialise hint."""
    d2 = [(w.x - x) ** 2 + (w.y - y) ** 2 for w in W]
    return int(np.argmin(d2))


def _nearest_index_incremental(
    W: List[Waypoint], x: float, y: float, i_hint: int, radius: int = 12
) -> int:
    """
    O(2·radius) incremental nearest-waypoint search.  [Improvement C]

    Assumes the sim vehicle moves forward along the raceline between calls,
    so the true nearest index shifts by at most `radius` steps per call.
    radius=12 covers ~1.2 m at 0.1 m waypoint spacing — safe for
    v=8 m/s, dt=0.02 s (0.16 m/step).
    """
    n = len(W)
    best_d2 = float("inf")
    best_i = i_hint
    for di in range(-radius, radius + 1):
        j = (i_hint + di) % n
        d2 = (W[j].x - x) ** 2 + (W[j].y - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = j
    return best_i


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


def _pure_pursuit(
    state: State, W: List[Waypoint], ld: float, L: float, i_near: int
) -> float:
    """
    Compute pure-pursuit steering angle.

    `i_near` must be pre-computed by the caller (full or incremental search).
    Accepting it as a parameter avoids a redundant O(N) _nearest_index call
    inside the sim loop, where i_sim_near is already available.  [Improvement C]
    """
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

    def spawn(self, s: State) -> None:
        self.state = State(s.x, s.y, s.yaw, s.v)

    def step(self, delta: float, v_target: float) -> None:
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


# ─────────────────────────────────────────────
# Objective  [Improvement B: ALAT_MAX_CTE]
# ─────────────────────────────────────────────

def _score(
    max_a_lat: float,
    avg_cte: float,
    ld: float,
    objective: str,
    v_exit: float,
    beta: float,
    a_lat_max: float,
    w_cte: float,
) -> float:
    """
    ALAT_MAX_CTE (kw2 default):
        Hard exclude:  max_a_lat > a_lat_max  →  score = -∞
        Otherwise:     score = ld - w_cte × avg_cte
        → Selects LARGEST LD within a_lat budget.
        → CTE penalty prevents path-deviation oscillation with over-large LD.
        → w_cte=0: pure ALAT_MAX (LD only).  w_cte=∞: pure DEV.
        Reversal threshold: ΔLD / w_cte  (e.g. 0.3m / 0.5 = 0.6m CTE gap
        needed to swap LD=1.2m ↔ LD=0.9m).

    ALAT (legacy):
        score = -max_a_lat  (minimise; large LD tends to win — see plan §0)

    VEL / DEV / CONVEX: kept for ablation.
    """
    if objective == "ALAT_MAX_CTE":
        if max_a_lat > a_lat_max:
            return -1e9
        return ld - w_cte * avg_cte

    if objective == "ALAT_MAX":
        # Pure version without CTE guard — w_cte=0 case
        return ld if max_a_lat <= a_lat_max else -1e9

    if objective == "ALAT":
        return -max_a_lat

    if objective == "VEL":
        return v_exit

    if objective == "DEV":
        return -avg_cte

    # CONVEX
    return beta * v_exit - (1.0 - beta) * avg_cte


# ─────────────────────────────────────────────
# Dual safety filter helper  [Improvement A]
# ─────────────────────────────────────────────

def _is_ld_safe(
    ld: float,
    kw: float,          # κ_window(i, scan_dist) — pre-computed
    alpha: float,
    kappa_min: float,
    kappa_allow: float, # a_lat_max / (C_dyn × v_ref²)  [units: 1/m]
) -> bool:
    """
    Two independent safety conditions for accepting an LD candidate.

    Condition 1 — Geometric (turning circle):
        ld ≤ α / max(κ_window, κ_min)
        The lookahead point must be reachable given the upcoming curvature.

    Condition 2 — Speed-aware (lateral acceleration):   [Improvement A]
        κ_window ≤ κ_allow = a_lat_max / (C_dyn × v_ref²)   [1/m ✓]
        The curvature in the scan window must be drivable at current speed.
        This condition is INDEPENDENT of ld — it gates on the track geometry
        at the candidate's scan horizon, not on ld directly.

    Both must hold for the candidate to enter the valid set.

    Note on dimensionality:
        κ_allow  [1/m] = a_lat_max [m/s²] / (C_dyn [1] × v² [m²/s²])
                       = [m/s²] / [m/s²] × [m]  =  [1/m]   ✓
    """
    kw_eff = max(kw, kappa_min)

    # Condition 1: geometric
    if ld > alpha / kw_eff:
        return False

    # Condition 2: speed-aware curvature limit
    if kw > kappa_allow:
        return False

    return True


def _nearest_candidate(v: float, candidates: List[float]) -> float:
    return min(candidates, key=lambda c: abs(c - v))


def _postprocess_ld_sequence(
    pi: List[float],
    candidates: List[float],
    smooth_window: int = 0,
    jump_limit: float = 0.0,
) -> List[float]:
    """
    Optional LD post-processing for continuity:
      1) circular median smoothing (odd window >= 3)
      2) per-step jump clamp (|Δld| <= jump_limit)
    """
    if not pi:
        return pi

    out = list(pi)
    n = len(out)
    cand = sorted(candidates)

    # 1) Circular median smoothing
    if smooth_window >= 3:
        if smooth_window % 2 == 0:
            smooth_window += 1
        half = smooth_window // 2
        smoothed: List[float] = []
        for i in range(n):
            vals = [out[(i + di) % n] for di in range(-half, half + 1)]
            med = float(np.median(vals))
            smoothed.append(_nearest_candidate(med, cand))
        out = smoothed

    # 2) Sequential jump clamp
    if jump_limit > 0.0 and n >= 2:
        clamped = [out[0]]
        for i in range(1, n):
            prev = clamped[-1]
            cur = out[i]
            diff = cur - prev
            if abs(diff) > jump_limit:
                cur = prev + math.copysign(jump_limit, diff)
            clamped.append(_nearest_candidate(cur, cand))
        out = clamped

    return out


# ─────────────────────────────────────────────
# Main assignment
# ─────────────────────────────────────────────

def kw2_lookahead_assignment(
    W: List[Waypoint],
    occ_map,
    L_candidates: List[float],
    alpha: float,
    a_lat_max: float = 7.5,
    c_dyn: float = 2.0,
    kw_margin_t: float = 0.3,
    w_cte: float = 0.5,
    objective: str = "ALAT_MAX_CTE",
    beta: float = 0.6,
    wheelbase: float = 0.3302,
    kappa_min: float = 0.005,
    steps_cap: int = 80,
    goal_tol: float = 0.5,
    min_sim_time: float = 0.2,
    track_half_width: float = 1.5,
    offtrack_margin: float = 0.1,
    boundary_buffer: float = 0.05,
    ld_smooth_window: int = 0,
    ld_jump_limit: float = 0.0,
    dynamic_v_target: bool = True,
) -> List[float]:
    """
    κ-Window v2 + Kinematic Sim lookahead assignment.

    Step 1  Extended κ-window + dual safety filter  [D + A]
    ─────────────────────────────────────────────────────────
      scan_dist  = ld + v_ref × kw_margin_t         [D]
      kw         = κ_window_max(i, scan_dist)

      κ_allow    = a_lat_max / (C_dyn × v_ref²)     [A, 1/m]

      Condition 1:  ld    ≤ α / max(kw, κ_min)      geometric
      Condition 2:  kw    ≤ κ_allow                  speed-aware
      valid_ld   = { ld | both conditions hold }

    Step 2  Dynamic-v sim  [C]
    ─────────────────────────────────────────────────────────
      Spawn at (W[i].x, W[i].y, W[i].heading, v=W[i].v_ref)
      Each step: v_target = W[nearest_sim_wp].v_ref

    Step 3  ALAT_MAX_CTE scoring  [B]
    ─────────────────────────────────────────────────────────
      Hard exclude: max_a_lat > a_lat_max
      Score:        ld - w_cte × avg_cte
      π[i]        = argmax(score)

    Fallback: min(L_candidates) if valid_ld is empty.
    Shortcut: skip sim if len(valid_ld) == 1.
    """
    sim = _KinematicBicycleSim(L=wheelbase, dt=0.02, a_acc=2.0, a_brake=4.0)
    min_steps = int(min_sim_time / sim.dt)
    L_candidates = sorted(L_candidates)
    pi: List[float] = []
    # Keep a margin from benchmark off-track threshold:
    # |lateral_error| <= track_half_width + offtrack_margin
    cte_limit = max(0.1, track_half_width + offtrack_margin - boundary_buffer)

    for i, wp in enumerate(W):
        v_ref = max(wp.v_ref, 0.1)

        # ── Step 1: Dual safety filter ───────────────────────────────────
        # κ_allow: max curvature safely drivable at v_ref  [1/m]
        kappa_allow = a_lat_max / (c_dyn * v_ref ** 2)

        valid_ld: List[float] = []
        for ld in L_candidates:
            scan_dist = ld + v_ref * kw_margin_t   # [D] extended scan
            kw = kappa_window_max(W, i, scan_dist)
            if _is_ld_safe(ld, kw, alpha, kappa_min, kappa_allow):
                valid_ld.append(ld)

        if not valid_ld:
            pi.append(L_candidates[0])
            continue

        if len(valid_ld) == 1:
            pi.append(valid_ld[0])
            continue

        # ── Step 2 + 3: Sim scoring ──────────────────────────────────────
        best_ld: Optional[float] = None
        best_score = float("-inf")

        for ld in valid_ld:
            T_init = State(wp.x, wp.y, wp.heading, v=v_ref)
            sim.spawn(T_init)

            # [C] Incremental index: full O(N) scan once, then O(radius)
            i_sim_near = _nearest_index(W, T_init.x, T_init.y)
            v_target = v_ref  # ablation [C] fixed; overwritten each step if dynamic_v_target

            i_goal = _waypoint_ahead(W, i_sim_near, ld)
            gx, gy = W[i_goal].x, W[i_goal].y

            delta_acc = 0.0
            max_a_lat = 0.0
            max_cte = 0.0
            step_count = 0
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

                # [C] Dynamic v_target + reuse i_sim_near for pure pursuit
                # Incremental O(2·radius) search replaces two O(N) scans:
                #   one for v_target, one that _pure_pursuit used to do internally.
                i_sim_near = _nearest_index_incremental(
                    W, s.x, s.y, i_sim_near
                )
                if dynamic_v_target:
                    v_target = W[i_sim_near].v_ref
                # else: v_target stays = v_ref (spawn waypoint speed, ablation [C])

                delta = _pure_pursuit(s, W, ld, wheelbase, i_sim_near)
                sim.step(delta, v_target)

                # CTE: distance to nearest raceline point
                cte = math.hypot(W[i_sim_near].x - s.x, W[i_sim_near].y - s.y)
                if cte > max_cte:
                    max_cte = cte
                if max_cte > cte_limit:
                    crashed = True
                    break
                delta_acc += cte * sim.dt

                kappa_sim = abs(math.tan(delta)) / max(wheelbase, 1e-6)
                a_lat = (s.v ** 2) * kappa_sim
                if a_lat > max_a_lat:
                    max_a_lat = a_lat

            if crashed:
                continue

            v_exit = sim.state.v
            avg_cte = delta_acc / (step_count * sim.dt) if step_count > 0 else float("inf")

            # [B] ALAT_MAX_CTE scoring
            s_val = _score(
                max_a_lat=max_a_lat,
                avg_cte=avg_cte,
                ld=ld,
                objective=objective,
                v_exit=v_exit,
                beta=beta,
                a_lat_max=a_lat_max,
                w_cte=w_cte,
            )
            if objective in ("ALAT_MAX_CTE", "ALAT_MAX") and s_val <= -1e8:
                continue
            if s_val > best_score:
                best_score = s_val
                best_ld = ld

        if best_ld is None:
            # All valid candidates were rejected/crashed in simulation.
            pi.append(valid_ld[0])
        else:
            pi.append(best_ld)

        if (i + 1) % 200 == 0:
            print(f"  [{i+1}/{len(W)}] done...")

    pi = _postprocess_ld_sequence(
        pi,
        candidates=L_candidates,
        smooth_window=ld_smooth_window,
        jump_limit=ld_jump_limit,
    )
    return pi


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description=(
            "κ-Window Lookahead Assignment v2 (kw2).\n"
            "Improvements over kw_sim: speed-aware κ filter [A], ALAT_MAX_CTE [B],\n"
            "dynamic v_target [C], extended κ-window scan [D].\n"
            "See report/KW2_IMPROVEMENT_PLAN.md for design rationale."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # I/O
    ap.add_argument("--in_csv",   required=True, help="Input raceline CSV (x_m, y_m, kappa_1pm, v_mps, ...)")
    ap.add_argument("--out_csv",  required=True, help="Output CSV with ld_m column")
    ap.add_argument("--map_yaml", required=True, help="Map YAML for occupancy collision check")

    # κ-window filter
    ap.add_argument("--alpha", type=float, default=None,
                    help="Geometric safety factor LD ≤ α/κ. Omit with --auto_alpha. Default 0.30.")
    ap.add_argument("--auto_alpha", action="store_true",
                    help="Auto-compute α = κ_p90 × LD_min + 0.01.")
    ap.add_argument("--ld_candidates", nargs="+", type=float,
                    default=[0.6, 0.9, 1.2, 1.6, 2.0],
                    help="LD candidates in metres. Default: 0.6 0.9 1.2 1.6 2.0")
    ap.add_argument("--kappa_min", type=float, default=0.005,
                    help="Floor curvature to prevent div-by-zero (1/m). Default 0.005.")

    # [A] Speed-aware κ filter
    ap.add_argument("--a_lat_max", type=float, default=7.5,
                    help="[A+B] Lateral acceleration limit (m/s²). Default 7.5.")
    ap.add_argument("--c_dyn", type=float, default=2.0,
                    help="[A] Dynamic correction factor: κ_allow = a_lat_max/(c_dyn×v²). "
                         "Theoretical worst-case = 2.0. Default 2.0.")

    # [D] Extended κ-window margin
    ap.add_argument("--kw_margin_t", type=float, default=0.3,
                    help="[D] Extra scan beyond LD: scan_dist = ld + v_ref×kw_margin_t (s). "
                         "Default 0.3 s → +2.25 m at 7.5 m/s.")

    # [B] ALAT_MAX_CTE
    ap.add_argument("--w_cte", type=float, default=0.5,
                    help="[B] CTE penalty weight in score = ld - w_cte×avg_cte. "
                         "w_cte=0 → pure ALAT_MAX. Default 0.5.")
    ap.add_argument("--objective",
                    choices=["ALAT_MAX_CTE", "ALAT_MAX", "ALAT", "VEL", "DEV", "CONVEX"],
                    default="ALAT_MAX_CTE",
                    help="[B] Scoring objective. Default ALAT_MAX_CTE.")

    # Sim parameters
    ap.add_argument("--beta",         type=float, default=0.6,   help="CONVEX weight β (unused for ALAT_MAX_CTE)")
    ap.add_argument("--wheelbase",    type=float, default=0.3302, help="Vehicle wheelbase (m)")
    ap.add_argument("--steps_cap",    type=int,   default=80,    help="Max sim steps per candidate. Default 80 (1.6 s).")
    ap.add_argument("--goal_tol",     type=float, default=0.5,   help="Goal arrival tolerance (m)")
    ap.add_argument("--min_sim_time", type=float, default=0.2,   help="Min sim time before goal check (s)")
    ap.add_argument("--track_half_width", type=float, default=1.5,
                    help="Track half width for offline CTE guard (m). Default 1.5.")
    ap.add_argument("--offtrack_margin", type=float, default=0.1,
                    help="Offtrack margin for offline CTE guard (m). Default 0.1.")
    ap.add_argument("--boundary_buffer", type=float, default=0.05,
                    help="Safety buffer inside boundary for offline CTE guard (m). Default 0.05.")
    ap.add_argument("--ld_smooth_window", type=int, default=0,
                    help="Post LD circular median window (odd >=3). 0 disables. Default 0.")
    ap.add_argument("--ld_jump_limit", type=float, default=0.0,
                    help="Post LD max per-waypoint jump clamp (m). 0 disables. Default 0.")
    ap.add_argument("--no_dynamic_v_target", action="store_true",
                    help="[C ablation] Fix v_target at spawn waypoint speed (kw_sim-style). "
                         "Default: update v_target each sim step from nearest waypoint.")

    args = ap.parse_args()

    # ── Banner ────────────────────────────────────────────────────────────
    print("=" * 62)
    print("  κ-Window Lookahead Assignment  v2  (kw2)")
    print("=" * 62)
    print(f"  in_csv        : {args.in_csv}")
    print(f"  out_csv       : {args.out_csv}")
    print(f"  ld_candidates : {args.ld_candidates}")
    print(f"  objective     : {args.objective}")
    print(f"  a_lat_max     : {args.a_lat_max} m/s²   [A+B]")
    print(f"  c_dyn         : {args.c_dyn}             [A]  κ_allow = a_lat_max/(c_dyn×v²)")
    print(f"  kw_margin_t   : {args.kw_margin_t} s     [D]  scan = ld + v×{args.kw_margin_t}")
    print(f"  w_cte         : {args.w_cte}             [B]  score = ld - {args.w_cte}×avg_cte")
    print(f"  steps_cap     : {args.steps_cap} × 0.02 s = {args.steps_cap*0.02:.1f} s")
    print(f"  cte_guard     : half_width={args.track_half_width}, margin={args.offtrack_margin}, "
          f"buffer={args.boundary_buffer}")
    print(f"  ld_post       : smooth_window={args.ld_smooth_window}, jump_limit={args.ld_jump_limit}")
    print(f"  dynamic_v_target: {not args.no_dynamic_v_target}  [C]  (False = fixed v_target, ablation)")

    # ── Load CSV ──────────────────────────────────────────────────────────
    rows = []
    fieldnames = None
    with open(args.in_csv, "r") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)

    if fieldnames is None or "x_m" not in fieldnames or "y_m" not in fieldnames:
        raise ValueError(f"CSV must have x_m and y_m columns. Found: {fieldnames}")

    # ── Heading ───────────────────────────────────────────────────────────
    has_heading = "heading" in (fieldnames or [])
    if not has_heading:
        print("[INFO] 'heading' absent — computing from x/y.")
        xs = [float(r["x_m"]) for r in rows]
        ys = [float(r["y_m"]) for r in rows]
        dx = np.roll(xs, -1) - np.array(xs)
        dy = np.roll(ys, -1) - np.array(ys)
        headings = np.arctan2(dy, dx).tolist()
        for i, row in enumerate(rows):
            row["heading"] = str(headings[i])

    # ── Curvature ─────────────────────────────────────────────────────────
    kappa_col = next(
        (c for c in ("kappa_1pm", "kappa", "curvature") if c in (fieldnames or [])),
        None,
    )
    if kappa_col is None:
        raise ValueError(f"CSV must have kappa_1pm / kappa / curvature. Found: {fieldnames}")
    print(f"  kappa column  : '{kappa_col}'")

    # ── Speed ─────────────────────────────────────────────────────────────
    if "v_mps" not in (fieldnames or []):
        print("[WARN] 'v_mps' absent — using 3.0 m/s default.")
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
    v_vals     = [w.v_ref for w in W]

    # ── Alpha resolution ──────────────────────────────────────────────────
    if args.alpha is not None:
        alpha = args.alpha
        alpha_src = "manual"
    elif args.auto_alpha:
        k_p90 = float(np.percentile(kappa_vals, 90))
        ld_min = min(args.ld_candidates)
        alpha = round(k_p90 * ld_min + 0.01, 4)
        alpha_src = f"auto (κ_p90={k_p90:.4f} × {ld_min} + 0.01)"
    else:
        alpha = 0.30
        alpha_src = "default"

    print(f"\n  Raceline : {len(W)} waypoints")
    print(f"  |κ| range: [{min(kappa_vals):.4f}, {max(kappa_vals):.4f}] 1/m")
    print(f"  v range  : [{min(v_vals):.2f}, {max(v_vals):.2f}] m/s")
    print(f"  α        : {alpha:.4f}  [{alpha_src}]")

    # Representative operating point diagnostics
    v90 = float(np.percentile(v_vals, 90))
    k90 = float(np.percentile(kappa_vals, 90))
    kappa_allow_diag = args.a_lat_max / (args.c_dyn * v90 ** 2)
    ld_safe_geom_diag = alpha / max(k90, args.kappa_min)
    binding = "speed [A]" if kappa_allow_diag < k90 else "geometry"
    print(f"\n  At v_p90={v90:.2f} m/s, κ_p90={k90:.4f} 1/m:")
    print(f"    κ_allow (speed limit)  = {kappa_allow_diag:.4f} 1/m   [A]")
    print(f"    κ_p90 vs κ_allow       : {'κ_p90 > κ_allow → cond.2 rejects some LD at high-v corners' if k90 > kappa_allow_diag else 'κ_p90 ≤ κ_allow → cond.2 rarely triggers'}")
    print(f"    LD_safe_geom (cond.1)  = {ld_safe_geom_diag:.2f} m")
    print(f"    Binding constraint     : {binding}")

    # ── Load map ──────────────────────────────────────────────────────────
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from pp_adaptive.utils.map_loader import OccupancyMap
    occ_map = OccupancyMap(args.map_yaml)

    # ── Compute ───────────────────────────────────────────────────────────
    print(f"\n[RUN] kw2 — {len(W)} waypoints × {len(args.ld_candidates)} candidates ...")
    pi = kw2_lookahead_assignment(
        W=W,
        occ_map=occ_map,
        L_candidates=args.ld_candidates,
        alpha=alpha,
        a_lat_max=args.a_lat_max,
        c_dyn=args.c_dyn,
        kw_margin_t=args.kw_margin_t,
        w_cte=args.w_cte,
        objective=args.objective,
        beta=args.beta,
        wheelbase=args.wheelbase,
        kappa_min=args.kappa_min,
        steps_cap=args.steps_cap,
        goal_tol=args.goal_tol,
        min_sim_time=args.min_sim_time,
        track_half_width=args.track_half_width,
        offtrack_margin=args.offtrack_margin,
        boundary_buffer=args.boundary_buffer,
        ld_smooth_window=args.ld_smooth_window,
        ld_jump_limit=args.ld_jump_limit,
        dynamic_v_target=not args.no_dynamic_v_target,
    )

    # ── Distribution + safety report ─────────────────────────────────────
    cnt = Counter(pi)
    total = len(pi)
    print("\n[RESULT] LD distribution:")
    for ld_val in sorted(cnt):
        bar = "█" * int(50 * cnt[ld_val] / total)
        print(f"  LD={ld_val:.1f}m: {cnt[ld_val]:5d} wp ({100*cnt[ld_val]/total:5.1f}%)  {bar}")

    violations = sum(
        1 for idx, ld_val in enumerate(pi)
        if abs(W[idx].kappa) > 1e-6 and ld_val > 1.0 / abs(W[idx].kappa)
    )
    print(f"\n  LD > R violations : {violations} / {total} ({100*violations/total:.2f}%)  [target: 0%]")

    # ── Write output ──────────────────────────────────────────────────────
    for i, row in enumerate(rows):
        row["ld_m"] = str(pi[i])

    out_path = os.path.abspath(args.out_csv)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

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
    print(f"     ld_m range: [{min(pi):.2f}, {max(pi):.2f}] m")


if __name__ == "__main__":
    main()
