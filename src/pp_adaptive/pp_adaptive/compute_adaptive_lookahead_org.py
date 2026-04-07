#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline Adaptive Lookahead Label Assignment — Faithful Reproduction
Algorithm 1 of Sukhil & Behl (arXiv:2111.08873, 2021)

Structural faithfulness to the paper:
  1. Spawn at waypoint position  : T_i = (w_i.x, w_i.y, w_i.heading, v_exit_{i-1})
                                   Not the exit position of the previous simulation.
  2. v_target during simulation  : uses nearest waypoint v_ref at every sim step,
                                   following the raceline velocity profile.
  3. δ = ∫ CTE dt                : area under the cross-track error curve (paper Fig.2).
  4. Default parameters          : β=0.5, L={1.0,1.5,2.0}m, steps_cap=80,
                                   goal_tol=0.5, min_sim_time=0.2 (high-speed adjusted).

Key difference from compute_adaptive_lookahead.py:
  - compute_adaptive_lookahead.py propagates exit *position/heading* to next spawn.
  - THIS file propagates only exit *velocity* and spawns at waypoint position.

Usage (faithful paper reproduction, high-speed parameters):
  python compute_adaptive_lookahead_org.py \\
    --in_csv  racelines/Budapest_optimal_rl.csv \\
    --map_yaml /path/to/Budapest_map.yaml \\
    --out_csv  racelines/Budapest_optimal_rl_alg1_org.csv \\
    --ld_candidates 1.0 1.5 2.0 \\
    --beta 0.5 --objective CONVEX \\
    --delta_mode integral \\
    --steps_cap 80 --goal_tol 0.5 --min_sim_time 0.2
"""

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Literal, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pp_adaptive.utils.map_loader import OccupancyMap
from pp_adaptive.utils.math_utils import wrap_to_pi


# ============================================================
# Data structures
# ============================================================
@dataclass
class Waypoint:
    x: float
    y: float
    heading: float
    v_ref: float


@dataclass
class State:
    x: float
    y: float
    yaw: float
    v: float


ObjectiveType = Literal["VEL", "DEV", "CONVEX"]


# ============================================================
# Geometry helpers
# ============================================================
def nearest_index(W: List[Waypoint], x: float, y: float) -> int:
    d2 = [(w.x - x) ** 2 + (w.y - y) ** 2 for w in W]
    return int(np.argmin(d2))


def waypoint_ahead(W: List[Waypoint], i0: int, ld: float) -> int:
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


def cross_track_error(W: List[Waypoint], x: float, y: float) -> float:
    i = nearest_index(W, x, y)
    return math.hypot(W[i].x - x, W[i].y - y)


# ============================================================
# Kinematic Bicycle Simulator
# ============================================================
class KinematicBicycleSim:
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
        s.yaw = wrap_to_pi(s.yaw + yaw_rate * self.dt)
        s.x += s.v * math.cos(s.yaw) * self.dt
        s.y += s.v * math.sin(s.yaw) * self.dt


# ============================================================
# Pure Pursuit (Ackermann)
# ============================================================
def pure_pursuit(state: State, W: List[Waypoint], ld: float, L: float) -> float:
    i_near = nearest_index(W, state.x, state.y)
    i_goal = waypoint_ahead(W, i_near, ld)
    gx, gy = W[i_goal].x, W[i_goal].y
    alpha = wrap_to_pi(math.atan2(gy - state.y, gx - state.x) - state.yaw)
    return math.atan2(2.0 * L * math.sin(alpha), ld)


# ============================================================
# Objective
# ============================================================
def score(v_exit: float, delta: float, objective: ObjectiveType, beta: float) -> float:
    if objective == "VEL":
        return v_exit
    if objective == "DEV":
        return -delta
    return beta * v_exit - (1.0 - beta) * delta


# ============================================================
# Algorithm 1 — Faithful Reproduction
# ============================================================
def adaptive_lookahead_label_assignment_org(
    W: List[Waypoint],
    occ_map: OccupancyMap,
    L_candidates: List[float],
    objective: ObjectiveType,
    beta: float,
    wheelbase: float,
    steps_cap: int = 80,
    goal_tol: float = 0.5,
    min_sim_time: float = 0.2,
    delta_mode: str = "integral",
) -> List[float]:
    """
    Faithful Algorithm 1 reproduction.

    Key difference from compute_adaptive_lookahead.py:
      - Spawn position: waypoint w_i coordinates  (not exit position of previous sim)
      - Spawn velocity: v_exit from previous best  (paper: v_i+1 = v_exit_i)
      - v_target during sim: nearest waypoint v_ref at each step (raceline profile)
    """
    sim = KinematicBicycleSim(L=wheelbase, dt=0.02, a_acc=2.0, a_brake=4.0)
    min_steps = int(min_sim_time / sim.dt)

    pi: List[float] = []
    N = len(W)

    # Paper: T_init with v_init = 0
    carry_v: float = 0.0  # Only velocity is propagated between waypoints

    for i in range(N):
        # ── Paper: R = SpawnCar(T_i) ──────────────────────────────────────
        # T_i = (w_i.x, w_i.y, w_i.heading, carry_v)
        # STRUCTURAL FIX: always spawn at waypoint position, not exit position
        spawn_state = State(W[i].x, W[i].y, W[i].heading, carry_v)

        best_ld = L_candidates[0]
        best_score = -1e9
        best_exit_v: float = carry_v  # v_i+1 = v_exit_i (paper eq.)

        for ld in L_candidates:
            sim.spawn(spawn_state)
            delta_acc = 0.0
            step_count = 0

            i_near = nearest_index(W, spawn_state.x, spawn_state.y)
            i_goal = waypoint_ahead(W, i_near, ld)
            gx, gy = W[i_goal].x, W[i_goal].y

            crashed = False

            for step in range(steps_cap):
                s = sim.state
                assert s is not None
                step_count = step + 1

                if occ_map.is_collision(s.x, s.y):
                    crashed = True
                    break

                if step >= min_steps:
                    if math.hypot(s.x - gx, s.y - gy) < goal_tol:
                        break

                delta_steer = pure_pursuit(s, W, ld, wheelbase)

                # ── STRUCTURAL FIX: track raceline velocity profile ────────
                # Use nearest waypoint's v_ref at each step (not fixed W[i].v_ref)
                i_near_now = nearest_index(W, s.x, s.y)
                v_target = W[i_near_now].v_ref

                sim.step(delta_steer, v_target)
                delta_acc += cross_track_error(W, s.x, s.y) * sim.dt

            if crashed:
                v_exit = 0.0
                delta_val = float("inf")
            else:
                v_exit = sim.state.v
                if step_count > 0:
                    if delta_mode == "integral":
                        delta_val = delta_acc
                    else:
                        delta_val = delta_acc / (step_count * sim.dt)
                else:
                    delta_val = float("inf")

            s_val = score(v_exit, delta_val, objective, beta)
            if s_val > best_score:
                best_score = s_val
                best_ld = ld
                best_exit_v = v_exit  # v_i+1 = v_exit_i (paper)

        pi.append(best_ld)

        # ── Paper: v_i+1 = v_exit_i (only velocity propagated) ───────────
        carry_v = best_exit_v

    return pi


# ============================================================
# Entry point
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        description="Compute adaptive lookahead labels — faithful Algorithm 1 (Sukhil & Behl 2021)"
    )
    ap.add_argument("--in_csv", required=True)
    ap.add_argument("--map_yaml", required=True)
    ap.add_argument("--out_csv", required=True)
    ap.add_argument("--wheelbase", type=float, default=0.3302)
    ap.add_argument("--objective", choices=["VEL", "DEV", "CONVEX"], default="CONVEX")
    ap.add_argument("--beta", type=float, default=0.5,
                    help="Convex combination weight (paper optimal: 0.5)")
    ap.add_argument("--ld_candidates", nargs="+", type=float,
                    default=[1.0, 1.5, 2.0],
                    help="Paper original: 1.0 1.5 2.0")
    ap.add_argument("--steps_cap", type=int, default=80,
                    help="Max sim steps (80=1.6s; paper has no cap)")
    ap.add_argument("--goal_tol", type=float, default=0.5)
    ap.add_argument("--min_sim_time", type=float, default=0.2)
    ap.add_argument("--delta_mode", choices=["integral", "normalized"], default="integral",
                    help="'integral'=area under curve (paper), 'normalized'=time-averaged")
    args = ap.parse_args()

    print("[INFO] Loading raceline:", args.in_csv)

    rows = []
    fieldnames = None
    with open(args.in_csv, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)

    if fieldnames is None or 'x_m' not in fieldnames or 'y_m' not in fieldnames:
        raise ValueError(f"CSV must contain x_m and y_m columns. Found: {fieldnames}")

    has_heading = 'heading' in fieldnames
    if not has_heading:
        print("[INFO] heading column not found. Computing from x/y.")
        x_values = [float(row['x_m']) for row in rows]
        y_values = [float(row['y_m']) for row in rows]
        dx = np.roll(x_values, -1) - x_values
        dy = np.roll(y_values, -1) - y_values
        headings = np.arctan2(dy, dx)
        for idx, row in enumerate(rows):
            row['heading'] = str(headings[idx])

    has_v = 'v_mps' in fieldnames
    if not has_v:
        print("[WARN] v_mps not found. Using 3.0 m/s.")
        for row in rows:
            row['v_mps'] = '3.0'

    W = [Waypoint(
        x=float(r['x_m']),
        y=float(r['y_m']),
        heading=float(r.get('heading', 0.0)),
        v_ref=float(r.get('v_mps', 3.0)),
    ) for r in rows]

    occ_map = OccupancyMap(args.map_yaml)

    print(f"[INFO] Computing Algorithm 1 (faithful, org) for {len(W)} waypoints...")
    print(f"  Wheelbase:    {args.wheelbase} m")
    print(f"  Objective:    {args.objective}  (beta={args.beta})")
    print(f"  Candidates:   {args.ld_candidates}")
    print(f"  Delta mode:   {args.delta_mode}")
    print(f"  Steps cap:    {args.steps_cap}  goal_tol={args.goal_tol}  min_sim_time={args.min_sim_time}s")
    print(f"  Spawn:        waypoint position (paper-faithful)")
    print(f"  v_target:     nearest waypoint v_ref per step (raceline profile)")

    pi = adaptive_lookahead_label_assignment_org(
        W=W,
        occ_map=occ_map,
        L_candidates=args.ld_candidates,
        objective=args.objective,
        beta=args.beta,
        wheelbase=args.wheelbase,
        steps_cap=args.steps_cap,
        goal_tol=args.goal_tol,
        min_sim_time=args.min_sim_time,
        delta_mode=args.delta_mode,
    )

    # Print distribution
    from collections import Counter
    dist = Counter(pi)
    total = len(pi)
    print(f"[INFO] LD distribution:")
    for ld in sorted(dist.keys()):
        print(f"  {ld:.1f}m: {dist[ld]:5d} ({dist[ld]/total*100:.1f}%)")

    for idx, row in enumerate(rows):
        row['ld_m'] = str(pi[idx])

    out_path = os.path.abspath(args.out_csv)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    output_fieldnames = list(fieldnames) if fieldnames else list(rows[0].keys())
    if 'ld_m' not in output_fieldnames:
        output_fieldnames.append('ld_m')
    if not has_heading and 'heading' not in output_fieldnames:
        output_fieldnames.insert(output_fieldnames.index('y_m') + 1, 'heading')

    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=output_fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[OK] Saved to: {out_path}")
    print(f"  ld_m range: [{min(pi):.2f}, {max(pi):.2f}] m")


if __name__ == "__main__":
    main()
