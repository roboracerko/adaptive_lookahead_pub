#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Speed-Proportional Lookahead Labeler  (pp_kv)

Assigns  Ld(i) = clip(k * v_ref(i),  ld_min,  ld_max)  to each waypoint.
No kinematic simulation required — direct analytic formula.

Reviewer baseline (R1a):
  "Adding low-effort baselines such as Ld = k·v would contextualize
   the gains beyond the prior adaptive method."

Design:
  k = 0.15 s  →  v=7.5 m/s → Ld=1.125 m  (midpoint of kw_sim range)
  The gain k is chosen so that the mean assigned Ld across the six-map
  benchmark lies between pp_fixed (1.0 m) and kw_sim (avg ~1.3 m).

  Unlike kw_sim, pp_kv has NO curvature awareness:
    - A 7 m/s straight   gets Ld = 1.05 m  (same as a 7 m/s tight corner)
    - A 5 m/s tight corner gets Ld = 0.75 m (only thanks to lower speed)
  This is the structural weakness we expect kw_sim to outperform.

Usage:
  python compute_adaptive_lookahead_kv.py \
    --in_csv  racelines/Budapest_map_optimal_rl.csv \
    --out_csv racelines/Budapest_map_optimal_rl_kv.csv \
    --k 0.15 --ld_min 0.6 --ld_max 2.0
"""

import argparse
import csv
import os
import sys

import numpy as np


# ─────────────────────────────────────────────
# CSV helpers
# ─────────────────────────────────────────────

def load_raceline(path: str):
    """Load raceline CSV; return (list-of-dicts, fieldnames)."""
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty raceline: {path}")
    if 'v_mps' not in fields:
        raise ValueError(f"Missing 'v_mps' column in {path}. Fields: {fields}")
    return rows, fields


def save_raceline(rows, fields, out_path: str):
    """Write raceline CSV, adding 'ld_m' column if absent."""
    if 'ld_m' not in fields:
        fields = list(fields) + ['ld_m']
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


# ─────────────────────────────────────────────
# Assignment
# ─────────────────────────────────────────────

def assign_kv(rows, k: float, ld_min: float, ld_max: float):
    """Assign speed-proportional lookahead to each waypoint row (in-place)."""
    ld_vals = []
    for row in rows:
        v = float(row['v_mps'])
        ld = float(np.clip(k * v, ld_min, ld_max))
        row['ld_m'] = f'{ld:.6f}'
        ld_vals.append(ld)
    return np.array(ld_vals)


# ─────────────────────────────────────────────
# Summary statistics
# ─────────────────────────────────────────────

def print_summary(ld_vals: np.ndarray, k: float, ld_min: float, ld_max: float):
    n = len(ld_vals)
    print(f"[pp_kv]  k={k:.3f} s,  ld_min={ld_min} m,  ld_max={ld_max} m")
    print(f"  N = {n} waypoints")
    print(f"  Ld:  mean={ld_vals.mean():.3f}  std={ld_vals.std():.3f}"
          f"  min={ld_vals.min():.3f}  max={ld_vals.max():.3f}  [m]")

    # Histogram in 0.1 m bins
    bins = np.arange(ld_min, ld_max + 0.15, 0.1)
    counts, edges = np.histogram(ld_vals, bins=bins)
    print("  Distribution:")
    for lo, hi, cnt in zip(edges[:-1], edges[1:], counts):
        if cnt > 0:
            bar = '█' * int(20 * cnt / n + 0.5)
            print(f"    [{lo:.1f},{hi:.1f})  {cnt:4d}  ({100*cnt/n:5.1f}%)  {bar}")

    # Clipping report
    n_min = int((ld_vals <= ld_min + 1e-6).sum())
    n_max = int((ld_vals >= ld_max - 1e-6).sum())
    if n_min:
        print(f"  ⚠  {n_min} waypoints clipped to ld_min={ld_min} m")
    if n_max:
        print(f"  ⚠  {n_max} waypoints clipped to ld_max={ld_max} m")


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='Speed-proportional lookahead labeler (pp_kv baseline).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--in_csv',  required=True,
                    help='Input raceline CSV (must contain v_mps column)')
    ap.add_argument('--out_csv', required=True,
                    help='Output CSV with ld_m column added')
    ap.add_argument('--k',       type=float, default=0.15,
                    help='Proportionality gain [s]: Ld = k * v_ref')
    ap.add_argument('--ld_min',  type=float, default=0.6,
                    help='Minimum allowed lookahead [m]')
    ap.add_argument('--ld_max',  type=float, default=2.0,
                    help='Maximum allowed lookahead [m]')
    args = ap.parse_args()

    if args.k <= 0:
        ap.error(f'--k must be positive, got {args.k}')
    if args.ld_min >= args.ld_max:
        ap.error(f'--ld_min must be < --ld_max')

    rows, fields = load_raceline(args.in_csv)
    ld_vals = assign_kv(rows, k=args.k, ld_min=args.ld_min, ld_max=args.ld_max)
    save_raceline(rows, fields, args.out_csv)
    print_summary(ld_vals, k=args.k, ld_min=args.ld_min, ld_max=args.ld_max)
    print(f"  Saved → {args.out_csv}")


if __name__ == '__main__':
    main()
