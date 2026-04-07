#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Curvature-Conditioned Lookahead Labeler  (pp_curv)

Assigns  Ld(i) = clip(alpha_h / sqrt(max(|kappa(i)|, kappa_min)),  ld_min,  ld_max)
to each waypoint.  No kinematic simulation required — direct analytic formula.

Reviewer baseline (R1b):
  "a simple curvature-conditioned heuristic
   (e.g., Ld ∝ 1/√|κ|, with reasonable bounds)"

Design:
  alpha_h = 0.35  →  |κ|=0.10 m⁻¹ → Ld=1.11 m  (moderate corner)
                      |κ|=0.25 m⁻¹ → Ld=0.70 m  (tight corner)
                      |κ|=0.01 m⁻¹ → Ld=3.50 m  (straight, clipped to ld_max)

  Unlike kw_sim, pp_curv has NO speed awareness:
    - A 4 m/s tight corner and an 8 m/s tight corner get the same Ld.
    - The dynamic safety condition  κ_allow = a_lat_max / (C_dyn * v²)
      is absent: the same curvature is treated identically at any speed.
  This is the structural limitation we expect kw_sim to outperform.

Usage:
  python compute_adaptive_lookahead_curv.py \
    --in_csv  racelines/Budapest_map_optimal_rl.csv \
    --out_csv racelines/Budapest_map_optimal_rl_curv.csv \
    --alpha_h 0.35 --kappa_min 0.01 --ld_min 0.6 --ld_max 2.0
"""

import argparse
import csv
import math
import os
import sys

import numpy as np


# ─────────────────────────────────────────────
# CSV helpers  (same pattern as compute_adaptive_lookahead_kv.py)
# ─────────────────────────────────────────────

def load_raceline(path: str):
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty raceline: {path}")
    if 'kappa_1pm' not in fields:
        raise ValueError(
            f"Missing 'kappa_1pm' column in {path}.\n"
            f"Available columns: {fields}"
        )
    return rows, fields


def save_raceline(rows, fields, out_path: str):
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

def assign_curv(rows, alpha_h: float, kappa_min: float,
                ld_min: float, ld_max: float):
    """Assign curvature-conditioned lookahead in-place."""
    ld_vals = []
    kappa_vals = []
    for row in rows:
        kappa = abs(float(row['kappa_1pm']))
        kappa_eff = max(kappa, kappa_min)
        ld = float(np.clip(alpha_h / math.sqrt(kappa_eff), ld_min, ld_max))
        row['ld_m'] = f'{ld:.6f}'
        ld_vals.append(ld)
        kappa_vals.append(kappa)
    return np.array(ld_vals), np.array(kappa_vals)


# ─────────────────────────────────────────────
# Summary statistics
# ─────────────────────────────────────────────

def print_summary(ld_vals: np.ndarray, kappa_vals: np.ndarray,
                  alpha_h: float, kappa_min: float,
                  ld_min: float, ld_max: float):
    n = len(ld_vals)
    print(f"[pp_curv]  alpha_h={alpha_h:.3f} m^0.5,  "
          f"kappa_min={kappa_min} m⁻¹,  "
          f"ld_min={ld_min} m,  ld_max={ld_max} m")
    print(f"  N = {n} waypoints")
    print(f"  |κ|:  mean={kappa_vals.mean():.4f}  "
          f"std={kappa_vals.std():.4f}  "
          f"max={kappa_vals.max():.4f}  [m⁻¹]")
    print(f"  Ld:   mean={ld_vals.mean():.3f}  "
          f"std={ld_vals.std():.3f}  "
          f"min={ld_vals.min():.3f}  "
          f"max={ld_vals.max():.3f}  [m]")

    # Correlation check: should be strongly negative
    if len(kappa_vals) > 10:
        # simple rank correlation proxy
        from scipy.stats import spearmanr
        try:
            rho, pval = spearmanr(kappa_vals, ld_vals)
            print(f"  Spearman ρ(|κ|, Ld) = {rho:.3f}  (p={pval:.2e})"
                  f"  ← should be strongly negative")
        except ImportError:
            pass  # scipy optional

    # Distribution histogram
    bins = np.arange(ld_min, ld_max + 0.15, 0.1)
    counts, edges = np.histogram(ld_vals, bins=bins)
    print("  Ld distribution:")
    for lo, hi, cnt in zip(edges[:-1], edges[1:], counts):
        if cnt > 0:
            bar = '█' * int(20 * cnt / n + 0.5)
            print(f"    [{lo:.1f},{hi:.1f})  {cnt:4d}  ({100*cnt/n:5.1f}%)  {bar}")

    # High-curvature waypoints
    n_high = int((kappa_vals > 0.1).sum())
    print(f"  |κ|>0.1 m⁻¹: {n_high}/{n} ({100*n_high/n:.1f}%) waypoints")

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
        description='Curvature-conditioned lookahead labeler (pp_curv baseline).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument('--in_csv',    required=True,
                    help='Input raceline CSV (must contain kappa_1pm column)')
    ap.add_argument('--out_csv',   required=True,
                    help='Output CSV with ld_m column added')
    ap.add_argument('--alpha_h',   type=float, default=0.35,
                    help='Scaling coefficient: Ld = alpha_h / sqrt(|kappa|)  [m^0.5 units]')
    ap.add_argument('--kappa_min', type=float, default=0.01,
                    help='Floor curvature to prevent Ld → ∞ on near-straight segments [m⁻¹]')
    ap.add_argument('--ld_min',    type=float, default=0.6,
                    help='Minimum allowed lookahead [m]')
    ap.add_argument('--ld_max',    type=float, default=2.0,
                    help='Maximum allowed lookahead [m]')
    args = ap.parse_args()

    if args.alpha_h <= 0:
        ap.error(f'--alpha_h must be positive, got {args.alpha_h}')
    if args.kappa_min <= 0:
        ap.error(f'--kappa_min must be positive, got {args.kappa_min}')
    if args.ld_min >= args.ld_max:
        ap.error('--ld_min must be < --ld_max')

    rows, fields = load_raceline(args.in_csv)
    ld_vals, kappa_vals = assign_curv(
        rows,
        alpha_h=args.alpha_h,
        kappa_min=args.kappa_min,
        ld_min=args.ld_min,
        ld_max=args.ld_max,
    )
    save_raceline(rows, fields, args.out_csv)
    print_summary(ld_vals, kappa_vals,
                  alpha_h=args.alpha_h, kappa_min=args.kappa_min,
                  ld_min=args.ld_min, ld_max=args.ld_max)
    print(f"  Saved → {args.out_csv}")


if __name__ == '__main__':
    main()
