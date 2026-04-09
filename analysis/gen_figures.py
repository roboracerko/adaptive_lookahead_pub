"""Generate summary figures from benchmark outputs."""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import LinearSegmentedColormap
import os

import argparse as _argparse
_ap = _argparse.ArgumentParser(add_help=False)
_ap.add_argument('--raceline-dir', default=None)
_ap.add_argument('--output-dir', default=None)
_known, _ = _ap.parse_known_args()

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.dirname(_THIS_DIR)

RACELINE_DIR = _known.raceline_dir or os.path.join(_WS_ROOT, 'src', 'pp_adaptive', 'racelines')
FALLBACK_RACELINE_DIR = os.path.join(_WS_ROOT, 'src', 'pp_core', 'racelines')
OUT_DIR = _known.output_dir or os.path.join(_WS_ROOT, 'results', 'figures')
os.makedirs(OUT_DIR, exist_ok=True)

# IEEE single-column: 3.5 in; double: 7.16 in
plt.rcParams.update({
    'font.size': 8, 'font.family': 'serif',
    'axes.labelsize': 8, 'axes.titlesize': 8,
    'xtick.labelsize': 7, 'ytick.labelsize': 7,
    'legend.fontsize': 7, 'lines.linewidth': 0.8,
    'axes.linewidth': 0.6, 'grid.linewidth': 0.4,
    'figure.dpi': 300,
})

COL_K  = 'kappa_1pm'
COL_LD = 'ld_m'

C_KIN  = '#D94E1F'   # kinSim: burnt orange
C_KW   = '#1F77B4'   # kw: blue
C_FIX  = '#555555'   # pp_fixed: grey


def read_raceline(csv_name):
    for directory in [RACELINE_DIR, FALLBACK_RACELINE_DIR]:
        candidate = os.path.join(directory, csv_name)
        if os.path.exists(candidate):
            return pd.read_csv(candidate)
    raise FileNotFoundError(csv_name)

# ─────────────────────────────────────────────────────────────
# Fig 1 — κ vs L_d scatter  (Budapest, two panels)
# ─────────────────────────────────────────────────────────────
def fig1_scatter():
    kin = read_raceline('Budapest_map_optimal_rl_alg1_org.csv')
    kw  = read_raceline('Budapest_map_optimal_rl_kw2_tuned_auto.csv')

    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.4), sharey=False)
    fig.subplots_adjust(wspace=0.35)

    rho_kin = np.corrcoef(np.abs(kin[COL_K]), kin[COL_LD])[0,1]

    for ax, df, color, label, rho in [
        (axes[0], kin, C_KIN, r'\texttt{kinSim}', 0.094),
        (axes[1], kw,  C_KW,  r'\texttt{kw}',    -0.807),
    ]:
        kappa_abs = np.abs(df[COL_K])
        ld = df[COL_LD]

        # jitter L_d slightly (it's discrete)
        np.random.seed(42)
        ld_j = ld + np.random.uniform(-0.015, 0.015, len(ld))

        ax.scatter(kappa_abs, ld_j, c=color, s=2, alpha=0.35, linewidths=0,
                   rasterized=True)

        ax.set_xlabel(r'$|\kappa|$ [m$^{-1}$]')
        ax.set_ylabel(r'$L_d$ [m]')
        ax.set_title(f'{label}  ($\\rho_s = {rho:+.3f}$)', usetex=False)
        ax.axvline(0.1, color='gray', lw=0.7, ls='--', label=r'$|\kappa|=0.1$')
        ax.set_xlim(left=0)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', framealpha=0.7)

    axes[0].set_title(r'$\mathtt{kinSim}$  ($\rho_s = +0.094$)')
    axes[1].set_title(r'$\mathtt{kw}$  ($\rho_s = -0.807$)')

    fig.suptitle('Budapest raceline: curvature vs. assigned lookahead distance',
                 fontsize=8, y=1.01)
    fig.savefig(f'{OUT_DIR}/fig_scatter.pdf', bbox_inches='tight')
    plt.close()
    print('fig_scatter.pdf done')


# ─────────────────────────────────────────────────────────────
# Fig 2 — Completion rate + Lateral error heatmap (2 panels)
# ─────────────────────────────────────────────────────────────
def fig2_heatmap():
    maps = ['Budapest', 'Spielberg', 'ICRA', 'Skir', 'Austin', 'BrandsHatch']
    methods_cr  = ['pp\_fixed', 'kw', 'kinSim']
    methods_lat = ['pp\_fixed', 'kw']

    cr = np.array([
        [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],  # pp_fixed
        [ 96.7, 100.0, 100.0, 100.0, 100.0, 100.0],  # kw
        [  0.0, 100.0,   0.0,   0.0,   0.0, 100.0],  # kinSim
    ])

    lat = np.array([
        [0.0695, 0.0556, 0.0645, 0.0624, 0.0567, 0.0656],  # pp_fixed
        [0.0639, 0.0468, 0.0606, 0.0478, 0.0502, 0.0597],  # kw
    ])

    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.2))
    fig.subplots_adjust(wspace=0.5)

    # — completion rate heatmap —
    ax = axes[0]
    cmap_cr = LinearSegmentedColormap.from_list('cr', ['#D94E1F','#FFDD77','#1A7A3A'])
    im = ax.imshow(cr, cmap=cmap_cr, vmin=0, vmax=100, aspect='auto')
    ax.set_xticks(range(6)); ax.set_xticklabels(maps, rotation=35, ha='right', fontsize=6.5)
    ax.set_yticks(range(3)); ax.set_yticklabels([r'$\mathtt{pp\_fixed}$',
                                                  r'$\mathtt{kw}$',
                                                  r'$\mathtt{kinSim}$'], fontsize=7)
    for i in range(3):
        for j in range(6):
            ax.text(j, i, f'{cr[i,j]:.0f}%', ha='center', va='center',
                    fontsize=6, color='white' if cr[i,j] < 40 else 'black')
    ax.set_title('Completion rate [%]', fontsize=8)
    plt.colorbar(im, ax=ax, fraction=0.04, pad=0.02)

    # — lateral error heatmap —
    ax = axes[1]
    cmap_lat = LinearSegmentedColormap.from_list('lat', ['#1F77B4','#FFDD77','#D94E1F'])
    im2 = ax.imshow(lat, cmap=cmap_lat, vmin=0.040, vmax=0.075, aspect='auto')
    ax.set_xticks(range(6)); ax.set_xticklabels(maps, rotation=35, ha='right', fontsize=6.5)
    ax.set_yticks(range(2)); ax.set_yticklabels([r'$\mathtt{pp\_fixed}$',
                                                   r'$\mathtt{kw}$'], fontsize=7)
    for i in range(2):
        for j in range(6):
            ax.text(j, i, f'{lat[i,j]:.3f}', ha='center', va='center',
                    fontsize=6, color='white' if lat[i,j] > 0.065 else 'black')
    ax.set_title('Lateral error RMS [m]', fontsize=8)
    plt.colorbar(im2, ax=ax, fraction=0.04, pad=0.02)

    fig.suptitle('Per-map benchmark results across six maps ($N=30$ each)',
                 fontsize=8, y=1.02)
    fig.savefig(f'{OUT_DIR}/fig_heatmap.pdf', bbox_inches='tight')
    plt.close()
    print('fig_heatmap.pdf done')


# ─────────────────────────────────────────────────────────────
# Fig 3 — Steering jerk per map (grouped bars, log scale)
# ─────────────────────────────────────────────────────────────
def fig3_jerk():
    maps = ['Budapest', 'Spielberg', 'ICRA', 'Skir', 'Austin', 'BrandsHatch']
    n = len(maps)

    mean_fix = np.array([ 538.5,  324.8, 2725.7,  47.6, 158.8,  869.4])
    std_fix  = np.array([ 918.1,  407.9, 2849.8,   1.5, 187.6, 1032.6])
    mean_kw  = np.array([  47.6,   42.4,   47.3,  63.9,  48.2,   49.0])
    std_kw   = np.array([   9.3,    7.8,    8.6,   4.5,   8.3,   10.2])
    # kinSim: only 2 maps with data
    mean_kin = np.array([   np.nan, 29.0, np.nan, np.nan, np.nan, 35.9])
    std_kin  = np.array([   np.nan,  3.3, np.nan, np.nan, np.nan,  2.9])

    x = np.arange(n)
    w = 0.26
    fig, ax = plt.subplots(figsize=(7.16, 2.8))

    bars_fix = ax.bar(x - w, mean_fix, w, yerr=std_fix, color=C_FIX,
                      alpha=0.85, capsize=2, label=r'$\mathtt{pp\_fixed}$',
                      error_kw={'elinewidth': 0.7})
    bars_kw  = ax.bar(x,     mean_kw,  w, yerr=std_kw,  color=C_KW,
                      alpha=0.85, capsize=2, label=r'$\mathtt{kw}$',
                      error_kw={'elinewidth': 0.7})

    # kinSim only where data exists
    valid = ~np.isnan(mean_kin)
    ax.bar(x[valid] + w, mean_kin[valid], w, yerr=std_kin[valid],
           color=C_KIN, alpha=0.85, capsize=2, label=r'$\mathtt{kinSim}$',
           error_kw={'elinewidth': 0.7})
    # ghost bars for consistent spacing
    ax.bar(x[~valid] + w, 0, w, color=C_KIN, alpha=0.0)

    ax.set_yscale('log')
    ax.set_xticks(x); ax.set_xticklabels(maps, fontsize=7)
    ax.set_ylabel(r'Steering jerk RMS [rad/s$^2$]  (log scale)')
    ax.set_title('Per-map steering jerk RMS: $\\mathtt{kw}$ vs baselines ($N=30$)',
                 fontsize=8)
    ax.legend(loc='upper left', framealpha=0.85)
    ax.grid(axis='y', alpha=0.3, which='both')
    ax.set_ylim(bottom=10)

    # annotate ICRA outlier
    ax.annotate('ICRA:\npp\_fixed\n2725.7', xy=(2 - w, 2725.7),
                xytext=(2.6, 1800), fontsize=5.5,
                arrowprops=dict(arrowstyle='->', lw=0.6, color='gray'),
                color='gray')

    fig.savefig(f'{OUT_DIR}/fig_jerk.pdf', bbox_inches='tight')
    plt.close()
    print('fig_jerk.pdf done')


if __name__ == '__main__':
    fig1_scatter()
    fig2_heatmap()
    fig3_jerk()
    print('All done.')
