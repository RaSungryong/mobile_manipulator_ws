#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Render a 2D Ra heatmap from a `*_ra_map.csv` produced by a full-scan task.

Usage:
    ra_map_plotter.py <csv_path> [-o <png_path>] [--interpolate] [--show]

CSV schema (13 cols):
    group_id, point_id, x, y, z,
    ra_mean, ra_std, ra_min, ra_max, num_samples,
    success, execution_message, validated_at
"""
import argparse
import os
import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv_path', help='Ra map CSV (output of scan_full_* task)')
    ap.add_argument('-o', '--output', default=None,
                    help='Output PNG path (default: <csv_basename>.png alongside CSV)')
    ap.add_argument('--interpolate', action='store_true',
                    help='Also render a griddata-interpolated continuous heatmap')
    ap.add_argument('--show', action='store_true', help='Display plot interactively')
    ap.add_argument('--metric', default='ra_mean',
                    choices=['ra_mean', 'ra_std', 'ra_min', 'ra_max'],
                    help='Ra column to plot (default: ra_mean)')
    args = ap.parse_args()

    if args.show:
        matplotlib.use('TkAgg', force=True)

    if not os.path.isfile(args.csv_path):
        sys.exit(f"[error] CSV not found: {args.csv_path}")

    df = pd.read_csv(args.csv_path)

    required = {'x', 'y', args.metric, 'group_id', 'point_id'}
    missing = required - set(df.columns)
    if missing:
        sys.exit(f"[error] CSV missing columns: {missing}")

    valid = df.dropna(subset=['x', 'y', args.metric]).copy()
    if valid.empty:
        sys.exit(f"[error] No rows with valid x/y/{args.metric} — "
                 f"scan not yet run or all points failed.")

    failed = len(df) - len(valid)
    print(f"[info] Loaded {len(df)} rows, {len(valid)} with valid {args.metric} "
          f"({failed} pending/failed)")

    groups = sorted(valid['group_id'].unique())
    print(f"[info] Groups: {groups}")
    for g in groups:
        sub = valid[valid['group_id'] == g]
        print(f"  group {g}: {len(sub)} points, "
              f"x=[{sub.x.min():.3f}, {sub.x.max():.3f}]  "
              f"y=[{sub.y.min():.3f}, {sub.y.max():.3f}]  "
              f"{args.metric}=[{sub[args.metric].min():.4f}, {sub[args.metric].max():.4f}]")

    fig, axes = plt.subplots(
        1, 2 if args.interpolate else 1,
        figsize=(14 if args.interpolate else 8, 7), squeeze=False
    )

    vmin, vmax = valid[args.metric].min(), valid[args.metric].max()
    cmap = 'viridis'

    # Scatter plot (raw points)
    ax = axes[0][0]
    sc = ax.scatter(valid['x'], valid['y'], c=valid[args.metric],
                    cmap=cmap, vmin=vmin, vmax=vmax, s=18, edgecolors='none')
    plt.colorbar(sc, ax=ax, label=args.metric)
    ax.set_xlabel('world x (m)')
    ax.set_ylabel('world y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.set_title(f'Raw scan points ({args.metric})')

    # Annotate group centers
    for g in groups:
        sub = valid[valid['group_id'] == g]
        cx, cy = sub['x'].mean(), sub['y'].mean()
        ax.annotate(f'g{g}', (cx, cy), color='white', fontsize=9, weight='bold',
                    ha='center', va='center',
                    bbox=dict(boxstyle='round,pad=0.2', fc='black', alpha=0.5, ec='none'))

    # Optional interpolated continuous heatmap
    if args.interpolate:
        from scipy.interpolate import griddata
        ax2 = axes[0][1]
        # Interpolate each group separately to avoid bridging disjoint lines
        xs = np.linspace(valid['x'].min(), valid['x'].max(), 200)
        ys = np.linspace(valid['y'].min(), valid['y'].max(), 200)
        X, Y = np.meshgrid(xs, ys)
        Z_full = np.full_like(X, np.nan, dtype=float)
        for g in groups:
            sub = valid[valid['group_id'] == g]
            if len(sub) < 4:
                continue
            pts = sub[['x', 'y']].values
            vals = sub[args.metric].values
            # Limit interp region to this group's bbox (+5cm margin)
            mx_min, mx_max = sub.x.min() - 0.05, sub.x.max() + 0.05
            my_min, my_max = sub.y.min() - 0.05, sub.y.max() + 0.05
            mask = (X >= mx_min) & (X <= mx_max) & (Y >= my_min) & (Y <= my_max)
            Zi = griddata(pts, vals, (X[mask], Y[mask]), method='linear')
            Z_full[mask] = np.where(np.isnan(Zi), Z_full[mask], Zi)
        im = ax2.pcolormesh(X, Y, Z_full, cmap=cmap, vmin=vmin, vmax=vmax,
                            shading='auto')
        plt.colorbar(im, ax=ax2, label=args.metric)
        ax2.set_xlabel('world x (m)')
        ax2.set_ylabel('world y (m)')
        ax2.set_aspect('equal', adjustable='datalim')
        ax2.grid(alpha=0.3)
        ax2.set_title(f'Interpolated ({args.metric}, per-group linear)')

    fig.suptitle(f'Ra map — {os.path.basename(args.csv_path)}  '
                 f'({len(valid)}/{len(df)} points)')
    fig.tight_layout()

    out_path = args.output or os.path.splitext(args.csv_path)[0] + '.png'
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"[done] Heatmap written to: {out_path}")

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
