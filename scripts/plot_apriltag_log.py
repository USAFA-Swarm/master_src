#!/usr/bin/env python3
"""
Plot AprilTag offset logs produced by the apriltag_logger ROS2 node.

Usage:
    python3 scripts/plot_apriltag_log.py ~/apriltag_logs/apriltag_20260430_120000.csv
    python3 scripts/plot_apriltag_log.py <csv> --tag-id 0 --output-dir graphs
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_csv(path, tag_id_filter=None):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            tid = int(row['tag_id'])
            if tag_id_filter is not None and tid != tag_id_filter:
                continue
            rows.append({
                't':       float(row['timestamp_sec']),
                'tag_id':  tid,
                'x':       float(row['x_m']),
                'y':       float(row['y_m']),
                'z':       float(row['z_m']),
                'lateral': float(row['lateral_m']),
            })
    return rows


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _save(fig, out_dir, name):
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / name
    fig.savefig(p, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {p}')


# ---------------------------------------------------------------------------
# Individual plots
# ---------------------------------------------------------------------------

def plot_offset_vs_time(t, values, ylabel, title, stem, out_dir, hline=None):
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, values, linewidth=1.5, color='tab:blue')
    if hline is not None:
        ax.axhline(hline, color='#888888', linestyle='--', linewidth=0.9, label='zero')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    if hline is not None:
        ax.legend(fontsize=9)
    fig.tight_layout()
    _save(fig, out_dir, stem)


def plot_topdown(t, x, y, stem, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))
    norm = plt.Normalize(t.min(), t.max())
    sc = ax.scatter(x, y, c=t, cmap='viridis', norm=norm, s=12, alpha=0.85)
    cb = fig.colorbar(sc, ax=ax, shrink=0.75)
    cb.set_label('Time (s)', fontsize=10)
    ax.axhline(0, color='#999999', linestyle='--', linewidth=0.9)
    ax.axvline(0, color='#999999', linestyle='--', linewidth=0.9)
    # Mark start and end points
    ax.plot(x[0], y[0], 'o', color='#2ecc71', markersize=10, label='Start', zorder=5)
    ax.plot(x[-1], y[-1], 's', color='#e74c3c', markersize=10, label='End', zorder=5)
    ax.set_xlabel('Right offset (m)', fontsize=11)
    ax.set_ylabel('Down offset (m)', fontsize=11)
    ax.set_title('Drone Position Relative to Tag Center (top-down view)', fontsize=12)
    ax.set_aspect('equal')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    _save(fig, out_dir, stem)


def plot_combined(t, x, y, z, tag_label, stem, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))

    # X vs time
    axes[0, 0].plot(t, x, linewidth=1.5, color='tab:blue')
    axes[0, 0].axhline(0, color='#888888', linestyle='--', linewidth=0.9)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Right offset (m)')
    axes[0, 0].set_title('X Offset vs Time')
    axes[0, 0].grid(True, alpha=0.3)

    # Y vs time
    axes[0, 1].plot(t, y, linewidth=1.5, color='tab:orange')
    axes[0, 1].axhline(0, color='#888888', linestyle='--', linewidth=0.9)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Down offset (m)')
    axes[0, 1].set_title('Y Offset vs Time')
    axes[0, 1].grid(True, alpha=0.3)

    # Z vs time
    axes[1, 0].plot(t, z, linewidth=1.5, color='tab:green')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Range / depth (m)')
    axes[1, 0].set_title('Z (Range to Tag) vs Time')
    axes[1, 0].grid(True, alpha=0.3)

    # Top-down scatter
    norm = plt.Normalize(t.min(), t.max())
    sc = axes[1, 1].scatter(x, y, c=t, cmap='viridis', norm=norm, s=12, alpha=0.85)
    cb = fig.colorbar(sc, ax=axes[1, 1], shrink=0.85)
    cb.set_label('Time (s)', fontsize=9)
    axes[1, 1].axhline(0, color='#999999', linestyle='--', linewidth=0.9)
    axes[1, 1].axvline(0, color='#999999', linestyle='--', linewidth=0.9)
    axes[1, 1].plot(x[0], y[0], 'o', color='#2ecc71', markersize=8, label='Start', zorder=5)
    axes[1, 1].plot(x[-1], y[-1], 's', color='#e74c3c', markersize=8, label='End', zorder=5)
    axes[1, 1].set_xlabel('Right offset (m)')
    axes[1, 1].set_ylabel('Down offset (m)')
    axes[1, 1].set_title('Top-down Approach')
    axes[1, 1].set_aspect('equal')
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle(
        f'AprilTag Relative Position — Precision Landing Approach ({tag_label})',
        fontsize=13, fontweight='bold',
    )
    fig.tight_layout()
    _save(fig, out_dir, stem)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Plot AprilTag offset logs from apriltag_logger CSV'
    )
    parser.add_argument('csv', help='Path to apriltag_*.csv produced by apriltag_logger')
    parser.add_argument('--tag-id', type=int, default=None,
                        help='Filter to a single tag ID (default: all tags)')
    parser.add_argument('--output-dir', default='graphs',
                        help='Directory for output PNGs (default: graphs/)')
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f'ERROR: {csv_path} not found', file=sys.stderr)
        sys.exit(1)

    rows = load_csv(csv_path, tag_id_filter=args.tag_id)
    if not rows:
        print('ERROR: No matching rows in CSV (check --tag-id filter)', file=sys.stderr)
        sys.exit(1)

    tag_label = f'Tag {args.tag_id}' if args.tag_id is not None else 'All tags'
    print(f'Loaded {len(rows)} rows from {csv_path}  ({tag_label})')

    t       = np.array([r['t']       for r in rows])
    x       = np.array([r['x']       for r in rows])
    y       = np.array([r['y']       for r in rows])
    z       = np.array([r['z']       for r in rows])
    lateral = np.array([r['lateral'] for r in rows])

    out_dir = Path(args.output_dir)
    ts_str  = csv_path.stem.replace('apriltag_', '')

    print('Generating plots...')
    plot_offset_vs_time(t, x, 'Right offset (m)',
                        f'X Offset vs Time — {tag_label}',
                        f'apriltag_x_{ts_str}.png', out_dir, hline=0)
    plot_offset_vs_time(t, y, 'Down offset (m)',
                        f'Y Offset vs Time — {tag_label}',
                        f'apriltag_y_{ts_str}.png', out_dir, hline=0)
    plot_offset_vs_time(t, z, 'Range / depth (m)',
                        f'Z (Range to Tag) vs Time — {tag_label}',
                        f'apriltag_z_{ts_str}.png', out_dir)
    plot_topdown(t, x, y, f'apriltag_topdown_{ts_str}.png', out_dir)
    plot_combined(t, x, y, z, tag_label,
                  f'apriltag_combined_{ts_str}.png', out_dir)

    print(f'Done. All plots saved to {out_dir}/')
    print()
    print(f'  Duration:       {t[-1]:.1f} s')
    print(f'  Final range:    {z[-1]:.3f} m')
    print(f'  Final lateral:  {lateral[-1]:.3f} m')


if __name__ == '__main__':
    main()
