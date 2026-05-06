#!/usr/bin/env python3
"""
Plot SITL flight path logs produced by the flight_logger ROS2 node.

Usage:
    python3 scripts/plot_flight_log.py ~/flight_logs/flight_20260430_120000.csv
    python3 scripts/plot_flight_log.py <csv> --output-dir graphs
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers '3d' projection

PHASE_COLORS = {
    'TAKEOFF': '#2ecc71',
    'SEARCH':  '#3498db',
    'TRANSIT': '#e67e22',
    'CRUISE':  '#9b59b6',
    'LAND':    '#e74c3c',
}


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                't': float(row['timestamp_sec']),
                'x': float(row['x_m']),
                'y': float(row['y_m']),
                'z': float(row['z_m']),
            })
    return rows


# ---------------------------------------------------------------------------
# Phase detection
# ---------------------------------------------------------------------------

def _smooth(arr, window=7):
    w = min(window, len(arr))
    return np.convolve(arr, np.ones(w) / w, mode='same')


def _circularity(xs, ys):
    """Path-length / chord-length ratio — high = circular, low = linear."""
    if len(xs) < 3:
        return 1.0
    dx, dy = np.diff(xs), np.diff(ys)
    path = float(np.sum(np.sqrt(dx**2 + dy**2)))
    chord = float(np.sqrt((xs[-1] - xs[0])**2 + (ys[-1] - ys[0])**2)) + 1e-6
    return path / chord


def detect_phases(t, x, y, z):
    """
    Returns list of (time_sec, label) marking phase start times.
    Detects TAKEOFF, SEARCH (or CRUISE), TRANSIT (if distinguishable), LAND.
    Returns [] if altitude never exceeds threshold (e.g. ground-only log).
    """
    z_s = _smooth(z)
    z_max = float(np.max(z_s))
    if z_max < 0.3:
        return []

    thresh = 0.55 * z_max

    # TAKEOFF end: first sample above threshold while climbing
    takeoff_end = None
    for i in range(1, len(z_s)):
        if z_s[i] > thresh and z_s[i] >= z_s[i - 1]:
            takeoff_end = i
            break

    # LAND start: last sample above threshold before final descent
    land_start = None
    for i in range(len(z_s) - 2, -1, -1):
        if z_s[i] > thresh and z_s[i] >= z_s[i + 1]:
            land_start = i
            break

    if takeoff_end is None or land_start is None or takeoff_end >= land_start:
        return [(t[0], 'TAKEOFF')]

    # Try to split cruise into SEARCH + TRANSIT by comparing circularity of each half
    cruise_x = x[takeoff_end:land_start]
    cruise_y = y[takeoff_end:land_start]
    n = len(cruise_x)

    if n > 40:
        half = n // 2
        c1 = _circularity(cruise_x[:half], cruise_y[:half])
        c2 = _circularity(cruise_x[half:], cruise_y[half:])
        if c1 > 2.5 and c2 < c1 * 0.55:
            return [
                (t[0],                     'TAKEOFF'),
                (t[takeoff_end],           'SEARCH'),
                (t[takeoff_end + half],    'TRANSIT'),
                (t[land_start],            'LAND'),
            ]

    return [
        (t[0],           'TAKEOFF'),
        (t[takeoff_end], 'CRUISE'),
        (t[land_start],  'LAND'),
    ]


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _phase_segments(t, phases):
    """Convert [(time, label)] into [(start_idx, end_idx, color, label)]."""
    if not phases:
        return [(0, len(t) - 1, '#3498db', 'FLIGHT')]

    indices = [int(np.searchsorted(t, pt)) for pt, _ in phases]
    indices = [max(0, min(i, len(t) - 1)) for i in indices]
    segments = []
    for i, (_, label) in enumerate(phases):
        start = indices[i]
        end = (indices[i + 1] - 1) if i + 1 < len(indices) else len(t) - 1
        end = min(end, len(t) - 1)
        segments.append((start, end, PHASE_COLORS.get(label, '#888888'), label))
    return segments


def _add_phase_vlines(ax, t, phases):
    """Draw dashed vertical lines and labels for each phase transition."""
    ymin, ymax = ax.get_ylim()
    label_y = ymin + (ymax - ymin) * 0.82
    span = t[-1] - t[0]
    for pt, label in phases[1:]:  # skip index-0 (starts at t=0)
        ax.axvline(pt, color='gray', linestyle='--', linewidth=0.9, alpha=0.7)
        ax.text(pt + span * 0.005, label_y, label,
                fontsize=8, rotation=90, va='top', ha='left', color='#555555')


def _save(fig, out_dir, name):
    out_dir.mkdir(parents=True, exist_ok=True)
    p = out_dir / name
    fig.savefig(p, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {p}')


# ---------------------------------------------------------------------------
# Individual plots
# ---------------------------------------------------------------------------

def plot_timeseries(t, values, ylabel, title, phases, stem, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    for start, end, color, label in _phase_segments(t, phases):
        ax.plot(t[start:end + 1], values[start:end + 1],
                color=color, linewidth=1.8, label=label)
    _add_phase_vlines(ax, t, phases)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    _save(fig, out_dir, stem)


def plot_3d(t, x, y, z, phases, stem, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for start, end, color, label in _phase_segments(t, phases):
        xs, ys, zs = x[start:end + 1], y[start:end + 1], z[start:end + 1]
        ax.plot(xs, ys, zs, color=color, linewidth=1.8, label=label)

        # Directional arrows — ~8 evenly spaced per segment
        n = end - start
        step = max(1, n // 8)
        arrow_scale = max(0.2, (x.max() - x.min() + y.max() - y.min()) * 0.025)
        for i in range(0, n - step, step):
            ai = start + i
            bi = start + i + step
            dx = x[bi] - x[ai]
            dy = y[bi] - y[ai]
            dz = z[bi] - z[ai]
            mag = (dx**2 + dy**2 + dz**2) ** 0.5 + 1e-9
            ax.quiver(x[ai], y[ai], z[ai],
                      dx / mag * arrow_scale,
                      dy / mag * arrow_scale,
                      dz / mag * arrow_scale,
                      color=color, arrow_length_ratio=0.4, linewidth=0.7)

    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D Flight Trajectory')
    ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    _save(fig, out_dir, stem)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Plot SITL/real flight path from flight_logger CSV'
    )
    parser.add_argument('csv', help='Path to flight_*.csv produced by flight_logger')
    parser.add_argument('--output-dir', default='graphs',
                        help='Directory for output PNGs (default: graphs/)')
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f'ERROR: {csv_path} not found', file=sys.stderr)
        sys.exit(1)

    rows = load_csv(csv_path)
    if len(rows) < 5:
        print('ERROR: Too few rows to plot', file=sys.stderr)
        sys.exit(1)

    t = np.array([r['t'] for r in rows])
    x = np.array([r['x'] for r in rows])
    y = np.array([r['y'] for r in rows])
    z = np.array([r['z'] for r in rows])

    print(f'Loaded {len(rows)} samples  |  duration={t[-1]:.1f}s  |  max alt={z.max():.2f}m')

    phases = detect_phases(t, x, y, z)
    if phases:
        print('Phases: ' + '  →  '.join(f'{lbl}@{pt:.1f}s' for pt, lbl in phases))
    else:
        print('No altitude phases detected — plotting without phase markers')

    out_dir = Path(args.output_dir)
    ts_str = csv_path.stem.replace('flight_', '')

    print('Generating plots...')
    plot_timeseries(t, x, 'East (m)', 'X Position (East) vs Time',
                    phases, f'flight_x_{ts_str}.png', out_dir)
    plot_timeseries(t, y, 'North (m)', 'Y Position (North) vs Time',
                    phases, f'flight_y_{ts_str}.png', out_dir)
    plot_timeseries(t, z, 'Altitude (m)', 'Altitude vs Time',
                    phases, f'flight_z_{ts_str}.png', out_dir)
    plot_3d(t, x, y, z, phases, f'flight_3d_{ts_str}.png', out_dir)

    print(f'Done. All plots saved to {out_dir}/')


if __name__ == '__main__':
    main()
