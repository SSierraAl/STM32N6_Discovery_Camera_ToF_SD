#!/usr/bin/env python3
"""
VL53L5CX ToF Data Analysis & Report Generator
===============================================
Reads CSV datalog files from vl53l5cx_datalogger.py and generates
professional plots and statistics for engineering reports.

Usage:
  python vl53l5cx_analysis.py              # Auto-detect latest datalog file
  python vl53l5cx_analysis.py <file.csv>   # Analyze specific file

Output:
  - analysis_report_<timestamp>.pdf  (full report with all plots)
  - analysis_report_<timestamp>.html (interactive HTML report)
  - Plots folder with individual PNG files

Requires:
  pip install numpy pandas matplotlib seaborn scipy
"""

import sys
import os
import glob
import warnings
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
import seaborn as sns
from scipy import stats

warnings.filterwarnings('ignore')

# ================================================================
# STYLE CONFIGURATION (Engineering Report Style)
# ================================================================
sns.set_style("whitegrid")
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'sans-serif',
    'axes.titlesize': 12,
    'axes.labelsize': 11,
    'figure.titlesize': 14,
    'figure.figsize': (12, 8),
    'figure.dpi': 150,
    'savefig.dpi': 150,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# ================================================================
# MOTION PARAMETER TUNING (keep in sync with Inc/app_config.h,
# SECTION 9 "ToF Detection (VL53L5CX)")
# ================================================================
# Current firmware settings — marked on plots and highlighted in the sweep
# table. Each variable mirrors exactly one firmware define:
#   MOTION_THRESH      <-> VL53L5CX_DET_MOTION_THRESH
#   MIN_AFFECTED_ZONES <-> VL53L5CX_DET_MIN_AFFECTED_ZONES
#   MOTION_MIN_ZONES   <-> VL53L5CX_DET_MOTION_MIN_ZONES      (sensor plugin, annotation only)
#   PERSIST_FRAMES     <-> VL53L5CX_DET_MOTION_PERSIST_FRAMES (sensor plugin, annotation only)
#   EXTRA_NOISE        <-> VL53L5CX_DET_MOTION_EXTRA_NOISE    (sensor plugin, annotation only)
#
# What the sweep (compute_threshold_sweep) actually replays:
#   triggered_frame = (number of zones with motion >= T) >= M
# using the per-zone motion values already recorded in the CSV. It does NOT
# include the signal channel (VL53L5CX_DET_THRESHOLD_PCT /
# VL53L5CX_DET_MIN_SIGNAL), the fresh-data status gate (5/6/9), the baseline,
# or the sensor plugin's temporal behavior — so PERSIST_FRAMES / EXTRA_NOISE
# / MOTION_MIN_ZONES are annotation-only here: changing them requires a
# firmware edit + a new datalog.
#
# Workflow after the sweep: edit VL53L5CX_DET_MOTION_THRESH and
# VL53L5CX_DET_MIN_AFFECTED_ZONES in Inc/app_config.h (SECTION 9),
# rebuild/flash, re-datalog, and update the values in this block.
MOTION_THRESH      = 60    # VL53L5CX_DET_MOTION_THRESH (4x4 grid)
MIN_AFFECTED_ZONES = 1     # VL53L5CX_DET_MIN_AFFECTED_ZONES (4x4 grid)
MOTION_MIN_ZONES   = 1     # VL53L5CX_DET_MOTION_MIN_ZONES (plugin global flag)
PERSIST_FRAMES     = 16    # VL53L5CX_DET_MOTION_PERSIST_FRAMES (plugin)
EXTRA_NOISE        = 0     # VL53L5CX_DET_MOTION_EXTRA_NOISE (plugin)

# Candidate values swept by the tuning plots/tables.
CANDIDATE_THRESHOLDS = [10, 15, 20, 30, 40, 50, 60, 80, 100, 120, 150]
CANDIDATE_MIN_ZONES  = [1, 2, 3]

# Global CSV columns written by datalogger.py from the MOTION / DETF lines
# (firmware DEBUG MODE 3 / DEBUG MODE 4). Older datalogs don't have them.
GLOBAL_CSV_COLS = ['motion_g1', 'motion_g2', 'motion_status',
                   'motion_nb_det', 'motion_nb_agg',
                   'detected', 'trigger', 'affected_count']

# ================================================================
# AUTO-DETECT LATEST DATALOG FILE
# ================================================================
def find_latest_datalog():
    """Find the most recent datalog CSV file."""
    patterns = [
        'datalog/tof_datalog_*.csv',
        'datalog/*.csv',
    ]
    for pattern in patterns:
        files = glob.glob(pattern)
        if files:
            files.sort(key=os.path.getmtime, reverse=True)
            return files[0]
    return None

# ================================================================
# LOAD DATA
# ================================================================
def load_datalog(filepath):
    """Load CSV datalog and extract zone data."""
    print(f"Loading: {filepath}")
    df = pd.read_csv(filepath)
    print(f"  Rows: {len(df)}, Columns: {len(df.columns)}")

    # Detect resolution from column count
    # Columns: frame, timestamp, temp, [optional motion/detection globals],
    # then N fields per zone
    # Fields (13): sig, dist, base_sig, base_dist, motion, drop, refl,
    #              ambient, sigma, status, spads, targets, valid
    n_global = sum(1 for c in GLOBAL_CSV_COLS if c in df.columns)
    n_data_cols = len(df.columns) - 3 - n_global  # subtract frame, timestamp, temp (+globals)
    # Support multiple formats: 13 fields (current datalogger), 12, 7
    if n_data_cols % 13 == 0:
        fields_per_zone = 13
    elif n_data_cols % 12 == 0:
        fields_per_zone = 12
    elif n_data_cols % 7 == 0:
        fields_per_zone = 7
    else:
        print(f"  ERROR: Cannot determine fields per zone ({n_data_cols} data columns)")
        sys.exit(1)
    zones_per_frame = n_data_cols // fields_per_zone
    print(f"  Detected: {zones_per_frame} zones")

    if zones_per_frame == 64:
        grid_size = 8
    elif zones_per_frame == 16:
        grid_size = 4
    else:
        print(f"  ERROR: Unexpected zone count {zones_per_frame}")
        sys.exit(1)

    return df, zones_per_frame, grid_size

# ================================================================
# EXTRACT PER-ZONE DATA
# ================================================================
def extract_zone_data(df, num_zones):
    """Extract per-zone DataFrames for analysis."""
    zone_data = {}
    for z in range(num_zones):
        zone_data[z] = {
            'sig': df[f'sig{z}'].values.astype(np.float64),
            'dist': df[f'dist{z}'].values.astype(np.float64),
            'base_sig': df[f'base_sig{z}'].values.astype(np.float64),
            'base_dist': df[f'base_dist{z}'].values.astype(np.float64),
            'motion': df[f'motion{z}'].values.astype(np.float64),
            'drop': df[f'drop{z}'].values.astype(np.float64),
            'refl': df[f'refl{z}'].values.astype(np.float64),
        }
        # Additional fields (new format)
        if f'ambient{z}' in df.columns:
            zone_data[z]['ambient'] = df[f'ambient{z}'].values.astype(np.float64)
        if f'sigma{z}' in df.columns:
            zone_data[z]['sigma'] = df[f'sigma{z}'].values.astype(np.float64)
        if f'status{z}' in df.columns:
            zone_data[z]['status'] = df[f'status{z}'].values.astype(np.float64)
        if f'spads{z}' in df.columns:
            zone_data[z]['spads'] = df[f'spads{z}'].values.astype(np.float64)
        if f'targets{z}' in df.columns:
            zone_data[z]['targets'] = df[f'targets{z}'].values.astype(np.float64)
        if f'valid{z}' in df.columns:
            zone_data[z]['valid'] = df[f'valid{z}'].values.astype(np.float64)
    return zone_data

# ================================================================
# COMPUTE STATISTICS
# ================================================================
def compute_zone_stats(zone_data, num_zones):
    """Compute comprehensive statistics for each zone."""
    all_stats = []
    for z in range(num_zones):
        sig = zone_data[z]['sig']
        dist = zone_data[z]['dist']
        drop = zone_data[z]['drop']
        motion = zone_data[z]['motion']
        refl = zone_data[z]['refl']

        # Filter out zeros (invalid readings)
        sig_valid = sig[sig > 0]
        dist_valid = dist[(dist > 0) & (dist < 5000)]

        s = {
            'zone': z,
            'samples': len(sig),
            'sig_valid': len(sig_valid),
            'sig_mean': np.mean(sig_valid) if len(sig_valid) > 0 else 0,
            'sig_std': np.std(sig_valid) if len(sig_valid) > 0 else 0,
            'sig_min': np.min(sig_valid) if len(sig_valid) > 0 else 0,
            'sig_max': np.max(sig_valid) if len(sig_valid) > 0 else 0,
            'sig_median': np.median(sig_valid) if len(sig_valid) > 0 else 0,
            'sig_cv': (np.std(sig_valid) / np.mean(sig_valid) * 100) if len(sig_valid) > 1 and np.mean(sig_valid) > 0 else 0,
            'dist_mean': np.mean(dist_valid) if len(dist_valid) > 0 else 0,
            'dist_std': np.std(dist_valid) if len(dist_valid) > 0 else 0,
            'dist_min': np.min(dist_valid) if len(dist_valid) > 0 else 0,
            'dist_max': np.max(dist_valid) if len(dist_valid) > 0 else 0,
            'dist_median': np.median(dist_valid) if len(dist_valid) > 0 else 0,
            'dist_cv': (np.std(dist_valid) / np.mean(dist_valid) * 100) if len(dist_valid) > 1 and np.mean(dist_valid) > 0 else 0,
            'drop_mean': np.mean(drop),
            'drop_std': np.std(drop),
            'drop_max': np.max(drop),
            'motion_mean': np.mean(motion),
            'motion_std': np.std(motion),
            'motion_max': np.max(motion),
            'motion_p95': np.percentile(motion, 95),
            'motion_p99': np.percentile(motion, 99),
            'motion_above_pct': 100.0 * np.mean(motion >= MOTION_THRESH),
            'refl_mean': np.mean(refl[refl > 0]) if len(refl[refl > 0]) > 0 else 0,
        }
        all_stats.append(s)
    return pd.DataFrame(all_stats)

# ================================================================
# MOTION PARAMETER SWEEP (for tuning)
# ================================================================
def build_motion_matrix(zone_data, num_zones):
    """Build a (frames x zones) matrix of per-zone motion values."""
    max_frame = max(len(zone_data[z]['motion']) for z in range(num_zones))
    M = np.zeros((max_frame, num_zones), dtype=np.float64)
    for z in range(num_zones):
        mz = zone_data[z]['motion']
        M[:len(mz), z] = mz
    return M

def compute_threshold_sweep(motion_matrix):
    """For each candidate (threshold T, min zones M), count the frames that
    would trip the app-level motion trigger:
        triggered = (# zones with motion >= T) >= M
    'events' = contiguous runs of triggered frames (proxy for how many
    separate camera snapshots a setting would cause). Mirrors the firmware
    logic using VL53L5CX_DET_MOTION_THRESH / VL53L5CX_DET_MIN_AFFECTED_ZONES.
    """
    n_frames = motion_matrix.shape[0]
    rows = []
    for T in CANDIDATE_THRESHOLDS:
        flagged = (motion_matrix >= T).sum(axis=1)  # zones above T per frame
        for m in CANDIDATE_MIN_ZONES:
            trig = flagged >= m
            n_trig = int(trig.sum())
            if n_frames > 1:
                events = int((~trig[:-1] & trig[1:]).sum()) + (1 if n_trig > 0 and trig[0] else 0)
            else:
                events = n_trig
            rows.append({
                'T': T, 'M': m,
                'triggered_frames': n_trig,
                'triggered_pct': (100.0 * n_trig / n_frames) if n_frames else 0.0,
                'events': events,
                'max_flagged': int(flagged.max()) if n_frames else 0,
            })
    return pd.DataFrame(rows)

# ================================================================
# PLOT GENERATION
# ================================================================
def create_report_plots(df, zone_data, stats_df, num_zones, grid_size, output_folder,
                        sweep_df=None, motion_matrix=None):
    """Create all plots for the analysis report."""
    os.makedirs(output_folder, exist_ok=True)
    plots = []

    # --- Plot 1: Signal per zone (box plot) ---
    fig = plt.figure(figsize=(14, 6))
    sig_values = [zone_data[z]['sig'][zone_data[z]['sig'] > 0] for z in range(num_zones)]
    bp = plt.boxplot(sig_values, patch_artist=True)
    colors = plt.cm.viridis(np.linspace(0.2, 0.8, num_zones))
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
    plt.title('Signal per SPAD - Distribution by Zone', fontsize=14)
    plt.ylabel('Signal (kcps/spad)')
    plt.xlabel('Zone')
    plt.xticks(range(1, num_zones + 1), [f'Z{i}' for i in range(num_zones)], rotation=45)
    plt.tight_layout()
    fname = os.path.join(output_folder, '01_signal_boxplot.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Signal Box Plot', fname))
    print(f"  [1/8] Signal box plot")

    # --- Plot 2: Distance per zone (box plot) ---
    fig = plt.figure(figsize=(14, 6))
    dist_values = [zone_data[z]['dist'][(zone_data[z]['dist'] > 0) & (zone_data[z]['dist'] < 5000)] for z in range(num_zones)]
    bp = plt.boxplot(dist_values, patch_artist=True)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
    plt.title('Distance - Distribution by Zone', fontsize=14)
    plt.ylabel('Distance (mm)')
    plt.xlabel('Zone')
    plt.xticks(range(1, num_zones + 1), [f'Z{i}' for i in range(num_zones)], rotation=45)
    plt.tight_layout()
    fname = os.path.join(output_folder, '02_distance_boxplot.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Distance Box Plot', fname))
    print(f"  [2/8] Distance box plot")

    # --- Plot 3: Signal heatmap (mean values) ---
    fig, ax = plt.subplots(figsize=(8, 7))
    heatmap = np.zeros((grid_size, grid_size))
    for z in range(num_zones):
        row = z // grid_size
        col = z % grid_size
        heatmap[row, col] = stats_df.iloc[z]['sig_mean']
    im = ax.imshow(heatmap, cmap='viridis', aspect='equal')
    plt.colorbar(im, label='Mean Signal (kcps/spad)')
    for i in range(grid_size):
        for j in range(grid_size):
            val = heatmap[i, j]
            ax.text(j, i, f'Z{i*grid_size+j}\n{val:.0f}', ha='center', va='center',
                    color='white' if val > np.max(heatmap) * 0.5 else 'black', fontsize=8)
    ax.set_title('Mean Signal per Zone')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    fname = os.path.join(output_folder, '03_signal_heatmap.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Signal Heatmap', fname))
    print(f"  [3/8] Signal heatmap")

    # --- Plot 4: Distance heatmap (mean values) ---
    fig, ax = plt.subplots(figsize=(8, 7))
    heatmap = np.zeros((grid_size, grid_size))
    for z in range(num_zones):
        row = z // grid_size
        col = z % grid_size
        heatmap[row, col] = stats_df.iloc[z]['dist_mean']
    im = ax.imshow(heatmap, cmap='plasma', aspect='equal')
    plt.colorbar(im, label='Mean Distance (mm)')
    for i in range(grid_size):
        for j in range(grid_size):
            val = heatmap[i, j]
            ax.text(j, i, f'Z{i*grid_size+j}\n{val:.0f}', ha='center', va='center',
                    color='white' if val > np.max(heatmap) * 0.5 else 'black', fontsize=8)
    ax.set_title('Mean Distance per Zone')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    fname = os.path.join(output_folder, '04_distance_heatmap.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Distance Heatmap', fname))
    print(f"  [4/8] Distance heatmap")

    # --- Plot 5: Signal stability (CV% heatmap) ---
    fig, ax = plt.subplots(figsize=(8, 7))
    heatmap = np.zeros((grid_size, grid_size))
    for z in range(num_zones):
        row = z // grid_size
        col = z % grid_size
        heatmap[row, col] = stats_df.iloc[z]['sig_cv']
    im = ax.imshow(heatmap, cmap='RdYlGn_r', aspect='equal')
    plt.colorbar(im, label='Signal CV (%)')
    for i in range(grid_size):
        for j in range(grid_size):
            val = heatmap[i, j]
            ax.text(j, i, f'{val:.1f}%', ha='center', va='center', fontsize=9)
    ax.set_title('Signal Stability (Coefficient of Variation)')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    fname = os.path.join(output_folder, '05_stability_heatmap.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Stability Heatmap', fname))
    print(f"  [5/9] Stability heatmap")

    # --- Plot 5.5: Motion Indicator heatmap (mean values) ---
    fig, ax = plt.subplots(figsize=(8, 7))
    heatmap = np.zeros((grid_size, grid_size))
    for z in range(num_zones):
        row = z // grid_size
        col = z % grid_size
        heatmap[row, col] = stats_df.iloc[z]['motion_mean']
    im = ax.imshow(heatmap, cmap='hot', aspect='equal')
    plt.colorbar(im, label='Mean Motion Indicator')
    for i in range(grid_size):
        for j in range(grid_size):
            val = heatmap[i, j]
            ax.text(j, i, f'Z{i*grid_size+j}\n{val:.0f}', ha='center', va='center',
                    color='white' if val > np.max(heatmap) * 0.5 else 'black', fontsize=8)
    ax.set_title('Mean Motion Indicator per Zone')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    fname = os.path.join(output_folder, '05b_motion_heatmap.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Motion Heatmap', fname))
    print(f"  [5b/9] Motion heatmap")

    # --- Plot 6: Time series (first 4 zones signal) ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    plot_zones = min(4, num_zones)
    for idx in range(plot_zones):
        ax = axes[idx // 2, idx % 2]
        sig = zone_data[idx]['sig']
        ax.plot(sig[:500], linewidth=0.5, color='steelblue')
        ax.axhline(np.mean(sig[sig > 0]), color='red', linestyle='--', label=f'Mean={np.mean(sig[sig > 0]):.0f}')
        ax.set_title(f'Zone {idx} - Signal Over Time')
        ax.set_ylabel('Signal (kcps/spad)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    plt.suptitle('Signal Time Series (First 4 Zones)', fontsize=14)
    plt.tight_layout()
    fname = os.path.join(output_folder, '06_timeseries_signal.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Time Series', fname))
    print(f"  [6/9] Time series")

    # --- Plot 7: Drop% distribution ---
    fig, ax = plt.subplots(figsize=(10, 6))
    all_drops = []
    for z in range(num_zones):
        all_drops.extend(zone_data[z]['drop'])
    ax.hist(all_drops, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
    ax.axvline(np.mean(all_drops), color='red', linestyle='--', label=f'Mean={np.mean(all_drops):.1f}%')
    ax.axvline(np.median(all_drops), color='orange', linestyle='--', label=f'Median={np.median(all_drops):.1f}%')
    ax.set_xlabel('Signal Drop (%)')
    ax.set_ylabel('Count')
    ax.set_title('Signal Drop Distribution (All Zones)')
    ax.legend()
    plt.tight_layout()
    fname = os.path.join(output_folder, '07_drop_distribution.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Drop Distribution', fname))
    print(f"  [7/9] Drop distribution")

    # --- Plot 8: Bar chart - mean signal per zone ---
    fig, ax = plt.subplots(figsize=(12, 5))
    means = [stats_df.iloc[z]['sig_mean'] for z in range(num_zones)]
    stds = [stats_df.iloc[z]['sig_std'] for z in range(num_zones)]
    x = np.arange(num_zones)
    bars = ax.bar(x, means, yerr=stds, capsize=3, color=colors, edgecolor='black', linewidth=0.5)
    ax.set_xlabel('Zone')
    ax.set_ylabel('Mean Signal (kcps/spad)')
    ax.set_title('Mean Signal per Zone (±1σ)')
    ax.set_xticks(x)
    ax.set_xticklabels([f'Z{i}' for i in range(num_zones)], rotation=45)
    plt.tight_layout()
    fname = os.path.join(output_folder, '08_signal_barchart.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Signal Bar Chart', fname))
    print(f"  [8/9] Signal bar chart")

    # --- Plot 9: Motion time series (first 4 zones) ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    for idx in range(plot_zones):
        ax = axes[idx // 2, idx % 2]
        mot = zone_data[idx]['motion']
        ax.plot(mot[:500], linewidth=0.5, color='darkorange')
        ax.axhline(np.mean(mot), color='red', linestyle='--', label=f'Mean={np.mean(mot):.0f}')
        ax.set_title(f'Zone {idx} - Motion Over Time')
        ax.set_ylabel('Motion Indicator')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    plt.suptitle('Motion Indicator Time Series (First 4 Zones)', fontsize=14)
    plt.tight_layout()
    fname = os.path.join(output_folder, '09_timeseries_motion.png')
    fig.savefig(fname)
    plt.close(fig)
    plots.append(('Motion Time Series', fname))
    print(f"  [9/14] Motion time series")

    # ================================================================
    # MOTION TUNING PLOTS (10-14)
    # ================================================================
    if motion_matrix is None:
        motion_matrix = build_motion_matrix(zone_data, num_zones)
    time_s = (df['frame'].values - df['frame'].values[0]) / 15.0  # approx frame period

    # --- Plot 10: Motion histogram (all zones, log scale) ---
    all_motion = motion_matrix.ravel()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.hist(all_motion, bins=80, color='#FF7F0E', edgecolor='black', alpha=0.8)
    ax.set_yscale('log')
    ax.set_xlabel('Per-zone motion value (plugin, temporally accumulated)')
    ax.set_ylabel('Number of frame-zone samples (log)')
    ax.set_title(f'Motion Histogram - {len(all_motion):,} samples\n'
                 f'(current MOTION_THRESH={MOTION_THRESH})')
    for T in CANDIDATE_THRESHOLDS:
        is_cur = (T == MOTION_THRESH)
        ax.axvline(T, color='red' if is_cur else 'gray',
                   linestyle='--' if is_cur else ':',
                   linewidth=2 if is_cur else 0.8,
                   alpha=1.0 if is_cur else 0.5)
    ax.annotate(f'current={MOTION_THRESH}', xy=(MOTION_THRESH, 1),
                xytext=(MOTION_THRESH + 15, 2),
                color='red', fontweight='bold',
                arrowprops=dict(arrowstyle='->', color='red'))
    plt.tight_layout()
    fname = os.path.join(output_folder, '10_motion_histogram.png')
    fig.savefig(fname); plt.close(fig)
    plots.append(('Motion Histogram (threshold selection)', fname))
    print("  [10/14] Motion histogram")

    # --- Plot 11: Per-frame max zone motion + firmware detection flags ---
    max_motion = motion_matrix.max(axis=1)
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(time_s[:len(max_motion)], max_motion, color='#D62728', linewidth=0.8)
    ax.axhline(MOTION_THRESH, color='red', linestyle='--', linewidth=1.5,
               label=f'MOTION_THRESH={MOTION_THRESH}')
    if 'detected' in df.columns:
        det = df['detected'].values
        ax.scatter(time_s[:len(det)][det == 1], max_motion[:len(det)][det == 1],
                   color='darkred', s=12, zorder=3,
                   label=f"Firmware-flagged frames ({int(det.sum())})")
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Max per-zone motion (frame)')
    ax.set_title('Per-Frame Max Zone Motion vs Firmware Detection Flags')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fname = os.path.join(output_folder, '11_max_motion_flags.png')
    fig.savefig(fname); plt.close(fig)
    plots.append(('Max Zone Motion vs Detection Flags', fname))
    print("  [11/14] Max motion vs flags")

    # --- Plot 12: Zones above candidate thresholds per frame ---
    fig, ax = plt.subplots(figsize=(14, 5))
    for T in (20, 30, 40, 60, 100):
        flagged = (motion_matrix >= T).sum(axis=1)
        is_cur = (T == MOTION_THRESH)
        ax.plot(time_s[:len(flagged)], flagged,
                color='red' if is_cur else 'steelblue',
                linewidth=1.4 if is_cur else 0.8,
                alpha=1.0 if is_cur else 0.7,
                label=f'T={T}')
    ax.axhline(MIN_AFFECTED_ZONES, color='green', linestyle=':', linewidth=1.5,
               label=f'MIN_AFFECTED_ZONES={MIN_AFFECTED_ZONES}')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Zones with motion >= T')
    ax.set_title('Flagged Zones per Frame (red = current threshold)')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fname = os.path.join(output_folder, '12_flagged_zones.png')
    fig.savefig(fname); plt.close(fig)
    plots.append(('Flagged Zones per Frame', fname))
    print("  [12/14] Flagged zones")

    # --- Plot 13: Threshold sweep (would-trigger % and events) ---
    if sweep_df is not None and len(sweep_df) > 0:
        fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharex=True)
        ax = axes[0]
        for m in CANDIDATE_MIN_ZONES:
            sub = sweep_df[sweep_df['M'] == m]
            is_cur = (m == MIN_AFFECTED_ZONES)
            ax.plot(sub['T'], sub['triggered_pct'], marker='o', markersize=4,
                    linewidth=2 if is_cur else 1,
                    label=f'MIN_AFFECTED_ZONES={m}')
        ax.axvline(MOTION_THRESH, color='red', linestyle='--', linewidth=1.5,
                   label=f'current threshold={MOTION_THRESH}')
        ax.set_yscale('log')
        ax.set_xlabel('Motion threshold (MOTION_THRESH candidate)')
        ax.set_ylabel('% of frames that would trigger (log)')
        ax.set_title('Triggered-Frame % vs Threshold')
        ax.legend(); ax.grid(True, alpha=0.3, which='both')

        ax = axes[1]
        for m in CANDIDATE_MIN_ZONES:
            sub = sweep_df[sweep_df['M'] == m]
            is_cur = (m == MIN_AFFECTED_ZONES)
            ax.plot(sub['T'], sub['events'], marker='o', markersize=4,
                    linewidth=2 if is_cur else 1,
                    label=f'MIN_AFFECTED_ZONES={m}')
        ax.axvline(MOTION_THRESH, color='red', linestyle='--', linewidth=1.5)
        ax.set_xlabel('Motion threshold (MOTION_THRESH candidate)')
        ax.set_ylabel('Detection events (contiguous runs)')
        ax.set_title('Event Count vs Threshold (lower = fewer snapshots)')
        ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        fname = os.path.join(output_folder, '13_threshold_sweep.png')
        fig.savefig(fname); plt.close(fig)
        plots.append(('Threshold Sweep (tuning)', fname))
        print("  [13/14] Threshold sweep")
    else:
        print("  [13/14] Skipped: no sweep data")

    # --- Plot 14: Motion plugin globals (MOTION line) ---
    if 'motion_g1' in df.columns and (df['motion_g1'].abs().sum() > 0 or df['motion_g2'].abs().sum() > 0):
        fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
        axes[0].plot(time_s, df['motion_g1'].values, color='#1F77B4', linewidth=0.8,
                     label='global_indicator_1 (g1)')
        axes[0].plot(time_s, df['motion_g2'].values, color='#FF7F0E', linewidth=0.8,
                     label='global_indicator_2 (g2)')
        axes[0].set_ylabel('Global motion indicator (raw)')
        axes[0].set_title(f'Motion Plugin Globals - MIN_ZONES={MOTION_MIN_ZONES}, '
                          f'PERSIST_FRAMES={PERSIST_FRAMES}, EXTRA_NOISE={EXTRA_NOISE}')
        axes[0].legend(); axes[0].grid(True, alpha=0.3)
        axes[1].step(time_s, df['motion_nb_det'].values, where='post',
                     color='#9467BD', label='nb_of_detected_aggregates')
        if 'motion_status' in df.columns:
            st = df['motion_status'].values
            axes[1].scatter(time_s[st == 1], df['motion_nb_det'].values[st == 1],
                            color='red', s=8, zorder=3,
                            label=f"status=1 frames ({int(st.sum())})")
        axes[1].axhline(MOTION_MIN_ZONES, color='green', linestyle=':',
                        label=f'MOTION_MIN_ZONES={MOTION_MIN_ZONES}')
        axes[1].set_xlabel('Time (s)'); axes[1].set_ylabel('Aggregates')
        axes[1].legend(); axes[1].grid(True, alpha=0.3)
        plt.tight_layout()
        fname = os.path.join(output_folder, '14_plugin_globals.png')
        fig.savefig(fname); plt.close(fig)
        plots.append(('Motion Plugin Globals', fname))
        print("  [14/14] Plugin globals")
    else:
        print("  [14/14] Skipped: no MOTION globals in CSV (old datalog format)")

    return plots

# ================================================================
# GENERATE PDF REPORT
# ================================================================
def generate_pdf_report(plots, stats_df, df, num_zones, grid_size, output_folder, filepath,
                        sweep_df=None):
    """Generate a comprehensive PDF report."""
    from matplotlib.backends.backend_pdf import PdfPages

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pdf_path = os.path.join(output_folder, f'analysis_report_{timestamp}.pdf')

    with PdfPages(pdf_path) as pdf:
        # --- Page 1: Title & Summary ---
        fig = plt.figure(figsize=(8.27, 11.69))  # A4
        fig.suptitle('VL53L5CX ToF Sensor - Data Analysis Report', fontsize=16, fontweight='bold', y=0.95)

        gs = gridspec.GridSpec(3, 1, height_ratios=[1, 2, 3])

        # Session info
        ax1 = fig.add_subplot(gs[0])
        ax1.axis('off')
        info_text = f"""
Session Information
  File:          {os.path.basename(filepath)}
  Timestamp:     {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
  Total Frames:  {len(df)}
  Resolution:    {grid_size}x{grid_size} ({num_zones} zones)
  Temperature:   {df['temp'].mean():.1f}°C (avg)
        """
        ax1.text(0.05, 0.5, info_text, fontsize=10, family='monospace', verticalalignment='center')

        # Key metrics
        ax2 = fig.add_subplot(gs[1])
        ax2.axis('off')
        all_sig = stats_df['sig_mean'].values
        all_dist = stats_df['dist_mean'].values
        metrics_text = f"""
Key Metrics (Averaged Across All Zones)
  Signal:        {np.mean(all_sig):.1f} ± {np.std(all_sig):.1f} kcps/spad
  Distance:      {np.mean(all_dist):.1f} ± {np.std(all_dist):.1f} mm
  Signal CV:     {stats_df['sig_cv'].mean():.1f}% (stability)
  Distance CV:   {stats_df['dist_cv'].mean():.1f}% (stability)
  Max Drop:      {stats_df['drop_max'].max():.1f}%
  Avg Drop:      {stats_df['drop_mean'].mean():.1f}%
        """
        ax2.text(0.05, 0.5, metrics_text, fontsize=10, family='monospace', verticalalignment='center')

        # Per-zone table
        ax3 = fig.add_subplot(gs[2])
        ax3.axis('off')
        table_data = []
        for z in range(num_zones):
            row = stats_df.iloc[z]
            table_data.append([
                f'Z{z}', f"{row['sig_mean']:.0f}", f"{row['sig_std']:.0f}",
                f"{row['sig_cv']:.1f}%", f"{row['dist_mean']:.0f}",
                f"{row['dist_std']:.0f}", f"{row['drop_max']:.1f}%"
            ])
        table = ax3.table(cellText=table_data,
                          colLabels=['Zone', 'Sig Mean', 'Sig Std', 'Sig CV%',
                                    'Dist Mean', 'Dist Std', 'Drop Max%'],
                          loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.2)
        # Color header row
        for j in range(7):
            table[(0, j)].set_facecolor('#4472C4')
            table[(0, j)].set_text_props(color='white', fontweight='bold')

        plt.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # --- Motion parameter sweep page ---
        if sweep_df is not None and len(sweep_df) > 0:
            fig = plt.figure(figsize=(8.27, 11.69))  # A4
            fig.suptitle('Motion Parameter Sweep (for tuning)', fontsize=16, fontweight='bold', y=0.97)
            ax = fig.add_subplot(111)
            ax.axis('off')
            sweep_rows = []
            for _, r in sweep_df.iterrows():
                is_cur = (r['T'] == MOTION_THRESH and r['M'] == MIN_AFFECTED_ZONES)
                sweep_rows.append([
                    f"{int(r['T'])}  <== current" if is_cur else str(int(r['T'])),
                    str(int(r['M'])),
                    str(int(r['triggered_frames'])),
                    f"{r['triggered_pct']:.2f}%",
                    str(int(r['events'])),
                    str(int(r['max_flagged'])),
                ])
            sweep_table = ax.table(
                cellText=sweep_rows,
                colLabels=['MOTION_THRESH', 'MIN_AFFECTED_ZONES', 'Triggered frames',
                           'Triggered %', 'Events (runs)', 'Max flagged zones'],
                loc='center', cellLoc='center')
            sweep_table.auto_set_font_size(False)
            sweep_table.set_fontsize(8)
            sweep_table.scale(1, 1.1)
            for j in range(6):
                sweep_table[(0, j)].set_facecolor('#4472C4')
                sweep_table[(0, j)].set_text_props(color='white', fontweight='bold')
            pdf.savefig(fig)
            plt.close(fig)

        # --- Plot pages ---
        for title, fname in plots:
            fig = plt.figure(figsize=(10, 7))
            ax = fig.add_subplot(111)
            ax.imshow(plt.imread(fname))
            ax.axis('off')
            fig.suptitle(title, fontsize=14, fontweight='bold')
            pdf.savefig(fig)
            plt.close(fig)

    print(f"\nPDF Report: {pdf_path}")
    return pdf_path

# ================================================================
# GENERATE HTML REPORT
# ================================================================
def generate_html_report(plots, stats_df, df, num_zones, grid_size, output_folder,
                         sweep_df=None):
    """Generate an interactive HTML report."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    html_path = os.path.join(output_folder, f'analysis_report_{timestamp}.html')

    # Build table rows
    table_rows = ''
    for z in range(num_zones):
        row = stats_df.iloc[z]
        table_rows += f"""<tr>
            <td>Z{z}</td>
            <td>{row['sig_mean']:.1f}</td><td>{row['sig_std']:.1f}</td><td>{row['sig_cv']:.1f}%</td>
            <td>{row['dist_mean']:.1f}</td><td>{row['dist_std']:.1f}</td><td>{row['dist_cv']:.1f}%</td>
            <td>{row['drop_mean']:.1f}%</td><td>{row['drop_max']:.1f}%</td>
            <td>{row['samples']}</td>
        </tr>\n"""

    # Build image section
    images_html = ''
    for title, fname in plots:
        rel_path = fname.replace('\\', '/')
        images_html += f"""
        <div class="plot-section">
            <h3>{title}</h3>
            <img src="{rel_path}" alt="{title}" class="plot-img">
        </div>"""

    # Build motion parameter sweep table
    sweep_rows_html = ''
    if sweep_df is not None and len(sweep_df) > 0:
        sweep_rows_html = """<table>
            <thead><tr>
                <th>MOTION_THRESH (T)</th><th>MIN_AFFECTED_ZONES (M)</th>
                <th>Triggered frames</th><th>Triggered %</th>
                <th>Events (contiguous runs)</th><th>Max flagged zones</th>
            </tr></thead>
            <tbody>
        """
        for _, r in sweep_df.iterrows():
            is_cur = (r['T'] == MOTION_THRESH and r['M'] == MIN_AFFECTED_ZONES)
            cls = ' style="background:#fff3cd;font-weight:bold;"' if is_cur else ''
            tag = ' (current)' if is_cur else ''
            sweep_rows_html += (f"<tr{cls}><td>{int(r['T'])}{tag}</td><td>{int(r['M'])}</td>"
                                f"<td>{int(r['triggered_frames'])}</td><td>{r['triggered_pct']:.2f}%</td>"
                                f"<td>{int(r['events'])}</td><td>{int(r['max_flagged'])}</td></tr>\n")
        sweep_rows_html += """</tbody></table>
        <p>Frames that <em>would</em> trip the app-level motion trigger for each candidate
        (MOTION_THRESH, MIN_AFFECTED_ZONES) setting. Tune so idle-time triggered % is ~0
        while genuine motion still produces events.</p>"""

    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>VL53L5CX ToF Analysis Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }}
        h1 {{ color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        .info-grid {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin: 20px 0; }}
        .info-card {{ background: #ecf0f1; padding: 15px; border-radius: 5px; border-left: 4px solid #3498db; }}
        .info-card h3 {{ margin: 0 0 5px 0; color: #2c3e50; font-size: 14px; }}
        .info-card p {{ margin: 0; font-size: 20px; font-weight: bold; color: #2980b9; }}
        table {{ width: 100%; border-collapse: collapse; margin: 20px 0; font-size: 12px; }}
        th {{ background: #3498db; color: white; padding: 8px; text-align: center; }}
        td {{ padding: 6px; text-align: center; border-bottom: 1px solid #ddd; }}
        tr:hover {{ background: #f0f8ff; }}
        .plot-section {{ margin: 30px 0; text-align: center; }}
        .plot-img {{ max-width: 100%; border: 1px solid #ddd; border-radius: 4px; }}
        .footer {{ margin-top: 40px; padding-top: 20px; border-top: 1px solid #ddd; color: #7f8c8d; font-size: 12px; text-align: center; }}
    </style>
</head>
<body>
<div class="container">
    <h1>VL53L5CX ToF Sensor - Data Analysis Report</h1>

    <div class="info-grid">
        <div class="info-card">
            <h3>Total Frames</h3>
            <p>{len(df):,}</p>
        </div>
        <div class="info-card">
            <h3>Resolution</h3>
            <p>{grid_size}x{grid_size} ({num_zones} zones)</p>
        </div>
        <div class="info-card">
            <h3>Avg Temperature</h3>
            <p>{df['temp'].mean():.1f}°C</p>
        </div>
        <div class="info-card">
            <h3>Avg Signal</h3>
            <p>{stats_df['sig_mean'].mean():.0f} ± {stats_df['sig_std'].mean():.0f}</p>
        </div>
        <div class="info-card">
            <h3>Avg Distance</h3>
            <p>{stats_df['dist_mean'].mean():.0f} ± {stats_df['dist_std'].mean():.0f} mm</p>
        </div>
        <div class="info-card">
            <h3>Signal Stability (CV)</h3>
            <p>{stats_df['sig_cv'].mean():.1f}%</p>
        </div>
    </div>

    <h2>Per-Zone Statistics</h2>
    <table>
        <thead>
            <tr>
                <th>Zone</th>
                <th>Signal Mean</th><th>Signal Std</th><th>Signal CV%</th>
                <th>Dist Mean</th><th>Dist Std</th><th>Dist CV%</th>
                <th>Drop Mean%</th><th>Drop Max%</th>
                <th>Samples</th>
            </tr>
        </thead>
        <tbody>
            {table_rows}
        </tbody>
    </table>

    <h2>Motion Parameter Sweep (for tuning)</h2>
    {sweep_rows_html}

    <h2>Plots & Visualizations</h2>
    {images_html}

    <div class="footer">
        <p>Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | VL53L5CX ToF Data Analysis Tool</p>
    </div>
</div>
</body>
</html>"""

    with open(html_path, 'w') as f:
        f.write(html_content)

    print(f"HTML Report: {html_path}")
    return html_path

# ================================================================
# MAIN
# ================================================================
def main():
    print("=" * 60)
    print("VL53L5CX ToF Data Analysis & Report Generator")
    print("=" * 60)

    # Find input file
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = find_latest_datalog()

    if filepath is None:
        print("ERROR: No datalog file found in 'datalog/' folder.")
        print("Run vl53l5cx_datalogger.py first to collect data.")
        sys.exit(1)

    if not os.path.exists(filepath):
        print(f"ERROR: File not found: {filepath}")
        sys.exit(1)

    # Load data
    df, num_zones, grid_size = load_datalog(filepath)

    # Extract zone data
    print("\nExtracting zone data...")
    zone_data = extract_zone_data(df, num_zones)

    # Compute statistics
    print("Computing statistics...")
    stats_df = compute_zone_stats(zone_data, num_zones)

    # Print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY ({len(df)} frames, {num_zones} zones)")
    print(f"{'='*60}")
    print(f"{'Zone':>5} {'Sig Mean':>10} {'Sig Std':>10} {'Sig CV%':>8} {'Dist Mean':>10} {'Dist Std':>10} {'Drop Max':>10}")
    print("-" * 65)
    for z in range(num_zones):
        row = stats_df.iloc[z]
        print(f"Z{z:>2} {row['sig_mean']:>10.0f} {row['sig_std']:>10.0f} {row['sig_cv']:>7.1f}% {row['dist_mean']:>10.0f} {row['dist_std']:>10.0f} {row['drop_max']:>9.1f}%")

    # Create output folder
    base_name = os.path.splitext(os.path.basename(filepath))[0]
    output_folder = os.path.join(os.path.dirname(filepath) or '.', f'{base_name}_plots')

    # Motion/detection summary (from MOTION / DETF debug lines)
    if 'detected' in df.columns:
        n_detected = int(df['detected'].sum())
        print(f"\n  App-detected frames : {n_detected} ({100.0*n_detected/len(df):.1f}%)")
        print(f"  Trigger sources     : per-zone={int((df['trigger']==1).sum())}, "
              f"plugin-global={int((df['trigger']==2).sum())}")
        aff = df.loc[df['detected'] == 1, 'affected_count']
        if len(aff) > 0:
            print(f"  Affected zones (det): max={int(aff.max())}, mean={aff.mean():.2f}")
    if 'motion_g1' in df.columns:
        print(f"  Plugin g1/g2        : max={int(df['motion_g1'].max())}/{int(df['motion_g2'].max())}")
        print(f"  Plugin status=1     : {int(df['motion_status'].sum())} frames, "
              f"agg max={int(df['motion_nb_det'].max())}")
        print(f"  Firmware settings used for overlays: MOTION_THRESH={MOTION_THRESH}, "
              f"MIN_AFFECTED_ZONES={MIN_AFFECTED_ZONES}, MOTION_MIN_ZONES={MOTION_MIN_ZONES}, "
              f"PERSIST_FRAMES={PERSIST_FRAMES}, EXTRA_NOISE={EXTRA_NOISE}")
        print("  (update the tuning constants at the top of analysis.py if you change firmware)")

    # Motion parameter sweep
    print("\n  Running motion parameter sweep...")
    motion_matrix = build_motion_matrix(zone_data, num_zones)
    sweep_df = compute_threshold_sweep(motion_matrix)
    sweep_out = os.path.join(output_folder, 'motion_threshold_sweep.csv')
    sweep_df.to_csv(sweep_out, index=False)
    print(f"  Sweep table: {sweep_out}")
    cur = sweep_df[(sweep_df['T'] == MOTION_THRESH) & (sweep_df['M'] == MIN_AFFECTED_ZONES)]
    if len(cur) > 0:
        c = cur.iloc[0]
        print(f"  Current setting (T={MOTION_THRESH}, M={MIN_AFFECTED_ZONES}): "
              f"would trigger {c['triggered_pct']:.2f}% of frames -> {int(c['events'])} events")

    # Generate plots
    print(f"\nGenerating plots -> {output_folder}/")
    plots = create_report_plots(df, zone_data, stats_df, num_zones, grid_size, output_folder,
                                sweep_df=sweep_df, motion_matrix=motion_matrix)

    # Generate PDF report
    print("\nGenerating PDF report...")
    pdf_path = generate_pdf_report(plots, stats_df, df, num_zones, grid_size, output_folder, filepath,
                                   sweep_df=sweep_df)

    # Generate HTML report
    print("Generating HTML report...")
    html_path = generate_html_report(plots, stats_df, df, num_zones, grid_size, output_folder,
                                     sweep_df=sweep_df)

    print(f"\n{'='*60}")
    print("DONE!")
    print(f"  PDF:  {pdf_path}")
    print(f"  HTML: {html_path}")
    print(f"  Plots: {output_folder}/")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()