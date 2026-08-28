#!/usr/bin/env python3
"""
VL53L5CX ToF Datalogger
========================
Captures signal, distance, reflectance per zone from serial output.
Computes real-time statistics (mean, std, min, max, median).
Exports to CSV for report generation.

Usage:
  python vl53l5cx_datalogger.py

Features:
  - Records ZFRAME (compact) and ALLPARAM (detailed) data
  - Real-time statistics per zone
  - CSV export (one file per session)
  - Summary report at end of session
  - Configurable resolution (4x4 or 8x8)

Requires:
  pip install pyserial numpy pandas pyqtgraph
"""

import sys
import os
import csv
import time
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from datetime import datetime
from collections import defaultdict, deque

# ================================================================
# CONFIGURATION
# ================================================================
SERIAL_PORT = 'COM7'     # <-- Change to your port
BAUD_RATE   = 115200     # Must match STM32 UART baud rate

# Resolution: must match firmware (VL53L5CX_DET_RESOLUTION)
RESOLUTION  = 4          # 4 = 4x4 (16 zones), 8 = 8x8 (64 zones)

# Data saving
SAVE_DATA = True         # Enable CSV logging
DATA_FOLDER = 'datalog'  # Folder for log files

# Compact ZFRAME format: temp + 5 fields per zone
COMPACT_ZONE_FIELDS = 5  # sig, dist, base_sig, base_dist, motion
F_SIG      = 0
F_DIST     = 1
F_BASE_SIG = 2
F_BASE_DIST = 3
F_MOTION   = 4

# ALLPARAM format: 12 fields per zone
# Fields: sig, base_sig, dist, base_dist, ambient, sigma, reflect, status, spads, targets, drop_pct, valid
ALLPARAM_ZONE_FIELDS = 12

# ALLPARAM field indices
AP_SIG       = 0
AP_BASE_SIG  = 1
AP_DIST      = 2
AP_BASE_DIST = 3
AP_AMBIENT   = 4
AP_SIGMA     = 5
AP_REFLECT   = 6
AP_STATUS    = 7
AP_SPADS     = 8
AP_TARGETS   = 9
AP_DROP_PCT  = 10
AP_VALID     = 11

# Derived
if RESOLUTION == 8:
    NUM_ZONES = 64
    GRID_SIZE = 8
else:
    NUM_ZONES = 16
    GRID_SIZE = 4

# ================================================================
# SERIAL CONNECTION
# ================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
    print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")
except Exception as e:
    print(f"Serial error: {e}")
    sys.exit(1)

serial_buffer = ""

# Shared motion buffer — populated by MOTION line, consumed by ALLPARAM frame
last_motion_data = []

# Motion-plugin globals + per-frame app detection state (MOTION / DETF lines,
# firmware DEBUG MODE 3/4 in vl53l5cx_detection.h)
last_motion_global = {'g1': 0, 'g2': 0, 'status': 0, 'nb_det': 0, 'nb_agg': 0}
last_detf = {'detected': 0, 'trigger': 0, 'affected': 0}
detected_frames_total = 0
motion_status_frames_total = 0
motion_g1_max = 0
motion_g2_max = 0

# ================================================================
# DATA STORAGE
# ================================================================
# Per-zone data accumulators (unlimited - stores all samples)
zone_signal_samples = defaultdict(list)
zone_distance_samples = defaultdict(list)
zone_reflectance_samples = defaultdict(list)
zone_baseline_samples = defaultdict(list)
zone_motion_samples = defaultdict(list)
zone_drop_samples = defaultdict(list)

# Timestamps for each frame
frame_timestamps = []
frame_count = 0

# ================================================================
# CSV FILE SETUP
# ================================================================
def setup_csv():
    """Create CSV file for this session."""
    if not SAVE_DATA:
        return None

    if not os.path.exists(DATA_FOLDER):
        os.makedirs(DATA_FOLDER)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(DATA_FOLDER, f"tof_datalog_{timestamp}.csv")

    # Write header
    header = ['frame', 'timestamp']
    header.append('temp')
    # Motion-plugin globals + per-frame detection state (MOTION / DETF lines).
    # Firmware emits ZFRAME before MOTION/DETF, so these columns carry the
    # previous interval's values — negligible for statistical tuning.
    header.extend(['motion_g1', 'motion_g2', 'motion_status',
                   'motion_nb_det', 'motion_nb_agg',
                   'detected', 'trigger', 'affected_count'])
    for z in range(NUM_ZONES):
        header.extend([
            f'sig{z}', f'dist{z}', f'base_sig{z}', f'base_dist{z}',
            f'motion{z}', f'drop{z}', f'refl{z}',
            f'ambient{z}', f'sigma{z}', f'status{z}', f'spads{z}', f'targets{z}', f'valid{z}'
        ])

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)

    print(f"Data logging to: {filename}")
    return filename

data_file = setup_csv()

# ================================================================
# STATISTICS COMPUTATION
# ================================================================
def compute_stats(values):
    """Compute statistics for a list of values."""
    if not values:
        return {'count': 0, 'mean': 0, 'std': 0, 'min': 0, 'max': 0, 'median': 0}
    arr = np.array(values, dtype=np.float64)
    return {
        'count': len(arr),
        'mean': float(np.mean(arr)),
        'std': float(np.std(arr)),
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
        'median': float(np.median(arr)),
    }

def get_zone_stats(z):
    """Get statistics for a specific zone."""
    return {
        'signal': compute_stats(zone_signal_samples[z]),
        'distance': compute_stats(zone_distance_samples[z]),
        'reflectance': compute_stats(zone_reflectance_samples[z]),
        'baseline': compute_stats(zone_baseline_samples[z]),
        'motion': compute_stats(zone_motion_samples[z]),
        'drop': compute_stats(zone_drop_samples[z]),
    }

# ================================================================
# CSV WRITER
# ================================================================
def save_frame_to_csv(zones, motion, temp):
    """Append a single frame to the CSV file."""
    if not SAVE_DATA or data_file is None:
        return

    try:
        ts = datetime.now().isoformat()
        row = [frame_count, ts, temp,
               last_motion_global['g1'], last_motion_global['g2'],
               last_motion_global['status'], last_motion_global['nb_det'],
               last_motion_global['nb_agg'],
               last_detf['detected'], last_detf['trigger'], last_detf['affected']]

        for z in range(NUM_ZONES):
            row.extend([
                zones[z]['sig'],
                zones[z]['dist'],
                zones[z]['base'],
                zones[z]['bdist'],
                motion[z] if z < len(motion) else 0,
                zones[z]['drop'],
                zones[z]['refl'],
                zones[z].get('ambient', 0),
                zones[z].get('sigma', 0),
                zones[z].get('status', 0),
                zones[z].get('spads', 0),
                zones[z].get('targets', 0),
                zones[z].get('valid', 0)
            ])

        with open(data_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    except Exception as e:
        print(f"Error saving CSV: {e}")

# ================================================================
# SUMMARY REPORT GENERATOR
# ================================================================
def generate_summary_report():
    """Generate a statistics summary report (CSV format)."""
    if not SAVE_DATA or data_file is None:
        return None

    # Derive report filename from data filename
    base = data_file.replace('.csv', '')
    report_file = base + '_summary.csv'

    try:
        with open(report_file, 'w', newline='') as f:
            writer = csv.writer(f)

            # Header
            writer.writerow(['VL53L5CX ToF Sensor - Datalog Summary Report'])
            writer.writerow(['Generated', datetime.now().isoformat()])
            writer.writerow(['Session File', data_file])
            writer.writerow(['Total Frames', frame_count])
            writer.writerow(['Resolution', f'{GRID_SIZE}x{GRID_SIZE} ({NUM_ZONES} zones)'])
            writer.writerow([])

            # Per-zone statistics
            writer.writerow(['ZONE STATISTICS'])
            writer.writerow([
                'Zone',
                'Signal_Mean', 'Signal_Std', 'Signal_Min', 'Signal_Max', 'Signal_Median',
                'Distance_Mean', 'Distance_Std', 'Distance_Min', 'Distance_Max', 'Distance_Median',
                'Reflectance_Mean', 'Reflectance_Std', 'Reflectance_Min', 'Reflectance_Max',
                'Baseline_Mean', 'Baseline_Std',
                'Motion_Mean', 'Motion_Std', 'Motion_Max',
                'Drop_Mean', 'Drop_Std', 'Drop_Max',
                'Samples'
            ])

            for z in range(NUM_ZONES):
                stats = get_zone_stats(z)
                writer.writerow([
                    f'Z{z}',
                    f"{stats['signal']['mean']:.1f}",
                    f"{stats['signal']['std']:.1f}",
                    f"{stats['signal']['min']:.1f}",
                    f"{stats['signal']['max']:.1f}",
                    f"{stats['signal']['median']:.1f}",
                    f"{stats['distance']['mean']:.1f}",
                    f"{stats['distance']['std']:.1f}",
                    f"{stats['distance']['min']:.1f}",
                    f"{stats['distance']['max']:.1f}",
                    f"{stats['distance']['median']:.1f}",
                    f"{stats['reflectance']['mean']:.1f}",
                    f"{stats['reflectance']['std']:.1f}",
                    f"{stats['reflectance']['min']:.1f}",
                    f"{stats['reflectance']['max']:.1f}",
                    f"{stats['baseline']['mean']:.1f}",
                    f"{stats['baseline']['std']:.1f}",
                    f"{stats['motion']['mean']:.1f}",
                    f"{stats['motion']['std']:.1f}",
                    f"{stats['motion']['max']:.1f}",
                    f"{stats['drop']['mean']:.1f}",
                    f"{stats['drop']['std']:.1f}",
                    f"{stats['drop']['max']:.1f}",
                    stats['signal']['count']
                ])

            writer.writerow([])

            # Global statistics (averaged across all zones)
            writer.writerow(['GLOBAL STATISTICS (averaged across all zones)'])
            all_signal = []
            all_distance = []
            all_reflectance = []
            for z in range(NUM_ZONES):
                all_signal.extend(zone_signal_samples[z])
                all_distance.extend(zone_distance_samples[z])
                all_reflectance.extend(zone_reflectance_samples[z])

            global_signal = compute_stats(all_signal) if all_signal else {}
            global_distance = compute_stats(all_distance) if all_distance else {}
            global_reflectance = compute_stats(all_reflectance) if all_reflectance else {}

            writer.writerow(['Parameter', 'Mean', 'Std', 'Min', 'Max', 'Median', 'Total Samples'])
            writer.writerow([
                'Signal (kcps/spad)',
                f"{global_signal.get('mean', 0):.1f}",
                f"{global_signal.get('std', 0):.1f}",
                f"{global_signal.get('min', 0):.1f}",
                f"{global_signal.get('max', 0):.1f}",
                f"{global_signal.get('median', 0):.1f}",
                len(all_signal)
            ])
            writer.writerow([
                'Distance (mm)',
                f"{global_distance.get('mean', 0):.1f}",
                f"{global_distance.get('std', 0):.1f}",
                f"{global_distance.get('min', 0):.1f}",
                f"{global_distance.get('max', 0):.1f}",
                f"{global_distance.get('median', 0):.1f}",
                len(all_distance)
            ])
            writer.writerow([
                'Reflectance (%)',
                f"{global_reflectance.get('mean', 0):.1f}",
                f"{global_reflectance.get('std', 0):.1f}",
                f"{global_reflectance.get('min', 0):.1f}",
                f"{global_reflectance.get('max', 0):.1f}",
                f"{global_reflectance.get('median', 0):.1f}",
                len(all_reflectance)
            ])

            # Motion plugin + app detection stats (MOTION / DETF lines,
            # for tuning MOTION_THRESH / MIN_AFFECTED_ZONES / plugin params)
            writer.writerow([])
            writer.writerow(['MOTION & DETECTION (tuning reference)'])
            writer.writerow(['Metric', 'Value'])
            writer.writerow(['App-detected frames (DETF detected=1)', detected_frames_total])
            writer.writerow(['Plugin-global-motion frames (MOTION status=1)', motion_status_frames_total])
            writer.writerow(['Plugin global indicator g1 max', motion_g1_max])
            writer.writerow(['Plugin global indicator g2 max', motion_g2_max])

        print(f"\nSummary report saved to: {report_file}")
        return report_file

    except Exception as e:
        print(f"Error generating report: {e}")
        return None

# ================================================================
# PARSING FUNCTIONS
# ================================================================
def parse_compact_zframe_line(line):
    """Parse compact ZFRAME: temp + 5 fields per zone."""
    parts = line.split(',')
    actual_zones = (len(parts) - 2) // COMPACT_ZONE_FIELDS
    if actual_zones not in (16, 64):
        return None
    if actual_zones != NUM_ZONES:
        return None
    if len(parts) != 2 + NUM_ZONES * COMPACT_ZONE_FIELDS:
        return None

    try:
        temp = int(parts[1])
    except ValueError:
        return None

    zones = []
    motion = []
    for z in range(actual_zones):
        base_idx = 2 + z * COMPACT_ZONE_FIELDS
        try:
            sig      = int(parts[base_idx + F_SIG])
            dist     = int(parts[base_idx + F_DIST])
            base_sig = int(parts[base_idx + F_BASE_SIG])
            base_dist = int(parts[base_idx + F_BASE_DIST])
            mot      = int(parts[base_idx + F_MOTION])

            drop_pct = 0
            if base_sig > 0:
                drop_pct = abs(base_sig - sig) * 100 // base_sig

            zones.append({
                'sig': sig, 'base': base_sig,
                'dist': dist, 'bdist': base_dist,
                'refl': 0,
                'drop': drop_pct, 'valid': 1 if sig > 0 else 0
            })

            if mot < 0 or mot > 65535:
                mot = 0
            motion.append(mot)
        except (ValueError, IndexError):
            return None

    return {'temp': temp, 'zones': zones, 'motion': motion}


def parse_allparam_line(line):
    """Parse ALLPARAM: temp + 12 fields per zone."""
    parts = line.split(',')
    actual_zones = (len(parts) - 2) // ALLPARAM_ZONE_FIELDS
    if actual_zones not in (16, 64):
        return None
    if actual_zones != NUM_ZONES:
        return None
    if len(parts) != 2 + NUM_ZONES * ALLPARAM_ZONE_FIELDS:
        return None

    try:
        temp = int(parts[1])
    except ValueError:
        return None

    zones = []
    for z in range(NUM_ZONES):
        base_idx = 2 + z * ALLPARAM_ZONE_FIELDS
        try:
            sig       = int(parts[base_idx + 0])
            base_sig  = int(parts[base_idx + 1])
            dist      = int(parts[base_idx + 2])
            base_dist = int(parts[base_idx + 3])
            ambient   = int(parts[base_idx + 4])
            sigma     = int(parts[base_idx + 5])
            reflect   = int(parts[base_idx + 6])
            status    = int(parts[base_idx + 7])
            spads     = int(parts[base_idx + 8])
            targets   = int(parts[base_idx + 9])
            drop_pct  = int(parts[base_idx + 10])
            valid     = int(parts[base_idx + 11])

            zones.append({
                'sig': sig, 'base': base_sig,
                'dist': dist, 'bdist': base_dist,
                'refl': reflect,
                'drop': drop_pct, 'valid': valid,
                'ambient': ambient, 'sigma': sigma,
                'status': status, 'spads': spads, 'targets': targets
            })
        except (ValueError, IndexError):
            return None

    # Use motion from last MOTION line (firmware emits ALLPARAM then MOTION on next line)
    motion = list(last_motion_data) if len(last_motion_data) >= NUM_ZONES else [0] * NUM_ZONES
    return {'temp': temp, 'zones': zones, 'motion': motion}


def parse_motion_line(line):
    """Parse MOTION line: MOTION,g1,g2,status,nb_detected,nb_agg,motion0,...,motionN
    (also handles the EXTMOTION, variant from the dual-sensor external ToF)"""
    global motion_g1_max, motion_g2_max, motion_status_frames_total
    parts = line.split(',')
    # Expected: prefix, g1, g2, status, nb_det, nb_agg, then NUM_ZONES motion values
    if len(parts) < 6 + NUM_ZONES:
        return
    try:
        g1 = int(parts[1])
        g2 = int(parts[2])
        last_motion_global['g1'] = g1
        last_motion_global['g2'] = g2
        last_motion_global['status'] = int(parts[3])
        last_motion_global['nb_det'] = int(parts[4])
        last_motion_global['nb_agg'] = int(parts[5])
        if g1 > motion_g1_max:
            motion_g1_max = g1
        if g2 > motion_g2_max:
            motion_g2_max = g2
        if last_motion_global['status'] == 1:
            motion_status_frames_total += 1
        last_motion_data.clear()
        for i in range(6, 6 + NUM_ZONES):
            m = int(parts[i])
            last_motion_data.append(0 if (m < 0 or m > 65535) else m)
    except (ValueError, IndexError):
        pass


def parse_detf_line(line):
    """Parse DETF line: DETF,detected,trigger_source,affected_count
    (also handles the EXTDETF, variant from the dual-sensor external ToF)"""
    global detected_frames_total
    parts = line.split(',')
    if len(parts) < 4:
        return
    try:
        last_detf['detected'] = int(parts[1])
        last_detf['trigger'] = int(parts[2])
        last_detf['affected'] = int(parts[3])
        if last_detf['detected'] == 1:
            detected_frames_total += 1
    except (ValueError, IndexError):
        pass


# ================================================================
# PYQTGRAPH UI
# ================================================================
app = QtWidgets.QApplication(sys.argv)

win = QtWidgets.QWidget()
win.setWindowTitle(f"VL53L5CX Datalogger - {GRID_SIZE}x{GRID_SIZE} Zones")
win.resize(1400, 900)

main_layout = QtWidgets.QVBoxLayout()
tabs = QtWidgets.QTabWidget()

# --- TAB 1: Signal Heatmap ---
tab1 = pg.GraphicsLayoutWidget(title="Signal per SPAD (Heatmap)")
tab1.resize(1300, 500)
heatmap_signal_plot = tab1.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Signal (kcps/spad)")
heatmap_signal_plot.hideAxis('left')
heatmap_signal_plot.hideAxis('bottom')
heatmap_signal_view = heatmap_signal_plot.getViewBox()
heatmap_signal_view.setAspectLocked(True)
heatmap_signal_img = pg.ImageItem()
heatmap_signal_view.addItem(heatmap_signal_img)
try:
    cmap = pg.colormap.get('viridis')
except Exception:
    cmap = pg.ColorMap([0.0, 1.0], [[0, 0, 0], [255, 255, 255]])
heatmap_signal_img.setColorMap(cmap)
heatmap_signal_img.setLevels([0, 2000])

# Add zone labels
for row in range(GRID_SIZE):
    for col in range(GRID_SIZE):
        z = row * GRID_SIZE + col
        font_size = 9 if GRID_SIZE <= 4 else 7
        text = pg.TextItem(f"Z{z}", color=(200, 200, 200), anchor=(0.5, 0.5))
        text.setPos(col + 0.5, row + 0.5)
        text.setFont(pg.QtGui.QFont('Arial', font_size, pg.QtGui.QFont.Bold))
        heatmap_signal_view.addItem(text)

tabs.addTab(tab1, "Signal Heatmap")

# --- TAB 2: Distance Heatmap ---
tab2 = pg.GraphicsLayoutWidget(title="Distance (Heatmap)")
tab2.resize(1300, 500)
heatmap_dist_plot = tab2.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Distance (mm)")
heatmap_dist_plot.hideAxis('left')
heatmap_dist_plot.hideAxis('bottom')
heatmap_dist_view = heatmap_dist_plot.getViewBox()
heatmap_dist_view.setAspectLocked(True)
heatmap_dist_img = pg.ImageItem()
heatmap_dist_view.addItem(heatmap_dist_img)
heatmap_dist_img.setColorMap(cmap)
heatmap_dist_img.setLevels([0, 200])

for row in range(GRID_SIZE):
    for col in range(GRID_SIZE):
        z = row * GRID_SIZE + col
        font_size = 9 if GRID_SIZE <= 4 else 7
        text = pg.TextItem(f"Z{z}", color=(200, 200, 200), anchor=(0.5, 0.5))
        text.setPos(col + 0.5, row + 0.5)
        text.setFont(pg.QtGui.QFont('Arial', font_size, pg.QtGui.QFont.Bold))
        heatmap_dist_view.addItem(text)

tabs.addTab(tab2, "Distance Heatmap")

# --- TAB 2.5: Motion Indicator Heatmap ---
tab25 = pg.GraphicsLayoutWidget(title="Motion Indicator (Heatmap)")
tab25.resize(1300, 500)
heatmap_motion_plot = tab25.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Motion Indicator")
heatmap_motion_plot.hideAxis('left')
heatmap_motion_plot.hideAxis('bottom')
heatmap_motion_view = heatmap_motion_plot.getViewBox()
heatmap_motion_view.setAspectLocked(True)
heatmap_motion_img = pg.ImageItem()
heatmap_motion_view.addItem(heatmap_motion_img)
try:
    motion_cmap = pg.colormap.get('hot')
except Exception:
    motion_cmap = pg.ColorMap([0.0, 0.5, 1.0], [[0, 0, 0], [255, 100, 0], [255, 255, 0]])
heatmap_motion_img.setColorMap(motion_cmap)
heatmap_motion_img.setLevels([0, 100])

for row in range(GRID_SIZE):
    for col in range(GRID_SIZE):
        z = row * GRID_SIZE + col
        font_size = 9 if GRID_SIZE <= 4 else 7
        text = pg.TextItem(f"Z{z}", color=(200, 200, 200), anchor=(0.5, 0.5))
        text.setPos(col + 0.5, row + 0.5)
        text.setFont(pg.QtGui.QFont('Arial', font_size, pg.QtGui.QFont.Bold))
        heatmap_motion_view.addItem(text)

tabs.addTab(tab25, "Motion Heatmap")

# --- TAB 3: Statistics Table ---
tab3 = QtWidgets.QWidget()
tab3_layout = QtWidgets.QVBoxLayout()
tab3_layout.addWidget(QtWidgets.QLabel("Real-Time Zone Statistics (updated every 50 frames)"))

stats_table = QtWidgets.QTableWidget()
stats_table.setColumnCount(10)
stats_table.setHorizontalHeaderLabels([
    'Zone', 'Signal Mean', 'Signal Std', 'Distance Mean', 'Distance Std',
    'Reflect Mean', 'Reflect Std', 'Drop Mean', 'Drop Max', 'Samples'
])
stats_table.setRowCount(NUM_ZONES)
for z in range(NUM_ZONES):
    item = QtWidgets.QTableWidgetItem(f'Z{z}')
    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
    stats_table.setItem(z, 0, item)
    for c in range(1, 10):
        item = QtWidgets.QTableWidgetItem('--')
        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
        stats_table.setItem(z, c, item)
stats_table.setEditTriggers(QtWidgets.QTableWidget.NoEditTriggers)
stats_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
tab3_layout.addWidget(stats_table)

tabs.addTab(tab3, "Statistics Table")

# --- TAB 4: Session Info ---
tab4 = QtWidgets.QWidget()
tab4_layout = QtWidgets.QVBoxLayout()

info_label = QtWidgets.QLabel()
info_label.setTextFormat(QtCore.Qt.RichText)
info_label.setWordWrap(True)
info_label.setStyleSheet("font-family: monospace; font-size: 12pt;")
tab4_layout.addWidget(info_label)

# Buttons
btn_layout = QtWidgets.QHBoxLayout()

export_btn = QtWidgets.QPushButton("Export Summary Report")
export_btn.setStyleSheet("font-size: 14pt; padding: 10px;")
btn_layout.addWidget(export_btn)

stop_btn = QtWidgets.QPushButton("Stop Logging")
stop_btn.setStyleSheet("font-size: 14pt; padding: 10px; background-color: #ff4444; color: white;")
btn_layout.addWidget(stop_btn)

tab4_layout.addLayout(btn_layout)
tabs.addTab(tab4, "Session Control")

main_layout.addWidget(tabs)
win.setLayout(main_layout)
win.show()

# ================================================================
# UPDATE FUNCTION
# ================================================================
last_stats_update = 0
logging_active = True

def update_data():
    global frame_count, serial_buffer, last_stats_update, logging_active

    if not logging_active:
        return

    try:
        while ser.in_waiting > 0:
            raw = ser.read(ser.in_waiting)
            serial_buffer += raw.decode('utf-8', errors='ignore')

        while '\n' in serial_buffer:
            line, serial_buffer = serial_buffer.split('\n', 1)
            chunk = line.strip().rstrip('\r')
            if not chunk:
                continue

            # Parse MOTION line separately — it comes after ALLPARAM
            if chunk.startswith("MOTION,") or chunk.startswith("EXTMOTION,"):
                parse_motion_line(chunk)
                continue

            # Parse DETF line — per-frame app detection state (DEBUG MODE 4)
            if chunk.startswith("DETF,") or chunk.startswith("EXTDETF,"):
                parse_detf_line(chunk)
                continue

            data = None

            if chunk.startswith("ZFRAME"):
                data = parse_compact_zframe_line(chunk)
            elif chunk.startswith("ALLPARAM,"):
                data = parse_allparam_line(chunk)

            if data is None:
                continue

            frame_count += 1

            # Debug: print first few frames to console
            if frame_count <= 3:
                print(f"\n[Frame {frame_count}] Parsed: temp={data['temp']}, zones={len(data['zones'])}")
                print(f"  Z0: sig={data['zones'][0]['sig']}, dist={data['zones'][0]['dist']}, base={data['zones'][0]['base']}, bdist={data['zones'][0]['bdist']}, refl={data['zones'][0]['refl']}")
                print(f"  Motion[0:4]={data.get('motion', [0])[:4]}")
            zones = data['zones']
            motion = data.get('motion', [0] * NUM_ZONES)
            temp = data.get('temp', 0)

            # Accumulate per-zone samples
            for z in range(NUM_ZONES):
                zone_signal_samples[z].append(zones[z]['sig'])
                zone_distance_samples[z].append(zones[z]['dist'])
                zone_reflectance_samples[z].append(zones[z]['refl'])
                zone_baseline_samples[z].append(zones[z]['base'])
                zone_motion_samples[z].append(motion[z] if z < len(motion) else 0)
                zone_drop_samples[z].append(zones[z]['drop'])

            # Save to CSV
            save_frame_to_csv(zones, motion, temp)

            # Update heatmaps
            signal_map = np.zeros((GRID_SIZE, GRID_SIZE))
            dist_map = np.zeros((GRID_SIZE, GRID_SIZE))
            for z in range(NUM_ZONES):
                row = z // GRID_SIZE
                col = z % GRID_SIZE
                signal_map[row, col] = zones[z]['sig']
                dist_map[row, col] = zones[z]['dist']

            # No rotation: zone 0 is top-left
            # signal_map = np.rot90(signal_map, k=-1)
            # dist_map = np.rot90(dist_map, k=-1)

            heatmap_signal_img.setImage(signal_map, autoLevels=False)
            max_sig = np.max(signal_map)
            if max_sig > 0:
                heatmap_signal_img.setLevels([0, max_sig * 1.2])

            heatmap_dist_img.setImage(dist_map, autoLevels=False)
            max_dist = np.max(dist_map)
            if max_dist > 0:
                heatmap_dist_img.setLevels([0, max_dist * 1.2])

            # Update motion heatmap
            motion_map = np.zeros((GRID_SIZE, GRID_SIZE))
            for z in range(NUM_ZONES):
                row = z // GRID_SIZE
                col = z % GRID_SIZE
                motion_map[row, col] = motion[z] if z < len(motion) else 0
            heatmap_motion_img.setImage(motion_map, autoLevels=False)
            max_motion = np.max(motion_map)
            if max_motion > 0:
                heatmap_motion_img.setLevels([0, max_motion * 1.2])

            # Update statistics table every 50 frames
            if frame_count - last_stats_update >= 50:
                last_stats_update = frame_count
                print(f"\n[Stats Update] Frame {frame_count}, updating table...")

                for z in range(NUM_ZONES):
                    zstats = get_zone_stats(z)
                    row_items = [
                        f"{zstats['signal']['mean']:.0f}",
                        f"{zstats['signal']['std']:.0f}",
                        f"{zstats['distance']['mean']:.0f}",
                        f"{zstats['distance']['std']:.0f}",
                        f"{zstats['reflectance']['mean']:.0f}",
                        f"{zstats['reflectance']['std']:.0f}",
                        f"{zstats['drop']['mean']:.1f}",
                        f"{zstats['drop']['max']:.1f}",
                        f"{zstats['signal']['count']}"
                    ]
                    for c, val in enumerate(row_items):
                        stats_table.setItem(z, c + 1, QtWidgets.QTableWidgetItem(val))

                # Update session info (use zone 0 stats as reference)
                ref_stats = get_zone_stats(0)
                info_label.setText(f"""
Session Information
  Frames Logged:    {frame_count}
  Zones:            {NUM_ZONES} ({GRID_SIZE}x{GRID_SIZE})
  Temp:             {temp}°C
  CSV File:         {data_file or 'N/A'}
  Samples per zone: {ref_stats['signal']['count']}
  Motion g1/g2:     {last_motion_global['g1']} / {last_motion_global['g2']} (status={last_motion_global['status']})
  App detected:     {detected_frames_total} frames | Plugin status: {motion_status_frames_total} frames

  Press 'Export Summary Report' to generate statistics CSV.
  Press 'Stop Logging' to finalize and exit.
                """)
                print(f"[Stats] Z0: mean_sig={ref_stats['signal']['mean']:.0f}, std={ref_stats['signal']['std']:.0f}, samples={ref_stats['signal']['count']}")

    except Exception as e:
        print(f"Error: {e}")

def on_export():
    """Export summary report."""
    generate_summary_report()
    samples = len(zone_signal_samples[0]) if zone_signal_samples[0] else 0
    info_label.setText(f"Report Exported! Frames: {frame_count} | Samples/zone: {samples}")

def on_stop():
    """Stop logging and generate final report."""
    global logging_active
    logging_active = False

    # Generate final summary
    report = generate_summary_report()

    info_label.setText(f"""
Logging Stopped
  Total Frames:   {frame_count}
  Data File:      {data_file or 'N/A'}
  Summary Report: {report or 'N/A'}

  Application will close in 3 seconds...
    """)

    # Close after delay
    QtCore.QTimer.singleShot(3000, lambda: app.quit())

export_btn.clicked.connect(on_export)
stop_btn.clicked.connect(on_stop)

# Keyboard shortcut: 'S' to stop, 'E' to export
class KeyFilter(QtCore.QObject):
    def eventFilter(self, obj, event):
        if event.type() == event.Type.KeyPress:
            if event.key() == QtCore.Qt.Key_S:
                on_stop()
            elif event.key() == QtCore.Qt.Key_E:
                on_export()
        return super().eventFilter(obj, event)

key_filter = KeyFilter()
win.installEventFilter(key_filter)
win.setFocusPolicy(QtCore.Qt.StrongFocus)

def close_app():
    # Always generate report on exit
    generate_summary_report()
    if ser.is_open:
        ser.close()
    print(f"\nSession complete. Frames: {frame_count}")

app.aboutToQuit.connect(close_app)

timer = QtCore.QTimer()
timer.timeout.connect(update_data)
timer.start(30)  # ~33Hz update rate

print("=" * 60)
print(f"VL53L5CX Datalogger - {GRID_SIZE}x{GRID_SIZE} Zones")
print("=" * 60)
print(f"Port: {SERIAL_PORT} | Baud: {BAUD_RATE}")
print(f"Resolution: {GRID_SIZE}x{GRID_SIZE} ({NUM_ZONES} zones)")
print(f"Logging: {'ENABLED' if SAVE_DATA else 'DISABLED'}")
if SAVE_DATA:
    print(f"Data file: {data_file}")
print("")
print("Keys: 'E' = Export Report | 'S' = Stop & Exit")
print("=" * 60)

if __name__ == '__main__':
    sys.exit(app.exec_())