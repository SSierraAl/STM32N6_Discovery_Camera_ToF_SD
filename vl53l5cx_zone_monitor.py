#!/usr/bin/env python3
"""
VL53L5CX 4x4 Zone Monitor - Real-time signal visualization with ALL parameters
===============================================================================
Parses extended ZFRAME, ALLPARAM, and MOTION debug output from STM32 firmware.

EXTENDED ZFRAME format (from vl53l5cx_detection.c):
  ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...,motion0,...,motion15

  Per zone (12 fields each):
    - sigN    = current signal_per_spad (kcps/spad)
    - baseN   = baseline signal_per_spad
    - distN   = current distance_mm
    - bdistN  = baseline distance_mm
    - ambN    = ambient_per_spad (kcps/spad)
    - sigmaN  = range_sigma_mm (mm)
    - reflN   = reflectance (%)
    - statusN = target_status (0/5/9 = valid)
    - spadsN  = nb_spads_enabled
    - targsN  = nb_target_detected
    - dropN   = signal drop percentage (%)
    - validN  = zone valid flag (0 or 1)

  Global:
    - temp    = silicon temperature (°C)
    - motion0..motion15 = motion indicator per zone

ALLPARAM format (detailed, emitted every 10 frames):
  ALLPARAM,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...

MOTION format:
  MOTION,global1,global2,status,nb_detected,nb_agg,motion0,...,motion15

Usage:
  python vl53l5cx_zone_monitor.py

Requires:
  pip install pyserial pyqtgraph numpy
"""

import sys
import os
import csv
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from datetime import datetime
from collections import deque

# ================================================================
# CONFIGURATION
# ================================================================
SERIAL_PORT = 'COM6'   # <-- Change to your port
BAUD_RATE   = 115200    # Must match STM32 UART baud rate
MAX_POINTS  = 150       # History length on plots

# Resolution: set manually to match firmware (VL53L5CX_DET_RESOLUTION in header)
#   4 = 4x4 (16 zones), 8 = 8x8 (64 zones)
RESOLUTION  = 8         # MUST match firmware setting!

# Data saving configuration
SAVE_DATA = True        # Enable/disable data saving to CSV
DATA_FOLDER = 'zone_data'  # Folder to store data files

# Field count per zone in extended ZFRAME
ZONE_FIELDS = 12  # sig, base, dist, bdist, amb, sigma, refl, status, spads, targs, drop, valid

# Derived from RESOLUTION (do not change manually)
if RESOLUTION == 8:
    NUM_ZONES = 64
    GRID_SIZE = 8
else:
    NUM_ZONES = 16
    GRID_SIZE = 4

# Zone field indices
F_SIG    = 0  # signal_per_spad
F_BASE   = 1  # baseline signal_per_spad
F_DIST   = 2  # distance_mm
F_BDIST  = 3  # baseline distance_mm
F_AMB    = 4  # ambient_per_spad
F_SIGMA  = 5  # range_sigma_mm
F_REFL   = 6  # reflectance %
F_STATUS = 7  # target_status
F_SPADS  = 8  # nb_spads_enabled
F_TARGS  = 9  # nb_target_detected
F_DROP   = 10 # signal drop %
F_VALID  = 11 # zone valid flag

# Color map for zones (viridis palette, extended for 8x8 = 64 zones)
ZONE_COLORS_16 = [
    (68, 1, 84), (72, 35, 116), (64, 67, 135), (52, 94, 141),
    (41, 120, 142), (32, 144, 140), (34, 167, 132), (68, 190, 112),
    (121, 209, 81), (189, 222, 38), (253, 231, 37), (253, 231, 37),
    (253, 231, 37), (253, 231, 37), (253, 231, 37), (253, 231, 37),
]

# Generate 64 colors for 8x8 mode (interpolate viridis)
import colorsys
def generate_viridis_colors(n):
    colors = []
    for i in range(n):
        t = i / max(n - 1, 1)
        # Approximate viridis: blue → purple → teal → green → yellow
        if t < 0.25:
            r, g, b = 68 + t*400, 1 + t*120, 84 + t*200
        elif t < 0.5:
            r, g, b = 168 - t*200, 31 + t*500, 234 - t*300
        elif t < 0.75:
            r, g, b = 68 + t*300, 181 + t*200, 134 - t*100
        else:
            r, g, b = 253, 231 - t*100, 37
        colors.append((int(r), int(g), int(b)))
    return colors

ZONE_COLORS = ZONE_COLORS_16 if NUM_ZONES <= 16 else generate_viridis_colors(64)

# ================================================================
# DATA SAVING SETUP
# ================================================================
def setup_data_saving():
    """Create data folder and return filename for today's session."""
    if not SAVE_DATA:
        return None
    
    if not os.path.exists(DATA_FOLDER):
        os.makedirs(DATA_FOLDER)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(DATA_FOLDER, f"zone_data_{timestamp}.csv")
    
    # Write CSV header
    header = ['timestamp', 'frame']
    for z in range(NUM_ZONES):
        header.extend([
            f'sig{z}', f'base{z}', f'dist{z}', f'bdist{z}',
            f'amb{z}', f'sigma{z}', f'refl{z}', f'status{z}',
            f'spads{z}', f'targs{z}', f'drop{z}', f'valid{z}',
            f'motion{z}'
        ])
    header.extend(['temp', 'total_detections'])
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
    
    print(f"Data will be saved to: {filename}")
    return filename


# ================================================================
# SERIAL CONNECTION
# ================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")
except Exception as e:
    print(f"Serial error: {e}")
    sys.exit(1)

# Initialize data saving
data_file = setup_data_saving()

# ================================================================
# PYQTGRAPH SETUP
# ================================================================
app = QtWidgets.QApplication(sys.argv)

# Create main window with tabbed interface
win = QtWidgets.QWidget()
win.setWindowTitle(f"VL53L5CX {GRID_SIZE}x{GRID_SIZE} Zone Monitor - All Parameters")
win.resize(1600, 1000)

main_layout = QtWidgets.QVBoxLayout()
tabs = QtWidgets.QTabWidget()

# ================================================================
# TAB 1: Signal & Distance (Original Views)
# ================================================================
tab1 = pg.GraphicsLayoutWidget(title="Signal & Distance")
tab1.resize(1400, 600)

# --- Plot 1: Signal per SPAD ---
plot_signal = tab1.addPlot(title="Signal per SPAD (Current vs Baseline)")
plot_signal.setYRange(0, 1000)
plot_signal.setLabel('left', 'Signal', units='kcps/spad')
plot_signal.setLabel('bottom', 'Samples')
plot_signal.showGrid(x=True, y=True, alpha=0.3)
plot_signal.addLegend(offset=(10, 10))

# --- Plot 2: Distance ---
plot_distance = tab1.addPlot(title="Distance (Current vs Baseline)")
plot_distance.setYRange(0, 200)
plot_distance.setLabel('left', 'Distance', units='mm')
plot_distance.setLabel('bottom', 'Samples')
plot_distance.showGrid(x=True, y=True, alpha=0.3)
plot_distance.addLegend(offset=(10, 10))

tabs.addTab(tab1, "Signal & Distance")

# ================================================================
# TAB 2: Detection Metrics
# ================================================================
tab2 = pg.GraphicsLayoutWidget(title="Detection Metrics")
tab2.resize(1400, 600)

# --- Plot 3: Signal Drop Percentage ---
plot_drop = tab2.addPlot(title="Signal Drop % (Detection Metric)")
plot_drop.setYRange(-5, 30)
plot_drop.setLabel('left', 'Drop', units='%')
plot_drop.setLabel('bottom', 'Samples')
plot_drop.showGrid(x=True, y=True, alpha=0.3)
plot_drop.addLegend(offset=(10, 10))

threshold_line = pg.InfiniteLine(
    pos=6,
    angle=0,
    pen=pg.mkPen(color='r', width=1.5, style=QtCore.Qt.DashLine),
    label='Threshold (6%)',
    labelOpts={'color': 'r', 'position': 0.95}
)
plot_drop.addItem(threshold_line, ignoreBounds=True)

# --- Plot 4: Ambient Noise ---
plot_ambient = tab2.addPlot(title="Ambient Noise per SPAD")
plot_ambient.setYRange(0, 500)
plot_ambient.setLabel('left', 'Ambient', units='kcps/spad')
plot_ambient.setLabel('bottom', 'Samples')
plot_ambient.showGrid(x=True, y=True, alpha=0.3)
plot_ambient.addLegend(offset=(10, 10))

# --- Plot 5: Range Sigma ---
plot_sigma = tab2.addPlot(title="Range Sigma (Measurement Uncertainty)")
plot_sigma.setYRange(0, 100)
plot_sigma.setLabel('left', 'Sigma', units='mm')
plot_sigma.setLabel('bottom', 'Samples')
plot_sigma.showGrid(x=True, y=True, alpha=0.3)
plot_sigma.addLegend(offset=(10, 10))

tabs.addTab(tab2, "Detection Metrics")

# ================================================================
# TAB 3: Advanced Parameters
# ================================================================
tab3 = pg.GraphicsLayoutWidget(title="Advanced Parameters")
tab3.resize(1400, 600)

# --- Plot 6: Reflectance ---
plot_reflect = tab3.addPlot(title="Reflectance (%)")
plot_reflect.setYRange(0, 100)
plot_reflect.setLabel('left', 'Reflectance', units='%')
plot_reflect.setLabel('bottom', 'Samples')
plot_reflect.showGrid(x=True, y=True, alpha=0.3)
plot_reflect.addLegend(offset=(10, 10))

# --- Plot 7: SPADs Enabled ---
plot_spads = tab3.addPlot(title="SPADs Enabled")
plot_spads.setYRange(0, 5000)
plot_spads.setLabel('left', 'SPADs', units='count')
plot_spads.setLabel('bottom', 'Samples')
plot_spads.showGrid(x=True, y=True, alpha=0.3)
plot_spads.addLegend(offset=(10, 10))

# --- Plot 8: Motion Indicator ---
plot_motion = tab3.addPlot(title="Motion Indicator per Zone")
plot_motion.setYRange(0, 1000)
plot_motion.setLabel('left', 'Motion', units='power')
plot_motion.setLabel('bottom', 'Samples')
plot_motion.showGrid(x=True, y=True, alpha=0.3)
plot_motion.addLegend(offset=(10, 10))

motion_threshold_line = pg.InfiniteLine(
    pos=20,
    angle=0,
    pen=pg.mkPen(color='orange', width=1.5, style=QtCore.Qt.DashLine),
    label='Motion Threshold (20)',
    labelOpts={'color': 'orange', 'position': 0.95}
)
plot_motion.addItem(motion_threshold_line, ignoreBounds=True)

tabs.addTab(tab3, "Advanced")

# ================================================================
# TAB 4: Heatmaps & Status
# ================================================================
tab4 = pg.GraphicsLayoutWidget(title="Heatmaps & Status")
tab4.resize(1400, 600)

# --- Heatmap 1: Signal Drop ---
heatmap_drop_plot = tab4.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Zone Heatmap (Signal Drop %)")
heatmap_drop_plot.hideAxis('left')
heatmap_drop_plot.hideAxis('bottom')
heatmap_drop_view = heatmap_drop_plot.getViewBox()
heatmap_drop_view.setAspectLocked(True)
heatmap_drop_img = pg.ImageItem()
heatmap_drop_view.addItem(heatmap_drop_img)

# --- Heatmap 2: Distance ---
heatmap_dist_plot = tab4.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Zone Heatmap (Distance mm)")
heatmap_dist_plot.hideAxis('left')
heatmap_dist_plot.hideAxis('bottom')
heatmap_dist_view = heatmap_dist_plot.getViewBox()
heatmap_dist_view.setAspectLocked(True)
heatmap_dist_img = pg.ImageItem()
heatmap_dist_view.addItem(heatmap_dist_img)

# --- Heatmap 3: Motion ---
heatmap_motion_plot = tab4.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Zone Heatmap (Motion)")
heatmap_motion_plot.hideAxis('left')
heatmap_motion_plot.hideAxis('bottom')
heatmap_motion_view = heatmap_motion_plot.getViewBox()
heatmap_motion_view.setAspectLocked(True)
heatmap_motion_img = pg.ImageItem()
heatmap_motion_view.addItem(heatmap_motion_img)

# --- Heatmap 4: Reflectance ---
tab4.nextRow()
heatmap_reflect_plot = tab4.addPlot(title=f"{GRID_SIZE}x{GRID_SIZE} Zone Heatmap (Reflectance %)")
heatmap_reflect_plot.hideAxis('left')
heatmap_reflect_plot.hideAxis('bottom')
heatmap_reflect_view = heatmap_reflect_plot.getViewBox()
heatmap_reflect_view.setAspectLocked(True)
heatmap_reflect_img = pg.ImageItem()
heatmap_reflect_view.addItem(heatmap_reflect_img)

# Set colormaps
try:
    colormap = pg.colormap.get('viridis')
except Exception:
    colormap = pg.ColorMap([0.0, 1.0], [[0, 0, 0], [255, 255, 255]])

heatmap_drop_img.setColorMap(colormap)
heatmap_drop_img.setLevels([0, 30])
heatmap_dist_img.setColorMap(colormap)
heatmap_dist_img.setLevels([0, 200])
heatmap_motion_img.setColorMap(colormap)
heatmap_motion_img.setLevels([0, 500])
heatmap_reflect_img.setColorMap(colormap)
heatmap_reflect_img.setLevels([0, 100])

# Add zone labels to drop heatmap
for row in range(GRID_SIZE):
    for col in range(GRID_SIZE):
        z = row * GRID_SIZE + col
        font_size = 9 if GRID_SIZE <= 4 else 7
        text = pg.TextItem(f"Z{z}", color=(200, 200, 200), anchor=(0.5, 0.5))
        text.setPos(col + 0.5, row + 0.5)
        text.setFont(pg.QtGui.QFont('Arial', font_size, pg.QtGui.QFont.Bold))
        heatmap_drop_view.addItem(text)

# --- Temperature Display ---
temp_plot = tab4.addPlot()
temp_plot.setMaximumHeight(60)
temp_plot.hideAxis('left')
temp_plot.hideAxis('bottom')
temp_text = pg.TextItem("Temp: --°C", color='#FFFF00', anchor=(0.5, 0.5))
temp_text.setFont(pg.QtGui.QFont('Arial', 14, pg.QtGui.QFont.Bold))
temp_plot.addItem(temp_text, ignoreBounds=True)
temp_text.setPos(0.5, 0.5)

# --- Status Panel ---
status_plot = tab4.addPlot()
status_plot.setMaximumHeight(60)
status_plot.hideAxis('left')
status_plot.hideAxis('bottom')
status_text = pg.TextItem("WAITING FOR DATA...", color='#888888', anchor=(0.5, 0.5))
status_text.setFont(pg.QtGui.QFont('Arial', 14, pg.QtGui.QFont.Bold))
status_plot.addItem(status_text, ignoreBounds=True)
status_text.setPos(0.5, 0.5)

tabs.addTab(tab4, "Heatmaps")

# --- Add tabs to main layout ---
main_layout.addWidget(tabs)
win.setLayout(main_layout)
win.show()

# ================================================================
# DATA STORAGE - All parameters per zone
# ================================================================
# Signal data
zone_signal_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_baseline_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

# Distance data
zone_distance_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_bdistance_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

# Detection metrics
zone_drop_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_ambient_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_sigma_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

# Advanced params
zone_reflect_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_spads_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_status_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_targets_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_valid_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

# Motion data
zone_motion_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

# Global data
temp_history = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
motion_global1_history = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
motion_global2_history = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

# ================================================================
# PLOT CURVES
# ================================================================
signal_curves = {}
signal_baseline_curves = {}
distance_curves = {}
distance_baseline_curves = {}
drop_curves = {}
ambient_curves = {}
sigma_curves = {}
reflect_curves = {}
spads_curves = {}
motion_curves = {}

for z in range(NUM_ZONES):
    color = ZONE_COLORS[z % len(ZONE_COLORS)]
    pen_main = pg.mkPen(color=color, width=2 if z < GRID_SIZE else 1, style=QtCore.Qt.SolidLine)
    pen_base = pg.mkPen(color=color, width=1, style=QtCore.Qt.DotLine)
    label = f'Z{z}' if z < GRID_SIZE else None

    # Tab 1: Signal & Distance
    signal_curves[z] = plot_signal.plot(name=label, pen=pen_main)
    signal_baseline_curves[z] = plot_signal.plot(pen=pen_base)
    distance_curves[z] = plot_distance.plot(name=label, pen=pen_main)
    distance_baseline_curves[z] = plot_distance.plot(pen=pen_base)

    # Tab 2: Detection Metrics
    drop_curves[z] = plot_drop.plot(name=label, pen=pen_main)
    ambient_curves[z] = plot_ambient.plot(name=label, pen=pen_main)
    sigma_curves[z] = plot_sigma.plot(name=label, pen=pen_main)

    # Tab 3: Advanced
    reflect_curves[z] = plot_reflect.plot(name=label, pen=pen_main)
    spads_curves[z] = plot_spads.plot(name=label, pen=pen_main)
    motion_curves[z] = plot_motion.plot(name=label, pen=pen_main)

frame_count = 0
total_detections = 0
current_temp = 0

# Store detection events as vertical markers
detection_markers = deque(maxlen=30)

# ================================================================
# CSV DATA WRITER
# ================================================================
def save_frame_to_csv(zones, motion, temp, detections):
    """Append a single frame of data to the CSV file."""
    if not SAVE_DATA or data_file is None:
        return
    
    try:
        timestamp = datetime.now().isoformat()
        row = [timestamp, frame_count]
        
        for z in range(NUM_ZONES):
            row.extend([
                zones[z]['sig'], zones[z]['base'],
                zones[z]['dist'], zones[z]['bdist'],
                zones[z]['amb'], zones[z]['sigma'],
                zones[z]['refl'], zones[z]['status'],
                zones[z]['spads'], zones[z]['targs'],
                zones[z]['drop'], zones[z]['valid'],
                motion[z] if z < len(motion) else 0
            ])
        
        row.extend([temp, detections])
        
        with open(data_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    except Exception as e:
        print(f"Error saving to CSV: {e}")

# ================================================================
# PARSING FUNCTIONS
# ================================================================

def parse_extended_zframe_line(line):
    """
    Parse extended ZFRAME format:
    ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...,motion0,...,motionN

    Layout after split:
      parts[0]   = "ZFRAME" (label)
      parts[1]   = temp
      parts[2:]  = zone data (12 fields per zone × N zones)
      last N     = motion data

    Resolution is set manually via RESOLUTION config (4 or 8).
    """
    parts = line.split(',')

    min_parts = 2 + NUM_ZONES * ZONE_FIELDS + 1
    if len(parts) < min_parts:
        return None

    try:
        temp = int(parts[1])
    except ValueError:
        return None

    zones = []
    for z in range(NUM_ZONES):
        base_idx = 2 + z * ZONE_FIELDS   # offset by 2: label + temp
        if base_idx + ZONE_FIELDS > len(parts):
            return None
        try:
            zone_data = {
                'sig':    int(parts[base_idx + F_SIG]),
                'base':   int(parts[base_idx + F_BASE]),
                'dist':   int(parts[base_idx + F_DIST]),
                'bdist':  int(parts[base_idx + F_BDIST]),
                'amb':    int(parts[base_idx + F_AMB]),
                'sigma':  int(parts[base_idx + F_SIGMA]),
                'refl':   int(parts[base_idx + F_REFL]),
                'status': int(parts[base_idx + F_STATUS]),
                'spads':  int(parts[base_idx + F_SPADS]),
                'targs':  int(parts[base_idx + F_TARGS]),
                'drop':   int(parts[base_idx + F_DROP]),
                'valid':  int(parts[base_idx + F_VALID]),
            }
            zones.append(zone_data)
        except (ValueError, IndexError):
            return None

    # Parse motion data (last NUM_ZONES fields)
    motion_start = 2 + NUM_ZONES * ZONE_FIELDS
    motion = []
    for m in range(NUM_ZONES):
        if motion_start + m < len(parts):
            try:
                motion.append(int(parts[motion_start + m]))
            except ValueError:
                motion.append(0)
        else:
            motion.append(0)

    return {'temp': temp, 'zones': zones, 'motion': motion}


def parse_legacy_zframe_line(line):
    """
    Parse legacy ZFRAME format (backward compatible):
    ZFRAME,sig0,base0,dist0,bdist0,...,sig15,base15,dist15,bdist15

    parts[0] = "ZFRAME", parts[1..64] = zone data (4 fields × 16 zones)
    """
    parts = line.split(',')
    if len(parts) != 1 + NUM_ZONES * 4:
        return None

    zones = []
    for z in range(NUM_ZONES):
        base_idx = 1 + z * 4
        try:
            sig   = int(parts[base_idx])
            base  = int(parts[base_idx + 1])
            dist  = int(parts[base_idx + 2])
            bdist = int(parts[base_idx + 3])

            drop_pct = 0.0
            if base > 0:
                drop_pct = abs(base - sig) * 100.0 / base

            zones.append({
                'sig': sig, 'base': base,
                'dist': dist, 'bdist': bdist,
                'amb': 0, 'sigma': 0, 'refl': 0,
                'status': 0, 'spads': 0, 'targs': 0,
                'drop': drop_pct, 'valid': 1 if sig > 0 else 0
            })
        except (ValueError, IndexError):
            return None

    return {'temp': 0, 'zones': zones, 'motion': [0] * NUM_ZONES}


def parse_allparam_line(line):
    """
    Parse ALLPARAM format (same as extended ZFRAME but prefixed with ALLPARAM):
    ALLPARAM,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...

    parts[0] = "ALLPARAM", parts[1] = temp, parts[2:] = zone data
    """
    # Replace "ALLPARAM" with "ZFRAME" so the extended parser handles it
    zframe_line = "ZFRAME" + line[9:]  # "ALLPARAM" is 8 chars, "ZFRAME" is 6 → line[9:] skips "ALLPARAM,"
    return parse_extended_zframe_line(zframe_line)


def parse_motion_line(line):
    """
    Parse MOTION format:
    MOTION,global1,global2,status,nb_detected,nb_agg,motion0,...,motion15
    """
    parts = line.split(',')
    if len(parts) < 6 + NUM_ZONES:
        return None

    try:
        return {
            'global1': int(parts[1]),
            'global2': int(parts[2]),
            'status': int(parts[3]),
            'nb_detected': int(parts[4]),
            'nb_agg': int(parts[5]),
            'motion': [int(parts[6 + m]) for m in range(NUM_ZONES)]
        }
    except ValueError:
        return None


# ================================================================
# UPDATE FUNCTION
# ================================================================
def update_data():
    global frame_count, total_detections, current_temp

    try:
        while ser.in_waiting > 0:
            chunk = ser.readline().decode('utf-8', errors='ignore').strip()
            if not chunk:
                continue

            # Handle DET lines (firmware-triggered detection events)
            if chunk.startswith("DET,"):
                total_detections += 1
                det_parts = chunk.split(',')
                det_num = det_parts[1] if len(det_parts) > 1 else "?"
                zone_info = ", ".join(det_parts[2:]) if len(det_parts) > 2 else "unknown"
                print(f"\n*** DETECTION #{total_detections} (FW #{det_num}) *** Zones: {zone_info}")

                marker_pos = len(zone_drop_data[0]) - 1
                for z in range(NUM_ZONES):
                    m = pg.InfiniteLine(
                        pos=marker_pos, angle=90,
                        pen=pg.mkPen(color='w', width=1, style=QtCore.Qt.DotLine)
                    )
                    plot_drop.addItem(m, ignoreBounds=True)
                    detection_markers.append(m)

                status_text.setText(
                    f"*** DETECTION #{total_detections} *** | Zones: {zone_info}"
                )
                status_text.setColor('#FF0000')
                continue

            # Handle MOTION lines
            if chunk.startswith("MOTION,"):
                motion_data = parse_motion_line(chunk)
                if motion_data:
                    motion_global1_history.append(motion_data['global1'])
                    motion_global2_history.append(motion_data['global2'])
                continue

            # Handle ALLPARAM lines (richer data, same structure)
            if chunk.startswith("ALLPARAM,"):
                data = parse_allparam_line(chunk)
                if data is None:
                    continue
                # Process same as ZFRAME but don't increment frame_count
                process_zone_data(data)
                continue

            # Handle ZFRAME lines (per-zone data)
            if chunk.startswith("ZFRAME"):
                # Try extended format first, fall back to legacy
                data = parse_extended_zframe_line(chunk)
                if data is None:
                    data = parse_legacy_zframe_line(chunk)
                if data is None:
                    continue

                frame_count += 1
                process_zone_data(data)
                continue

    except Exception as e:
        print(f"Error: {e}")


def process_zone_data(data):
    """Process parsed zone data and update all plots."""
    global current_temp

    zones = data['zones']
    motion = data.get('motion', [0] * NUM_ZONES)
    current_temp = data.get('temp', 0)

    # Update temperature display
    temp_text.setText(f"Temp: {current_temp}°C")
    if current_temp > 60:
        temp_text.setColor('#FF0000')
    elif current_temp > 45:
        temp_text.setColor('#FFA500')
    else:
        temp_text.setColor('#00FF00')

    # Update all zone data
    for z in range(NUM_ZONES):
        zone_signal_data[z].append(zones[z]['sig'])
        zone_baseline_data[z].append(zones[z]['base'])
        zone_distance_data[z].append(zones[z]['dist'])
        zone_bdistance_data[z].append(zones[z]['bdist'])
        zone_drop_data[z].append(zones[z]['drop'])
        zone_ambient_data[z].append(zones[z]['amb'])
        zone_sigma_data[z].append(zones[z]['sigma'])
        zone_reflect_data[z].append(zones[z]['refl'])
        zone_spads_data[z].append(zones[z]['spads'])
        zone_status_data[z].append(zones[z]['status'])
        zone_targets_data[z].append(zones[z]['targs'])
        zone_valid_data[z].append(zones[z]['valid'])
        zone_motion_data[z].append(motion[z] if z < len(motion) else 0)

        # Update curves - Tab 1: Signal & Distance
        signal_curves[z].setData(list(zone_signal_data[z]))
        signal_baseline_curves[z].setData(list(zone_baseline_data[z]))
        distance_curves[z].setData(list(zone_distance_data[z]))
        distance_baseline_curves[z].setData(list(zone_bdistance_data[z]))

        # Update curves - Tab 2: Detection Metrics
        drop_curves[z].setData(list(zone_drop_data[z]))
        ambient_curves[z].setData(list(zone_ambient_data[z]))
        sigma_curves[z].setData(list(zone_sigma_data[z]))

        # Update curves - Tab 3: Advanced
        reflect_curves[z].setData(list(zone_reflect_data[z]))
        spads_curves[z].setData(list(zone_spads_data[z]))
        motion_curves[z].setData(list(zone_motion_data[z]))

    # Update heatmaps (use GRID_SIZE: 4 for 4x4, 8 for 8x8)
    heatmap_drop = np.zeros((GRID_SIZE, GRID_SIZE))
    heatmap_dist = np.zeros((GRID_SIZE, GRID_SIZE))
    heatmap_motion = np.zeros((GRID_SIZE, GRID_SIZE))
    heatmap_reflect = np.zeros((GRID_SIZE, GRID_SIZE))

    for z in range(NUM_ZONES):
        row = z // GRID_SIZE
        col = z % GRID_SIZE
        heatmap_drop[row, col] = zones[z]['drop']
        heatmap_dist[row, col] = zones[z]['dist']
        heatmap_motion[row, col] = motion[z] if z < len(motion) else 0
        heatmap_reflect[row, col] = zones[z]['refl']

    # Rotate for correct orientation
    heatmap_drop = np.rot90(heatmap_drop, k=-1)
    heatmap_dist = np.rot90(heatmap_dist, k=-1)
    heatmap_motion = np.rot90(heatmap_motion, k=-1)
    heatmap_reflect = np.rot90(heatmap_reflect, k=-1)

    heatmap_drop_img.setImage(heatmap_drop, autoLevels=False)
    heatmap_dist_img.setImage(heatmap_dist, autoLevels=False)
    heatmap_motion_img.setImage(heatmap_motion, autoLevels=False)
    heatmap_reflect_img.setImage(heatmap_reflect, autoLevels=False)

    # Save data to CSV
    save_frame_to_csv(zones, motion, current_temp, total_detections)

    # Update status text
    max_drop_zone = max(range(NUM_ZONES), key=lambda z: zones[z]['drop'])
    max_drop = zones[max_drop_zone]['drop']

    if max_drop > 6:
        status_text.setText(
            f"FRAME #{frame_count} | DROP: {max_drop}% in Z{max_drop_zone} | "
            f"SIG: {zones[max_drop_zone]['sig']}/{zones[max_drop_zone]['base']} | "
            f"DIST: {zones[max_drop_zone]['dist']}/{zones[max_drop_zone]['bdist']}mm | "
            f"AMBIENT: {zones[max_drop_zone]['amb']} | SIGMA: {zones[max_drop_zone]['sigma']}mm | "
            f"REFLECT: {zones[max_drop_zone]['refl']}% | "
            f"Detects: {total_detections}"
        )
        status_text.setColor('#FF3300')
    else:
        status_text.setText(
            f"FRAME #{frame_count} | OK | "
            f"MAX DROP: {max_drop}% in Z{max_drop_zone} | "
            f"Detects: {total_detections}"
        )
        status_text.setColor('#00FF00')

    # Auto-scale plots
    max_sig = max(zones[z]['sig'] for z in range(NUM_ZONES))
    max_base = max(zones[z]['base'] for z in range(NUM_ZONES))
    sig_max = max(max_sig, max_base, 1)
    if sig_max > 0:
        plot_signal.setYRange(0, sig_max * 1.2)

    max_dist = max(zones[z]['dist'] for z in range(NUM_ZONES))
    max_bdist = max(zones[z]['bdist'] for z in range(NUM_ZONES))
    dist_max = max(max_dist, max_bdist, 1)
    if dist_max > 0:
        plot_distance.setYRange(0, dist_max * 1.2)

    max_amb = max(zones[z]['amb'] for z in range(NUM_ZONES))
    if max_amb > 0:
        plot_ambient.setYRange(0, max_amb * 1.2)

    max_sigma_val = max(zones[z]['sigma'] for z in range(NUM_ZONES))
    if max_sigma_val > 0:
        plot_sigma.setYRange(0, max_sigma_val * 1.5)

    max_drop_val = max(zones[z]['drop'] for z in range(NUM_ZONES))
    drop_max = max(max_drop_val, 10)
    if drop_max > 0:
        plot_drop.setYRange(-5, drop_max * 1.5)

    max_spads = max(zones[z]['spads'] for z in range(NUM_ZONES))
    if max_spads > 0:
        plot_spads.setYRange(0, max_spads * 1.2)

    max_motion_val = max(motion) if motion else 0
    if max_motion_val > 0:
        plot_motion.setYRange(0, max(max_motion_val * 1.5, 100))
        heatmap_motion_img.setLevels([0, max(max_motion_val * 1.5, 500)])


def on_key(event):
    if event.key() == QtCore.Qt.Key_R:
        for z in range(NUM_ZONES):
            for dq in [zone_signal_data[z], zone_baseline_data[z],
                       zone_distance_data[z], zone_bdistance_data[z],
                       zone_drop_data[z], zone_ambient_data[z],
                       zone_sigma_data[z], zone_reflect_data[z],
                       zone_spads_data[z], zone_motion_data[z]]:
                dq.clear()
                dq.extend([0] * MAX_POINTS)
        global frame_count
        frame_count = 0
        print("Data reset!")

# Install event filter for key press on the main window
class KeyFilter(QtCore.QObject):
    def eventFilter(self, obj, event):
        if event.type() == event.Type.KeyPress:
            on_key(event)
        return super().eventFilter(obj, event)

key_filter = KeyFilter()
win.installEventFilter(key_filter)
# Make window focusable so it receives key events
win.setFocusPolicy(QtCore.Qt.StrongFocus)

def close_app():
    timer.stop()
    if SAVE_DATA and data_file:
        print(f"\nData saved to: {data_file}")
        print(f"Total frames recorded: {frame_count}")
    if ser.is_open:
        ser.close()
        print(f"Port closed. Frames: {frame_count}, Detections: {total_detections}")

app.aboutToQuit.connect(close_app)

timer = QtCore.QTimer()
timer.timeout.connect(update_data)
timer.start(10)

print("=" * 70)
print(f"VL53L5CX {GRID_SIZE}x{GRID_SIZE} Zone Monitor - ALL PARAMETERS ENABLED")
print("=" * 70)
print(f"Port: {SERIAL_PORT} | Baud: {BAUD_RATE}")
print(f"Resolution: {GRID_SIZE}x{GRID_SIZE} ({NUM_ZONES} zones) — must match firmware!")
print("")
print("Available Parameters per Zone:")
print("  - Signal per SPAD (current + baseline)")
print("  - Distance mm (current + baseline)")
print("  - Ambient Noise per SPAD")
print("  - Range Sigma mm")
print("  - Reflectance %")
print("  - Target Status")
print("  - SPADs Enabled")
print("  - Targets Detected")
print("  - Signal Drop %")
print("  - Zone Valid Flag")
print("  - Motion Indicator")
print("")
print("Global Parameters:")
print("  - Silicon Temperature")
print("  - Motion Global Indicators")
print("")
print("Tabs: Signal & Distance | Detection Metrics | Advanced | Heatmaps")
print("Press 'R' to reset data")
if SAVE_DATA:
    print(f"\nDATA SAVING: ENABLED -> {data_file}")
else:
    print("\nDATA SAVING: DISABLED")
print("=" * 70)

if __name__ == '__main__':
    sys.exit(app.exec_())