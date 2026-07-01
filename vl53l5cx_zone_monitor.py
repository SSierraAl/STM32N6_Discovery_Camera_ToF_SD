#!/usr/bin/env python3
"""
VL53L5CX 4x4 Zone Monitor - Real-time signal visualization
============================================================
Parses ZFRAME debug output from STM32 insect detection firmware.

ZFRAME format (from main.c):
  ZFRAME,sig0,base0,dist0,bdist0,sig1,base1,dist1,bdist1,...,sig15,base15,dist15,bdist15

Where for each zone N (0-15):
  - sigN   = current signal_per_spad
  - baseN  = baseline signal_per_spad
  - distN  = current distance_mm
  - bdistN = baseline distance_mm

Usage:
  python vl53l5cx_zone_monitor.py

Requires:
  pip install pyserial pyqtgraph numpy
"""

import sys
import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from collections import deque

# ================================================================
# CONFIGURATION
# ================================================================
SERIAL_PORT = 'COM7'   # <-- Change to your port
BAUD_RATE   = 115200    # Must match STM32 UART baud rate
MAX_POINTS  = 150       # History length on plots
NUM_ZONES   = 16        # 4x4 = 16 zones

# Color map for zones
ZONE_COLORS = [
    (68, 1, 84), (72, 35, 116), (64, 67, 135), (52, 94, 141),
    (41, 120, 142), (32, 144, 140), (34, 167, 132), (68, 190, 112),
    (121, 209, 81), (189, 222, 38), (253, 231, 37), (253, 231, 37),
    (253, 231, 37), (253, 231, 37), (253, 231, 37), (253, 231, 37),
]

# ================================================================
# SERIAL CONNECTION
# ================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")
except Exception as e:
    print(f"Serial error: {e}")
    sys.exit(1)

# ================================================================
# PYQTGRAPH SETUP
# ================================================================
app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="VL53L5CX 4x4 Zone Monitor")
win.resize(1400, 900)

# --- Plot 1: Signal per SPAD ---
plot_signal = win.addPlot(title="Signal per SPAD (Current vs Baseline)")
plot_signal.setYRange(0, 1000)
plot_signal.setLabel('left', 'Signal', units='kcps/spad')
plot_signal.setLabel('bottom', 'Samples')
plot_signal.showGrid(x=True, y=True, alpha=0.3)
plot_signal.addLegend(offset=(10, 10))

# --- Plot 2: Distance ---
plot_distance = win.addPlot(title="Distance (Current vs Baseline)")
plot_distance.setYRange(0, 200)
plot_distance.setLabel('left', 'Distance', units='mm')
plot_distance.setLabel('bottom', 'Samples')
plot_distance.showGrid(x=True, y=True, alpha=0.3)
plot_distance.addLegend(offset=(10, 10))

# --- Plot 3: Signal Drop Percentage ---
plot_drop = win.addPlot(title="Signal Drop % (Detection Metric)")
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

# --- Plot 4: 4x4 Heatmap ---
win.nextRow()
heatmap_plot = win.addPlot(title="4x4 Zone Heatmap (Signal Drop %)")
heatmap_plot.hideAxis('left')
heatmap_plot.hideAxis('bottom')
heatmap_view = heatmap_plot.getViewBox()
heatmap_view.setAspectLocked(True)
heatmap_img = pg.ImageItem()
heatmap_view.addItem(heatmap_img)
# Use viridis colormap (available in all pyqtgraph versions)
try:
    colormap = pg.colormap.get('viridis')
except Exception:
    # Fallback: create simple grayscale colormap
    colormap = pg.ColorMap(
        [0.0, 1.0],
        [[0, 0, 0], [255, 255, 255]]
    )
heatmap_img.setColorMap(colormap)
heatmap_img.setLevels([0, 30])

zone_labels = []
for row in range(4):
    for col in range(4):
        z = row * 4 + col
        text = pg.TextItem(f"Z{z}", color=(200, 200, 200), anchor=(0.5, 0.5))
        text.setPos(col + 0.5, row + 0.5)
        text.setFont(pg.QtGui.QFont('Arial', 9, pg.QtGui.QFont.Bold))
        heatmap_view.addItem(text)
        zone_labels.append(text)

# --- Status Panel ---
status_plot = win.addPlot()
status_plot.setMaximumHeight(60)
status_plot.hideAxis('left')
status_plot.hideAxis('bottom')
status_text = pg.TextItem("WAITING FOR DATA...", color='#888888', anchor=(0.5, 0.5))
status_text.setFont(pg.QtGui.QFont('Arial', 14, pg.QtGui.QFont.Bold))
status_plot.addItem(status_text, ignoreBounds=True)
status_text.setPos(0.5, 0.5)

# --- Data Storage ---
zone_signal_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_baseline_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_distance_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
zone_drop_data = {z: deque([0] * MAX_POINTS, maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

signal_curves = {}
signal_baseline_curves = {}
distance_curves = {}
drop_curves = {}

for z in range(NUM_ZONES):
    color = ZONE_COLORS[z]
    pen_main = pg.mkPen(color=color, width=2 if z < 4 else 1, style=QtCore.Qt.SolidLine)
    pen_base = pg.mkPen(color=color, width=1, style=QtCore.Qt.DotLine)
    label = f'Z{z}' if z < 4 else None
    
    signal_curves[z] = plot_signal.plot(name=label, pen=pen_main)
    signal_baseline_curves[z] = plot_signal.plot(pen=pen_base)
    distance_curves[z] = plot_distance.plot(name=label, pen=pen_main)
    drop_curves[z] = plot_drop.plot(name=label, pen=pen_main)

frame_count = 0
total_detections = 0

# Store detection events as vertical markers on the drop plot
detection_markers = deque(maxlen=30)

# ================================================================
# PARSING AND UPDATE
# ================================================================
def parse_zframe_line(line):
    parts = line.split(',')
    if len(parts) != 65:
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
                'drop_pct': drop_pct
            })
        except (ValueError, IndexError):
            return None
    
    return zones

def update_data():
    global frame_count, total_detections
    
    try:
        while ser.in_waiting > 0:
            chunk = ser.readline().decode('utf-8', errors='ignore').strip()
            if not chunk:
                continue

            # Handle DET lines (firmware-triggered detection events)
            if chunk.startswith("DET,"):
                total_detections += 1
                # Parse: DET,capture_num,Z3:15,Z7:12
                det_parts = chunk.split(',')
                det_num = det_parts[1] if len(det_parts) > 1 else "?"
                zone_info = ", ".join(det_parts[2:]) if len(det_parts) > 2 else "unknown"
                print(f"\n*** DETECTION #{total_detections} (FW #{det_num}) *** Zones: {zone_info}")
                
                # Add vertical marker on all plots at current position
                marker_pos = len(zone_drop_data[0]) - 1
                for z in range(NUM_ZONES):
                    m = pg.InfiniteLine(
                        pos=marker_pos, angle=90,
                        pen=pg.mkPen(color='w', width=1, style=QtCore.Qt.DotLine)
                    )
                    plot_drop.addItem(m, ignoreBounds=True)
                    detection_markers.append(m)
                
                # Flash status
                status_text.setText(
                    f"*** DETECTION #{total_detections} *** | Zones: {zone_info}"
                )
                status_text.setColor('#FF0000')
                heatmap_img.setLevels([0, 50])
                continue

            # Handle ZFRAME lines (per-zone data)
            if not chunk.startswith("ZFRAME"):
                continue
            
            zones = parse_zframe_line(chunk)
            if zones is None:
                continue
            
            frame_count += 1
            
            # Restore heatmap levels after flash
            heatmap_img.setLevels([0, 30])
            
            for z in range(NUM_ZONES):
                zone_signal_data[z].append(zones[z]['sig'])
                zone_baseline_data[z].append(zones[z]['base'])
                zone_distance_data[z].append(zones[z]['dist'])
                zone_drop_data[z].append(zones[z]['drop_pct'])
                
                signal_curves[z].setData(list(zone_signal_data[z]))
                signal_baseline_curves[z].setData(list(zone_baseline_data[z]))
                distance_curves[z].setData(list(zone_distance_data[z]))
                drop_curves[z].setData(list(zone_drop_data[z]))
            
            # Update heatmap
            heatmap_data = np.zeros((4, 4))
            for z in range(NUM_ZONES):
                row = z // 4
                col = z % 4
                heatmap_data[row, col] = zones[z]['drop_pct']
            
            heatmap_data = np.rot90(heatmap_data, k=-1)
            heatmap_img.setImage(heatmap_data, autoLevels=False)
            
            # Update status
            max_drop_zone = max(range(NUM_ZONES), key=lambda z: zones[z]['drop_pct'])
            max_drop = zones[max_drop_zone]['drop_pct']
            
            if max_drop > 6:
                status_text.setText(
                    f"FRAME #{frame_count} | DROP: {max_drop:.1f}% in Z{max_drop_zone} | "
                    f"SIG: {zones[max_drop_zone]['sig']}/{zones[max_drop_zone]['base']} | "
                    f"DIST: {zones[max_drop_zone]['dist']}/{zones[max_drop_zone]['bdist']}mm | "
                    f"Detects: {total_detections}"
                )
                status_text.setColor('#FF3300')
            else:
                status_text.setText(
                    f"FRAME #{frame_count} | OK | "
                    f"MAX DROP: {max_drop:.1f}% in Z{max_drop_zone} | "
                    f"Detects: {total_detections}"
                )
                status_text.setColor('#00FF00')
            
            # Auto-scale (only when values are non-zero to avoid division issues)
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
            
            max_drop_val = max(zones[z]['drop_pct'] for z in range(NUM_ZONES))
            drop_max = max(max_drop_val, 10)
            if drop_max > 0:
                plot_drop.setYRange(-5, drop_max * 1.5)
            
    except Exception as e:
        print(f"Error: {e}")

def on_key(event):
    if event.key() == QtCore.Qt.Key_R:
        for z in range(NUM_ZONES):
            zone_signal_data[z].clear()
            zone_distance_data[z].clear()
            zone_drop_data[z].clear()
            zone_signal_data[z].extend([0] * MAX_POINTS)
            zone_distance_data[z].extend([0] * MAX_POINTS)
            zone_drop_data[z].extend([0] * MAX_POINTS)
        global frame_count
        frame_count = 0
        print("Data reset!")

win.scene().keyPressEvent = on_key

def close_app():
    timer.stop()
    if ser.is_open:
        ser.close()
        print(f"\nPort closed. Frames: {frame_count}, Detections: {total_detections}")

app.aboutToQuit.connect(close_app)

timer = QtCore.QTimer()
timer.timeout.connect(update_data)
timer.start(10)

print("=" * 60)
print("VL53L5CX 4x4 Zone Monitor")
print(f"Port: {SERIAL_PORT} | Baud: {BAUD_RATE}")
print("Press 'R' to reset data")
print("=" * 60)

if __name__ == '__main__':
    sys.exit(app.exec_())