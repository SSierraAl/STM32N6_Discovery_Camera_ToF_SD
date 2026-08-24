#!/usr/bin/env python3
"""
VL53L5CX Zone Monitor - Real-time ToF Sensor Visualization
============================================================
Parses ZFRAME and EXT,ZFRAME debug output from STM32 firmware.
Dual sensor mode: S1 (Primary) + EXT (Guardian)

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
SERIAL_PORT      = 'COM6'
BAUD_RATE        = 115200
MAX_POINTS       = 100
GRID_SIZE        = 4       # 4x4 grid
NUM_ZONES        = GRID_SIZE * GRID_SIZE  # 16
PLOT_ZONES       = 8       # zones to show on line charts
COMPACT_FIELDS   = 5       # sig, dist, base_sig, base_dist, motion
DUAL_SENSOR      = False

# ================================================================
# COLORS
# ================================================================
BG      = '#1a1a2e'
PLOT_BG = '#16213e'
TXT     = '#e0e0e0'
CYAN    = '#00d4ff'
GREEN   = '#00ff88'
RED     = '#ff4466'
YELLOW  = '#ffcc00'
ORANGE  = '#ff8800'
PINK    = '#ff66aa'

ZONE_COLORS = [
    CYAN, '#00b8d4', '#0097a7', '#00838f',
    '#26a69a', '#66bb6a', '#9ccc65', YELLOW,
]

pg.setConfigOption('background', BG)
pg.setConfigOption('foreground', TXT)

# ================================================================
# SERIAL
# ================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
    print(f"[OK] {SERIAL_PORT} @ {BAUD_RATE}")
except Exception as e:
    print(f"[ERR] {e}"); sys.exit(1)

serial_buffer = ""

# ================================================================
# SENSOR CLASS
# ================================================================
class Sensor:
    def __init__(self, name, color, tab_widget):
        self.name = name
        self.col  = color
        self.frames = 0
        self.temp = 0

        # rolling data
        self.sig    = {z: deque(maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
        self.dist   = {z: deque(maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
        self.drop   = {z: deque(maxlen=MAX_POINTS) for z in range(NUM_ZONES)}
        self.motion = {z: deque(maxlen=MAX_POINTS) for z in range(NUM_ZONES)}

        # tab
        tab = QtWidgets.QWidget()
        gl  = QtWidgets.QGridLayout(tab)
        gl.setContentsMargins(6, 6, 6, 6)
        gl.setSpacing(6)

        # left: line plots
        left = QtWidgets.QVBoxLayout()

        self.p1 = self._plot("Signal per SPAD", 0, 1000)
        self.p2 = self._plot("Distance (mm)",    0, 200)
        self.p3 = self._plot("Drop %",           0, 30)
        self.p3.addItem(pg.InfiniteLine(pos=6, angle=0,
            pen=pg.mkPen(RED, width=1, style=QtCore.Qt.DashLine)))
        self.p4 = self._plot("Motion",           0, 100)
        self.p4.addItem(pg.InfiniteLine(pos=40, angle=0,
            pen=pg.mkPen(ORANGE, width=1, style=QtCore.Qt.DashLine)))

        left.addWidget(self.p1)
        left.addWidget(self.p2)
        left.addWidget(self.p3)
        left.addWidget(self.p4)

        # right: heatmap + info labels
        right = QtWidgets.QVBoxLayout()

        # drop heatmap
        self.hm = pg.PlotWidget()
        self.hm.setBackground(PLOT_BG)
        self.hm.hideAxis('left'); self.hm.hideAxis('bottom')
        self.hm.getViewBox().setAspectLocked(True)
        self.img = pg.ImageItem()
        self.hm.addItem(self.img)
        # simple grayscale-like lookup: low=black, high=red
        lut = np.zeros((256, 4), dtype=np.uint8)
        for i in range(256):
            t = i / 255.0
            if t < 0.5:
                lut[i] = [0, int(255 * (t * 2)), 0, 255]  # green
            else:
                lut[i] = [255, int(255 * (1 - (t - 0.5) * 2)), 0, 255]  # yellow->red
        self.img.setLookupTable(lut)
        self.img.setLevels([0, 25])
        self.hm.setTitle(f"{GRID_SIZE}x{GRID_SIZE} Drop%")
        right.addWidget(self.hm)

        # info labels (plain QLabel, no pyqtgraph)
        self.lbl_status = QtWidgets.QLabel(f"{name}: WAITING")
        self.lbl_status.setStyleSheet(f"color:{TXT};background:{PLOT_BG};font-family:Consolas;font-size:11pt;padding:4px;border-radius:3px")
        right.addWidget(self.lbl_status)

        self.lbl_temp = QtWidgets.QLabel(f"{name}: --C")
        self.lbl_temp.setStyleSheet(f"color:{YELLOW};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")
        right.addWidget(self.lbl_temp)

        gl.addLayout(left, 0, 0)
        gl.addLayout(right, 0, 1)
        tab_widget.addTab(tab, name)

        # curves
        self.c_sig = [self.p1.plot(pen=pg.mkPen(ZONE_COLORS[z%len(ZONE_COLORS)], width=1.2)) for z in range(PLOT_ZONES)]
        self.c_dist = [self.p2.plot(pen=pg.mkPen(ZONE_COLORS[z%len(ZONE_COLORS)], width=1.2)) for z in range(PLOT_ZONES)]
        self.c_drop = [self.p3.plot(pen=pg.mkPen(ZONE_COLORS[z%len(ZONE_COLORS)], width=1.2)) for z in range(PLOT_ZONES)]
        self.c_mot  = [self.p4.plot(pen=pg.mkPen(ZONE_COLORS[z%len(ZONE_COLORS)], width=1.2)) for z in range(PLOT_ZONES)]

    def _plot(self, title, ymin, ymax):
        p = pg.PlotWidget(title=f" {title}")
        p.setBackground(PLOT_BG)
        p.setLabel('bottom', 'Samples', color=TXT)
        p.showGrid(True, True, alpha=0.15)
        p.setYRange(ymin, ymax)
        return p

    def update(self, zones, motion):
        self.frames += 1
        for z in range(NUM_ZONES):
            self.sig[z].append(zones[z]['sig'])
            self.dist[z].append(zones[z]['dist'])
            self.drop[z].append(zones[z]['drop'])
            self.motion[z].append(motion[z] if z < len(motion) else 0)

        for z in range(PLOT_ZONES):
            self.c_sig[z].setData(list(self.sig[z]))
            self.c_dist[z].setData(list(self.dist[z]))
            self.c_drop[z].setData(list(self.drop[z]))
            self.c_mot[z].setData(list(self.motion[z]))

        # heatmap
        hm = np.zeros((GRID_SIZE, GRID_SIZE))
        for z in range(NUM_ZONES):
            hm[z // GRID_SIZE, z % GRID_SIZE] = zones[z]['drop']
        self.img.setImage(hm, autoLevels=False)

        # status
        mz = max(range(NUM_ZONES), key=lambda z: zones[z]['drop'])
        md = zones[mz]['drop']
        if md > 6:
            self.lbl_status.setText(f"[!!] {self.name} #{self.frames} DROP {md}% Z{mz}")
            self.lbl_status.setStyleSheet(self.lbl_status.styleSheet().replace(TXT, RED))
        else:
            self.lbl_status.setText(f"[OK] {self.name} #{self.frames} maxDrop {md}%")
            self.lbl_status.setStyleSheet(self.lbl_status.styleSheet().replace(TXT, GREEN))

        # temp
        self.temp = zones[0].get('_temp', self.temp)
        tc = RED if self.temp > 60 else (ORANGE if self.temp > 45 else GREEN)
        self.lbl_temp.setText(f"{self.name}: {self.temp}C")
        self.lbl_temp.setStyleSheet(f"color:{tc};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")

        # autoscale
        mx = max(zones[z]['sig'] for z in range(NUM_ZONES))
        if mx > 0: self.p1.setYRange(0, mx * 1.3)
        mx2 = max(zones[z]['dist'] for z in range(NUM_ZONES))
        if mx2 > 0: self.p2.setYRange(0, mx2 * 1.3)
        mx3 = max(motion) if motion else 0
        if mx3 > 0: self.p4.setYRange(0, max(mx3 * 1.5, 100))

    def reset(self):
        for z in range(NUM_ZONES):
            for d in [self.sig[z], self.dist[z], self.drop[z], self.motion[z]]:
                d.clear()
        self.frames = 0

# ================================================================
# GUI
# ================================================================
app = QtWidgets.QApplication(sys.argv)
app.setStyle('Fusion')
pal = app.palette()
pal.setColor(pg.QtGui.QPalette.Window, pg.mkColor(BG))
pal.setColor(pg.QtGui.QPalette.WindowText, pg.mkColor(TXT))
pal.setColor(pg.QtGui.QPalette.Button, pg.mkColor(PLOT_BG))
app.setPalette(pal)

win = QtWidgets.QMainWindow()
win.setWindowTitle("VL53L5CX Zone Monitor")
win.resize(1600, 1000)
cw = QtWidgets.QWidget()
win.setCentralWidget(cw)
ml = QtWidgets.QVBoxLayout(cw)

tabs = QtWidgets.QTabWidget()
ml.addWidget(tabs)

gbar = QtWidgets.QLabel("WAITING FOR DATA...")
gbar.setAlignment(QtCore.Qt.AlignCenter)
gbar.setStyleSheet(f"color:{CYAN};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")
ml.addWidget(gbar)

s1 = Sensor("S1 (Primary)", CYAN, tabs)
s2 = Sensor("EXT (Guardian)", PINK, tabs) if DUAL_SENSOR else None
win.show()

total_det = 0

# ================================================================
# PARSING
# ================================================================
def parse(line):
    parts = line.split(',')
    if (len(parts) - 2) // COMPACT_FIELDS != NUM_ZONES:
        return None
    try:
        temp = int(parts[1])
    except ValueError:
        return None
    zones = []; motion = []
    for z in range(NUM_ZONES):
        i = 2 + z * COMPACT_FIELDS
        try:
            sig = int(parts[i]); dist = int(parts[i+1])
            bs = int(parts[i+2]); bd = int(parts[i+3])
            mot = int(parts[i+4])
            dr = abs(bs - sig) * 100 // bs if bs > 0 else 0
            zones.append({'sig': sig, 'base': bs, 'dist': dist, 'bdist': bd, 'drop': dr, '_temp': temp})
            motion.append(mot)
        except (ValueError, IndexError):
            return None
    return zones, motion

# ================================================================
# SERIAL LOOP
# ================================================================
def tick():
    global serial_buffer, total_det
    try:
        while ser.in_waiting > 0:
            serial_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        while '\n' in serial_buffer:
            line, serial_buffer = serial_buffer.split('\n', 1)
            line = line.strip().rstrip('\r')
            if not line: continue

            if line.startswith("EXT,ZFRAME"):
                if s2:
                    d = parse("ZFRAME" + line[4:])
                    if d: s2.update(*d)
                continue
            if line.startswith("ZFRAME"):
                d = parse(line)
                if d: s1.update(*d)
                continue
            if line.startswith("[EXT]") and "Detection confirmed" in line:
                total_det += 1
                gbar.setText(f"[EXT->S1] Guardian woke primary! ({total_det})")
                gbar.setStyleSheet(gbar.styleSheet().replace(CYAN, PINK))
                continue
            if line.startswith(">>> INSECT DETECTED"):
                total_det += 1
                gbar.setText(f"{line} ({total_det})")
                gbar.setStyleSheet(gbar.styleSheet().replace(CYAN, RED))
                continue
    except Exception as e:
        print(f"[ERR] {e}")

# ================================================================
# KEY: R = reset
# ================================================================
class KF(QtCore.QObject):
    def eventFilter(self, obj, ev):
        if ev.type() == ev.Type.KeyPress and ev.key() == QtCore.Qt.Key_R:
            s1.reset()
            if s2: s2.reset()
            nonlocal_total()
        return super().eventFilter(obj, ev)

def nonlocal_total():
    global total_det
    total_det = 0

win.installEventFilter(KF())
win.setFocusPolicy(QtCore.Qt.StrongFocus)

# ================================================================
# RUN
# ================================================================
timer = QtCore.QTimer()
timer.timeout.connect(tick)
timer.start(40)
app.aboutToQuit.connect(lambda: ser.close() if ser.is_open else None)
print(f"Port:{SERIAL_PORT} Grid:{GRID_SIZE}x{GRID_SIZE} Dual:{'yes' if DUAL_SENSOR else 'no'} [R=reset]")
sys.exit(app.exec_())