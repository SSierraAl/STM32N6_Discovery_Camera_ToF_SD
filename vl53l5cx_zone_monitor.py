#!/usr/bin/env python3
"""
VL53L5CX Zone Monitor - Real-time ToF Sensor Visualization
============================================================
Parses ZFRAME and EXT,ZFRAME debug output from STM32 firmware.
Dual sensor mode: S1 (Primary) + EXT (Guardian)

Usage:
  python vl53l5cx_zone_monitor.py

  The toolbar has Connect / Disconnect buttons. If the USB port is
  unplugged while running, the status turns red (DISCONNECTED) and the
  plots freeze; re-plug and press Connect to resume without restarting.

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
GRID_SIZE        = 4       # MUST match VL53L5CX_DET_RESOLUTION in app_config.h (4 or 8)
NUM_ZONES        = GRID_SIZE * GRID_SIZE  # 16
PLOT_ZONES       = 16       # zones to show on line charts
COMPACT_FIELDS   = 5       # sig, dist, base_sig, base_dist, motion
DUAL_SENSOR      = False    # match VL53L5CX_DUAL_SENSOR in app_config.h

# ==================================================================
# Zone-grid orientation (display layout)
# ==================================================================
# Firmware data order is Z0..Z15, row by row (z = 4*row + col).
# With both flips False (the layout validated against the manual's
# SPAD figure): Z0 bottom-left, the z=0..3 row at the BOTTOM,
# Z15 top-right. The heatmap title shows the active combination:
#
#   (V=F,H=F) validated layout (manual SPAD figure)   Z0 bottom-left
#   (V=T,H=F) vertical flip                           Z0 top-left
#   (V=F,H=T) horizontal flip (scene view through the
#             Rx lens: Z0 sees the target top-right)  Z0 bottom-right
#   (V=T,H=T) 180-deg rotation of (V=F,H=F)           Z0 top-right
FLIP_V = True
FLIP_H = False

# pyqtgraph 0.14: ImageItem's default mode ("col-major") draws the
# array's FIRST axis horizontally, so hm[row][col] would show its rows
# as screen columns. Transposing before setImage makes the matrix rows
# run down the screen (matching FLIP_V and the zone order). Keep True.
COLOR_TRANSPOSE = True

# Draw the Z0..Z15 overlay on the heatmap (handy to verify orientation;
# set False for a clean display).
SHOW_ZONE_LABELS = True

# ================================================================
# DETECTION THRESHOLDS — mirror Inc/app_config.h, SECTION 9
# ================================================================
# Used ONLY for this script's host-side overlay: the dashed threshold
# lines on the plots, the heatmap title, the status labels and the
# "host trigger" prediction printed per frame. Change them HERE to
# PREVIEW a new setting while live data streams; to make the change
# stick, edit the matching define in app_config.h (SECTION 9),
# rebuild and flash the firmware. The startup console line prints
# the active values.
#
#   This script's var  | app_config.h define                 | 4x4 | 8x8
#   -------------------+-------------------------------------+-----+----
#   THRESHOLD_PCT      | VL53L5CX_DET_THRESHOLD_PCT          |   6 |  15   (drop % strictly greater than)
#   MOTION_THRESH      | VL53L5CX_DET_MOTION_THRESH          |  60 | 100   (zone motion value >= this)
#   MIN_AFFECTED_ZONES | VL53L5CX_DET_MIN_AFFECTED_ZONES     |   1 |   2   (zones needed for a trigger)
#   MIN_SIGNAL         | VL53L5CX_DET_MIN_SIGNAL             | 500 | 500   (signal floor, kcps/spad)
#
# Sensor-side ST plugin values — ANNOTATION ONLY (this script never
# reads them; the plugin runs inside the sensor's GO2, so changing
# them requires a firmware edit + a new datalog; see
# TUNING_GUIDE.md section 4):
PLUGIN_MIN_ZONES   = 1     # VL53L5CX_DET_MOTION_MIN_ZONES       (plugin global-motion flag)
PLUGIN_PERSIST     = 16    # VL53L5CX_DET_MOTION_PERSIST_FRAMES  (temporal accumulation)
PLUGIN_EXTRA_NOISE = 0     # VL53L5CX_DET_MOTION_EXTRA_NOISE     (noise margin)

THRESHOLD_PCT      = 6     # VL53L5CX_DET_THRESHOLD_PCT      (4x4: 6, 8x8: 15)
MOTION_THRESH      = 60    # VL53L5CX_DET_MOTION_THRESH      (4x4: 60, 8x8: 100)
MIN_AFFECTED_ZONES = 1     # VL53L5CX_DET_MIN_AFFECTED_ZONES (4x4: 1, 8x8: 2)
MIN_SIGNAL         = 500   # VL53L5CX_DET_MIN_SIGNAL         (kcps/spad)

# EXT (guardian) sensor thresholds — dual mode only. Mirror the
# independent VL53L5CX_EXT_* defines in app_config.h SECTION 9 (EXT
# block). They drive ONLY the EXT tab: its overlay lines, heatmap
# title and host trigger prediction (S1 keeps the primary values).
# Defaults = current firmware values; 8x8 in parentheses.
EXT_THRESHOLD_PCT      = 6     # VL53L5CX_EXT_THRESHOLD_PCT      (4x4: 6, 8x8: 15)
EXT_MOTION_THRESH      = 60    # VL53L5CX_EXT_MOTION_THRESH      (4x4: 60, 8x8: 100)
EXT_MIN_AFFECTED_ZONES = 1     # VL53L5CX_EXT_MIN_AFFECTED_ZONES (4x4: 1, 8x8: 2)
EXT_MIN_SIGNAL         = 500   # VL53L5CX_EXT_MIN_SIGNAL         (kcps/spad)

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
# SERIAL (opened by connect_port(), auto-attempted at startup.
# If the port is missing the window shows CONNECT FAILED /
# DISCONNECTED and waits for the Connect button instead of exiting.)
# ================================================================
ser = None
serial_buffer = ""

# ================================================================
# SENSOR CLASS
# ================================================================
class Sensor:
    def __init__(self, name, color, tab_widget, thr, define_prefix="VL53L5CX_DET_"):
        self.name = name
        self.col  = color
        self.frames = 0
        self.temp = 0
        # per-sensor host overlay thresholds:
        # (threshold_pct, motion_thresh, min_affected_zones, min_signal)
        # S1 gets the primary VL53L5CX_DET_* values, the EXT guardian its
        # own VL53L5CX_EXT_* values (independent EXT configuration).
        self.t_pct, self.t_mot, self.t_zones, self.t_sig = thr
        self.t_prefix = define_prefix

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
        self.p3.addItem(pg.InfiniteLine(pos=self.t_pct, angle=0,
            pen=pg.mkPen(RED, width=1, style=QtCore.Qt.DashLine),
            label=f'Drop% > {self.t_pct} ({self.t_prefix}THRESHOLD_PCT)',
            labelOpts={'color': RED, 'position': 0.95}))
        self.p4 = self._plot("Motion",           0, 100)
        self.p4.addItem(pg.InfiniteLine(pos=self.t_mot, angle=0,
            pen=pg.mkPen(ORANGE, width=1, style=QtCore.Qt.DashLine),
            label=f'Motion >= {self.t_mot} ({self.t_prefix}MOTION_THRESH)',
            labelOpts={'color': ORANGE, 'position': 0.95}))

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
        self.hm.setTitle(f"{GRID_SIZE}x{GRID_SIZE} Drop%  [FLIP V={int(FLIP_V)} H={int(FLIP_H)}]  (host trigger: > {self.t_pct}% and sig >= {self.t_sig})")
        # Z-ID overlay: placed with the same FLIP logic as the colors, so
        # the labels always sit under the matching colored cells.
        if SHOW_ZONE_LABELS:
            for z in range(NUM_ZONES):
                r, c = z // GRID_SIZE, z % GRID_SIZE
                if FLIP_V:
                    r = GRID_SIZE - 1 - r
                if FLIP_H:
                    c = GRID_SIZE - 1 - c
                ti = pg.TextItem(f'Z{z}', color='#e8e8e8', anchor=(0.5, 0.5))
                ti.setPos(c, r)
                self.hm.addItem(ti)
        right.addWidget(self.hm)

        # info labels (plain QLabel, no pyqtgraph)
        self.lbl_status = QtWidgets.QLabel(f"{name}: WAITING")
        self.lbl_status.setStyleSheet(f"color:{TXT};background:{PLOT_BG};font-family:Consolas;font-size:11pt;padding:4px;border-radius:3px")
        right.addWidget(self.lbl_status)

        self.lbl_temp = QtWidgets.QLabel(f"{name}: --C")
        self.lbl_temp.setStyleSheet(f"color:{YELLOW};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")
        right.addWidget(self.lbl_temp)

        # host-side trigger prediction (replays the firmware logic from
        # the ZFRAME data alone — no per-zone status, so no status gate)
        self.lbl_host = QtWidgets.QLabel(f"host: 0/{self.t_zones} zones (no trigger)")
        self.lbl_host.setStyleSheet(f"color:{TXT};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")
        right.addWidget(self.lbl_host)

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

        # heatmap (orientation: see FLIP_V / FLIP_H / COLOR_TRANSPOSE above)
        hm = np.zeros((GRID_SIZE, GRID_SIZE))
        for z in range(NUM_ZONES):
            hm[z // GRID_SIZE, z % GRID_SIZE] = zones[z]['drop']
        if FLIP_V:
            hm = hm[::-1, :].copy()      # top <-> bottom
        if FLIP_H:
            hm = hm[:, ::-1].copy()      # left <-> right
        if COLOR_TRANSPOSE:
            hm = hm.T.copy()             # pyqtgraph col-major correction (keep True)
        self.img.setImage(hm, autoLevels=False)

        # status + host-side trigger replica
        # Same math as the firmware (TUNING_GUIDE.md section 4, "How a
        # trigger is computed"): a zone counts when
        #   (sig >= MIN_SIGNAL and drop % > THRESHOLD_PCT)   [signal]
        #   or (motion >= MOTION_THRESH)                     [motion]
        # a trigger needs >= MIN_AFFECTED_ZONES such zones.
        # ZFRAME carries no per-zone status, so the replica cannot apply
        # the firmware's fresh-data gate (status 5/6/9): it predicts
        # "at least" what the firmware will fire.
        mz = max(range(NUM_ZONES), key=lambda z: zones[z]['drop'])
        md = zones[mz]['drop']
        sig_hit = [z for z in range(NUM_ZONES)
                   if zones[z]['sig'] >= self.t_sig and zones[z]['drop'] > self.t_pct]
        mot_hit = [z for z in range(NUM_ZONES)
                   if (motion[z] if z < len(motion) else 0) >= self.t_mot]
        host_hit = sorted(set(sig_hit) | set(mot_hit))
        if host_hit and len(host_hit) >= self.t_zones:
            self.lbl_status.setText(f"[TRIG] {self.name} #{self.frames} zones={len(host_hit)} (sig {len(sig_hit)}, mot {len(mot_hit)})")
            self.lbl_status.setStyleSheet(self.lbl_status.styleSheet().replace(TXT, RED))
        elif md > self.t_pct:
            self.lbl_status.setText(f"[!!] {self.name} #{self.frames} DROP {md}% Z{mz} (no host trigger)")
            self.lbl_status.setStyleSheet(self.lbl_status.styleSheet().replace(TXT, YELLOW))
        else:
            self.lbl_status.setText(f"[OK] {self.name} #{self.frames} maxDrop {md}%")
            self.lbl_status.setStyleSheet(self.lbl_status.styleSheet().replace(TXT, GREEN))
        if host_hit:
            shown = host_hit[:8] + ['...'] if len(host_hit) > 8 else host_hit
            self.lbl_host.setText(f"host: {len(host_hit)}/{self.t_zones} zones {shown}")
            self.lbl_host.setStyleSheet(f"color:{RED};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")
        else:
            self.lbl_host.setText(f"host: 0/{self.t_zones} zones (no trigger)")
            self.lbl_host.setStyleSheet(f"color:{TXT};background:{PLOT_BG};font-family:Consolas;font-size:10pt;padding:4px;border-radius:3px")

        # temp
        self.temp = zones[0].get('_temp', self.temp)
        tc = RED if self.temp > 60 else (ORANGE if self.temp > 45 else GREEN)
        self.lbl_temp.setText(f"{self.name} Temperature : {self.temp}C")
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

# ---- link toolbar: Connect / Disconnect + link status ----
btn_connect = QtWidgets.QPushButton("Connect")
btn_disconnect = QtWidgets.QPushButton("Disconnect")
btn_disconnect.setEnabled(False)
btn_connect.setMinimumHeight(28)
btn_disconnect.setMinimumHeight(28)
for _b in (btn_connect, btn_disconnect):
    _b.setStyleSheet(f"font-family:Consolas;font-size:11pt;font-weight:bold;padding:4px 16px;background:{PLOT_BG};color:{TXT};border:1px solid {TXT};border-radius:3px")
lbl_link = QtWidgets.QLabel("CONNECTING...")
lbl_link.setStyleSheet(f"color:{YELLOW};font-family:Consolas;font-size:11pt;font-weight:bold;background:{PLOT_BG};padding:4px;border-radius:3px")
tb = QtWidgets.QHBoxLayout()
tb.addWidget(btn_connect); tb.addWidget(btn_disconnect); tb.addStretch(1); tb.addWidget(lbl_link)
ml.addLayout(tb)

tabs = QtWidgets.QTabWidget()
ml.addWidget(tabs)

gbar = QtWidgets.QLabel("WAITING FOR DATA...")
gbar.setAlignment(QtCore.Qt.AlignCenter)
gbar.setStyleSheet(f"color:{CYAN};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")
ml.addWidget(gbar)

s1 = Sensor("S1 (Primary)", CYAN, tabs,
            (THRESHOLD_PCT, MOTION_THRESH, MIN_AFFECTED_ZONES, MIN_SIGNAL))
s2 = Sensor("EXT (Guardian)", PINK, tabs,
            (EXT_THRESHOLD_PCT, EXT_MOTION_THRESH, EXT_MIN_AFFECTED_ZONES, EXT_MIN_SIGNAL),
            define_prefix="VL53L5CX_EXT_") if DUAL_SENSOR else None
win.show()

total_det = 0

# ================================================================
# SERIAL CONNECT / DISCONNECT
# ================================================================
def _set_link(connected, text, color):
    lbl_link.setText(text)
    lbl_link.setStyleSheet(f"color:{color};font-family:Consolas;font-size:11pt;font-weight:bold;background:{PLOT_BG};padding:4px;border-radius:3px")
    btn_connect.setEnabled(not connected)
    btn_disconnect.setEnabled(connected)

def _reset_host_data():
    global total_det
    s1.reset()
    if s2: s2.reset()
    total_det = 0
    gbar.setText("WAITING FOR DATA...")
    gbar.setStyleSheet(f"color:{CYAN};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")

def connect_port():
    global ser, serial_buffer
    if ser is not None and ser.is_open:
        return
    try:
        if ser is not None:
            try: ser.close()
            except Exception: pass
        # fresh serial object each time: a hot-unplugged handle is dead
        # on Windows and cannot be reused
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1.0)
    except Exception as e:
        print(f"[ERR] connect {SERIAL_PORT} failed: {e}")
        _set_link(False, f"CONNECT FAILED ({SERIAL_PORT}) — check port/cable, then press Connect", RED)
        gbar.setText("NOT CONNECTED")
        gbar.setStyleSheet(f"color:{RED};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")
        return
    serial_buffer = ""
    _reset_host_data()
    _set_link(True, f"CONNECTED {SERIAL_PORT} @ {BAUD_RATE}", GREEN)
    print(f"[OK] connected {SERIAL_PORT} @ {BAUD_RATE}")

def disconnect_port(reason="user"):
    global ser
    if ser is not None:
        try:
            if ser.is_open:
                ser.close()
        except Exception:
            pass
    if reason == "unplug":
        _set_link(False, f"DISCONNECTED ({SERIAL_PORT} lost) — re-plug USB, then press Connect", RED)
        gbar.setText("PORT LOST — press Connect to resume")
        gbar.setStyleSheet(f"color:{RED};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")
        print(f"[WARN] serial port lost (ClearCommError / unplugged?) — press Connect when re-plugged")
    else:
        _set_link(False, f"DISCONNECTED ({SERIAL_PORT}) — press Connect to resume", YELLOW)
        gbar.setText("NOT CONNECTED")
        gbar.setStyleSheet(f"color:{YELLOW};font-family:Consolas;font-size:13pt;font-weight:bold;background:{PLOT_BG};padding:6px;border-radius:4px")
        print(f"[OK] disconnected {SERIAL_PORT}")

btn_connect.clicked.connect(connect_port)
btn_disconnect.clicked.connect(lambda: disconnect_port("user"))

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
            dr = abs(bs - sig) * 100 // bs if bs > 0 else 0  # same formula as the firmware
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
    if ser is None or not ser.is_open:
        return
    try:
        while ser.in_waiting > 0:
            serial_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    except Exception:
        # Windows hot-unplug: ser.in_waiting -> ClearCommError fails with
        # PermissionError(13, ..., 22). Mark the link down once; the
        # Connect button re-opens a fresh port object.
        disconnect_port("unplug")
        return
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
app.aboutToQuit.connect(lambda: ser.close() if (ser is not None and ser.is_open) else None)
connect_port()   # auto-attempt at startup; if the port is missing, wait for Connect
print(f"Port:{SERIAL_PORT} Grid:{GRID_SIZE}x{GRID_SIZE} Dual:{'yes' if DUAL_SENSOR else 'no'} [R=reset] [Connect/Disconnect]")
print(f"S1  overlay thresholds (mirror VL53L5CX_DET_*): drop>{THRESHOLD_PCT}% sig>={MIN_SIGNAL} motion>={MOTION_THRESH} zones>={MIN_AFFECTED_ZONES}")
if DUAL_SENSOR:
    print(f"EXT overlay thresholds (mirror VL53L5CX_EXT_*): drop>{EXT_THRESHOLD_PCT}% sig>={EXT_MIN_SIGNAL} motion>={EXT_MOTION_THRESH} zones>={EXT_MIN_AFFECTED_ZONES}")
print(f"Sensor plugin values (annotation only, set in firmware): min_zones={PLUGIN_MIN_ZONES} persist={PLUGIN_PERSIST} extra_noise={PLUGIN_EXTRA_NOISE}")
sys.exit(app.exec_())