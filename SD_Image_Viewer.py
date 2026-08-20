"""
STM32 SD Card Snapshot Visualizer - Modern PySide6 Edition
"""

import sys
import os
import struct
import time
import subprocess
import threading
import queue
import cv2
import numpy as np

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QPushButton, QComboBox, QLabel, 
                               QListWidget, QGraphicsView, QGraphicsScene, 
                               QGraphicsPixmapItem, QMessageBox, QSplitter, 
                               QFileDialog, QInputDialog, QRadioButton, QButtonGroup, QFrame)
from PySide6.QtCore import Qt, QThread, Signal, Slot, QRectF
from PySide6.QtGui import QImage, QPixmap, QPainter, QAction, QIcon, QPalette, QColor

# ========================= CONFIG =========================
HEADER_SIZE = 64
BLOCK_SIZE = 512
HEADER_TAG = 0x49444745
FMT_NAME = {0: "YUV422", 1: "RGB565", 2: "GRAY8"}

# Fallback sizes if header is corrupt
SNAP_BLOCKS_FALLBACK = (HEADER_SIZE + (1296 * 972 * 2) + BLOCK_SIZE - 1) // BLOCK_SIZE

SNAP_BASE_NEW = 3072
SNAP_BASE_OLD = 1000

# How much of the card to zero-fill after a quick format, to destroy the old
# image headers in the region where photos are stored. 1024 MB holds ~100
# full-res images — far more than any realistic card fill. Change if needed.
SNAP_WIPE_MB = 1024

_gf = None
_gfd = None
_gpath = None

# Serializes every open/close/read of the shared drive handles. Without this,
# a second click (Scan / Refresh / Disconnect / image load) can swap the
# handle out from under a running thread and crash it.
_glock = threading.Lock()

# ========================= STYLESHEET =========================
# Modern, light-themed PyDracula-inspired UI
MODERN_QSS = """
QMainWindow {
    background-color: #f0f2f5;
}
QWidget {
    font-family: "Segoe UI", "Helvetica Neue", sans-serif;
    font-size: 13px;
    color: #333333;
}
/* Explicit dialog colors: native Windows dark theme must never leak in.
   Backgrounds come from the forced light palette; text/buttons are pinned here. */
QMessageBox {
    background-color: #ffffff;
}
QMessageBox QLabel {
    color: #111827;
    background: transparent;
}
QMessageBox QPushButton {
    background-color: #f3f4f6;
    color: #111827;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 6px 18px;
    font-weight: 600;
    min-width: 80px;
}
QMessageBox QPushButton:hover {
    background-color: #e5e7eb;
}
QInputDialog, QFileDialog {
    background-color: #ffffff;
    color: #111827;
}
QInputDialog QLabel, QFileDialog QLabel {
    color: #111827;
}
QInputDialog QLineEdit, QFileDialog QLineEdit {
    background-color: #ffffff;
    color: #111827;
    border: 1px solid #d1d5db;
    border-radius: 4px;
    padding: 4px;
    selection-background-color: #2563eb;
    selection-color: #ffffff;
}
QInputDialog QListView, QFileDialog QListView {
    background-color: #ffffff;
    alternate-background-color: #f3f4f6;
    color: #111827;
    border: 1px solid #e1e4e8;
}
QInputDialog QPushButton, QFileDialog QPushButton {
    background-color: #f3f4f6;
    color: #111827;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 5px 12px;
}
QInputDialog QPushButton:hover, QFileDialog QPushButton:hover {
    background-color: #e5e7eb;
}
QDialogButtonBox QPushButton {
    background-color: #f3f4f6;
    color: #111827;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 6px 18px;
    font-weight: 600;
    min-width: 80px;
}
QDialogButtonBox QPushButton:hover {
    background-color: #e5e7eb;
}
QStatusBar {
    background-color: #ffffff;
    color: #374151;
}
QStatusBar QLabel {
    color: #374151;
    background: transparent;
}
QToolTip {
    background-color: #ffffff;
    color: #111827;
    border: 1px solid #d1d5db;
    padding: 4px;
}
QFrame#ToolbarFrame {
    background-color: #ffffff;
    border-bottom: 1px solid #e1e4e8;
}
QFrame#ControlsFrame {
    background-color: #ffffff;
    border-top: 1px solid #e1e4e8;
    border-radius: 8px;
}
QPushButton {
    background-color: #f3f4f6;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 6px 12px;
    font-weight: 600;
    color: #374151;
}
QPushButton:hover {
    background-color: #e5e7eb;
}
QPushButton:pressed {
    background-color: #d1d5db;
}
QPushButton#PrimaryButton {
    background-color: #2563eb;
    color: white;
    border: none;
}
QPushButton#PrimaryButton:hover {
    background-color: #1d4ed8;
}
QPushButton#DangerButton {
    background-color: #dc2626;
    color: white;
    border: none;
}
QPushButton#DangerButton:hover {
    background-color: #b91c1c;
}
QPushButton#SuccessButton {
    background-color: #10b981;
    color: white;
    border: none;
}
QPushButton#SuccessButton:hover {
    background-color: #059669;
}
QComboBox {
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 5px;
    background-color: white;
}
QListWidget {
    background-color: white;
    border: 1px solid #e1e4e8;
    border-radius: 8px;
    padding: 5px;
}
QListWidget::item {
    padding: 8px;
    border-bottom: 1px solid #f3f4f6;
}
QListWidget::item:selected {
    background-color: #eff6ff;
    color: #1d4ed8;
    border-radius: 4px;
}
QGraphicsView {
    background-color: #ffffff; /* White background for image viewer */
    border: 1px solid #e1e4e8;
    border-radius: 8px;
}
"""

# ========================= DIALOG HELPER =========================
# Windows dark mode leaks into native QMessageBox (white text on dark bg,
# unreadable icons/buttons). Force a fixed light palette on every dialog
# so the app's look is independent of the OS theme.
DIALOG_QSS = """
QDialog, QMessageBox {
    background-color: #ffffff;
    color: #111827;
}
QDialog QLabel, QMessageBox QLabel {
    color: #111827;
    background: transparent;
    font-size: 13px;
}
QDialog QPushButton {
    background-color: #f3f4f6;
    color: #111827;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    padding: 6px 18px;
    font-weight: 600;
    min-width: 80px;
}
QDialog QPushButton:hover {
    background-color: #e5e7eb;
}
QDialog QPushButton:pressed {
    background-color: #d1d5db;
}
"""

def dialog(parent, kind, title, text):
    """Themed QMessageBox. `kind` = info | warning | critical | question.
    Always returns the QMessageBox.StandardButton result of exec()."""
    box = QMessageBox(parent)
    box.setStyleSheet(DIALOG_QSS)
    box.setWindowTitle(title)
    box.setText(text)
    box.setIcon({
        "info": QMessageBox.Icon.Information,
        "warning": QMessageBox.Icon.Warning,
        "critical": QMessageBox.Icon.Critical,
        "question": QMessageBox.Icon.Question,
    }[kind])
    if kind == "question":
        box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        box.setDefaultButton(QMessageBox.StandardButton.No)
    else:
        box.setStandardButtons(QMessageBox.StandardButton.Ok)
    return box.exec()

# ========================= I/O HELPERS =========================
def _open_source(path):
    """Open `path` for reading and cache the handle. Only called with
    _glock held. The path is cached ONLY after a successful open, so a
    failed open (busy card, renumbering, missing admin rights) is retried
    on the next call instead of poisoning the cache forever — the root
    cause of the old "'NoneType' object has no attribute 'seek'". """
    global _gf, _gfd, _gpath
    if _gpath == path and (_gf is not None or _gfd is not None):
        return
    _close_locked()
    _gpath = None
    if path.startswith('\\\\.\\PhysicalDrive'):
        flags = os.O_RDONLY | getattr(os, 'O_BINARY', 0)
        _gfd = os.open(path, flags)
    else:
        _gf = open(path, 'rb')
    _gpath = path

def _close_locked():
    """Close cached handles. Only called with _glock held."""
    global _gf, _gfd, _gpath
    if _gf:
        _gf.close()
        _gf = None
    if _gfd is not None:
        os.close(_gfd)
        _gfd = None
    _gpath = None

def close_drive():
    with _glock:
        _close_locked()

def _read_locked(path, offset, num_bytes):
    """Open if needed and read. Only called with _glock held.
    Returns (bytes, last_error)."""
    _open_source(path)
    try:
        if isinstance(_gfd, int):
            if hasattr(os, 'pread'):
                return os.pread(_gfd, num_bytes, offset), None
            os.lseek(_gfd, offset, os.SEEK_SET)
            return os.read(_gfd, num_bytes), None
        _gf.seek(offset, os.SEEK_SET)
        return _gf.read(num_bytes), None
    except Exception as e:
        return None, e

def _read_with_retry(path, offset, num_bytes, attempts=3, settle=0.4):
    """Thread-safe read of `num_bytes` at absolute `offset`. All reads of the
    shared handle are serialized under _glock; on failure the handle is
    re-opened and retried (cards can take a moment to settle after a
    format). Returns (bytes, first_error)."""
    last_err = None
    for i in range(attempts):
        with _glock:
            data, err = _read_locked(path, offset, num_bytes)
        if data is not None:
            return data, None
        last_err = err
        if i < attempts - 1:
            with _glock:
                _close_locked()
            time.sleep(settle)
    return None, last_err

def rblk(path, blk):
    data, _err = _read_with_retry(path, int(blk) * BLOCK_SIZE, BLOCK_SIZE)
    return data

def rbulk(path, start_blk, num_blks):
    data, _err = _read_with_retry(path, int(start_blk) * BLOCK_SIZE,
                                  int(num_blks) * BLOCK_SIZE)
    return data

def parse_hdr(b):
    if len(b) < HEADER_SIZE: return None
    head = struct.unpack('<8I', b[:32])
    return {
        'magic': head[0], 'width': head[1], 'height': head[2],
        'pixel_format': head[3], 'data_size': head[4],
        'timestamp': head[5], 'checksum': head[6], 'snap_id': head[7],
    }

# One PowerShell call lists every disk with number, size and friendly name.
# Unlike brute-force opening \\.\PhysicalDriveN, this does NOT open the
# devices, so a busy / just-formatted / re-enumerating card is still listed.
_DISK_INFO_PS = ('Get-Disk -ErrorAction SilentlyContinue | '
                 'ForEach-Object { "$($_.Number)|$($_.Size)|$($_.FriendlyName)" }')

def get_disk_info():
    """Return [{'num': int, 'size': bytes, 'friendly': str}, ...] for all disks.
    Falls back to brute-force open probing if PowerShell is unavailable."""
    infos = []
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command", _DISK_INFO_PS],
                           capture_output=True, text=True, timeout=30)
        for line in (r.stdout or '').splitlines():
            parts = line.strip().split('|')
            if len(parts) >= 3 and parts[0].strip().isdigit():
                try:
                    size = int(parts[1].strip() or 0)
                except ValueError:
                    size = 0
                infos.append({'num': int(parts[0].strip()),
                              'size': size,
                              'friendly': parts[2].strip()})
    except Exception:
        infos = []
    if not infos:  # fallback: open-probe each device
        for n in range(32):
            p = f'\\\\.\\PhysicalDrive{n}'
            try:
                with open(p, 'rb'):
                    infos.append({'num': n, 'size': 0, 'friendly': ''})
            except OSError:
                pass
    return infos

def get_drives():
    """Raw device paths of all non-system disks (backward-compatible API)."""
    return [f'\\\\.\\PhysicalDrive{d["num"]}' for d in get_disk_info() if d['num'] != 0]

def zero_fill(path, start_blk, num_blks):
    """Overwrite a block range with zeros directly on the physical drive.
    Used to 'delete' images (erases the header magic so the scanner skips it).
    Requires admin rights. Independent fd — does NOT touch the cached read handles."""
    flags = os.O_RDWR | getattr(os, 'O_BINARY', 0)
    fd = os.open(path, flags)
    try:
        chunk = b'\x00' * (64 * 1024)  # 64 KB write chunks
        offset = int(start_blk) * BLOCK_SIZE
        total = int(num_blks) * BLOCK_SIZE
        written = 0
        while written < total:
            n = min(len(chunk), total - written)
            os.lseek(fd, offset + written, os.SEEK_SET)
            os.write(fd, chunk[:n])
            written += n
    finally:
        os.close(fd)



# ==========================================
# ROBUST WHITE-BALANCE FOR STM32 CAMERA
# ==========================================
# Gray-world + percentile protection.
# Much more stable than pure fixed multipliers.

def apply_robust_white_balance(rgb_img, strength=0.5, stat_step=2):
    """
    Robust white balance for STM32 camera images.
    
    Strategy:
    1. Prefer the brightest pixels (white-patch) – works even when the
       scene is full of strong colors.
    2. Fall back to mid-tone gray-world only if there are almost no
       bright pixels.
    3. strength controls how aggressive the correction is (0.0–1.0).
    """
    if rgb_img is None or rgb_img.ndim != 3 or rgb_img.shape[2] != 3:
        return rgb_img

    img = rgb_img.astype(np.float32)

    # Gain statistics are computed on a strided subsample (4x fewer pixels).
    # The correction is global, so full resolution is only needed for the
    # final multiply — this removes the most expensive part of the decode.
    s = max(1, int(stat_step))
    sub = img[::s, ::s, :]
    lum = sub.mean(axis=2)

    # --- 1. White-patch: use the brightest 4 % of pixels -----------------
    threshold = np.percentile(lum, 96)
    bright_mask = lum >= threshold

    if bright_mask.sum() > 80:          # enough bright pixels
        means = sub[bright_mask].mean(axis=0)
    else:
        # --- 2. Fallback: classic mid-tone gray-world --------------------
        mid_mask = (lum > 40) & (lum < 210)
        if mid_mask.sum() < 100:
            return rgb_img              # image too extreme → leave it
        means = sub[mid_mask].mean(axis=0)

    means = np.maximum(means, 1.0)
    gray = means.mean()
    gains = gray / means

    # Soften the correction
    gains = 1.0 + (gains - 1.0) * strength

    # Apply + soft clip
    balanced = np.clip(img * gains, 0, 255)
    return balanced.astype(np.uint8)

def decode_image_data(img_data, width, height, pixel_format):
    """
    Convert raw YUV422 payload → QImage with robust white-balance.
    """
    if pixel_format != 0:
        return None
    if width <= 0 or height <= 0 or width > 4096 or height > 4096:
        return None

    expected_bytes = width * height * 2
    if len(img_data) < expected_bytes:
        return None

    try:
        arr = np.frombuffer(img_data, dtype=np.uint8, count=expected_bytes)
        arr = arr.reshape((height, width, 2))

        # One conversion pass (YUV422 -> RGB) instead of YUV->BGR->RGB
        rgb = cv2.cvtColor(arr, cv2.COLOR_YUV2RGB_YUY2)

        # Smart white-balance instead of fixed multipliers
        rgb = apply_robust_white_balance(rgb, strength=0.5)

        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        return qimg.copy()
    except Exception:
        return None



class Snapshot:
    def __init__(self, idx, block, header):
        self.idx = idx
        self.block = block
        self.header = header
        self.qimage = None

# ========================= THREADS =========================
class ScanThread(QThread):
    progress = Signal(str)
    finished = Signal(list)
    error = Signal(str)

    def __init__(self, drive):
        super().__init__()
        self.drive = drive
        self._cancel = False

    def cancel(self):
        """Asks the scan to stop at the next checkpoint (used by the
        'Cancel Scan' button). Safe to call from the GUI thread."""
        self._cancel = True

    def _cancelled(self):
        if self._cancel:
            return True
        try:
            if not self.isRunning():
                self._cancel = True
        except Exception:
            pass
        return self._cancel

    # Consecutive invalid headers tolerated before the stride scan gives up.
    # Generous on purpose: after a format done mid-run the first slots are
    # empty while valid images sit further back on the card.
    MAX_HEADER_GAPS = 30

    # Phase-2 "deep scan" parameters: read the card sequentially from the
    # first snapshot block and look for the header magic at every 512-byte
    # boundary. Fully layout-agnostic (no base/stride/resolution assumptions).
    DEEP_SCAN_MB = 512      # max area searched
    DEEP_CHUNK_MB = 4       # bytes per bulk read

    def validate_header(self, h):
        if not h or h.get('magic') != HEADER_TAG: return False
        w, hh = h.get('width', 0), h.get('height', 0)
        data_size = h.get('data_size', 0)
        if w == 0 or hh == 0 or w > 4096 or hh > 4096: return False
        
        # Bytes per pixel depends on the format (YUV422/RGB565 = 2, GRAY8 = 1).
        bpp = {0: 2, 1: 2, 2: 1}.get(h.get('pixel_format', 0), 2)
        expected = w * hh * bpp
        if expected > 0:
            tolerance = max(BLOCK_SIZE * 4, expected * 0.01)
            if abs(data_size - expected) > tolerance: return False
        return True

    def run(self):
        try:
            best_results = []
            first_err = None
            for snap_base, label in [(SNAP_BASE_NEW, "new"), (SNAP_BASE_OLD, "old")]:
                if self._cancelled():
                    return
                self.progress.emit(f"Scanning base {snap_base} ({label})...")

                probe = snap_base
                count = gaps = 0
                var_results = []

                # Variable stride scanning based on exact data_size
                while count < 300:
                    if self._cancelled():
                        return
                    raw, err = _read_with_retry(self.drive,
                                                probe * BLOCK_SIZE, BLOCK_SIZE)
                    if raw is None:
                        # Persistent read failure (card busy / no admin
                        # rights): remember it, stop trying this base.
                        if first_err is None:
                            first_err = err
                        break
                    if len(raw) < HEADER_SIZE:
                        break  # reached end of the readable card
                    h = parse_hdr(raw[:HEADER_SIZE])

                    if self.validate_header(h):
                        # Calculate exact blocks needed for this specific image
                        actual_nb = (HEADER_SIZE + h['data_size'] + BLOCK_SIZE - 1) // BLOCK_SIZE
                        if actual_nb < 1: actual_nb = SNAP_BLOCKS_FALLBACK

                        var_results.append(Snapshot(count, probe, h))
                        count += 1
                        gaps = 0
                        probe += actual_nb # Jump exactly to the next image
                    else:
                        gaps += 1
                        if gaps >= self.MAX_HEADER_GAPS: break
                        probe += SNAP_BLOCKS_FALLBACK

                if len(var_results) > len(best_results):
                    best_results = var_results

            if self._cancelled():
                return

            if best_results:
                self.finished.emit(best_results)
                return

            # Phase 2: the stride scan found nothing. This happens e.g. when the
            # card was formatted while the board kept running: the firmware kept
            # its snap_count, so the first slots from block 3072 are empty and
            # the real images start much further back. Fall back to a sequential
            # magic search.
            self.progress.emit("Stride scan found nothing - deep scanning first "
                               f"{self.DEEP_SCAN_MB} MB for header magic...")
            results, err = self._scan_magic()
            if results:
                self.finished.emit(results)
            elif first_err is not None:
                self.error.emit(str(first_err))
            elif err is not None:
                self.error.emit(str(err))
            else:
                self.finished.emit([])
        except Exception as e:
            self.error.emit(str(e))

    def _scan_magic(self):
        """Sequentially read the card from SNAP_BASE_NEW in DEEP_CHUNK_MB
        chunks and search for the little-endian header magic ('EGDI') at
        512-byte-aligned positions. Each hit is validated with the full
        header checks before being accepted."""
        results = []
        magic = struct.pack('<I', HEADER_TAG)  # b'EGDI'
        chunk_blocks = self.DEEP_CHUNK_MB * 1024 * 1024 // BLOCK_SIZE
        limit = SNAP_BASE_NEW + self.DEEP_SCAN_MB * 1024 * 1024 // BLOCK_SIZE
        err = None

        blk = SNAP_BASE_NEW
        while blk < limit and len(results) < 300:
            if self._cancelled():
                return results, err
            self.progress.emit(f"Deep scan: {blk * BLOCK_SIZE // (1024*1024)} / "
                               f"{limit * BLOCK_SIZE // (1024*1024)} MB ...")
            data, rerr = _read_with_retry(self.drive, blk * BLOCK_SIZE,
                                          chunk_blocks * BLOCK_SIZE)
            if data is None:
                # Persistent read failure — stop and report it.
                if err is None:
                    err = rerr
                break
            real_blocks = len(data) // BLOCK_SIZE
            if real_blocks < chunk_blocks:
                break  # reached end of readable area

            search_from = 0
            while search_from < len(data) - HEADER_SIZE:
                off = data.find(magic, search_from)
                if off == -1:
                    break
                if off % BLOCK_SIZE == 0:
                    h = parse_hdr(data[off:off + HEADER_SIZE])
                    if self.validate_header(h):
                        nb = (HEADER_SIZE + h['data_size'] + BLOCK_SIZE - 1) // BLOCK_SIZE
                        results.append(Snapshot(len(results), blk + off // BLOCK_SIZE, h))
                        search_from = off + nb * BLOCK_SIZE  # skip image body
                        continue
                search_from = off + 1
            blk += real_blocks

        return results, err

class LoadImageThread(QThread):
    # (qimage, snapshot, seq) — the GUI drops stale results (the user
    # clicked another row while this one was loading). Deliberately NOT
    # named `finished`/`error`: shadowing the built-in QThread signals is
    # fragile.
    image_ready = Signal(QImage, object, int)
    load_failed = Signal(object, int, str)   # (snapshot, seq, message)

    def __init__(self, drive, snapshot, seq):
        super().__init__()
        self.drive = drive
        self.snapshot = snapshot
        self.seq = seq

    def run(self):
        h = self.snapshot.header
        w = h['width']
        hh = h['height']
        data_size = h['data_size']
        expected_size = w * hh * 2 if h.get('pixel_format', 0) == 0 else w * hh

        if data_size <= 0 or abs(data_size - expected_size) > (BLOCK_SIZE * 4):
            data_size = expected_size

        nb = (HEADER_SIZE + data_size + BLOCK_SIZE - 1) // BLOCK_SIZE

        try:
            all_data = rbulk(self.drive, self.snapshot.block, nb)
            if not all_data or len(all_data) < nb * BLOCK_SIZE:
                self.load_failed.emit(self.snapshot, self.seq,
                                      "Incomplete read from SD card.")
                return

            img_data = bytes(all_data[HEADER_SIZE:HEADER_SIZE + data_size])

            # THIS is the only path that applies white-balance
            qimg = decode_image_data(img_data, w, hh, h.get('pixel_format', 0))

            if qimg is not None:
                self.image_ready.emit(qimg, self.snapshot, self.seq)
            else:
                self.load_failed.emit(
                    self.snapshot, self.seq,
                    f"Cannot decode image (format={h.get('pixel_format')}, "
                    f"size={w}x{hh}, data_len={len(img_data)})")
        except Exception as e:
            self.load_failed.emit(self.snapshot, self.seq, str(e))

class ExtractThread(QThread):
    """Bulk-read the selected images and save them as PNG files."""
    progress = Signal(str)
    finished_ok = Signal(str)
    failed = Signal(str)

    def __init__(self, drive, snapshots, out_dir):
        super().__init__()
        self.drive = drive
        self.snapshots = snapshots
        self.out_dir = out_dir
        self._cancel = False

    def cancel(self):
        self._cancel = True

    def run(self):
        total = len(self.snapshots)
        saved = 0
        try:
            for i, snap in enumerate(self.snapshots, 1):
                if self._cancel:
                    self.failed.emit(f"Cancelled after saving {saved} image(s).")
                    return
                self.progress.emit(f"Extracting {i}/{total}: #{snap.idx+1:03d} ...")
                h = snap.header
                w, hh = h['width'], h['height']
                data_size = h['data_size']
                expected_size = w * hh * 2
                if data_size <= 0 or abs(data_size - expected_size) > (BLOCK_SIZE * 4):
                    data_size = expected_size
                nb = (HEADER_SIZE + data_size + BLOCK_SIZE - 1) // BLOCK_SIZE

                all_data = rbulk(self.drive, snap.block, nb)
                if not all_data or len(all_data) < nb * BLOCK_SIZE:
                    self.failed.emit(
                        f"Short read for #{snap.idx+1:03d}:\ngot {len(all_data)} bytes, expected {nb*BLOCK_SIZE}.")
                    return

                qimg = decode_image_data(
                    bytes(all_data[HEADER_SIZE:HEADER_SIZE + data_size]), w, hh, h['pixel_format'])
                if qimg is None:
                    self.failed.emit(f"Cannot decode #{snap.idx+1:03d} (pixel format {h['pixel_format']}).")
                    return

                ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(h.get('timestamp', 0)))
                path = os.path.join(self.out_dir, f"snapshot_{snap.idx:03d}_{w}x{hh}_{ts}.png")
                if not qimg.save(path):
                    self.failed.emit(f"Failed to save:\n{path}")
                    return
                saved += 1
            self.finished_ok.emit(f"{saved} image(s) saved to:\n{self.out_dir}")
        except Exception as e:
            self.failed.emit(str(e))

class DeleteThread(QThread):
    """Erase selected images by zero-filling their fixed block ranges."""
    progress = Signal(str)
    finished_ok = Signal(int)
    failed = Signal(str)

    def __init__(self, drive, snapshots):
        super().__init__()
        self.drive = drive
        self.snapshots = snapshots
        self._cancel = False

    def cancel(self):
        self._cancel = True

    def run(self):
        snaps = sorted(self.snapshots, key=lambda s: s.block, reverse=True)
        total = len(snaps)
        try:
            for i, snap in enumerate(snaps, 1):
                if self._cancel:
                    self.failed.emit("Cancelled — the remaining images were not erased.")
                    return
                h = snap.header
                data_size = h['data_size']
                expected_size = h['width'] * h['height'] * 2
                if data_size <= 0 or abs(data_size - expected_size) > (BLOCK_SIZE * 4):
                    data_size = expected_size
                nb = (HEADER_SIZE + data_size + BLOCK_SIZE - 1) // BLOCK_SIZE
                self.progress.emit(
                    f"Erasing {i}/{total}: #{snap.idx+1:03d} @ block {snap.block} "
                    f"({nb*BLOCK_SIZE//(1024*1024)} MB) ...")
                zero_fill(self.drive, snap.block, nb)
            self.finished_ok.emit(total)
        except Exception as e:
            self.failed.emit(str(e))

class FormatThread(QThread):
    """Format the physical drive as FAT32, fast and live:

    Step 1 — QUICK format (Format-Volume -Quick, seconds). This is what makes
             the card a clean volume; it does NOT zero the data area, which is
             why a full -Full format used to appear 'stuck' for a long time.
    Step 2 — Wipe the photo-storage region (block 3072 .. SNAP_WIPE_MB) with
             real zero-filling and a REAL percent progress. This destroys the
             old image headers, so a scan afterwards shows 0 photos — the same
             end result a full format gives, in a fraction of the time.
    Step 3 — Verify (FAT32 boot signature + empty snapshot area) and report.

    The drive number can change after format (USB re-enumeration), so the card
    is re-located by its size before the wipe. Cancellable at any point.
    """
    progress = Signal(str)
    success = Signal(str, list)  # (message, fresh drive list)
    error = Signal(str, list)    # (message, fresh drive list)

    HEARTBEAT_SECS = 20
    WIPE_CHUNK = 1024 * 1024     # 1 MB zero chunks for the wipe

    def __init__(self, drive_num):
        super().__init__()
        self.drive_num = drive_num
        self._cancel = False

    def cancel(self):
        self._cancel = True

    @staticmethod
    def _all_disk_sizes():
        """{disk_number: size_bytes} from a single Get-Disk call."""
        return {d['num']: d['size'] for d in get_disk_info()}

    def _find_by_size(self, size_hint):
        """Re-locate a disk by size (survives USB renumbering)."""
        if not size_hint:
            return None
        sizes = self._all_disk_sizes()
        for num, sz in sizes.items():
            if num != 0 and sz == size_hint:
                return num
        return None

    def _run_live(self, argv, label, shell=False):
        """Run a long command, streaming its output lines + silence heartbeats.
        Returns (returncode, output_tail). Aborts if cancel() was called."""
        proc = subprocess.Popen(argv, shell=shell, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True,
                                errors='replace')
        lines_q = queue.Queue()

        def _reader(stream):
            try:
                for line in stream:
                    line = line.rstrip()
                    if line:
                        lines_q.put(line)
            except Exception:
                pass
            finally:
                try:
                    stream.close()
                except Exception:
                    pass

        threads = [threading.Thread(target=_reader, args=(s,), daemon=True)
                   for s in (proc.stdout, proc.stderr)]
        for t in threads:
            t.start()

        start = time.time()
        last_beat = 0.0
        tail = []
        try:
            while True:
                if self._cancel:
                    break
                try:
                    line = lines_q.get(timeout=1.0)
                    tail.append(line)
                    if len(tail) > 8:
                        tail.pop(0)
                    self.progress.emit(f"{label}: {line[:200]}")
                except queue.Empty:
                    now = time.time()
                    if now - last_beat >= self.HEARTBEAT_SECS:
                        last_beat = now
                        mm, ss = divmod(int(now - start), 60)
                        self.progress.emit(
                            f"{label} still running — elapsed {mm:02d}:{ss:02d} "
                            "(full format zeroes the whole card — do NOT unplug; "
                            "Cancel to abort)")
                if proc.poll() is not None and lines_q.empty():
                    break
        finally:
            if proc.poll() is None:
                proc.kill()
            rc = proc.wait()
            for t in threads:
                t.join(timeout=2)
        return rc, "\n".join(tail)

    def _verify(self, size_hint):
        """Post-format verification: re-locate the drive (its number can change
        after USB re-enumeration), check the FAT boot sector and confirm the
        snapshot area is truly empty. Returns (message, fresh drive list)."""
        time.sleep(3)  # let Windows settle the USB re-enumeration
        drives = get_drives()
        target = self._find_by_size(size_hint)
        if target is None and drives:
            target = int(drives[0].replace('\\\\.\\PhysicalDrive', ''))
        if target is None:
            return ("Format finished, but the card could not be re-located for "
                    "verification — check it in Windows Explorer.", drives)

        parts = [f"PhysicalDrive{target} formatted as FAT32 (quick)"]
        pdrv = f'\\\\.\\PhysicalDrive{target}'
        try:
            boot = rblk(pdrv, 0)
            if boot and len(boot) >= 512 and boot[510] == 0x55 and boot[511] == 0xAA:
                label = boot[71:82].split(b'\x00')[0].decode('ascii', 'replace')
                parts.append("boot sector OK" + (f" (label='{label}')" if label else ""))
            else:
                parts.append("WARNING: no FAT boot signature at block 0")
            snap = rblk(pdrv, SNAP_BASE_NEW)
            if snap and not any(snap):
                parts.append("snapshot area verified EMPTY (0 old images)")
            elif snap:
                h = parse_hdr(snap[:HEADER_SIZE])
                if h and h.get('magic') == HEADER_TAG:
                    parts.append("WARNING: an old image header is still present — "
                                 "run Format again")
                else:
                    parts.append("snapshot area holds no valid image headers")
        except Exception as e:
            parts.append(f"verification read failed: {e}")
        return "; ".join(parts), drives

    def run(self):
        try:
            # Release our cached read handle before Windows re-initializes the device
            close_drive()
            size_hint = self._all_disk_sizes().get(self.drive_num)

            # ---------- STEP 1: quick FAT32 format (seconds) ----------
            self.progress.emit(f"Step 1/3: quick FAT32 format of "
                               f"PhysicalDrive{self.drive_num} (a few seconds)...")
            subprocess.run(f"mountvol {self.drive_num} /p", shell=True,
                           capture_output=True)

            formatted = False
            tail = ""
            ps = (
                f"$vol = Get-Disk -Number {self.drive_num} | Get-Partition | Get-Volume; "
                f"if ($vol) {{ Format-Volume -DriveLetter $vol.DriveLetter -FileSystem FAT32 "
                f"-NewFileSystemLabel SD_CARD -Quick -Force -Confirm:$false -ErrorAction Stop }} "
                f"else {{ Format-Volume -DiskNumber {self.drive_num} -FileSystem FAT32 "
                f"-NewFileSystemLabel SD_CARD -Quick -Force -Confirm:$false -ErrorAction Stop }}"
            )
            rc, tail = self._run_live(["powershell", "-NoProfile", "-Command", ps], "fmt")
            if not self._cancel and rc == 0:
                formatted = True
            else:
                # Fallback: format.exe quick format via the volume letter
                self.progress.emit("PowerShell quick format failed — trying "
                                   "format.exe fallback...")
                letter = ""
                try:
                    r = subprocess.run(
                        ["powershell", "-NoProfile", "-Command",
                         f"(Get-Disk -Number {self.drive_num} | Get-Partition | Get-Volume).DriveLetter"],
                        capture_output=True, text=True, timeout=60)
                    letter = r.stdout.strip()
                except Exception:
                    pass
                if self._cancel or not letter:
                    self.error.emit(f"Format error (cannot locate a volume to "
                                    f"format):\n{tail}", get_drives())
                    return
                rc2, tail2 = self._run_live(f"format {letter}: /FS:FAT32 /Q /Y /V:SD_CARD",
                                            "fmt", shell=True)
                if self._cancel:
                    self.error.emit("Format CANCELLED by user during the fallback.",
                                    get_drives())
                    return
                if rc2 != 0:
                    self.error.emit(f"Format error:\n{tail}\n{tail2}", get_drives())
                    return
                formatted = True

            if not formatted:
                self.error.emit("Format CANCELLED by user. Run Format again to finish.",
                                get_drives())
                return

            # ---------- STEP 2: wipe the photo-storage region ----------
            target = self._find_by_size(size_hint) or self.drive_num
            self.progress.emit(f"Step 2/3: wiping old photo data "
                               f"({SNAP_WIPE_MB} MB) on PhysicalDrive{target}...")
            path = f'\\\\.\\PhysicalDrive{target}'
            flags = os.O_RDWR | getattr(os, 'O_BINARY', 0)
            fd = None
            for _attempt in range(15):  # the card can take a few seconds to settle
                if self._cancel:
                    break
                try:
                    fd = os.open(path, flags)
                    break
                except OSError:
                    time.sleep(1.0)
            if fd is None and target != self.drive_num:
                fd = os.open(f'\\\\.\\PhysicalDrive{self.drive_num}', flags)
            if fd is None:
                self.error.emit("Format done, but the card could not be opened "
                                "for the photo-area wipe. Run Format again.",
                                get_drives())
                return

            wipe_block = SNAP_BASE_NEW
            total = SNAP_WIPE_MB * 1024 * 1024
            chunk = b'\x00' * self.WIPE_CHUNK
            written = 0
            last_pct = -1
            try:
                while written < total:
                    if self._cancel:
                        break
                    os.lseek(fd, wipe_block * BLOCK_SIZE + written, os.SEEK_SET)
                    os.write(fd, chunk)
                    written += len(chunk)
                    pct = written * 100 // total
                    if pct >= last_pct + 2:  # update every 2%
                        last_pct = pct
                        self.progress.emit(
                            f"Step 2/3: wiping old photo data — {pct}% "
                            f"({written // (1024*1024)}/{SNAP_WIPE_MB} MB)...")
            finally:
                os.close(fd)

            if self._cancel:
                self.error.emit("Wipe CANCELLED by user. The volume is formatted "
                                "but old photos may still be visible — run "
                                "Format again to finish the wipe.", get_drives())
                return

            # ---------- STEP 3: verify ----------
            self.progress.emit("Step 3/3: verifying...")
            msg, drives = self._verify(size_hint)
            self.success.emit("Format complete (quick FAT32 + photo-area wipe). " + msg,
                              drives)
        except Exception as e:
            self.error.emit(f"Unexpected error during format:\n{e}", get_drives())

class DriveListThread(QThread):
    """Enumerate the disks (PowerShell Get-Disk) off the GUI thread so the
    window opens instantly instead of waiting 2-5 s for the drive list."""
    finished = Signal(list)

    def run(self):
        try:
            self.finished.emit(get_disk_info())
        except Exception:
            self.finished.emit([])

# ========================= GUI =========================
class InteractiveGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.image_item = QGraphicsPixmapItem()
        self.image_item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        self.scene.addItem(self.image_item)
        
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        
        self.current_scale = 1.0

    def set_image(self, qimage):
        # Reuse the persistent item (no scene.clear()) — cheaper and
        # flicker-free when switching between images.
        self.image_item.setPixmap(QPixmap.fromImage(qimage))
        self.scene.setSceneRect(self.image_item.boundingRect())
        self.fit_to_window()

    def fit_to_window(self):
        if not self.image_item.pixmap().isNull():
            self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
            self.current_scale = self.transform().m11()

    def set_explicit_scale(self, scale_factor):
        if not self.image_item.pixmap().isNull():
            self.resetTransform()
            self.scale(scale_factor, scale_factor)
            self.current_scale = scale_factor

    def wheelEvent(self, event):
        zoom_in_factor = 1.15
        zoom_out_factor = 1.0 / zoom_in_factor
        
        if not self.image_item.pixmap().isNull():
            if event.angleDelta().y() > 0:
                self.scale(zoom_in_factor, zoom_in_factor)
            else:
                self.scale(zoom_out_factor, zoom_out_factor)
            self.current_scale = self.transform().m11()


class SDVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 SD Card Snapshot Visualizer")
        self.resize(1300, 850)

        self.drive = None
        self.snapshots = []
        self.current_snap = None
        self._drive_list_thread = None
        # Monotonic request counter: every image load gets a seq, results
        # carrying an older seq are dropped (user already moved on).
        self._load_seq = 0
        # Strong refs to worker QThreads until they finish. A second fast
        # click used to reassign self.load_thread and let the previous
        # thread be garbage-collected WHILE STILL RUNNING -> "QThread:
        # Destroyed while thread is still running" (the crash).
        self._bg_threads = set()

        self._build_ui()
        # Enumerate drives in the background so the window appears instantly
        # (the Get-Disk PowerShell call can take a couple of seconds).
        self.request_drive_list()

    def _build_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # --- Top Toolbar ---
        toolbar = QFrame()
        toolbar.setObjectName("ToolbarFrame")
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(15, 10, 15, 10)

        toolbar_layout.addWidget(QLabel("<b>Drive:</b>"))
        self.drive_combo = QComboBox()
        self.drive_combo.setMinimumWidth(300)
        # itemData holds the raw device path; the visible text is a friendly
        # label (size + name) so renumbering after a format is recognizable.
        self.drive_combo.currentIndexChanged.connect(self.set_drive)
        toolbar_layout.addWidget(self.drive_combo)

        self.btn_refresh = QPushButton("↻ Refresh")
        self.btn_refresh.clicked.connect(self.request_drive_list)
        toolbar_layout.addWidget(self.btn_refresh)

        self.btn_scan = QPushButton("Scan Snapshots")
        self.btn_scan.setObjectName("PrimaryButton")
        self.btn_scan.clicked.connect(self.scan_snapshots)
        toolbar_layout.addWidget(self.btn_scan)

        self.btn_cancel_scan = QPushButton("✖ Cancel Scan")
        self.btn_cancel_scan.setObjectName("DangerButton")
        self.btn_cancel_scan.clicked.connect(self.cancel_scan)
        self.btn_cancel_scan.hide()
        toolbar_layout.addWidget(self.btn_cancel_scan)

        toolbar_layout.addSpacing(20)

        self.btn_extract = QPushButton("Extract Selected")
        self.btn_extract.setObjectName("SuccessButton")
        self.btn_extract.clicked.connect(self.extract_selected)
        toolbar_layout.addWidget(self.btn_extract)

        self.btn_delete = QPushButton("🗑 Delete Selected")
        self.btn_delete.setObjectName("DangerButton")
        self.btn_delete.clicked.connect(self.delete_selected)
        toolbar_layout.addWidget(self.btn_delete)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self.disconnect_drive)
        toolbar_layout.addWidget(self.btn_disconnect)

        toolbar_layout.addStretch()

        #self.btn_format = QPushButton("⚠️ Format SD")
        #self.btn_format.setObjectName("DangerButton")
        #self.btn_format.clicked.connect(self.format_card)
        #toolbar_layout.addWidget(self.btn_format)

        self.btn_cancel_fmt = QPushButton("✖ Cancel Format")
        self.btn_cancel_fmt.setObjectName("DangerButton")
        self.btn_cancel_fmt.clicked.connect(self.cancel_format)
        self.btn_cancel_fmt.hide()
        toolbar_layout.addWidget(self.btn_cancel_fmt)

        main_layout.addWidget(toolbar)

        # --- Main Splitter ---
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setContentsMargins(10, 10, 10, 10)
        main_layout.addWidget(splitter, 1)

        # Left Panel (List)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        title_lbl = QLabel("Snapshots")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; margin-bottom: 5px;")
        left_layout.addWidget(title_lbl)
        
        self.listbox = QListWidget()
        self.listbox.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.listbox.currentRowChanged.connect(self.on_select)
        left_layout.addWidget(self.listbox)
        splitter.addWidget(left_widget)

        # Right Panel (Viewer + Controls)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.viewer = InteractiveGraphicsView()
        right_layout.addWidget(self.viewer, 1)

        # Bottom Controls
        controls = QFrame()
        controls.setObjectName("ControlsFrame")
        ctrl_layout = QVBoxLayout(controls)
        
        scale_lbl = QLabel("<b>Image Scale</b>")
        ctrl_layout.addWidget(scale_lbl)

        scale_opts_layout = QHBoxLayout()
        self.scale_group = QButtonGroup(self)
        
        scales = [("Fit to Window", 0), ("0.25", 0.25), ("0.5", 0.5), 
                  ("0.75", 0.75), ("1.0", 1.0), ("1.5", 1.5), ("2.0", 2.0)]
        
        for i, (text, val) in enumerate(scales):
            rb = QRadioButton(text)
            if i == 0: rb.setChecked(True)
            self.scale_group.addButton(rb, i)
            scale_opts_layout.addWidget(rb)
            
            # Use lambda to capture the value
            if val == 0:
                rb.clicked.connect(self.viewer.fit_to_window)
            else:
                rb.clicked.connect(lambda checked, v=val: self.viewer.set_explicit_scale(v))
                
        scale_opts_layout.addStretch()
        ctrl_layout.addLayout(scale_opts_layout)

        save_layout = QHBoxLayout()
        btn_save_orig = QPushButton("💾 Save Original")
        btn_save_orig.setObjectName("SuccessButton")
        btn_save_orig.clicked.connect(lambda: self.save_image(scale_mode="original"))
        save_layout.addWidget(btn_save_orig)

        btn_save_scaled = QPushButton("💾 Save at Current Scale")
        btn_save_scaled.clicked.connect(lambda: self.save_image(scale_mode="scaled"))
        save_layout.addWidget(btn_save_scaled)
        
        save_layout.addStretch()
        ctrl_layout.addLayout(save_layout)

        right_layout.addWidget(controls)
        splitter.addWidget(right_widget)
        
        splitter.setSizes([350, 950])

        # Status Bar
        self.statusBar().showMessage("Ready — run as Administrator for physical-disk access.")

    # --- Logic Methods ---
    def refresh_drives(self, disk_infos=None):
        """Pure UI update from a disk list (see DriveListThread). Drives are
        listed via Get-Disk (no device opens), so a busy or just-reformatted
        card cannot 'disappear'. The previously selected drive is re-selected
        by its raw path."""
        previous = self.drive
        self._load_seq += 1  # invalidate any in-flight image loads
        close_drive()
        self.drive_combo.blockSignals(True)
        self.drive_combo.clear()
        sel_idx = 0
        for d in (disk_infos if disk_infos is not None else []):
            if d['num'] == 0:
                continue
            path = f"\\\\.\\PhysicalDrive{d['num']}"
            if d['size'] > 0:
                gb = d['size'] / (1024 ** 3)
                size_str = (f"{gb:.0f} GB" if gb >= 1 else f"{d['size']/(1024**2):.0f} MB")
            else:
                size_str = "?"
            label = f"PhysicalDrive{d['num']}  ({size_str}"
            if d['friendly']:
                label += f", {d['friendly']}"
            label += ")"
            self.drive_combo.addItem(label, path)
            if path == previous:
                sel_idx = self.drive_combo.count() - 1
        self.drive_combo.blockSignals(False)
        if self.drive_combo.count() > 0:
            self.drive_combo.setCurrentIndex(sel_idx)
            self.drive = self.drive_combo.itemData(sel_idx)
        else:
            self.drive = None

    def set_drive(self, index):
        if index is not None and index >= 0:
            self.drive = self.drive_combo.itemData(index)
            self._load_seq += 1  # invalidate any in-flight image loads
            close_drive()  # invalidate the cached handle for the previous drive
        else:
            self.drive = None
            self._load_seq += 1

    # --- Drive listing (background) + UI busy-state helpers ---
    def request_drive_list(self):
        """Asynchronously (re)enumerate disks in the background so the window
        opens immediately; the combo box fills in when the list arrives."""
        self._set_ui_busy(True, wait_drives=True)
        self.statusBar().showMessage("Detecting drives...")
        self._drive_list_thread = DriveListThread()
        self._drive_list_thread.finished.connect(self._on_drives_listed)
        self._track_bg(self._drive_list_thread, self._drive_list_thread.finished)
        self._drive_list_thread.start()

    def _track_bg(self, th, *done_sigs):
        """Hold a strong reference to a worker QThread until one of its
        completion signals fires, so it is never garbage-collected while it
        is still running (the root cause of the crash on rapid clicking)."""
        self._bg_threads.add(th)
        for sig in done_sigs:
            sig.connect(lambda *a, t=th: self._bg_threads.discard(t))

    @Slot(list)
    def _on_drives_listed(self, infos):
        self._set_ui_busy(False)
        self.refresh_drives(infos)
        self.statusBar().showMessage(
            f"{self.drive_combo.count()} removable disk(s) detected — "
            "run as Administrator for physical-disk access.")

    def _op_running(self):
        for name in ("scan_thread", "extract_thread", "delete_thread",
                     "format_thread", "_drive_list_thread"):
            t = getattr(self, name, None)
            if t is not None and t.isRunning():
                return True
        return False

    def _set_ui_busy(self, busy, wait_drives=False):
        """Enable/disable the drive-dependent buttons. While wait_drives=True
        the combo box and Refresh are also locked (the drive list is being
        (re)built, so a stale selection must not trigger set_drive)."""
        for b in (self.btn_scan, self.btn_extract, self.btn_delete,
                  self.btn_disconnect):
            b.setEnabled(not busy)
        self.drive_combo.setEnabled(not busy and not wait_drives)
        self.btn_refresh.setEnabled(not busy and not wait_drives)
        scanning = (getattr(self, "scan_thread", None) is not None
                    and self.scan_thread.isRunning())
        self.btn_cancel_scan.setVisible(scanning)

    def disconnect_drive(self):
        close_drive()
        self.statusBar().showMessage("Drive handle closed — it reopens on next use.")

    def scan_snapshots(self):
        if not self.drive:
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return
        self.statusBar().showMessage("Scanning SD Card using variable stride...")
        self._load_seq += 1
        self.current_snap = None
        self.listbox.clear()
        self.snapshots.clear()
        self._set_ui_busy(True)
        self.btn_scan.setEnabled(False)
        self.btn_cancel_scan.setVisible(True)

        self.scan_thread = ScanThread(self.drive)
        self.scan_thread.progress.connect(lambda msg: self.statusBar().showMessage(msg))
        self.scan_thread.finished.connect(self._on_scan_finished)
        self.scan_thread.error.connect(self._on_scan_error)
        self.scan_thread.start()

    def cancel_scan(self):
        th = getattr(self, "scan_thread", None)
        if th is not None and th.isRunning():
            th.cancel()
            self.statusBar().showMessage("Cancelling scan...")

    @Slot(list)
    def _on_scan_finished(self, results):
        self._set_ui_busy(False)
        self.snapshots = results
        self.current_snap = None
        for s in self.snapshots:
            fmt = FMT_NAME.get(s.header['pixel_format'], "?")
            ts = time.strftime("%Y-%m-%d %H:%M", time.localtime(s.header.get('timestamp', 0)))
            self.listbox.addItem(f"#{s.idx+1:03d} | {s.header['width']}x{s.header['height']} | {fmt} | {ts}")

        self.statusBar().showMessage(f"Scan complete. Found {len(self.snapshots)} snapshots.")

    @Slot(str)
    def _on_scan_error(self, msg):
        self._set_ui_busy(False)
        hint = ""
        low = msg.lower()
        if "denied" in low or "permission" in low or "winerror 5" in low:
            hint = ("\n\nThe card could not be opened for raw reading.\n"
                    "Make sure you are running this program AS ADMINISTRATOR "
                    "and that no other program (Explorer, antivirus) has it locked.")
        dialog(self, "critical", "Scan failed",
               f"Could not read the SD card:\n{msg}{hint}")

    def on_select(self, row):
        if row < 0 or row >= len(self.snapshots): return
        # While a scan / extract / delete / format is running, the card is
        # busy and its block map may be changing — ignore list clicks so we
        # never read a shared handle out from under another thread.
        if any(getattr(self, n, None) is not None and getattr(self, n, None).isRunning()
               for n in ("scan_thread", "extract_thread", "delete_thread", "format_thread")):
            return
        self.current_snap = self.snapshots[row]

        if self.current_snap.qimage:
            self.viewer.set_image(self.current_snap.qimage)
            self.statusBar().showMessage("Loaded from cache.")
            return

        self._load_seq += 1
        snap = self.current_snap
        self.statusBar().showMessage(f"Loading {snap.header['width']}x{snap.header['height']} image...")
        th = LoadImageThread(self.drive, snap, self._load_seq)
        th.image_ready.connect(self._on_image_ready)
        th.load_failed.connect(self._on_load_failed)
        self._track_bg(th, th.image_ready, th.load_failed, th.finished)
        th.start()

    @Slot(QImage, object, int)
    def _on_image_ready(self, qimg, snap, seq):
        # Stale-result guard: apply only if this is the latest request AND
        # that same image is still the selected one. The decoded pixels are
        # still cached on `snap`, so a later re-click is instant.
        if seq != self._load_seq or self.current_snap is not snap:
            return
        snap.qimage = qimg
        self.viewer.set_image(qimg)

        # Reset scale radio buttons to "Fit"
        self.scale_group.button(0).setChecked(True)
        self.statusBar().showMessage("Image loaded.")

    @Slot(object, int, str)
    def _on_load_failed(self, snap, seq, msg):
        if seq != self._load_seq or self.current_snap is not snap:
            return
        self.statusBar().showMessage(f"Error: {msg}")

    def save_image(self, scale_mode="original"):
        if not self.current_snap or not self.current_snap.qimage:
            QMessageBox.warning(self, "Warning", "Select an image first.")
            return

        out_dir = QFileDialog.getExistingDirectory(
            self, "Select Output Directory", options=QFileDialog.Option.DontUseNativeDialog)
        if not out_dir: return

        try:
            filename = f"snapshot_{self.current_snap.idx:03d}_{scale_mode}.png"
            path = os.path.join(out_dir, filename)
            
            img_to_save = self.current_snap.qimage

            if scale_mode == "scaled":
                # Create a QImage of the scaled, visible area (or scaled entire image)
                scale = self.viewer.current_scale
                new_w = int(img_to_save.width() * scale)
                new_h = int(img_to_save.height() * scale)
                
                # Smooth scaling directly through QImage
                img_to_save = img_to_save.scaled(new_w, new_h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
                
                # Apply sharpening if scale > 1.1 using basic convolution
                if scale > 1.1:
                    img_to_save = self._apply_sharpening(img_to_save)

            if img_to_save.save(path):
                dialog(self, "info", "Success", f"Saved to:\n{path}")
            else:
                dialog(self, "critical", "Error", "Failed to save image.")
        except Exception as e:
            dialog(self, "critical", "Error", str(e))

    def _apply_sharpening(self, qimage):
        """Applies a lightweight sharpening kernel to emulate ImageEnhance.Sharpness"""
        # Convert QImage to CV2
        img_format = qimage.format()
        qimage = qimage.convertToFormat(QImage.Format.Format_RGB888)
        ptr = qimage.constBits()
        arr = np.array(ptr).reshape(qimage.height(), qimage.width(), 3)
        
        # Define unsharp mask kernel
        kernel = np.array([[0, -0.5, 0], 
                           [-0.5, 3, -0.5], 
                           [0, -0.5, 0]])
        sharpened = cv2.filter2D(arr, -1, kernel)
        
        # Convert back
        h, w, ch = sharpened.shape
        return QImage(sharpened.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def extract_selected(self):
        rows = self.listbox.selectedItems()
        if not rows:
            dialog(self, "warning", "Warning", "Select one or more images first.")
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return
        snaps = [self.snapshots[self.listbox.row(i)] for i in rows]
        out_dir = QFileDialog.getExistingDirectory(
            self, "Select Output Directory", options=QFileDialog.Option.DontUseNativeDialog)
        if not out_dir:
            return

        self.extract_thread = ExtractThread(self.drive, snaps, out_dir)
        self.extract_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.extract_thread.finished_ok.connect(self._on_extract_ok)
        self.extract_thread.failed.connect(self._on_extract_failed)
        self.extract_thread.start()
        self._set_ui_busy(True)

    def _on_extract_ok(self, msg):
        self._set_ui_busy(False)
        self.statusBar().showMessage("Extraction complete.")
        dialog(self, "info", "Success", msg)

    def _on_extract_failed(self, msg):
        self._set_ui_busy(False)
        self.statusBar().showMessage("Extraction failed.")
        dialog(self, "critical", "Error", msg)

    def delete_selected(self):
        rows = self.listbox.selectedItems()
        if not rows:
            dialog(self, "warning", "Warning", "Select one or more images first.")
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return
        snaps = [self.snapshots[self.listbox.row(i)] for i in rows]
        total_mb = sum(
            (HEADER_SIZE + s.header['data_size']) // (1024 * 1024) for s in snaps)

        reply = dialog(self, "question", "Delete Selected",
            f"Erasing {len(snaps)} image(s) (~{total_mb} MB) by zero-filling their block ranges.\n"
            "They will no longer be found by the scanner.\n\nContinue?")
        if reply != QMessageBox.StandardButton.Yes:
            return

        self.delete_thread = DeleteThread(self.drive, snaps)
        self.delete_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.delete_thread.finished_ok.connect(self._on_delete_ok)
        self.delete_thread.failed.connect(self._on_delete_failed)
        self.delete_thread.start()
        self._set_ui_busy(True)

    def _on_delete_ok(self, count):
        self._set_ui_busy(False)
        self.statusBar().showMessage(f"{count} image(s) erased. Re-scanning...")
        self.scan_snapshots()

    def _on_delete_failed(self, msg):
        self._set_ui_busy(False)
        self.statusBar().showMessage("Delete failed.")
        dialog(self, "critical", "Error", msg)

    def format_card(self):
        if not self.drive:
            dialog(self, "warning", "Error", "Select a drive first.")
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return
        drive_num = self.drive.replace('\\\\.\\PhysicalDrive', '')

        reply = dialog(self, "question", "⚠️ FORMAT SD CARD",    f"Are you SURE you want to format the PhysicalDrive{drive_num} drive as FAT32?\n"
        "This action will DELETE ALL data!\nOnly continue if you are 100% sure.")

        if reply != QMessageBox.StandardButton.Yes:
            return

        dlg_in = QInputDialog(self)
        dlg_in.setStyleSheet(DIALOG_QSS)
        dlg_in.setWindowTitle("Final Confirmation")
        dlg_in.setInputMode(QInputDialog.InputMode.TextInput)
        dlg_in.setLabelText(f"Type 'YES' to confirm:")
        ok = dlg_in.exec()
        confirm = dlg_in.textValue()
        if not (ok and confirm.strip().upper() in ("YES", "SI")):
            return

        self._set_formatting_ui(True)
        self.statusBar().showMessage(
            f"Formatting PhysicalDrive{drive_num} (quick format + photo-area wipe)...")
        self.format_thread = FormatThread(drive_num)
        self.format_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.format_thread.success.connect(self._on_format_ok)
        self.format_thread.error.connect(self._on_format_failed)
        self.format_thread.start()

    def cancel_format(self):
        th = getattr(self, "format_thread", None)
        if th is not None and th.isRunning():
            th.cancel()
            self.statusBar().showMessage("Cancelling format...")

    def _set_formatting_ui(self, active):
        for b in (self.btn_refresh, self.btn_scan, self.btn_extract,
                  self.btn_delete, self.btn_disconnect):
            b.setEnabled(not active)
        self.drive_combo.setEnabled(not active)
        self.btn_cancel_fmt.setVisible(active)

    def _on_format_ok(self, msg, drives):
        self._set_formatting_ui(False)
        self.listbox.clear()
        self.snapshots.clear()
        self.current_snap = None
        self.request_drive_list()
        self.statusBar().showMessage("Format complete.")
        dialog(self, "info", "Success", msg)

    def _on_format_failed(self, msg, drives):
        self._set_formatting_ui(False)
        self.request_drive_list()
        self.statusBar().showMessage("Format failed.")
        dialog(self, "critical", "Error", msg)

    def closeEvent(self, event):
        # Politely stop any running threads before releasing the handle.
        for name in ("scan_thread", "extract_thread", "delete_thread",
                     "format_thread"):
            t = getattr(self, name, None)
            if t is not None and t.isRunning():
                t.cancel()
        # Wait for in-flight image loads / drive listings so no QThread is
        # destroyed while running.
        for t in list(self._bg_threads):
            t.wait(2000)
        for name in ("scan_thread", "extract_thread", "delete_thread",
                     "format_thread"):
            t = getattr(self, name, None)
            if t is not None:
                t.wait(3000)
        close_drive()
        event.accept()

if __name__ == "__main__":
    if os.name != 'nt':
        print("This script is designed for Windows PhysicalDrive access.")
        sys.exit(1)
        
    app = QApplication(sys.argv)

    # Force a fixed light theme (Fusion + light palette) so the app NEVER
    # inherits the Windows dark mode. Without this, native-styled dialogs
    # (QMessageBox, file pickers) get a dark background while the stylesheet
    # keeps dark text -> unreadable.
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(240, 242, 245))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(51, 51, 51))
    palette.setColor(QPalette.ColorRole.Base, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(243, 244, 246))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(17, 24, 39))
    palette.setColor(QPalette.ColorRole.Text, QColor(31, 41, 55))
    palette.setColor(QPalette.ColorRole.Button, QColor(243, 244, 246))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(17, 24, 39))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(37, 99, 235))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.PlaceholderText, QColor(156, 163, 175))
    palette.setColor(QPalette.ColorRole.Link, QColor(37, 99, 235))
    app.setPalette(palette)

    # Apply PyDracula inspired light theme
    app.setStyleSheet(MODERN_QSS)
    
    window = SDVisualizer()
    window.show()
    sys.exit(app.exec())