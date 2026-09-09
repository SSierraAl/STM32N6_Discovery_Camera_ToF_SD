"""
STM32 SD Card Snapshot Visualizer - integrated SD journal edition.

This wrapper keeps the validated viewer in _sd_image_viewer_core.py and adds
only persistent STM32 append-journal management.

"Format SD" / "Prepare SD" does NOT format the Windows filesystem. New cards
must be formatted externally first. The button uses exactly the same raw erase
path as Delete Selected, then resets the STM32 journal to block 3072 / ID 0.
"""

import errno
import os
import struct
import sys
import time
import zlib

import _sd_image_viewer_core as core

# ========================= PERSISTENT SD JOURNAL =========================
JOURNAL_MAGIC = 0x314A4453  # "SDJ1" little endian
JOURNAL_VERSION = 1
JOURNAL_BLOCK_A = core.SNAP_BASE_NEW - 2
JOURNAL_BLOCK_B = core.SNAP_BASE_NEW - 1
JOURNAL_FMT_NOCRC = "<6I"
JOURNAL_FMT = "<7I"
JOURNAL_SIZE = struct.calcsize(JOURNAL_FMT)


def _safe_close_fd(fd):
    if fd is None:
        return
    try:
        os.close(fd)
    except OSError as exc:
        if exc.errno != errno.EBADF:
            raise


def _safe_core_close_locked():
    """Make cached read-handle close idempotent; do not change write behavior."""
    if core._gf is not None:
        try:
            core._gf.close()
        except (OSError, ValueError):
            pass
        core._gf = None
    if core._gfd is not None:
        _safe_close_fd(core._gfd)
        core._gfd = None
    core._gpath = None


core._close_locked = _safe_core_close_locked


def _raw_open(path, write=False):
    flags = (os.O_RDWR if write else os.O_RDONLY) | getattr(os, "O_BINARY", 0)
    return os.open(path, flags)


def _read_block(path, block):
    fd = _raw_open(path, False)
    try:
        os.lseek(fd, int(block) * core.BLOCK_SIZE, os.SEEK_SET)
        return os.read(fd, core.BLOCK_SIZE)
    finally:
        _safe_close_fd(fd)


def _write_block_like_delete(path, block, data):
    """Write one raw block using the same CRT path as core.zero_fill()."""
    if len(data) != core.BLOCK_SIZE:
        raise ValueError("journal write must be exactly one 512-byte block")
    fd = _raw_open(path, True)
    try:
        os.lseek(fd, int(block) * core.BLOCK_SIZE, os.SEEK_SET)
        written = os.write(fd, data)
        if written != core.BLOCK_SIZE:
            raise OSError(
                f"short journal write at block {block}: "
                f"{written}/{core.BLOCK_SIZE} bytes"
            )
    finally:
        _safe_close_fd(fd)


def _journal_crc(payload):
    return zlib.crc32(payload) & 0xFFFFFFFF


def _decode_journal(raw, slot):
    if raw is None or len(raw) < JOURNAL_SIZE:
        return None
    vals = struct.unpack_from(JOURNAL_FMT, raw, 0)
    rec = {
        "magic": vals[0],
        "version": vals[1],
        "sequence": vals[2],
        "next_block": vals[3],
        "next_snap_id": vals[4],
        "snap_base": vals[5],
        "crc32": vals[6],
        "slot": slot,
    }
    payload = struct.pack(JOURNAL_FMT_NOCRC, *vals[:6])
    if rec["magic"] != JOURNAL_MAGIC or rec["version"] != JOURNAL_VERSION:
        return None
    if rec["snap_base"] != core.SNAP_BASE_NEW:
        return None
    if rec["next_block"] < core.SNAP_BASE_NEW:
        return None
    if rec["crc32"] != _journal_crc(payload):
        return None
    return rec


def _newer_journal(a, b):
    delta = (a["sequence"] - b["sequence"]) & 0xFFFFFFFF
    return a if delta != 0 and delta < 0x80000000 else b


def read_journal(path):
    a = _decode_journal(_read_block(path, JOURNAL_BLOCK_A), 0)
    b = _decode_journal(_read_block(path, JOURNAL_BLOCK_B), 1)
    if a and b:
        return _newer_journal(a, b)
    return a or b


def _pack_journal(sequence, next_block, next_snap_id):
    values = (
        JOURNAL_MAGIC,
        JOURNAL_VERSION,
        sequence & 0xFFFFFFFF,
        int(next_block),
        int(next_snap_id),
        core.SNAP_BASE_NEW,
    )
    payload = struct.pack(JOURNAL_FMT_NOCRC, *values)
    raw = payload + struct.pack("<I", _journal_crc(payload))
    return raw + b"\x00" * (core.BLOCK_SIZE - len(raw))


def write_journal(path, next_block, next_snap_id, duplicate_if_new=True):
    if int(next_block) < core.SNAP_BASE_NEW:
        raise ValueError(f"next_block must be >= {core.SNAP_BASE_NEW}")
    if int(next_snap_id) < 0:
        raise ValueError("next_snap_id must be >= 0")

    current = read_journal(path)
    sequence = ((current["sequence"] + 1) & 0xFFFFFFFF) if current else 1
    slot = (1 - current["slot"]) if current else 0
    block = JOURNAL_BLOCK_B if slot else JOURNAL_BLOCK_A
    raw = _pack_journal(sequence, next_block, next_snap_id)

    _write_block_like_delete(path, block, raw)
    verify = _decode_journal(_read_block(path, block), slot)
    if not verify:
        raise OSError(f"journal read-back failed at block {block}")
    if verify["next_block"] != int(next_block) or verify["next_snap_id"] != int(next_snap_id):
        raise OSError("journal read-back values do not match")

    if current is None and duplicate_if_new:
        return write_journal(path, next_block, next_snap_id, duplicate_if_new=False)
    return read_journal(path)


def reset_journal(path):
    """Clear A/B with the validated delete writer, then create a fresh pair."""
    # Reuse the exact writer that Delete Selected already uses successfully.
    core.zero_fill(path, JOURNAL_BLOCK_A, 1)
    core.zero_fill(path, JOURNAL_BLOCK_B, 1)
    rec = write_journal(path, core.SNAP_BASE_NEW, 0)
    if not rec or rec["next_block"] != core.SNAP_BASE_NEW or rec["next_snap_id"] != 0:
        raise OSError("fresh journal verification failed")
    return rec


def _snapshot_blocks(snap):
    h = snap.header
    data_size = h.get("data_size", 0)
    bpp = 1 if h.get("pixel_format", 0) == 2 else 2
    expected = h.get("width", 0) * h.get("height", 0) * bpp
    if data_size <= 0 or abs(data_size - expected) > core.BLOCK_SIZE * 4:
        data_size = expected
    return (core.HEADER_SIZE + data_size + core.BLOCK_SIZE - 1) // core.BLOCK_SIZE


def find_last_valid_image(path, before_block):
    """Find the physically last surviving header before journal.next_block."""
    before_block = max(int(before_block), core.SNAP_BASE_NEW)
    if before_block <= core.SNAP_BASE_NEW:
        return None

    chunk_blocks = (16 * 1024 * 1024) // core.BLOCK_SIZE
    magic = struct.pack("<I", core.HEADER_TAG)
    end_block = before_block

    while end_block > core.SNAP_BASE_NEW:
        start_block = max(core.SNAP_BASE_NEW, end_block - chunk_blocks)
        data = core.rbulk(path, start_block, end_block - start_block)
        if not data:
            break

        pos = len(data)
        while pos > 0:
            hit = data.rfind(magic, 0, pos)
            if hit < 0:
                break
            pos = hit
            if hit % core.BLOCK_SIZE:
                continue
            h = core.parse_hdr(data[hit : hit + core.HEADER_SIZE])
            if not h or h.get("magic") != core.HEADER_TAG:
                continue
            w, hh = h.get("width", 0), h.get("height", 0)
            if w <= 0 or hh <= 0 or w > 4096 or hh > 4096:
                continue
            bpp = 1 if h.get("pixel_format", 0) == 2 else 2
            expected = w * hh * bpp
            if h.get("data_size", 0) != expected:
                continue
            blocks = (core.HEADER_SIZE + expected + core.BLOCK_SIZE - 1) // core.BLOCK_SIZE
            block = start_block + hit // core.BLOCK_SIZE
            image_end = block + blocks
            if image_end <= before_block:
                return {
                    "block": block,
                    "end_block": image_end,
                    "snap_id": int(h.get("snap_id", 0)),
                }
        end_block = start_block
    return None


# ========================= DELETE + JOURNAL =========================
class JournalDeleteThread(core.QThread):
    progress = core.Signal(str)
    finished_ok = core.Signal(int)
    failed = core.Signal(str)

    def __init__(self, drive, snapshots):
        super().__init__()
        self.drive = drive
        self.snapshots = list(snapshots)
        self._cancel = False
        self.journal_note = ""

    def cancel(self):
        self._cancel = True

    def run(self):
        snaps = sorted(self.snapshots, key=lambda s: s.block, reverse=True)
        total = len(snaps)
        try:
            core.close_drive()
            current = read_journal(self.drive)

            # IMPORTANT: this is exactly the validated Delete Selected writer.
            for i, snap in enumerate(snaps, 1):
                if self._cancel:
                    self.failed.emit("Cancelled — remaining images were not erased.")
                    return
                nb = _snapshot_blocks(snap)
                self.progress.emit(
                    f"Erasing {i}/{total}: #{snap.idx+1:03d} @ block "
                    f"{snap.block} ({nb * core.BLOCK_SIZE // (1024*1024)} MB)..."
                )
                core.zero_fill(self.drive, snap.block, nb)

            if current is None:
                self.journal_note = (
                    "Images erased; no valid journal exists yet. "
                    "Use Format SD / Prepare SD before reconnecting to STM32."
                )
                self.finished_ok.emit(total)
                return

            last = find_last_valid_image(self.drive, current["next_block"])
            if last is None:
                next_block = core.SNAP_BASE_NEW
                next_id = 0
            else:
                next_block = last["end_block"]
                next_id = last["snap_id"] + 1

            if next_block != current["next_block"] or next_id != current["next_snap_id"]:
                rec = write_journal(self.drive, next_block, next_id)
                self.journal_note = (
                    f"Journal adjusted: next block {rec['next_block']}, "
                    f"next image ID {rec['next_snap_id']}."
                )
            else:
                self.journal_note = (
                    f"Journal unchanged at block {current['next_block']} / "
                    f"ID {current['next_snap_id']}."
                )

            self.finished_ok.emit(total)
        except Exception as exc:
            self.failed.emit(str(exc))


# ========================= PREPARE SD =========================
class PrepareSDThread(core.QThread):
    progress = core.Signal(str)
    success = core.Signal(str, list)
    error = core.Signal(str, list)

    def __init__(self, drive, snapshots):
        super().__init__()
        self.drive = drive
        self.snapshots = list(snapshots)
        self._cancel = False

    def cancel(self):
        self._cancel = True

    def run(self):
        try:
            core.close_drive()
            current = read_journal(self.drive)

            # If the journal says images were written, require the user to Scan
            # first. Prepare SD must never invent/delete unknown ranges.
            if current and current["next_block"] > core.SNAP_BASE_NEW and not self.snapshots:
                self.error.emit(
                    "This card contains STM32 data according to its journal, but the "
                    "viewer currently has no scanned snapshots.\n\nRun 'Scan Snapshots' "
                    "first, then click Format SD / Prepare SD again.",
                    core.get_drives(),
                )
                return

            snaps = sorted(self.snapshots, key=lambda s: s.block, reverse=True)
            self.progress.emit(
                f"Step 1/2: deleting {len(snaps)} scanned STM32 photo(s) using "
                "the same Delete Selected function..."
            )

            # EXACT same operation as Delete Selected.
            for i, snap in enumerate(snaps, 1):
                if self._cancel:
                    self.error.emit(
                        "Prepare SD cancelled. Journal was NOT reset.",
                        core.get_drives(),
                    )
                    return
                nb = _snapshot_blocks(snap)
                self.progress.emit(
                    f"Step 1/2: deleting {i}/{len(snaps)} @ block {snap.block}..."
                )
                core.zero_fill(self.drive, snap.block, nb)

            # If we had a trusted journal, do a small safety verification that no
            # surviving valid header remains before moving the pointer backwards.
            if current and current["next_block"] > core.SNAP_BASE_NEW:
                remaining = find_last_valid_image(self.drive, current["next_block"])
                if remaining is not None:
                    raise OSError(
                        f"a valid image still remains at block {remaining['block']}; "
                        "journal was NOT reset"
                    )

            if self._cancel:
                self.error.emit(
                    "Prepare SD cancelled. Journal was NOT reset.",
                    core.get_drives(),
                )
                return

            self.progress.emit("Step 2/2: resetting journal A/B to block 3072 / ID 0...")
            rec = reset_journal(self.drive)
            self.success.emit(
                f"SD prepared. Deleted {len(snaps)} STM32 photo(s). "
                f"Journal reset at blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B}; "
                f"next block {rec['next_block']}, ID {rec['next_snap_id']}. "
                "The Windows filesystem was not formatted.",
                core.get_drives(),
            )
        except Exception as exc:
            self.error.emit(
                f"Prepare SD failed:\n{exc}\n\n"
                "The same raw erase path as Delete Selected was used. "
                "The journal is not moved backwards after a failed delete.",
                core.get_drives(),
            )


# Core SDVisualizer.format_card expects FormatThread(drive_num), but this wrapper
# overrides format_card below and directly creates PrepareSDThread instead.


# ========================= INTEGRATED UI =========================
class SDVisualizer(core.SDVisualizer):
    def __init__(self):
        self._last_delete_note = ""
        super().__init__()

    def _build_ui(self):
        super()._build_ui()
        central_layout = self.centralWidget().layout()
        toolbar = central_layout.itemAt(0).widget()
        toolbar_layout = toolbar.layout()

        self.btn_format = core.QPushButton("⚠ Format SD")
        self.btn_format.setObjectName("DangerButton")
        self.btn_format.setToolTip(
            "STM32 Prepare/Reset only. Uses the same erase function as Delete "
            "Selected, then resets the journal. Does NOT format FAT32."
        )
        self.btn_format.clicked.connect(self.format_card)
        toolbar_layout.insertWidget(2, self.btn_format)

    def _set_ui_busy(self, busy, wait_drives=False):
        super()._set_ui_busy(busy, wait_drives)
        if hasattr(self, "btn_format"):
            self.btn_format.setEnabled(not busy and not wait_drives)

    def _set_formatting_ui(self, active):
        super()._set_formatting_ui(active)
        if hasattr(self, "btn_format"):
            self.btn_format.setEnabled(not active)

    @core.Slot(list)
    def _on_scan_finished(self, results):
        super()._on_scan_finished(results)
        try:
            rec = read_journal(self.drive) if self.drive else None
        except Exception:
            rec = None

        if rec:
            status = (
                f"Scan complete. Found {len(self.snapshots)} snapshots. "
                f"Journal → next block {rec['next_block']}, ID {rec['next_snap_id']}."
            )
        else:
            status = (
                f"Scan complete. Found {len(self.snapshots)} snapshots. "
                "No valid STM32 journal — Format SD prepares it after external formatting."
            )
        if self._last_delete_note:
            status += " " + self._last_delete_note
            self._last_delete_note = ""
        self.statusBar().showMessage(status)

    def delete_selected(self):
        rows = self.listbox.selectedItems()
        if not rows:
            core.dialog(self, "warning", "Warning", "Select one or more images first.")
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return

        snaps = [self.snapshots[self.listbox.row(item)] for item in rows]
        reply = core.dialog(
            self,
            "question",
            "Delete Selected",
            f"Erase {len(snaps)} selected image(s)?\n\n"
            "This uses the viewer's validated raw delete function. Interior gaps "
            "remain unused; deleting the latest photos automatically adjusts the journal.",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        self.delete_thread = JournalDeleteThread(self.drive, snaps)
        self.delete_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.delete_thread.finished_ok.connect(self._on_delete_ok)
        self.delete_thread.failed.connect(self._on_delete_failed)
        self.delete_thread.start()
        self._set_ui_busy(True)

    def _on_delete_ok(self, count):
        self._last_delete_note = getattr(self.delete_thread, "journal_note", "")
        self._set_ui_busy(False)
        self.statusBar().showMessage(f"{count} image(s) erased. Re-scanning...")
        self.scan_snapshots()

    def format_card(self):
        """Prepare/reset STM32 storage; no Windows filesystem formatting."""
        if not self.drive:
            core.dialog(self, "warning", "Error", "Select a drive first.")
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation to finish...")
            return

        reply = core.dialog(
            self,
            "question",
            "⚠ PREPARE / RESET STM32 SD",
            "This does NOT format FAT32. Format new cards externally first.\n\n"
            "Prepare SD will:\n"
            " • erase ALL snapshots currently found by Scan Snapshots\n"
            " • use exactly the same erase operation as Delete Selected\n"
            " • reset journal A/B to block 3072 / image ID 0\n\n"
            "If this is an existing card, run Scan Snapshots first.\n\nContinue?",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        dlg = core.QInputDialog(self)
        dlg.setStyleSheet(core.DIALOG_QSS)
        dlg.setWindowTitle("Final Confirmation")
        dlg.setInputMode(core.QInputDialog.InputMode.TextInput)
        dlg.setLabelText("Type 'RESET' to confirm:")
        ok = dlg.exec()
        if not (ok and dlg.textValue().strip().upper() == "RESET"):
            return

        self._set_formatting_ui(True)
        self.statusBar().showMessage("Preparing SD using validated Delete Selected path...")
        self.format_thread = PrepareSDThread(self.drive, list(self.snapshots))
        self.format_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.format_thread.success.connect(self._on_format_ok)
        self.format_thread.error.connect(self._on_format_failed)
        self.format_thread.start()


# ========================= ENTRY POINT =========================
def _configure_app_palette(app):
    app.setStyle("Fusion")
    palette = core.QPalette()
    palette.setColor(core.QPalette.ColorRole.Window, core.QColor(240, 242, 245))
    palette.setColor(core.QPalette.ColorRole.WindowText, core.QColor(51, 51, 51))
    palette.setColor(core.QPalette.ColorRole.Base, core.QColor(255, 255, 255))
    palette.setColor(core.QPalette.ColorRole.AlternateBase, core.QColor(243, 244, 246))
    palette.setColor(core.QPalette.ColorRole.ToolTipBase, core.QColor(255, 255, 255))
    palette.setColor(core.QPalette.ColorRole.ToolTipText, core.QColor(17, 24, 39))
    palette.setColor(core.QPalette.ColorRole.Text, core.QColor(31, 41, 55))
    palette.setColor(core.QPalette.ColorRole.Button, core.QColor(243, 244, 246))
    palette.setColor(core.QPalette.ColorRole.ButtonText, core.QColor(17, 24, 39))
    palette.setColor(core.QPalette.ColorRole.Highlight, core.QColor(37, 99, 235))
    palette.setColor(core.QPalette.ColorRole.HighlightedText, core.QColor(255, 255, 255))
    palette.setColor(core.QPalette.ColorRole.PlaceholderText, core.QColor(156, 163, 175))
    palette.setColor(core.QPalette.ColorRole.Link, core.QColor(37, 99, 235))
    app.setPalette(palette)
    app.setStyleSheet(core.MODERN_QSS)


if __name__ == "__main__":
    if os.name != "nt":
        print("This script is designed for Windows PhysicalDrive access.")
        sys.exit(1)

    app = core.QApplication(sys.argv)
    _configure_app_palette(app)
    window = SDVisualizer()
    window.show()
    sys.exit(app.exec())
