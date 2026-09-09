"""
STM32 SD Card Snapshot Visualizer - integrated SD journal edition.

The validated image viewer remains in _sd_image_viewer_core.py. This entry point
adds persistent append-journal management for the STM32 raw image area.

Important: "Format SD" in this UI does NOT reformat the Windows filesystem.
Format the card externally first when needed. The button only erases STM32 RAW
photos that belong to this application and resets the redundant append journal.
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
    """Close a raw Windows descriptor without turning an already-invalid
    descriptor into a second, unrelated error."""
    if fd is None:
        return
    try:
        os.close(fd)
    except OSError as exc:
        if exc.errno != errno.EBADF:
            raise


def _safe_core_close_locked():
    """Idempotent replacement for the core cached-handle close helper."""
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


def _journal_open(path, write=False, attempts=8, settle=0.35):
    flags = (os.O_RDWR if write else os.O_RDONLY) | getattr(os, "O_BINARY", 0)
    last = None
    for attempt in range(attempts):
        try:
            return os.open(path, flags)
        except OSError as exc:
            last = exc
            if attempt + 1 < attempts:
                time.sleep(settle)
    raise last


def _read_block_fd(fd, block):
    os.lseek(fd, int(block) * core.BLOCK_SIZE, os.SEEK_SET)
    return os.read(fd, core.BLOCK_SIZE)


def _write_block_fd(fd, block, data):
    if len(data) != core.BLOCK_SIZE:
        raise ValueError("journal write must be exactly one 512-byte block")
    os.lseek(fd, int(block) * core.BLOCK_SIZE, os.SEEK_SET)
    written = os.write(fd, data)
    if written != core.BLOCK_SIZE:
        raise OSError(f"short journal write: {written}/{core.BLOCK_SIZE} bytes")
    try:
        os.fsync(fd)
    except OSError:
        pass


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
    fd = _journal_open(path, False)
    try:
        a = _decode_journal(_read_block_fd(fd, JOURNAL_BLOCK_A), 0)
        b = _decode_journal(_read_block_fd(fd, JOURNAL_BLOCK_B), 1)
    finally:
        _safe_close_fd(fd)
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
    """Write/advance the redundant A/B journal with read-back verification."""
    if next_block < core.SNAP_BASE_NEW:
        raise ValueError(f"next_block must be >= {core.SNAP_BASE_NEW}")
    if next_snap_id < 0:
        raise ValueError("next_snap_id must be >= 0")

    current = read_journal(path)
    sequence = ((current["sequence"] + 1) & 0xFFFFFFFF) if current else 1
    slot = (1 - current["slot"]) if current else 0
    block = JOURNAL_BLOCK_B if slot else JOURNAL_BLOCK_A
    raw = _pack_journal(sequence, next_block, next_snap_id)

    fd = _journal_open(path, True)
    try:
        _write_block_fd(fd, block, raw)
        verify = _decode_journal(_read_block_fd(fd, block), slot)
        if not verify:
            raise OSError("journal read-back verification failed")
        if (
            verify["sequence"] != sequence
            or verify["next_block"] != next_block
            or verify["next_snap_id"] != next_snap_id
        ):
            raise OSError("journal read-back values do not match")
    finally:
        _safe_close_fd(fd)

    if current is None and duplicate_if_new:
        return write_journal(path, next_block, next_snap_id, duplicate_if_new=False)
    return read_journal(path)


def reset_journal(path):
    """Reset BOTH journal slots, then recreate a fresh redundant A/B pair.

    The old records are cleared only after photo deletion/verification succeeds.
    If recreation fails, firmware sees no valid journal and blocks image writes
    rather than risking an overwrite.
    """
    zero = b"\x00" * core.BLOCK_SIZE
    fd = _journal_open(path, True)
    try:
        _write_block_fd(fd, JOURNAL_BLOCK_A, zero)
        _write_block_fd(fd, JOURNAL_BLOCK_B, zero)
    finally:
        _safe_close_fd(fd)

    rec = write_journal(path, core.SNAP_BASE_NEW, 0)
    if not rec or rec["next_block"] != core.SNAP_BASE_NEW or rec["next_snap_id"] != 0:
        raise OSError("fresh journal verification failed")
    return rec


def _parse_raw_header(raw):
    if raw is None or len(raw) < core.HEADER_SIZE:
        return None
    vals = struct.unpack_from("<11I", raw, 0)
    magic, width, height, pixel_format, data_size = vals[:5]
    if magic != core.HEADER_TAG:
        return None
    if width == 0 or height == 0 or width > 4096 or height > 4096:
        return None
    bpp = 1 if pixel_format == 2 else 2
    expected = width * height * bpp
    if data_size != expected:
        return None
    blocks = (core.HEADER_SIZE + data_size + core.BLOCK_SIZE - 1) // core.BLOCK_SIZE
    return {
        "width": width,
        "height": height,
        "pixel_format": pixel_format,
        "data_size": data_size,
        "snap_id": vals[7],
        "blocks": blocks,
    }


def find_last_valid_image(path, before_block, progress_cb=None):
    """Find the physically last surviving raw image before before_block."""
    before_block = max(int(before_block), core.SNAP_BASE_NEW)
    if before_block <= core.SNAP_BASE_NEW:
        return None

    chunk_blocks = (16 * 1024 * 1024) // core.BLOCK_SIZE
    magic = struct.pack("<I", core.HEADER_TAG)
    fd = _journal_open(path, False)
    try:
        end_block = before_block
        while end_block > core.SNAP_BASE_NEW:
            start_block = max(core.SNAP_BASE_NEW, end_block - chunk_blocks)
            block_count = end_block - start_block
            os.lseek(fd, start_block * core.BLOCK_SIZE, os.SEEK_SET)
            data = os.read(fd, block_count * core.BLOCK_SIZE)
            if not data:
                break

            if progress_cb:
                progress_cb(
                    f"Updating SD journal: checking blocks "
                    f"{start_block}..{end_block - 1} ..."
                )

            pos = len(data)
            while pos > 0:
                hit = data.rfind(magic, 0, pos)
                if hit < 0:
                    break
                pos = hit
                if hit % core.BLOCK_SIZE != 0:
                    continue
                h = _parse_raw_header(data[hit : hit + core.HEADER_SIZE])
                if not h:
                    continue
                block = start_block + hit // core.BLOCK_SIZE
                image_end = block + h["blocks"]
                if image_end <= before_block:
                    h["block"] = block
                    h["end_block"] = image_end
                    return h
            end_block = start_block
    finally:
        _safe_close_fd(fd)
    return None


def find_all_valid_images(path, end_block, progress_cb=None, cancel_cb=None):
    """Scan the STM32 image area up to a trusted journal end pointer.

    This is used only by Format/Reset SD. It is deliberately bounded by the
    journal's next_block so the viewer never scans or zeroes arbitrary unused
    parts of the card.
    """
    end_block = max(int(end_block), core.SNAP_BASE_NEW)
    if end_block <= core.SNAP_BASE_NEW:
        return []

    chunk_blocks = (16 * 1024 * 1024) // core.BLOCK_SIZE
    magic = struct.pack("<I", core.HEADER_TAG)
    found = []
    fd = _journal_open(path, False)
    try:
        start_block = core.SNAP_BASE_NEW
        while start_block < end_block:
            if cancel_cb and cancel_cb():
                return None
            count = min(chunk_blocks, end_block - start_block)
            os.lseek(fd, start_block * core.BLOCK_SIZE, os.SEEK_SET)
            data = os.read(fd, count * core.BLOCK_SIZE)
            if not data:
                raise OSError(
                    f"short read while scanning STM32 image area at block {start_block}"
                )

            if progress_cb:
                pct = int(100 * (start_block - core.SNAP_BASE_NEW + count) /
                          max(1, end_block - core.SNAP_BASE_NEW))
                progress_cb(f"Step 1/3: finding STM32 photos — {min(pct, 100)}%...")

            search_from = 0
            while search_from <= len(data) - core.HEADER_SIZE:
                hit = data.find(magic, search_from)
                if hit < 0:
                    break
                if hit % core.BLOCK_SIZE == 0:
                    h = _parse_raw_header(data[hit : hit + core.HEADER_SIZE])
                    if h:
                        block = start_block + hit // core.BLOCK_SIZE
                        image_end = block + h["blocks"]
                        if image_end <= end_block:
                            h["block"] = block
                            h["end_block"] = image_end
                            found.append(h)
                            search_from = hit + h["blocks"] * core.BLOCK_SIZE
                            continue
                search_from = hit + 1

            start_block += len(data) // core.BLOCK_SIZE
    finally:
        _safe_close_fd(fd)
    return found


def _snapshot_blocks(snap):
    h = snap.header
    data_size = h["data_size"]
    bpp = 1 if h.get("pixel_format", 0) == 2 else 2
    expected_size = h["width"] * h["height"] * bpp
    if data_size <= 0 or abs(data_size - expected_size) > (core.BLOCK_SIZE * 4):
        data_size = expected_size
    return (core.HEADER_SIZE + data_size + core.BLOCK_SIZE - 1) // core.BLOCK_SIZE


# ========================= DELETE + JOURNAL =========================
class JournalDeleteThread(core.QThread):
    progress = core.Signal(str)
    finished_ok = core.Signal(int)
    failed = core.Signal(str)

    def __init__(self, drive, snapshots):
        super().__init__()
        self.drive = drive
        self.snapshots = snapshots
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

            for i, snap in enumerate(snaps, 1):
                if self._cancel:
                    self.failed.emit(
                        "Cancelled — the remaining images were not erased."
                    )
                    return
                nb = _snapshot_blocks(snap)
                self.progress.emit(
                    f"Erasing {i}/{total}: #{snap.idx+1:03d} @ block "
                    f"{snap.block} ({nb * core.BLOCK_SIZE // (1024*1024)} MB) ..."
                )
                core.zero_fill(self.drive, snap.block, nb)

            if current is None:
                self.journal_note = (
                    "Images erased; no valid STM32 journal was found. "
                    "Use Format SD before reconnecting this card to the board."
                )
                self.finished_ok.emit(total)
                return

            last = find_last_valid_image(
                self.drive,
                current["next_block"],
                progress_cb=lambda m: self.progress.emit(m),
            )
            if last is None:
                next_block = core.SNAP_BASE_NEW
                next_id = 0
            else:
                next_block = last["end_block"]
                next_id = int(last["snap_id"]) + 1

            if (
                next_block != current["next_block"]
                or next_id != current["next_snap_id"]
            ):
                try:
                    rec = write_journal(self.drive, next_block, next_id)
                    self.journal_note = (
                        f"Journal adjusted: next block {rec['next_block']}, "
                        f"next image ID {rec['next_snap_id']}."
                    )
                except Exception as exc:
                    self.journal_note = (
                        f"Images erased, but journal update failed: {exc}. "
                        "Do not capture until the card is prepared again."
                    )
            else:
                self.journal_note = (
                    f"Images erased; journal unchanged at block "
                    f"{current['next_block']} / ID {current['next_snap_id']}."
                )

            self.finished_ok.emit(total)
        except Exception as exc:
            self.failed.emit(str(exc))


# ========================= PREPARE / RESET SD + JOURNAL =========================
class JournalFormatThread(core.QThread):
    """Reset the STM32 raw-image area without formatting the Windows volume.

    Existing valid-journal cards:
      1) scan only from block 3072 to journal.next_block,
      2) erase every valid STM32 RAW image found there,
      3) verify no valid image remains,
      4) recreate both journals at block 3072 / ID 0.

    New externally-formatted cards have no journal and no STM32 RAW images, so
    only step 4 is needed. If a no-journal card has images already visible in
    the UI, those known images are erased before the fresh journal is created.
    """

    progress = core.Signal(str)
    success = core.Signal(str, list)
    error = core.Signal(str, list)

    def __init__(self, drive, known_snapshots=None):
        super().__init__()
        self.drive = drive
        self.known_snapshots = list(known_snapshots or [])
        self._cancel = False

    def cancel(self):
        self._cancel = True

    def run(self):
        try:
            core.close_drive()
            self.progress.emit(
                "Step 1/3: reading STM32 journal and locating stored photos..."
            )
            current = read_journal(self.drive)

            if current is not None:
                images = find_all_valid_images(
                    self.drive,
                    current["next_block"],
                    progress_cb=lambda m: self.progress.emit(m),
                    cancel_cb=lambda: self._cancel,
                )
                if images is None or self._cancel:
                    self.error.emit(
                        "Prepare SD cancelled. Journal was not reset.",
                        core.get_drives(),
                    )
                    return
                erase_ranges = [(h["block"], h["blocks"]) for h in images]
                trusted_end = current["next_block"]
            else:
                # Intended use: a card that was already formatted externally.
                # If the user scanned a legacy/no-journal card first, erase the
                # exact photos already identified by the validated viewer.
                erase_ranges = [
                    (snap.block, _snapshot_blocks(snap))
                    for snap in sorted(
                        self.known_snapshots, key=lambda s: s.block, reverse=True
                    )
                ]
                trusted_end = None
                self.progress.emit(
                    "Step 1/3: no STM32 journal found — treating this as a "
                    "new/externally-formatted card."
                )

            self.progress.emit(
                f"Step 2/3: deleting {len(erase_ranges)} STM32 RAW photo(s)..."
            )
            for i, (block, blocks) in enumerate(erase_ranges, 1):
                if self._cancel:
                    self.error.emit(
                        "Prepare SD cancelled. Journal was not reset.",
                        core.get_drives(),
                    )
                    return
                self.progress.emit(
                    f"Step 2/3: deleting photo {i}/{len(erase_ranges)} "
                    f"at block {block}..."
                )
                core.zero_fill(self.drive, block, blocks)

            # For a card with a valid journal we know the exact used extent.
            # Verify it is empty BEFORE moving the journal pointer backwards.
            if trusted_end is not None:
                remaining = find_last_valid_image(
                    self.drive,
                    trusted_end,
                    progress_cb=lambda m: self.progress.emit(
                        "Step 2/3: verifying photo area..."
                    ),
                )
                if remaining is not None:
                    raise OSError(
                        f"photo reset verification failed: a valid image still "
                        f"exists at block {remaining['block']}; journal NOT reset"
                    )

            if self._cancel:
                self.error.emit(
                    "Prepare SD cancelled. Journal was not reset.",
                    core.get_drives(),
                )
                return

            self.progress.emit(
                "Step 3/3: resetting redundant STM32 journal to block 3072 / ID 0..."
            )
            rec = reset_journal(self.drive)

            self.success.emit(
                f"SD prepared for STM32. Deleted {len(erase_ranges)} RAW photo(s). "
                f"Journal A/B reset at blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B}; "
                f"next block {rec['next_block']}, ID {rec['next_snap_id']}. "
                "The Windows filesystem was NOT reformatted.",
                core.get_drives(),
            )
        except Exception as exc:
            self.error.emit(
                f"Prepare SD failed:\n{exc}\n\n"
                "No filesystem format was attempted.",
                core.get_drives(),
            )


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
            "STM32 reset only: delete STM32 RAW photos and reset the append "
            "journal. It does NOT format FAT32; format new cards externally first."
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
                f"Journal → next block {rec['next_block']}, "
                f"ID {rec['next_snap_id']}."
            )
        else:
            status = (
                f"Scan complete. Found {len(self.snapshots)} snapshots. "
                "No valid STM32 journal — use Format SD before capturing."
            )

        if self._last_delete_note:
            status += " " + self._last_delete_note
            self._last_delete_note = ""
        self.statusBar().showMessage(status)

    def delete_selected(self):
        rows = self.listbox.selectedItems()
        if not rows:
            core.dialog(
                self, "warning", "Warning", "Select one or more images first."
            )
            return
        if self._op_running():
            self.statusBar().showMessage(
                "Please wait for the current operation to finish..."
            )
            return

        snaps = [self.snapshots[self.listbox.row(item)] for item in rows]
        total_mb = sum(
            (core.HEADER_SIZE + s.header["data_size"]) // (1024 * 1024)
            for s in snaps
        )

        reply = core.dialog(
            self,
            "question",
            "Delete Selected",
            f"Erasing {len(snaps)} image(s) (~{total_mb} MB).\n"
            "Interior holes will stay unused. If the latest image(s) are deleted, "
            "the SD journal will automatically move to the last surviving image.\n\n"
            "Continue?",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        self.delete_thread = JournalDeleteThread(self.drive, snaps)
        self.delete_thread.progress.connect(
            lambda m: self.statusBar().showMessage(m)
        )
        self.delete_thread.finished_ok.connect(self._on_delete_ok)
        self.delete_thread.failed.connect(self._on_delete_failed)
        self.delete_thread.start()
        self._set_ui_busy(True)

    def _on_delete_ok(self, count):
        self._last_delete_note = getattr(
            self.delete_thread, "journal_note", ""
        )
        self._set_ui_busy(False)
        self.statusBar().showMessage(
            f"{count} image(s) erased. Re-scanning..."
        )
        self.scan_snapshots()

    def format_card(self):
        """Prepare/reset STM32 storage. No Windows filesystem formatting."""
        if not self.drive:
            core.dialog(self, "warning", "Error", "Select a drive first.")
            return
        if self._op_running():
            self.statusBar().showMessage(
                "Please wait for the current operation to finish..."
            )
            return

        drive_num = self.drive.replace("\\\\.\\PhysicalDrive", "")
        reply = core.dialog(
            self,
            "question",
            "⚠ PREPARE / RESET STM32 SD",
            f"PhysicalDrive{drive_num}\n\n"
            "This button does NOT format FAT32.\n"
            "Format a new card externally in the correct filesystem first.\n\n"
            "It WILL:\n"
            " • erase all STM32 RAW photos detected on this card\n"
            " • reset both STM32 journals\n"
            f" • restart storage at block {core.SNAP_BASE_NEW}, image ID 0\n\n"
            "Other filesystem files are not intentionally modified.\n\n"
            "Continue?",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        dlg_in = core.QInputDialog(self)
        dlg_in.setStyleSheet(core.DIALOG_QSS)
        dlg_in.setWindowTitle("Final Confirmation")
        dlg_in.setInputMode(core.QInputDialog.InputMode.TextInput)
        dlg_in.setLabelText("Type 'RESET' to confirm:")
        ok = dlg_in.exec()
        if not (ok and dlg_in.textValue().strip().upper() == "RESET"):
            return

        self._set_formatting_ui(True)
        self.statusBar().showMessage(
            f"Preparing PhysicalDrive{drive_num}: deleting STM32 photos + resetting journal..."
        )
        self.format_thread = JournalFormatThread(
            self.drive, known_snapshots=list(self.snapshots)
        )
        self.format_thread.progress.connect(
            lambda m: self.statusBar().showMessage(m)
        )
        self.format_thread.success.connect(self._on_format_ok)
        self.format_thread.error.connect(self._on_format_failed)
        self.format_thread.start()


# ========================= ENTRY POINT =========================
def _configure_app_palette(app):
    app.setStyle("Fusion")
    palette = core.QPalette()
    palette.setColor(
        core.QPalette.ColorRole.Window, core.QColor(240, 242, 245)
    )
    palette.setColor(
        core.QPalette.ColorRole.WindowText, core.QColor(51, 51, 51)
    )
    palette.setColor(core.QPalette.ColorRole.Base, core.QColor(255, 255, 255))
    palette.setColor(
        core.QPalette.ColorRole.AlternateBase, core.QColor(243, 244, 246)
    )
    palette.setColor(
        core.QPalette.ColorRole.ToolTipBase, core.QColor(255, 255, 255)
    )
    palette.setColor(
        core.QPalette.ColorRole.ToolTipText, core.QColor(17, 24, 39)
    )
    palette.setColor(core.QPalette.ColorRole.Text, core.QColor(31, 41, 55))
    palette.setColor(
        core.QPalette.ColorRole.Button, core.QColor(243, 244, 246)
    )
    palette.setColor(
        core.QPalette.ColorRole.ButtonText, core.QColor(17, 24, 39)
    )
    palette.setColor(
        core.QPalette.ColorRole.Highlight, core.QColor(37, 99, 235)
    )
    palette.setColor(
        core.QPalette.ColorRole.HighlightedText, core.QColor(255, 255, 255)
    )
    palette.setColor(
        core.QPalette.ColorRole.PlaceholderText, core.QColor(156, 163, 175)
    )
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
