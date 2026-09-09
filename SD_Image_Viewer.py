"""
STM32 SD Card Snapshot Visualizer - integrated SD journal edition.

The original viewer implementation lives in _sd_image_viewer_core.py so the
validated image decoding/scanning/formatting UI is preserved byte-for-byte.
This entry point adds only the persistent raw-SD journal workflow used by the
STM32 firmware:
  - Format SD initializes redundant journal sectors automatically.
  - Delete Selected recalculates the append point from the last surviving
    valid raw image; interior holes are intentionally left unused.
"""

import os
import struct
import sys
import zlib

import _sd_image_viewer_core as core

# ========================= PERSISTENT SD JOURNAL =========================
JOURNAL_MAGIC = 0x314A4453  # "SDJ1" little endian
JOURNAL_VERSION = 1
JOURNAL_BLOCK_A = core.SNAP_BASE_NEW - 2
JOURNAL_BLOCK_B = core.SNAP_BASE_NEW - 1
JOURNAL_FMT_NOCRC = '<6I'
JOURNAL_FMT = '<7I'
JOURNAL_SIZE = struct.calcsize(JOURNAL_FMT)


def _journal_open(path, write=False):
    flags = (os.O_RDWR if write else os.O_RDONLY) | getattr(os, 'O_BINARY', 0)
    return os.open(path, flags)


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
        'magic': vals[0], 'version': vals[1], 'sequence': vals[2],
        'next_block': vals[3], 'next_snap_id': vals[4],
        'snap_base': vals[5], 'crc32': vals[6], 'slot': slot,
    }
    payload = struct.pack(JOURNAL_FMT_NOCRC, *vals[:6])
    if rec['magic'] != JOURNAL_MAGIC or rec['version'] != JOURNAL_VERSION:
        return None
    if rec['snap_base'] != core.SNAP_BASE_NEW:
        return None
    if rec['next_block'] < core.SNAP_BASE_NEW:
        return None
    if rec['crc32'] != _journal_crc(payload):
        return None
    return rec


def _newer_journal(a, b):
    delta = (a['sequence'] - b['sequence']) & 0xFFFFFFFF
    return a if delta != 0 and delta < 0x80000000 else b


def read_journal(path):
    fd = _journal_open(path, False)
    try:
        a = _decode_journal(_read_block_fd(fd, JOURNAL_BLOCK_A), 0)
        b = _decode_journal(_read_block_fd(fd, JOURNAL_BLOCK_B), 1)
    finally:
        os.close(fd)
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
    raw = payload + struct.pack('<I', _journal_crc(payload))
    return raw + b'\x00' * (core.BLOCK_SIZE - len(raw))


def write_journal(path, next_block, next_snap_id, duplicate_if_new=True):
    """Atomically advance/rebuild the redundant A/B journal.

    A new card gets both sectors populated. Normal updates alternate sectors,
    matching the STM32 firmware's sequence-number selection logic.
    """
    if next_block < core.SNAP_BASE_NEW:
        raise ValueError(f"next_block must be >= {core.SNAP_BASE_NEW}")
    if next_snap_id < 0:
        raise ValueError("next_snap_id must be >= 0")

    current = read_journal(path)
    sequence = ((current['sequence'] + 1) & 0xFFFFFFFF) if current else 1
    slot = (1 - current['slot']) if current else 0
    block = JOURNAL_BLOCK_B if slot else JOURNAL_BLOCK_A
    raw = _pack_journal(sequence, next_block, next_snap_id)

    fd = _journal_open(path, True)
    try:
        _write_block_fd(fd, block, raw)
        verify = _decode_journal(_read_block_fd(fd, block), slot)
        if not verify:
            raise OSError("journal read-back verification failed")
        if verify['sequence'] != sequence or verify['next_block'] != next_block \
                or verify['next_snap_id'] != next_snap_id:
            raise OSError("journal read-back values do not match")
    finally:
        os.close(fd)

    if current is None and duplicate_if_new:
        return write_journal(path, next_block, next_snap_id, duplicate_if_new=False)
    return read_journal(path)


def _parse_raw_header(raw):
    """Strict raw-header validation matching the firmware storage format."""
    if raw is None or len(raw) < core.HEADER_SIZE:
        return None
    vals = struct.unpack_from('<11I', raw, 0)
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
        'width': width,
        'height': height,
        'pixel_format': pixel_format,
        'data_size': data_size,
        'snap_id': vals[7],
        'blocks': blocks,
    }


def find_last_valid_image(path, before_block, progress_cb=None):
    """Find the physically last surviving raw image before before_block.

    The scan moves backwards in 16 MB chunks. This is normally very fast when
    deleting the latest images, but it can safely scan all the way back to block
    3072 when all images were deleted. Interior holes are ignored and remain
    unused; only trailing free space is reclaimed.
    """
    before_block = max(int(before_block), core.SNAP_BASE_NEW)
    if before_block <= core.SNAP_BASE_NEW:
        return None

    chunk_blocks = (16 * 1024 * 1024) // core.BLOCK_SIZE
    magic = struct.pack('<I', core.HEADER_TAG)
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
                    f"Updating SD journal: checking blocks {start_block}..{end_block - 1} ...")

            pos = len(data)
            while pos > 0:
                hit = data.rfind(magic, 0, pos)
                if hit < 0:
                    break
                pos = hit
                if hit % core.BLOCK_SIZE != 0:
                    continue
                h = _parse_raw_header(data[hit:hit + core.HEADER_SIZE])
                if not h:
                    continue
                block = start_block + hit // core.BLOCK_SIZE
                image_end = block + h['blocks']
                if image_end <= before_block:
                    h['block'] = block
                    h['end_block'] = image_end
                    return h

            end_block = start_block
    finally:
        os.close(fd)
    return None


# ========================= DELETE + JOURNAL =========================
class JournalDeleteThread(core.QThread):
    """Zero-fill selected images, then safely move the journal to the last survivor."""
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
            # No shared cached read handle while raw write handles are open.
            core.close_drive()
            current = read_journal(self.drive)

            for i, snap in enumerate(snaps, 1):
                if self._cancel:
                    self.failed.emit("Cancelled — the remaining images were not erased.")
                    return
                h = snap.header
                data_size = h['data_size']
                bpp = 1 if h.get('pixel_format', 0) == 2 else 2
                expected_size = h['width'] * h['height'] * bpp
                if data_size <= 0 or abs(data_size - expected_size) > (core.BLOCK_SIZE * 4):
                    data_size = expected_size
                nb = (core.HEADER_SIZE + data_size + core.BLOCK_SIZE - 1) // core.BLOCK_SIZE
                self.progress.emit(
                    f"Erasing {i}/{total}: #{snap.idx+1:03d} @ block {snap.block} "
                    f"({nb * core.BLOCK_SIZE // (1024*1024)} MB) ...")
                core.zero_fill(self.drive, snap.block, nb)

            # A card without a journal is still allowed to delete images, but we
            # must not invent an append position from a potentially incomplete UI scan.
            if current is None:
                self.journal_note = (
                    "Images erased; no valid STM32 journal was found. Use Format SD "
                    "before reconnecting this card to the board.")
                self.finished_ok.emit(total)
                return

            # Recalculate from the physical card, not from the GUI list. This
            # safely handles prior holes, incomplete scans, Mode 0/4 mixtures and
            # the special case where no image remains.
            last = find_last_valid_image(
                self.drive,
                current['next_block'],
                progress_cb=lambda m: self.progress.emit(m),
            )
            if last is None:
                next_block = core.SNAP_BASE_NEW
                next_id = 0
            else:
                next_block = last['end_block']
                next_id = int(last['snap_id']) + 1

            if next_block != current['next_block'] or next_id != current['next_snap_id']:
                try:
                    rec = write_journal(self.drive, next_block, next_id)
                    self.journal_note = (
                        f"Journal adjusted: next block {rec['next_block']}, "
                        f"next image ID {rec['next_snap_id']}.")
                except Exception as exc:
                    # The images are already erased. Keep the operation successful
                    # and report the metadata problem separately; the old redundant
                    # journal remains the safe fallback in most failure cases.
                    self.journal_note = (
                        f"Images erased, but journal update failed: {exc}. "
                        "Do not capture until the card is prepared again.")
            else:
                self.journal_note = (
                    f"Images erased; journal unchanged at block {current['next_block']} "
                    f"/ ID {current['next_snap_id']}.")

            self.finished_ok.emit(total)
        except Exception as exc:
            self.failed.emit(str(exc))


# ========================= FORMAT + JOURNAL =========================
class JournalFormatThread(core.FormatThread):
    """Existing validated format flow plus automatic STM32 journal creation."""

    def _verify(self, size_hint):
        # Core FormatThread calls _verify only after the quick format and photo
        # area wipe completed. Initialize the journal before reporting success.
        core.close_drive()
        core.time.sleep(3)
        drives = core.get_drives()
        target = self._find_by_size(size_hint)
        if target is None and drives:
            target = int(drives[0].replace('\\\\.\\PhysicalDrive', ''))
        if target is None:
            return (
                "Format finished, but the card could not be re-located to initialize "
                "the STM32 journal. Reconnect it and run Format SD again.",
                drives,
            )

        path = f'\\\\.\\PhysicalDrive{target}'
        try:
            rec = write_journal(path, core.SNAP_BASE_NEW, 0)
            journal_msg = (
                f"STM32 journal ready (blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B}, "
                f"next block {rec['next_block']}, ID {rec['next_snap_id']})")
        except Exception as exc:
            return (
                f"Photo area was formatted, but STM32 journal initialization FAILED: {exc}",
                drives,
            )

        msg, fresh_drives = super()._verify(size_hint)
        return msg + "; " + journal_msg, fresh_drives


# Base format_card resolves FormatThread from its own module globals.
core.FormatThread = JournalFormatThread


# ========================= INTEGRATED UI =========================
class SDVisualizer(core.SDVisualizer):
    def __init__(self):
        self._last_delete_note = ""
        super().__init__()

    def _build_ui(self):
        super()._build_ui()

        # Add Format SD as the first action after the drive selector. The core
        # viewer already contains the full formatting workflow; it was simply
        # hidden in the previous UI.
        central_layout = self.centralWidget().layout()
        toolbar = central_layout.itemAt(0).widget()
        toolbar_layout = toolbar.layout()

        self.btn_format = core.QPushButton("⚠ Format SD")
        self.btn_format.setObjectName("DangerButton")
        self.btn_format.setToolTip(
            "Prepare a new SD card: format/wipe the photo area and create the STM32 append journal.")
        self.btn_format.clicked.connect(self.format_card)
        toolbar_layout.insertWidget(2, self.btn_format)

    def _set_ui_busy(self, busy, wait_drives=False):
        super()._set_ui_busy(busy, wait_drives)
        if hasattr(self, 'btn_format'):
            self.btn_format.setEnabled(not busy and not wait_drives)

    def _set_formatting_ui(self, active):
        super()._set_formatting_ui(active)
        if hasattr(self, 'btn_format'):
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
                f"Journal → next block {rec['next_block']}, ID {rec['next_snap_id']}.")
        else:
            status = (
                f"Scan complete. Found {len(self.snapshots)} snapshots. "
                "No valid STM32 journal — use Format SD before capturing.")

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
        total_mb = sum(
            (core.HEADER_SIZE + s.header['data_size']) // (1024 * 1024) for s in snaps)

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
        self.delete_thread.progress.connect(lambda m: self.statusBar().showMessage(m))
        self.delete_thread.finished_ok.connect(self._on_delete_ok)
        self.delete_thread.failed.connect(self._on_delete_failed)
        self.delete_thread.start()
        self._set_ui_busy(True)

    def _on_delete_ok(self, count):
        self._last_delete_note = getattr(self.delete_thread, 'journal_note', '')
        self._set_ui_busy(False)
        self.statusBar().showMessage(f"{count} image(s) erased. Re-scanning...")
        self.scan_snapshots()


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
    if os.name != 'nt':
        print("This script is designed for Windows PhysicalDrive access.")
        sys.exit(1)

    app = core.QApplication(sys.argv)
    _configure_app_palette(app)
    window = SDVisualizer()
    window.show()
    sys.exit(app.exec())
