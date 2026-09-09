"""
STM32 SD Card Snapshot Visualizer - Windows-safe raw erase wrapper.

Normal viewer behavior remains in _sd_image_viewer_core.py.
Raw writes are performed with the original core.zero_fill(), but Windows volumes
belonging to the selected PhysicalDrive are temporarily locked during Delete and
Format SD. This is required by Windows for raw writes inside a mounted volume.

Format SD does NOT format the filesystem:
  - first run Scan Snapshots,
  - delete every scanned STM32 photo,
  - clear journal blocks 3070/3071.
"""

import ctypes
import os
import subprocess
import sys
import time
from ctypes import wintypes

import _sd_image_viewer_core as core

JOURNAL_BLOCK_A = core.SNAP_BASE_NEW - 2
JOURNAL_BLOCK_B = core.SNAP_BASE_NEW - 1

_GENERIC_READ = 0x80000000
_GENERIC_WRITE = 0x40000000
_FILE_SHARE_READ = 0x00000001
_FILE_SHARE_WRITE = 0x00000002
_OPEN_EXISTING = 3
_FSCTL_LOCK_VOLUME = 0x00090018
_FSCTL_UNLOCK_VOLUME = 0x0009001C
_INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

_kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
_kernel32.CreateFileW.argtypes = [
    wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
    wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
]
_kernel32.CreateFileW.restype = wintypes.HANDLE
_kernel32.DeviceIoControl.argtypes = [
    wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD,
    wintypes.LPVOID, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD),
    wintypes.LPVOID,
]
_kernel32.DeviceIoControl.restype = wintypes.BOOL
_kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
_kernel32.CloseHandle.restype = wintypes.BOOL


def _disk_number(path):
    prefix = r"\\.\PhysicalDrive"
    if not path.startswith(prefix):
        raise ValueError(f"Not a PhysicalDrive path: {path}")
    return int(path[len(prefix):])


def _volume_letters_for_disk(path):
    disk = _disk_number(path)
    ps = (
        f"Get-Partition -DiskNumber {disk} -ErrorAction SilentlyContinue | "
        "Where-Object { $_.DriveLetter } | "
        "ForEach-Object { $_.DriveLetter }"
    )
    result = subprocess.run(
        ["powershell", "-NoProfile", "-Command", ps],
        capture_output=True,
        text=True,
        timeout=15,
    )
    if result.returncode != 0:
        raise OSError(
            f"Could not identify the mounted volume for PhysicalDrive{disk}: "
            f"{(result.stderr or result.stdout).strip()}"
        )
    letters = []
    for line in (result.stdout or "").splitlines():
        value = line.strip().upper()
        if len(value) == 1 and value.isalpha() and value not in letters:
            letters.append(value)
    return letters


def _ioctl(handle, code):
    returned = wintypes.DWORD(0)
    ctypes.set_last_error(0)
    ok = _kernel32.DeviceIoControl(
        handle, code, None, 0, None, 0, ctypes.byref(returned), None
    )
    return bool(ok), ctypes.get_last_error()


class _LockedVolumes:
    def __init__(self, drive):
        self.drive = drive
        self.handles = []

    def __enter__(self):
        # Scan/preview caches a read-only raw handle. It must be closed before
        # Windows can grant an exclusive lock for raw writes to the mounted SD.
        core.close_drive()
        time.sleep(0.1)

        letters = _volume_letters_for_disk(self.drive)
        try:
            for letter in letters:
                volume_path = rf"\\.\{letter}:"
                ctypes.set_last_error(0)
                handle = _kernel32.CreateFileW(
                    volume_path,
                    _GENERIC_READ | _GENERIC_WRITE,
                    _FILE_SHARE_READ | _FILE_SHARE_WRITE,
                    None,
                    _OPEN_EXISTING,
                    0,
                    None,
                )
                if handle == _INVALID_HANDLE_VALUE:
                    code = ctypes.get_last_error()
                    raise OSError(
                        code,
                        f"Cannot open volume {letter}: for raw-write lock: "
                        f"{ctypes.FormatError(code).strip()}",
                    )

                locked = False
                last_error = 0
                for _ in range(8):
                    locked, last_error = _ioctl(handle, _FSCTL_LOCK_VOLUME)
                    if locked:
                        break
                    time.sleep(0.15)

                if not locked:
                    _kernel32.CloseHandle(handle)
                    raise OSError(
                        last_error,
                        f"Cannot lock volume {letter}: for raw SD writing: "
                        f"{ctypes.FormatError(last_error).strip()}. "
                        "Close any Explorer window or program using the SD card and retry.",
                    )
                self.handles.append(handle)
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, exc_type, exc, tb):
        for handle in reversed(self.handles):
            try:
                _ioctl(handle, _FSCTL_UNLOCK_VOLUME)
            finally:
                _kernel32.CloseHandle(handle)
        self.handles.clear()
        return False


class LockedEraseThread(core.QThread):
    """Original delete algorithm, but with the Windows volume locked."""

    progress = core.Signal(str)
    finished_ok = core.Signal(int)
    failed = core.Signal(str)

    def __init__(self, drive, snapshots, clear_journal=False):
        super().__init__()
        self.drive = drive
        self.snapshots = list(snapshots)
        self.clear_journal = bool(clear_journal)
        self._cancel = False

    def cancel(self):
        self._cancel = True

    def run(self):
        snaps = sorted(self.snapshots, key=lambda s: s.block, reverse=True)
        total = len(snaps)
        try:
            with _LockedVolumes(self.drive):
                for i, snap in enumerate(snaps, 1):
                    if self._cancel:
                        self.failed.emit(
                            "Cancelled — the remaining images were not erased."
                        )
                        return

                    # Same block calculation used by the validated DeleteThread.
                    h = snap.header
                    data_size = h["data_size"]
                    expected_size = h["width"] * h["height"] * 2
                    if (
                        data_size <= 0
                        or abs(data_size - expected_size) > core.BLOCK_SIZE * 4
                    ):
                        data_size = expected_size
                    nb = (
                        core.HEADER_SIZE + data_size + core.BLOCK_SIZE - 1
                    ) // core.BLOCK_SIZE

                    self.progress.emit(
                        f"Erasing {i}/{total}: #{snap.idx+1:03d} @ block "
                        f"{snap.block} ({nb * core.BLOCK_SIZE // (1024*1024)} MB) ..."
                    )
                    core.zero_fill(self.drive, snap.block, nb)

                if self.clear_journal:
                    self.progress.emit(
                        f"Clearing journal blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B} ..."
                    )
                    core.zero_fill(self.drive, JOURNAL_BLOCK_A, 2)

            self.finished_ok.emit(total)
        except Exception as exc:
            self.failed.emit(str(exc))


class SDVisualizer(core.SDVisualizer):
    def __init__(self):
        self._scan_completed = False
        super().__init__()

    def _build_ui(self):
        super()._build_ui()

        central_layout = self.centralWidget().layout()
        toolbar = central_layout.itemAt(0).widget()
        toolbar_layout = toolbar.layout()

        self.btn_format = core.QPushButton("⚠ Format SD")
        self.btn_format.setObjectName("DangerButton")
        self.btn_format.setToolTip(
            "Delete all photos found by the last Scan and clear journal "
            "blocks 3070/3071. Does NOT format FAT32."
        )
        self.btn_format.clicked.connect(self.format_card)
        toolbar_layout.insertWidget(2, self.btn_format)

    def set_drive(self, index):
        self._scan_completed = False
        super().set_drive(index)

    def refresh_drives(self, disk_infos=None):
        self._scan_completed = False
        super().refresh_drives(disk_infos)

    @core.Slot(list)
    def _on_scan_finished(self, results):
        super()._on_scan_finished(results)
        self._scan_completed = True

    @core.Slot(str)
    def _on_scan_error(self, msg):
        self._scan_completed = False
        super()._on_scan_error(msg)

    def _set_ui_busy(self, busy, wait_drives=False):
        super()._set_ui_busy(busy, wait_drives)
        if hasattr(self, "btn_format"):
            self.btn_format.setEnabled(not busy and not wait_drives)

    def _set_formatting_ui(self, active):
        super()._set_formatting_ui(active)
        if hasattr(self, "btn_format"):
            self.btn_format.setEnabled(not active)

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
            "They will no longer be found by the scanner.\n\nContinue?",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        self.delete_thread = LockedEraseThread(self.drive, snaps)
        self.delete_thread.progress.connect(
            lambda msg: self.statusBar().showMessage(msg)
        )
        self.delete_thread.finished_ok.connect(self._on_delete_ok)
        self.delete_thread.failed.connect(self._on_delete_failed)
        self.delete_thread.start()
        self._set_ui_busy(True)

    def format_card(self):
        if not self.drive:
            core.dialog(self, "warning", "Error", "Select a drive first.")
            return
        if self._op_running():
            self.statusBar().showMessage(
                "Please wait for the current operation to finish..."
            )
            return
        if not self._scan_completed:
            core.dialog(
                self,
                "warning",
                "Scan required",
                "Run 'Scan Snapshots' first. Format SD only deletes photos "
                "found by that scan.",
            )
            return

        drive_num = self.drive.replace(r"\\.\PhysicalDrive", "")
        reply = core.dialog(
            self,
            "question",
            "⚠ RESET STM32 SD STORAGE",
            f"PhysicalDrive{drive_num}\n\n"
            f"Delete ALL {len(self.snapshots)} scanned STM32 photo(s) and "
            "clear journal blocks 3070/3071?\n\n"
            "The Windows filesystem is NOT formatted.",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        self._set_formatting_ui(True)
        self.format_thread = LockedEraseThread(
            self.drive, list(self.snapshots), clear_journal=True
        )
        self.format_thread.progress.connect(
            lambda msg: self.statusBar().showMessage(msg)
        )
        self.format_thread.finished_ok.connect(self._on_format_ok_simple)
        self.format_thread.failed.connect(self._on_format_failed_simple)
        self.format_thread.start()

    @core.Slot(int)
    def _on_format_ok_simple(self, count):
        self._set_formatting_ui(False)
        self._scan_completed = False
        self.listbox.clear()
        self.snapshots.clear()
        self.current_snap = None
        self.statusBar().showMessage(
            f"SD reset complete: {count} photo(s) deleted; journal A/B cleared."
        )
        core.dialog(
            self,
            "info",
            "SD ready",
            f"Deleted {count} STM32 photo(s).\n"
            f"Journal blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B} are empty.",
        )

    @core.Slot(str)
    def _on_format_failed_simple(self, msg):
        self._set_formatting_ui(False)
        self.statusBar().showMessage("Format SD failed.")
        core.dialog(self, "critical", "Format SD failed", msg)


if __name__ == "__main__":
    if os.name != "nt":
        print("This script is designed for Windows PhysicalDrive access.")
        sys.exit(1)

    app = core.QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = core.QPalette()
    palette.setColor(core.QPalette.ColorRole.Window, core.QColor(240, 242, 245))
    palette.setColor(core.QPalette.ColorRole.WindowText, core.QColor(51, 51, 51))
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

    window = SDVisualizer()
    window.show()
    sys.exit(app.exec())
