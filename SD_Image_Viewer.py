"""
STM32 SD Card Snapshot Visualizer - minimal journal-reset wrapper.

All normal viewer behavior comes directly from the validated
_sd_image_viewer_core.py implementation. In particular, Scan Snapshots and
Delete Selected are NOT overridden here.

"Format SD" does not format the Windows filesystem. The expected workflow is:
  1. format the card externally when needed,
  2. Scan Snapshots,
  3. Format SD = delete every scanned STM32 photo using the original
     DeleteThread, then clear journal sectors A/B (3070/3071).

The STM32 firmware recreates a fresh journal at block 3072 / image ID 0 when
both journal copies are invalid/empty.
"""

import os
import sys

import _sd_image_viewer_core as core

JOURNAL_BLOCK_A = core.SNAP_BASE_NEW - 2
JOURNAL_BLOCK_B = core.SNAP_BASE_NEW - 1


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
            "STM32 reset only: delete all photos found by the last Scan and "
            "clear journal blocks 3070/3071. Does NOT format FAT32."
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

    def _reset_journal_sectors(self):
        """Clear only the two 512-byte journal sectors using the viewer's
        original, already-validated raw zero_fill() implementation."""
        core.zero_fill(self.drive, JOURNAL_BLOCK_A, 2)

    def format_card(self):
        """Delete ALL snapshots from the last Scan, then clear journal A/B."""
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
                "that were found by that scan.",
            )
            return

        drive_num = self.drive.replace("\\\\.\\PhysicalDrive", "")
        reply = core.dialog(
            self,
            "question",
            "⚠ RESET STM32 SD STORAGE",
            f"PhysicalDrive{drive_num}\n\n"
            f"This will delete ALL {len(self.snapshots)} STM32 photo(s) found "
            "by the last Scan and clear journal blocks 3070/3071.\n\n"
            "It does NOT format the Windows filesystem.\n\nContinue?",
        )
        if reply != core.QMessageBox.StandardButton.Yes:
            return

        self._set_formatting_ui(True)

        if not self.snapshots:
            self.statusBar().showMessage("Clearing STM32 journal A/B...")
            try:
                self._reset_journal_sectors()
            except Exception as exc:
                self._set_formatting_ui(False)
                core.dialog(self, "critical", "Reset failed", str(exc))
                return
            self._finish_format_reset(0)
            return

        # IMPORTANT: use the exact same DeleteThread as the original
        # Delete Selected button. No alternate writer, wrapper, Win32 API, or
        # journal update is involved while deleting photos.
        self.format_thread = core.DeleteThread(self.drive, list(self.snapshots))
        self.format_thread.progress.connect(
            lambda msg: self.statusBar().showMessage(msg)
        )
        self.format_thread.finished_ok.connect(self._on_format_delete_ok)
        self.format_thread.failed.connect(self._on_format_delete_failed)
        self.format_thread.start()

    @core.Slot(int)
    def _on_format_delete_ok(self, count):
        self.statusBar().showMessage(
            f"{count} photo(s) erased. Clearing journal A/B..."
        )
        try:
            self._reset_journal_sectors()
        except Exception as exc:
            self._set_formatting_ui(False)
            core.dialog(
                self,
                "critical",
                "Journal reset failed",
                f"All scanned photos were erased, but journal blocks "
                f"{JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B} could not be cleared:\n{exc}",
            )
            return
        self._finish_format_reset(count)

    @core.Slot(str)
    def _on_format_delete_failed(self, msg):
        self._set_formatting_ui(False)
        self.statusBar().showMessage("Format SD stopped during photo deletion.")
        core.dialog(
            self,
            "critical",
            "Delete failed",
            "The original DeleteThread reported an error:\n" + msg,
        )

    def _finish_format_reset(self, count):
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
            f"Journal blocks {JOURNAL_BLOCK_A}/{JOURNAL_BLOCK_B} are empty.\n\n"
            "On the next STM32 capture, the firmware will create a fresh "
            f"journal at block {core.SNAP_BASE_NEW}, image ID 0.",
        )


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

    window = SDVisualizer()
    window.show()
    sys.exit(app.exec())
