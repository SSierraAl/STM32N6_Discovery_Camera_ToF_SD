"""MASSIF Studio: native Qt frontend for the STM32 SD snapshot viewer."""

from collections import OrderedDict
from pathlib import Path
import os
import sys

from PySide6.QtCore import QAbstractTableModel, QModelIndex, QSortFilterProxyModel, QTimer, Qt, Slot
from PySide6.QtGui import QAction, QIcon, QImage, QKeySequence, QPixmap
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFrame, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QSizePolicy, QSplitter, QStackedWidget, QTableView, QVBoxLayout, QWidget,
)

import _sd_image_viewer_core as core
from raw_image_metadata import image_filename, image_kind_label, time_label, utc_datetime

CACHE_BYTES = 96 * 1024 * 1024
COLOR_PROFILE = True
THEME = """
QMainWindow, QWidget#Workspace { background: #edf1ec; }
QWidget { color: #20332b; font: 10pt 'Segoe UI'; }
QFrame#Masthead { background: #152e25; border: 0; }
QFrame#Masthead QLabel { color: #f1f7ee; background: transparent; }
QLabel#Product { font-size: 19pt; font-weight: 700; }
QLabel#Edition { font-size: 9pt; color: #b4cdbd; }
QFrame#Card { background: #fffffe; border: 1px solid #dae4db; border-radius: 13px; }
QFrame#Canvas { background: #f4f7f3; border: 1px solid #dae4db; border-radius: 10px; }
QGraphicsView { background: #f4f7f3; border: 0; }
QWidget#Empty { background: #f4f7f3; }
QWidget#Empty QLabel { background: transparent; color: #4f695b; }
QLabel#EmptyTitle { font-size: 20pt; font-weight: 650; }
QLabel#Section { color: #20332b; font-size: 13pt; font-weight: 700; }
QLabel#Caption, QLabel#Count, QLabel#Meta, QLabel#Hint { color: #4f695b; font-size: 9pt; }
QLabel#Count { background: #e7f1e6; border-radius: 9px; padding: 4px 9px; font-weight: 650; }
QLabel#PreviewName { font-size: 12pt; font-weight: 650; }
QPushButton, QComboBox, QLineEdit {
    background: #f8faf7; border: 1px solid #cddbd0; border-radius: 7px; min-height: 24px;
    padding: 5px 11px;
}
QPushButton:hover, QComboBox:hover { background: #e7f0e7; border-color: #91ab99; }
QPushButton:pressed { background: #d7e6d8; }
QPushButton:disabled, QComboBox:disabled { color: #68786d; background: #eef2ee; border-color: #dce4dc; }
QPushButton:focus, QComboBox:focus, QLineEdit:focus { border-color: #33805c; }
QPushButton#Primary { background: #276045; color: white; border-color: #276045; font-weight: 650; }
QPushButton#Primary:hover { background: #1a4d36; }
QPushButton#Primary:disabled { background: #e7eee7; color: #65796b; border-color: #d7e2d7; }
QPushButton#Danger { color: #96443e; }
QComboBox QAbstractItemView {
    color: #20332b; background: #fffffe; border: 1px solid #cddbd0;
    selection-background-color: #dbead9; selection-color: #173f2b; padding: 5px;
}
QTableView {
    background: #fffffe; alternate-background-color: #f6f9f5; color: #20332b;
    border: 1px solid #e1e9e0; border-radius: 7px; outline: none;
    selection-background-color: #dbead9; selection-color: #173f2b;
    gridline-color: #edf1ed;
}
QTableView::item { padding: 0 8px; }
QTableView QScrollBar, QGraphicsView QScrollBar {
    background: #ffffff; border: 0; margin: 0;
}
QTableView QScrollBar:horizontal, QGraphicsView QScrollBar:horizontal { height: 13px; }
QTableView QScrollBar:vertical, QGraphicsView QScrollBar:vertical { width: 13px; }
QTableView QScrollBar::handle, QGraphicsView QScrollBar::handle {
    background: #c7d5cb; border: 2px solid #ffffff; border-radius: 6px;
}
QTableView QScrollBar::handle:horizontal, QGraphicsView QScrollBar::handle:horizontal { min-width: 24px; }
QTableView QScrollBar::handle:vertical, QGraphicsView QScrollBar::handle:vertical { min-height: 24px; }
QTableView QScrollBar::handle:hover, QGraphicsView QScrollBar::handle:hover { background: #9cb5a4; }
QTableView QScrollBar::add-line, QTableView QScrollBar::sub-line,
QGraphicsView QScrollBar::add-line, QGraphicsView QScrollBar::sub-line {
    background: transparent; border: 0; width: 0; height: 0;
}
QTableView QScrollBar::add-page, QTableView QScrollBar::sub-page,
QGraphicsView QScrollBar::add-page, QGraphicsView QScrollBar::sub-page {
    background: transparent;
}
QTableView::corner, QGraphicsView::corner { background: #ffffff; }
QHeaderView::section {
    border: none; border-bottom: 1px solid #dce6dd; background: #edf3ec;
    color: #365244; padding: 8px; font-weight: 650;
}
QSplitter::handle { background: transparent; }
QSplitter::handle:hover { background: #c5d9ca; }
QStatusBar { background: #e4ede4; color: #375345; }
QStatusBar::item { border: none; }
QToolTip { background: #f9fbf8; color: #20332b; border: 1px solid #a8c0ac; padding: 5px; }
"""


def styled_label(text, name):
    label = QLabel(text)
    label.setObjectName(name)
    label.setTextFormat(Qt.TextFormat.PlainText)
    return label


class CaptureModel(QAbstractTableModel):
    """Store metadata only; pixels are fetched on demand from the SD card."""

    COLUMNS = ("No.", "Resolution", "Date", "Time", "Category", "Name")

    def __init__(self, parent):
        super().__init__(parent)
        self.rows = []

    def replace(self, snapshots):
        rows = []
        for snap in snapshots:
            h = snap.header
            dt = utc_datetime(h)
            rows.append((
                f"#{snap.idx + 1:03d}", f"{h['width']} × {h['height']}",
                dt.date().isoformat() if dt else "—",
                dt.strftime("%H:%M") if dt else "—",
                (image_kind_label(h) or "Unspecified").replace("_", " "),
                image_filename(h, snap.idx, "original"),
                time_label(h),
            ))
        self.beginResetModel()
        self.rows = rows
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(self.rows)

    def columnCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(self.COLUMNS)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None
        row = self.rows[index.row()]
        if role == Qt.ItemDataRole.DisplayRole:
            return row[index.column()]
        if role == Qt.ItemDataRole.UserRole:
            value = row[index.column()]
            # Match filenames with either underscores or spaces.
            return f"{value} {value.replace('_', ' ')}" if index.column() == 5 else value
        if role == Qt.ItemDataRole.ToolTipRole:
            return row[-1] if index.column() in (2, 3) else row[index.column()]
        return None

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if orientation == Qt.Orientation.Horizontal and role == Qt.ItemDataRole.DisplayRole:
            return self.COLUMNS[section]
        if (orientation == Qt.Orientation.Horizontal and role == Qt.ItemDataRole.ToolTipRole
                and section in (2, 3)):
            return "Capture timestamp (UTC)"
        return None


class CaptureTable(QTableView):
    """Expose row selection in SD scan order for the existing delete handler."""

    def __init__(self, parent):
        super().__init__(parent)
        self.captures = CaptureModel(self)
        self.filtered = QSortFilterProxyModel(self)
        self.filtered.setSourceModel(self.captures)
        self.filtered.setFilterKeyColumn(-1)
        self.filtered.setFilterRole(Qt.ItemDataRole.UserRole)
        self.filtered.setFilterCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.setModel(self.filtered)
        self.setSelectionMode(self.SelectionMode.ExtendedSelection)
        self.setSelectionBehavior(self.SelectionBehavior.SelectRows)
        self.setEditTriggers(self.EditTrigger.NoEditTriggers)
        self.setAlternatingRowColors(True)
        self.setShowGrid(False)
        self.verticalHeader().hide()
        self.verticalHeader().setDefaultSectionSize(37)
        self.horizontalHeader().setStretchLastSection(True)
        for i, width in enumerate((58, 104, 108, 66, 128, 135)):
            self.setColumnWidth(i, width)
        self.setWordWrap(False)

    def clear(self):
        self.captures.replace([])

    def selectedItems(self):
        return self.selectionModel().selectedRows()

    def row(self, index):
        return self.filtered.mapToSource(index).row()


class StudioCanvas(core.InteractiveGraphicsView):
    """Fit after a resize until the user zooms; double click returns to fit."""

    def __init__(self):
        self.auto_fit = True
        super().__init__()
        self.setFrameShape(QFrame.Shape.NoFrame)

    def fit_to_window(self):
        self.auto_fit = True
        super().fit_to_window()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.auto_fit:
            self.fit_to_window()

    def mouseDoubleClickEvent(self, event):
        self.fit_to_window()
        event.accept()

    def set_explicit_scale(self, scale_factor):
        self.auto_fit = False
        super().set_explicit_scale(scale_factor)

    def wheelEvent(self, event):
        if self.image_item.pixmap().isNull():
            return
        delta = event.angleDelta().y()
        if delta:
            factor = 1.15 ** (delta / 120)
            target = min(8.0, max(0.04, self.current_scale * factor))
            self.scale(target / self.current_scale, target / self.current_scale)
            self.auto_fit = False
            self._update_scale()
        event.accept()


class PreviewJob(core.LoadImageThread):
    """Reuse the existing SD decoder, then render a QImage off the GUI thread."""

    rendered = core.Signal(QImage, QImage, int)

    def __init__(self, drive, snapshot, seq, settings, decoded, parent):
        super().__init__(drive, snapshot, seq)
        self.setParent(parent)
        self.settings = settings
        self.decoded = decoded
        self.image_ready.connect(self._render, Qt.ConnectionType.DirectConnection)

    def run(self):
        if self.decoded is not None:
            self._render(self.decoded, self.snapshot, self.seq)
        else:
            super().run()

    def _render(self, image, snapshot, seq):
        try:
            self.rendered.emit(image, core.presentation_image(
                image, self.settings[0], COLOR_PROFILE, self.settings[1]), seq)
        except Exception as exc:
            self.load_failed.emit(snapshot, seq, str(exc))


class SaveJob(core.QThread):
    result = core.Signal(bool, str)

    def __init__(self, image, path, parent):
        super().__init__(parent)
        self.image = image
        self.path = path

    def run(self):
        try:
            self.result.emit(self.image.save(str(self.path)), str(self.path))
        except Exception as exc:
            self.result.emit(False, str(exc))


class StudioUI:
    """Native frontend layered on the Windows-safe SD viewer backend."""

    def __init__(self):
        self._cache = OrderedDict()
        self._cache_size = 0
        self._pending = None
        self._preview_job = None
        self._save_job = None
        self._displayed = None
        self._closing = False
        self._busy = False
        self._scan_cancelled = False
        self._rescan_pending = False
        super().__init__()
        self.setWindowTitle("MASSIF Studio")
        self.resize(1480, 900)
        self.setMinimumSize(1040, 660)

    def _button(self, label, action, name=""):
        button = QPushButton(label)
        button.setObjectName(name)
        button.clicked.connect(action)
        return button

    def _shortcut(self, keys, callback):
        action = QAction(self)
        action.setShortcut(QKeySequence(keys))
        action.triggered.connect(callback)
        self.addAction(action)

    def _build_ui(self):
        root = QWidget()
        root.setObjectName("Workspace")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        head = QFrame()
        head.setObjectName("Masthead")
        mast = QHBoxLayout(head)
        mast.setContentsMargins(22, 11, 22, 11)
        mast.setSpacing(13)
        logo = Path(__file__).resolve().parent / "Logos" / "Massif_logo_white.png"
        pixmap = QPixmap(str(logo))
        if not pixmap.isNull():
            self.setWindowIcon(QIcon(pixmap))
            mark = QLabel()
            mark.setPixmap(pixmap.scaled(48, 48, Qt.AspectRatioMode.KeepAspectRatio,
                                         Qt.TransformationMode.SmoothTransformation))
            mast.addWidget(mark)
        brand = QVBoxLayout()
        brand.setSpacing(2)
        brand.addWidget(styled_label("MASSIF Studio", "Product"))
        brand.addWidget(styled_label("FIELD IMAGING  /  SNAPSHOT WORKSPACE", "Edition"))
        mast.addLayout(brand)
        mast.addStretch()
        mast.addWidget(styled_label("STM32N6  ·  SD CAPTURES", "Edition"))
        layout.addWidget(head)

        body = QVBoxLayout()
        body.setContentsMargins(16, 14, 16, 12)
        body.setSpacing(12)
        layout.addLayout(body, 1)
        source = QFrame()
        source.setObjectName("Card")
        source_line = QHBoxLayout(source)
        source_line.setContentsMargins(13, 8, 13, 8)
        source_line.addWidget(styled_label("SD SOURCE", "Caption"))
        self.drive_combo = QComboBox()
        self.drive_combo.setMinimumWidth(225)
        self.drive_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.drive_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.drive_combo.currentIndexChanged.connect(self.set_drive)
        source_line.addWidget(self.drive_combo, 1)
        self.btn_refresh = self._button("Refresh", self.request_drive_list)
        self.btn_scan = self._button("Scan SD", self.scan_snapshots, "Primary")
        self.btn_cancel_scan = self._button("Cancel scan", self.cancel_scan, "Danger")
        self.btn_disconnect = self._button("Disconnect", self.disconnect_drive)
        self.btn_cancel_fmt = self._button("Cancel reset", self.cancel_format, "Danger")
        self.btn_format = self._button("Reset SD catalog…", self.format_card, "Danger")
        self.btn_format.setToolTip("Erase scanned STM32 snapshots and reset their journal. Keeps the filesystem.")
        for button in (self.btn_refresh, self.btn_scan, self.btn_cancel_scan,
                       self.btn_disconnect, self.btn_cancel_fmt, self.btn_format):
            source_line.addWidget(button)
        self.btn_cancel_scan.hide()
        self.btn_cancel_fmt.hide()
        body.addWidget(source)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(12)
        splitter.setChildrenCollapsible(False)
        body.addWidget(splitter, 1)
        catalog = QFrame()
        catalog.setObjectName("Card")
        catalog.setMinimumWidth(380)
        side = QVBoxLayout(catalog)
        side.setContentsMargins(13, 12, 13, 12)
        side.setSpacing(10)
        heading = QHBoxLayout()
        heading.addWidget(styled_label("Capture library", "Section"))
        heading.addStretch()
        self.count_label = styled_label("0 captures", "Count")
        heading.addWidget(self.count_label)
        side.addLayout(heading)
        self.search = QLineEdit()
        self.search.setPlaceholderText("Filter by name, category, date or time")
        self.search.setClearButtonEnabled(True)
        side.addWidget(self.search)
        self.listbox = CaptureTable(catalog)
        self.search.textChanged.connect(self._filter)
        self.listbox.selectionModel().currentRowChanged.connect(self._on_current_row)
        self.listbox.selectionModel().selectionChanged.connect(self._selection_changed)
        self.listbox.captures.modelReset.connect(self._selection_changed)
        side.addWidget(self.listbox, 1)
        self.selection_label = styled_label("0 selected", "Hint")
        side.addWidget(self.selection_label)
        actions = QHBoxLayout()
        self.btn_extract = self._button("Export selected", self.extract_selected, "Primary")
        self.btn_download_all = self._button("Export all", self.download_all)
        self.btn_delete = self._button("Delete selected…", self.delete_selected, "Danger")
        # The inherited Windows wrapper expects this reference when updating action state.
        self._selected_download_button = self.btn_extract
        for button in (self.btn_extract, self.btn_download_all, self.btn_delete):
            actions.addWidget(button)
        side.addLayout(actions)
        splitter.addWidget(catalog)

        preview = QFrame()
        preview.setObjectName("Card")
        right = QVBoxLayout(preview)
        right.setContentsMargins(13, 12, 13, 12)
        right.setSpacing(10)
        top = QHBoxLayout()
        titles = QVBoxLayout()
        titles.setSpacing(4)
        titles.addWidget(styled_label("IMAGE PREVIEW", "Caption"))
        self.image_name_label = styled_label("Select a capture", "PreviewName")
        self.image_name_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.image_name_label.setWordWrap(True)
        titles.addWidget(self.image_name_label)
        self.metadata = styled_label("Source resolution and capture time appear here.", "Meta")
        self.metadata.setWordWrap(True)
        titles.addWidget(self.metadata)
        top.addLayout(titles, 1)
        self.btn_save = self._button("Save PNG", self.save_image, "Primary")
        self.btn_save.setToolTip("Save at the original pixel dimensions (Ctrl+S).")
        top.addWidget(self.btn_save, alignment=Qt.AlignmentFlag.AlignTop)
        right.addLayout(top)

        canvas = QFrame()
        canvas.setObjectName("Canvas")
        stage = QVBoxLayout(canvas)
        stage.setContentsMargins(7, 7, 7, 7)
        self.stack = QStackedWidget()
        stage.addWidget(self.stack)
        empty = QWidget()
        empty.setObjectName("Empty")
        intro = QVBoxLayout(empty)
        intro.addStretch()
        self.empty_title = styled_label("A closer look at every capture", "EmptyTitle")
        self.empty_hint = styled_label("Select an SD source, then scan to open your capture library.", "Hint")
        for label in (self.empty_title, self.empty_hint):
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setWordWrap(True)
            intro.addWidget(label)
        intro.addStretch()
        self.empty = empty
        self.viewer = StudioCanvas()
        self.stack.addWidget(empty)
        self.stack.addWidget(self.viewer)
        right.addWidget(canvas, 1)

        controls = QHBoxLayout()
        self.contrast_combo = QComboBox()
        self.contrast_combo.addItem("Original contrast", False)
        self.contrast_combo.addItem("Mild contrast", True)
        self.noise_combo = QComboBox()
        self.noise_combo.addItem("Off", 0)
        self.noise_combo.addItem("Color + luminance grain reduction", 2)
        for label, combo in (("Appearance", self.contrast_combo), ("Noise reduction", self.noise_combo)):
            controls.addWidget(styled_label(label, "Caption"))
            controls.addWidget(combo)
            combo.currentIndexChanged.connect(self._refresh_appearance)
        controls.addStretch()
        right.addLayout(controls)
        right.addWidget(styled_label("Scroll to zoom  ·  Drag to pan  ·  Double-click to fit", "Hint"))
        splitter.addWidget(preview)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([640, 785])
        self.statusBar().setSizeGripEnabled(False)
        self.statusBar().showMessage("Ready · Windows raw SD access requires Administrator.")
        self._shortcut("Ctrl+F", self.search.setFocus)
        self._shortcut("Ctrl+S", self.save_image)
        self._selection_changed()

    def _settings(self):
        return self.contrast_combo.currentData(), self.noise_combo.currentData()

    def _selection_changed(self, *_args):
        visible = self.listbox.filtered.rowCount()
        total = self.listbox.captures.rowCount()
        self.count_label.setText(f"{visible:,} / {total:,}" if visible != total else f"{total:,} captures")
        selected = len(self.listbox.selectedItems())
        self.selection_label.setText(f"{selected:,} selected")
        self.btn_scan.setEnabled(bool(self.drive) and not self._busy)
        self.btn_extract.setEnabled(bool(selected) and not self._busy)
        self.btn_delete.setEnabled(bool(selected) and not self._busy)
        self.btn_download_all.setEnabled(bool(total) and not self._busy)
        self.btn_format.setEnabled(getattr(self, "_scan_completed", False) and not self._busy)
        self.btn_save.setEnabled(self._displayed is not None and not self._busy and self._save_job is None)
        for control in (self.listbox, self.search, self.contrast_combo, self.noise_combo):
            control.setEnabled(not self._busy)

    def _filter(self, text):
        self.listbox.filtered.setFilterFixedString(text)
        if not self.listbox.currentIndex().isValid():
            self._clear_preview()
        self._selection_changed()

    def _clear_preview(self):
        self._load_seq += 1
        self._pending = None
        self.current_snap = self._displayed = None
        self.image_name_label.setText("Select a capture")
        self.metadata.setText("Source resolution and capture time appear here.")
        self.empty_title.setText("Select a capture" if self.snapshots else "Ready to explore")
        self.empty_hint.setText("Choose a row in the capture library." if self.snapshots
                                else "Select an SD source, then scan to open your capture library.")
        self.stack.setCurrentWidget(self.empty)
        self._selection_changed()

    def _reset_catalog(self):
        self._clear_preview()
        self._cache.clear()
        self._cache_size = 0
        self.listbox.clear()
        self.snapshots = []
        self._scan_completed = False
        self._selection_changed()

    def set_drive(self, index):
        previous = self.drive
        super().set_drive(index)
        if self.drive != previous:
            self._reset_catalog()

    def refresh_drives(self, disk_infos=None):
        super().refresh_drives(disk_infos)
        # Windows can reuse a PhysicalDrive number after a card swap.
        self._reset_catalog()

    def request_drive_list(self):
        if not super()._op_running() and self._preview_job is None and not self._busy:
            super().request_drive_list()

    def disconnect_drive(self):
        if not self._op_running() and not self._busy:
            super().disconnect_drive()
            self.drive_combo.setCurrentIndex(-1)
            self._reset_catalog()
            self.statusBar().showMessage("SD source disconnected.")

    def _op_running(self):
        return self._busy or self._preview_job is not None or super()._op_running()

    def _set_ui_busy(self, busy, wait_drives=False):
        self._busy = busy
        super()._set_ui_busy(busy, wait_drives)
        self._selection_changed()

    def _set_formatting_ui(self, active):
        self._busy = active
        super()._set_formatting_ui(active)
        self._selection_changed()

    def scan_snapshots(self):
        if self._closing:
            return
        if self.drive and not self._op_running():
            self._scan_cancelled = False
            self._reset_catalog()
            super().scan_snapshots()
        elif self.drive:
            self.statusBar().showMessage("Please wait for the current operation.")

    @Slot(list)
    def _on_scan_finished(self, results):
        if self._scan_cancelled or self._closing:
            return
        self._clear_preview()
        self.snapshots = results
        self.listbox.captures.replace(results)
        self._scan_completed = True
        self._set_ui_busy(False)
        self.btn_cancel_scan.hide()
        self.statusBar().showMessage(f"Scan complete · {len(results):,} captures.")

    @Slot(str)
    def _on_scan_error(self, message):
        if self._scan_cancelled or self._closing:
            return
        super()._on_scan_error(message)
        self.btn_cancel_scan.hide()

    def cancel_scan(self):
        if getattr(self, "scan_thread", None) is None or not self._busy:
            return
        self._scan_cancelled = True
        super().cancel_scan()
        self._finish_scan_cancel()

    def _finish_scan_cancel(self):
        # The shared scanner can return without a result signal on cancellation.
        if self._closing:
            return
        if self.scan_thread.isRunning():
            QTimer.singleShot(50, self._finish_scan_cancel)
        else:
            self._set_ui_busy(False)
            self.statusBar().showMessage("Scan cancelled.")

    @Slot(int)
    def _on_format_ok_simple(self, count):
        self._reset_catalog()
        if not self._closing:
            super()._on_format_ok_simple(count)

    @Slot(str)
    def _on_format_failed_simple(self, message):
        self._reset_catalog()
        if not self._closing:
            super()._on_format_failed_simple(message)

    @Slot(int)
    def _on_delete_ok(self, count):
        # A result signal can arrive before the worker actually exits.
        self._reset_catalog()
        self._rescan_pending = True
        self.statusBar().showMessage(f"{count} captures deleted · refreshing library…")
        if self.delete_thread.isRunning():
            self.delete_thread.finished.connect(self._rescan_after_delete)
            if not self.delete_thread.isRunning():
                QTimer.singleShot(0, self._rescan_after_delete)
        else:
            self._rescan_after_delete()

    @Slot()
    def _rescan_after_delete(self):
        if not self._rescan_pending or self._closing:
            return
        self._rescan_pending = False
        self._set_ui_busy(False)
        self.scan_snapshots()

    @Slot(str)
    def _on_delete_failed(self, message):
        self._reset_catalog()  # A cancelled delete can already have erased some captures.
        if not self._closing:
            super()._on_delete_failed(message)

    def _on_current_row(self, index, _previous):
        if self._busy or self._closing:
            return
        if not index.isValid():
            self._clear_preview()
            return
        self.on_select(self.listbox.row(index))

    def on_select(self, row):
        if self._busy or self._closing or not self.drive or not 0 <= row < len(self.snapshots):
            return
        self.current_snap = self.snapshots[row]
        self._queue_preview(preserve=False)

    def _refresh_appearance(self, _index):
        if self.current_snap is not None and not self._closing:
            self._queue_preview(preserve=True)

    def _cache_get(self, key):
        image = self._cache.get(key)
        if image is not None:
            self._cache.move_to_end(key)
        return image

    def _cache_put(self, key, image):
        old = self._cache.pop(key, None)
        if old is not None:
            self._cache_size -= old.sizeInBytes()
        self._cache[key] = image
        self._cache_size += image.sizeInBytes()
        while self._cache_size > CACHE_BYTES:
            _, old = self._cache.popitem(last=False)
            self._cache_size -= old.sizeInBytes()

    def _queue_preview(self, *, preserve):
        snap = self.current_snap
        settings = self._settings()
        self._load_seq += 1
        seq = self._load_seq
        self._pending = None
        self._displayed = None
        if not preserve:
            self.empty_title.setText(f"Opening capture #{snap.idx + 1:03d}")
            self.empty_hint.setText("Reading SD pixels and preparing the preview.")
            self.stack.setCurrentWidget(self.empty)
        self.image_name_label.setText(image_filename(snap.header, snap.idx, "original"))
        self.metadata.setText(f"{snap.header['width']} × {snap.header['height']} px"
                              f"  ·  {time_label(snap.header)}")
        cached = self._cache_get((snap, settings))
        if cached is not None:
            self._show_preview(cached, preserve)
        else:
            self._pending = (self.drive, snap, seq, settings, preserve)
            self.statusBar().showMessage(f"Preparing capture #{snap.idx+1:03d}…")
            if self._preview_job is None and not self._busy:
                self._start_preview()
        self._selection_changed()

    def _start_preview(self):
        if self._pending is None or self._closing:
            return
        drive, snap, seq, settings, preserve = self._pending
        self._pending = None
        job = PreviewJob(drive, snap, seq, settings, self._cache_get((snap, None)), self)
        self._preview_job = job
        job.preserve = preserve
        job.rendered.connect(self._preview_ready)
        job.load_failed.connect(self._on_load_failed)
        job.finished.connect(self._preview_finished)
        self.btn_refresh.setEnabled(False)
        self.drive_combo.setEnabled(False)
        job.start()

    @Slot(QImage, QImage, int)
    def _preview_ready(self, decoded, rendered, seq):
        job = self._preview_job
        if (self._closing or job is None or job.seq != seq or job.drive != self.drive
                or not any(s is job.snapshot for s in self.snapshots)):
            return
        self._cache_put((job.snapshot, None), decoded)
        self._cache_put((job.snapshot, job.settings), rendered)
        if job.seq == self._load_seq and job.snapshot is self.current_snap:
            self._show_preview(rendered, job.preserve)

    @Slot()
    def _preview_finished(self):
        job = self._preview_job
        self._preview_job = None
        job.deleteLater()
        if not self._busy and not self._closing:
            self.btn_refresh.setEnabled(True)
            self.drive_combo.setEnabled(True)
            self._start_preview()

    @Slot(object, int, str)
    def _on_load_failed(self, snap, seq, message):
        if self._closing or seq != self._load_seq or snap is not self.current_snap:
            return
        self.empty_title.setText("Preview unavailable")
        self.empty_hint.setText("Select the capture again to retry.")
        self.stack.setCurrentWidget(self.empty)
        self.statusBar().showMessage(f"Could not open capture: {message}")

    def _show_preview(self, image, preserve):
        visible = self.stack.currentWidget() is self.viewer
        self.stack.setCurrentWidget(self.viewer)
        if preserve and visible:
            # Same source dimensions: update the existing scene without refitting.
            self.viewer.image_item.setPixmap(QPixmap.fromImage(image))
            self.viewer._update_scale()
        else:
            self.viewer.set_image(image)
        self._displayed = image
        self.statusBar().showMessage(f"Capture #{self.current_snap.idx+1:03d} ready.")
        self._selection_changed()

    def save_image(self):
        if self._displayed is None or self._busy or self._save_job is not None:
            return
        snap, image = self.current_snap, self._displayed
        contrast, noise = self._settings()
        name = core.noise_filename(core.color_filename(core.quality_filename(
            image_filename(snap.header, snap.idx, "original"), contrast), COLOR_PROFILE), noise)
        folder = QFileDialog.getExistingDirectory(self, "Choose export folder")
        if not folder:
            return
        path = Path(folder) / name
        if path.exists() and core.dialog(self, "question", "Replace PNG?", f"Replace {name}?") != (
                core.QMessageBox.StandardButton.Yes):
            return
        job = self._save_job = SaveJob(image, path, self)
        job.result.connect(self._save_result)
        job.finished.connect(self._save_finished)
        self._selection_changed()
        self.statusBar().showMessage("Writing PNG…")
        job.start()

    @Slot(bool, str)
    def _save_result(self, ok, path):
        if self._closing:
            return
        if ok:
            self.statusBar().showMessage(f"Saved · {path}")
        else:
            core.dialog(self, "critical", "PNG export failed", path)

    @Slot()
    def _save_finished(self):
        self._save_job.deleteLater()
        self._save_job = None
        self._selection_changed()

    def extract_selected(self):
        self._export_batch([self.snapshots[self.listbox.row(i)] for i in self.listbox.selectedItems()])

    def download_all(self):
        # Explicitly export the whole scan even with a search filter active.
        self._export_batch(list(self.snapshots))

    def _export_batch(self, snapshots):
        if not snapshots or not self.drive:
            return
        if self._op_running():
            self.statusBar().showMessage("Please wait for the current operation.")
            return
        folder = QFileDialog.getExistingDirectory(self, "Choose export folder")
        if not folder:
            return
        contrast, noise = self._settings()
        self.extract_thread = core.ExtractThread(self.drive, snapshots, folder, contrast=contrast,
                                                  color_trial=COLOR_PROFILE, chroma_denoise=noise)
        self.extract_thread.progress.connect(self.statusBar().showMessage)
        self.extract_thread.finished_ok.connect(self._on_extract_ok)
        self.extract_thread.failed.connect(self._on_extract_failed)
        self._set_ui_busy(True)
        self.extract_thread.start()

    @Slot(str)
    def _on_extract_ok(self, message):
        if not self._closing:
            super()._on_extract_ok(message)

    @Slot(str)
    def _on_extract_failed(self, message):
        if not self._closing:
            super()._on_extract_failed(message)

    def closeEvent(self, event):
        # Never destroy a live worker on exit. Stop accepting new requests and
        # retry closing without blocking GUI events until outstanding work ends.
        self._closing = True
        self._pending = None
        for name in ("scan_thread", "extract_thread", "delete_thread", "format_thread"):
            worker = getattr(self, name, None)
            if worker is not None and worker.isRunning():
                worker.cancel()
        if self._preview_job is not None or self._save_job is not None or super()._op_running():
            event.ignore()
            self.statusBar().showMessage("Waiting for the current operation to finish…")
            QTimer.singleShot(150, self.close)
            return
        super().closeEvent(event)


def main():
    if os.name != "nt":
        print("MASSIF Studio reads physical SD drives on Windows. Run as Administrator.")
        return 1
    # The legacy launcher owns volume locking and journal reset on Windows.
    from SD_Image_Viewer import SDVisualizer

    class StudioWindow(StudioUI, SDVisualizer):
        pass

    app = QApplication(sys.argv)
    app.setApplicationName("MASSIF Studio")
    app.setStyle("Fusion")
    app.setStyleSheet(THEME)
    window = StudioWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
