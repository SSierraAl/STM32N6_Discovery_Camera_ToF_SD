"""Run with: python -m unittest test_viewer_image_quality -v.

Uses only synthetic pixels and temporary PNG files; no SD device is opened.
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

import _sd_image_viewer_core as viewer
from viewer_image_quality import apply_mild_contrast, quality_filename


def pixels(image):
    """Copy RGB pixels, excluding QImage row alignment padding."""
    image = image.convertToFormat(QImage.Format.Format_RGB888)
    rows = np.frombuffer(image.constBits(), dtype=np.uint8).reshape(
        image.height(), image.bytesPerLine())
    return rows[:, :image.width() * 3].reshape(image.height(), image.width(), 3).copy()


class QualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_curve_preserves_endpoints_order_and_input(self):
        ramp = np.repeat(np.arange(256, dtype=np.uint8)[None, :, None], 3, axis=2)
        original = ramp.copy()
        for strength in (0, 0.5, 1):
            out = apply_mild_contrast(ramp, strength)
            self.assertTrue(np.all(np.diff(out[0, :, 0].astype(int)) >= 0))
            self.assertEqual(int(out[0, 0, 0]), 0)
            self.assertEqual(int(out[0, -1, 0]), 255)
        np.testing.assert_array_equal(original, ramp)
        np.testing.assert_array_equal(apply_mild_contrast(ramp, 0), ramp)
        for strength in (-1, 2, float("nan")):
            with self.assertRaises(ValueError):
                apply_mild_contrast(ramp, strength)

    def test_padding_and_repeat_render_do_not_accumulate(self):
        # Width 5 gives 15 RGB bytes plus alignment padding per QImage row.
        image = QImage(5, 257, QImage.Format.Format_RGB888)
        image.fill(0xFF555555)
        original = pixels(image)
        adjusted = viewer.presentation_image(image, True)
        np.testing.assert_array_equal(pixels(adjusted), apply_mild_contrast(original))
        np.testing.assert_array_equal(pixels(image), original)
        np.testing.assert_array_equal(pixels(viewer.presentation_image(image, False)), original)
        np.testing.assert_array_equal(pixels(viewer.presentation_image(image, True)), pixels(adjusted))

    def test_toggle_and_save_keep_native_dimensions_and_selected_appearance(self):
        with patch.object(viewer.SDVisualizer, "request_drive_list"):
            window = viewer.SDVisualizer()
        window.contrast_combo.setCurrentIndex(1)
        window.color_combo.setCurrentIndex(0)
        window.noise_combo.setCurrentIndex(0)
        try:
            image = QImage(2592, 1944, QImage.Format.Format_RGB888)
            image.fill(0xFF555555)
            snapshot = viewer.Snapshot(0, 0, {})
            snapshot.qimage = image
            window.current_snap = snapshot
            window.viewer.set_image(image)
            window.viewer.set_explicit_scale(1.0)
            with tempfile.TemporaryDirectory() as directory:
                with patch.object(viewer.QFileDialog, "getExistingDirectory", return_value=directory), \
                        patch.object(viewer, "image_filename", return_value="test.png"), \
                        patch.object(viewer, "dialog"):
                    for index, filename in ((1, "test_contrast.png"), (0, "test.png")):
                        window.contrast_combo.setCurrentIndex(index)
                        self.assertAlmostEqual(window.viewer.current_scale, 1.0)
                        window.save_image("original")
                        loaded = QImage(str(Path(directory) / filename))
                        self.assertEqual((loaded.width(), loaded.height()), (2592, 1944))
                        expected = viewer.presentation_image(image, bool(index))
                        np.testing.assert_array_equal(pixels(loaded), pixels(expected))
            np.testing.assert_array_equal(pixels(image)[0, 0], [85, 85, 85])
        finally:
            window.close()

    def test_batch_export_matches_preview_and_names(self):
        image = QImage(6, 3, QImage.Format.Format_RGB888)
        image.fill(0xFF555555)
        snap = viewer.Snapshot(0, 0, {
            "width": 6, "height": 3, "data_size": 36, "pixel_format": 0})
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(viewer, "rbulk", return_value=bytes(512)), \
                    patch.object(viewer, "decode_image_data", return_value=image), \
                    patch.object(viewer, "image_filename", return_value="batch.png"):
                for enabled in (False, True):
                    worker = viewer.ExtractThread("unused", [snap], directory, contrast=enabled)
                    failures = []
                    worker.failed.connect(failures.append)
                    worker.run()
                    self.assertEqual(failures, [])
                    result = QImage(str(Path(directory) / quality_filename("batch.png", enabled)))
                    np.testing.assert_array_equal(pixels(result), pixels(viewer.presentation_image(image, enabled)))


if __name__ == "__main__":
    unittest.main()
