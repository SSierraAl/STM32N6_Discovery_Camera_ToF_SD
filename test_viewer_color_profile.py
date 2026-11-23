"""Desktop color regression tests; no camera or SD device is accessed."""

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
from test_viewer_image_quality import pixels
from viewer_color_profile import apply_experimental_color, color_filename


class ColorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_neutrals_identity_input_and_layout(self):
        ramp = np.repeat(np.arange(256, dtype=np.uint8)[None, :, None], 3, axis=2)
        np.testing.assert_array_equal(apply_experimental_color(ramp), ramp)
        rng = np.random.default_rng(13)
        image = rng.integers(0, 256, (257, 18, 3), dtype=np.uint8)
        original = image.copy()
        np.testing.assert_array_equal(apply_experimental_color(image, 0), image)
        candidate = apply_experimental_color(image)
        np.testing.assert_array_equal(image, original)
        np.testing.assert_array_equal(candidate[:, :, 0], image[:, :, 0])
        self.assertEqual(candidate.shape, image.shape)
        self.assertEqual(candidate.dtype, np.uint8)
        for strength in (-1, 1, float("nan")):
            with self.assertRaises(ValueError):
                apply_experimental_color(image, strength)

    def test_wb_is_restored_after_legacy_zero_diagnostic(self):
        # The old setting can remain 0.0 in a local checkout; the new central
        # parameter must still restore the expected white balance on decode.
        with patch.object(viewer, "VIEWER_WHITE_BALANCE_STRENGTH", 0.0), \
                patch.object(viewer, "apply_robust_white_balance", side_effect=lambda rgb, **kw: rgb) as wb:
            result = viewer.decode_image_data(bytes([150,128,150,128])*2, 2, 2, 0)
            self.assertIsNotNone(result)
            self.assertEqual(wb.call_args.kwargs["strength"], 1.0)

    def test_opt_in_toggle_and_individual_export(self):
        with patch.object(viewer.SDVisualizer, "request_drive_list"):
            window = viewer.SDVisualizer()
        try:
            self.assertTrue(window.color_combo.currentData())
            self.assertFalse(window.contrast_combo.currentData())
            window.contrast_combo.setCurrentIndex(1)
            window.noise_combo.setCurrentIndex(0)
            image = QImage(2592, 1944, QImage.Format.Format_RGB888)
            image.fill(0xFF8C572A)
            snap = viewer.Snapshot(0, 0, {})
            snap.qimage = image
            window.current_snap = snap
            window.viewer.set_image(image)
            window.viewer.set_explicit_scale(1.0)
            with tempfile.TemporaryDirectory() as directory:
                with patch.object(viewer.QFileDialog, "getExistingDirectory", return_value=directory), \
                        patch.object(viewer, "image_filename", return_value="sample.png"), \
                        patch.object(viewer, "dialog"):
                    for enabled in (True, False):
                        window.color_combo.setCurrentIndex(int(enabled))
                        self.assertAlmostEqual(window.viewer.current_scale, 1.0)
                        window.save_image("original")
                        name = color_filename("sample_contrast.png", enabled)
                        saved = QImage(str(Path(directory)/name))
                        self.assertEqual((saved.width(), saved.height()), (2592,1944))
                        expected = viewer.presentation_image(image, True, enabled)
                        np.testing.assert_array_equal(pixels(saved), pixels(expected))
            np.testing.assert_array_equal(pixels(image)[0,0], [140,87,42])
        finally:
            window.close()

    def test_all_batch_profiles_match_preview(self):
        image = QImage(6, 3, QImage.Format.Format_RGB888)
        image.fill(0xFF8C572A)
        snap = viewer.Snapshot(0, 0, {
            "width":6,"height":3,"data_size":36,"pixel_format":0})
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(viewer, "rbulk", return_value=bytes(512)), \
                    patch.object(viewer, "decode_image_data", return_value=image), \
                    patch.object(viewer, "image_filename", return_value="batch.png"):
                for contrast in (False,True):
                    for trial in (False,True):
                        worker = viewer.ExtractThread("unused",[snap],directory,contrast=contrast,color_trial=trial)
                        failures=[]
                        worker.failed.connect(failures.append)
                        worker.run()
                        self.assertEqual(failures,[])
                        name=color_filename(viewer.quality_filename("batch.png",contrast),trial)
                        saved=QImage(str(Path(directory)/name))
                        np.testing.assert_array_equal(pixels(saved),pixels(viewer.presentation_image(image,contrast,trial)))


if __name__ == "__main__":
    unittest.main()
