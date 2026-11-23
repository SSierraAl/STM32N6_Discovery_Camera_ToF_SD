"""Noise-effect, diagnostics and mode-2 export checks without physical SD."""

import os
os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

import _sd_image_viewer_core as viewer
import viewer_chroma_denoise as chroma
import viewer_luma_denoise as luma
from test_viewer_image_quality import pixels


class LumaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QApplication.instance() or QApplication([])

    def test_luminance_noise_is_reduced_without_color_cast(self):
        rng=np.random.default_rng(32)
        values=np.clip(150+rng.normal(0,4,(257,61)),0,255).astype(np.uint8)
        image=np.repeat(values[:,:,None],3,axis=2)
        original=image.copy()
        filtered=luma.apply_luma_denoise(image)
        self.assertLess(filtered[:,:,0].std(),image[:,:,0].std()*.85)
        np.testing.assert_array_equal(filtered[:,:,0],filtered[:,:,1])
        np.testing.assert_array_equal(filtered[:,:,0],filtered[:,:,2])
        np.testing.assert_array_equal(image,original)
        np.testing.assert_array_equal(luma.apply_luma_denoise(image,0),image)
        self.assertLessEqual(np.max(np.abs(filtered.astype(int)-image.astype(int))),4)
        for amount in (-1,2,float("nan")):
            with self.assertRaises(ValueError): luma.apply_luma_denoise(image,amount)

    def test_strong_edges_and_single_pixel_line_remain(self):
        image=np.full((257,63,3),160,np.uint8)
        image[:,32:]=[80,60,40]
        image[128]=[15,15,15]
        np.testing.assert_array_equal(luma.apply_luma_denoise(image),image)

    def test_strip_halos_match_full_frame_bilateral(self):
        rng=np.random.default_rng(17)
        image=np.clip(150+rng.normal(0,4,(259,37,3)),20,230).astype(np.uint8)
        src=image.astype(np.float32)
        y=.2126*src[:,:,0]+.7152*src[:,:,1]+.0722*src[:,:,2]
        smooth=cv2.bilateralFilter(y,luma.VIEWER_LUMA_DENOISE_DIAMETER,
            luma.VIEWER_LUMA_DENOISE_SIGMA_COLOR,luma.VIEWER_LUMA_DENOISE_SIGMA_SPACE,
            borderType=cv2.BORDER_REFLECT_101)
        delta=luma.VIEWER_LUMA_DENOISE_AMOUNT*np.clip(smooth-y,
            -luma.VIEWER_LUMA_DENOISE_MAX_DELTA,luma.VIEWER_LUMA_DENOISE_MAX_DELTA)
        expected=np.rint(np.clip(src+delta[:,:,None],0,255)).astype(np.uint8)
        np.testing.assert_array_equal(luma.apply_luma_denoise(image),expected)

    def test_effective_configuration_and_measured_change_are_visible(self):
        image=np.full((9,6,3),150,np.uint8)
        with patch.object(chroma,"VIEWER_CHROMA_DENOISE_AMOUNT",0.0):
            np.testing.assert_array_equal(chroma.apply_chroma_denoise(image),image)
            self.assertIn("amount=0",luma.noise_configuration(2))
        with patch.object(luma,"VIEWER_NOISE_DIAGNOSTICS",True),contextlib.redirect_stdout(io.StringIO()) as output:
            luma.log_noise_change(image,image+1,2)
        self.assertIn("sample_mean_abs_change=1.000",output.getvalue())
        self.assertIn("changed_pixels=100.0%",output.getvalue())

    def test_new_mode_preview_batch_and_original_size_single_save(self):
        rng=np.random.default_rng(19)
        rgb=np.clip(150+rng.normal(0,4,(9,6,3)),0,255).astype(np.uint8)
        image=QImage(rgb.data,6,9,18,QImage.Format.Format_RGB888).copy()
        snap=viewer.Snapshot(0,0,{"width":6,"height":9,"data_size":108,"pixel_format":0})
        with tempfile.TemporaryDirectory() as directory,patch.object(luma,"VIEWER_NOISE_DIAGNOSTICS",False):
            with patch.object(viewer,"rbulk",return_value=bytes(512)), \
                    patch.object(viewer,"decode_image_data",return_value=image), \
                    patch.object(viewer,"image_filename",return_value="batch.png"):
                for contrast in (False,True):
                    for trial in (False,True):
                        worker=viewer.ExtractThread("unused",[snap],directory,
                            contrast=contrast,color_trial=trial,chroma_denoise=2)
                        errors=[]
                        worker.failed.connect(errors.append)
                        worker.run()
                        self.assertEqual(errors,[])
                        name=luma.noise_filename(viewer.color_filename(viewer.quality_filename(
                            "batch.png",contrast),trial),2)
                        np.testing.assert_array_equal(pixels(QImage(str(Path(directory)/name))),
                            pixels(viewer.presentation_image(image,contrast,trial,2)))
            with patch.object(viewer.SDVisualizer,"request_drive_list"):
                window=viewer.SDVisualizer()
            try:
                self.assertEqual(window.noise_combo.currentData(),2)
                self.assertIn("Luma:",window.noise_info_label.text())
                # Full-size export through the real save method must retain 5 MP.
                full=QImage(2592,1944,QImage.Format.Format_RGB888)
                full.fill(0xFF969696)
                snap.qimage=full
                window.current_snap=snap
                window.viewer.set_image(full)
                window.viewer.set_explicit_scale(1.0)
                window.noise_combo.setCurrentIndex(0)
                self.assertEqual(window.noise_info_label.text(),"NR Off")
                window.noise_combo.setCurrentIndex(2)
                self.assertAlmostEqual(window.viewer.current_scale,1.0)
                with patch.object(viewer.QFileDialog,"getExistingDirectory",return_value=directory), \
                        patch.object(viewer,"image_filename",return_value="full.png"),patch.object(viewer,"dialog"):
                    window.save_image("original")
                saved=QImage(str(Path(directory)/"full_color_trial_color_luma_nr.png"))
                self.assertEqual((saved.width(),saved.height()),(2592,1944))
            finally:
                window.close()


if __name__ == "__main__": unittest.main()
