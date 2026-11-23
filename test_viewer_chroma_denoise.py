"""Regression checks for chroma smoothing and the desktop export pipeline."""

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
from viewer_chroma_denoise import apply_chroma_denoise, denoise_filename


def luminance(rgb):
    return rgb.astype(np.float32) @ np.array([.2126,.7152,.0722],np.float32)


class ChromaDenoiseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_chroma_noise_reduction_preserves_luminance_and_input(self):
        rng = np.random.default_rng(80)
        image = np.clip(150+rng.normal(0,5,(257,63,3)),0,255).astype(np.uint8)
        original = image.copy()
        filtered = apply_chroma_denoise(image)
        before = image[:,:,1].astype(float)-image[:,:,2]
        after = filtered[:,:,1].astype(float)-filtered[:,:,2]
        self.assertLess(after.std(), before.std()*.8)
        self.assertLess(np.max(np.abs(luminance(image)-luminance(filtered))), .6)
        np.testing.assert_array_equal(image,original)
        np.testing.assert_array_equal(apply_chroma_denoise(image,0),image)
        for amount in (-1,2,float("nan")):
            with self.assertRaises(ValueError): apply_chroma_denoise(image,amount)

    def test_colored_edge_single_pixel_detail_and_strip_seams(self):
        # Strong color step must not leak; a one-pixel dark line must remain.
        image=np.empty((257,64,3),np.uint8)
        image[:,:32]=[145,95,35]
        image[:,32:]=[35,95,145]
        image[128]=[25,25,25]
        filtered=apply_chroma_denoise(image)
        np.testing.assert_array_equal(filtered,image)
        # Flat patches and exact gray ramps remain unchanged, even at seams.
        gray=np.repeat(np.arange(257,dtype=np.uint16)[:,None,None]%256,3,axis=2).astype(np.uint8)
        np.testing.assert_array_equal(apply_chroma_denoise(gray),gray)

    def test_preview_and_all_batch_combinations(self):
        rng=np.random.default_rng(91)
        rgb=rng.integers(100,180,(9,6,3),dtype=np.uint8)
        image=QImage(rgb.data,6,9,18,QImage.Format.Format_RGB888).copy()
        snap=viewer.Snapshot(0,0,{"width":6,"height":9,"data_size":108,"pixel_format":0})
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(viewer,"rbulk",return_value=bytes(512)), \
                    patch.object(viewer,"decode_image_data",return_value=image), \
                    patch.object(viewer,"image_filename",return_value="batch.png"):
                for contrast in (False,True):
                    for trial in (False,True):
                        for denoise in (False,True):
                            worker=viewer.ExtractThread("unused",[snap],directory,
                                contrast=contrast,color_trial=trial,chroma_denoise=denoise)
                            failures=[]
                            worker.failed.connect(failures.append)
                            worker.run()
                            self.assertEqual(failures,[])
                            name=denoise_filename(viewer.color_filename(viewer.quality_filename(
                                "batch.png",contrast),trial),denoise)
                            saved=QImage(str(Path(directory)/name))
                            expected=viewer.presentation_image(image,contrast,trial,denoise)
                            np.testing.assert_array_equal(pixels(saved),pixels(expected))
        np.testing.assert_array_equal(pixels(image),rgb)

    def test_single_save_defaults_and_reversible_toggle(self):
        with patch.object(viewer.SDVisualizer,"request_drive_list"):
            window=viewer.SDVisualizer()
        try:
            self.assertFalse(window.contrast_combo.currentData())
            self.assertTrue(window.color_combo.currentData())
            self.assertTrue(window.noise_combo.currentData())
            rng=np.random.default_rng(11)
            rgb=rng.integers(100,180,(257,6,3),dtype=np.uint8)
            image=QImage(rgb.data,6,257,18,QImage.Format.Format_RGB888).copy()
            snap=viewer.Snapshot(0,0,{})
            snap.qimage=image
            window.current_snap=snap
            window.viewer.set_image(image)
            window.viewer.set_explicit_scale(1.0)
            with tempfile.TemporaryDirectory() as directory:
                with patch.object(viewer.QFileDialog,"getExistingDirectory",return_value=directory), \
                        patch.object(viewer,"image_filename",return_value="single.png"), \
                        patch.object(viewer,"dialog"):
                    for denoise in (False,True,False):
                        window.noise_combo.setCurrentIndex(int(denoise))
                        self.assertAlmostEqual(window.viewer.current_scale,1.0)
                        window.save_image("original")
                        name=denoise_filename("single_color_trial.png",denoise)
                        saved=QImage(str(Path(directory)/name))
                        self.assertEqual((saved.width(),saved.height()),(6,257))
                        np.testing.assert_array_equal(pixels(saved),pixels(viewer.presentation_image(
                            image,False,True,denoise)))
            np.testing.assert_array_equal(pixels(image),rgb)
        finally:
            window.close()


if __name__ == "__main__": unittest.main()
