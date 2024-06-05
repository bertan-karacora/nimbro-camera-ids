import ids_peak_ipl.ids_peak_ipl as idsp_ipl
import numpy as np


class CorrectorColors:
    def __init__(self, camera, name_matrix_correction="HQ"):
        self.camera = camera
        self.corrector_colors = None
        self.matrix_correction = None
        self.name_matrix_correction = name_matrix_correction

    def _init(self):
        self.corrector_colors = idsp_ipl.ColorCorrector()

        self.camera.set_entry("ColorCorrectionMatrix", "HQ")

        self.matrix_correction = np.array(
            [
                [
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain00"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain01"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain02"),
                ],
                [
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain10"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain11"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain12"),
                ],
                [
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain20"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain21"),
                    self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain22"),
                ],
            ]
        )

        factors_correction = idsp_ipl.ColorCorrectionFactors(*self.matrix_correction.flatten())
        self.corrector_colors.SetColorCorrectionFactors(factors_correction)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def process_image(self, image):
        self.corrector_colors.ProcessInPlace(image)
        return image
