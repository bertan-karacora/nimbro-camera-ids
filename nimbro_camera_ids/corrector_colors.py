import ids_peak_ipl.ids_peak_ipl as idsp_ipl
import numpy as np


class CorrectorColors:
    matrix_correction_hq = np.array(
        [
            [1.7813, -0.5898, -0.1875],
            [-0.4531, 1.8555, -0.3984],
            [0.0, -0.5625, 1.5664],
        ],
    )

    def __init__(self, camera, name_matrix_correction="HQ", enabled=True):
        self.camera = camera
        self.corrector_colors = None
        self.enabled = enabled
        self.matrix_correction = None
        self.name_matrix_correction = name_matrix_correction

        self._init()

    def _init(self):
        self.corrector_colors = idsp_ipl.ColorCorrector()

        # self.camera.set_entry("ColorCorrectionMatrix", self.name_matrix_correction)

        # self.matrix_correction = np.array(
        #     [
        #         [
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain00"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain01"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain02"),
        #         ],
        #         [
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain10"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain11"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain12"),
        #         ],
        #         [
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain20"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain21"),
        #             self.camera.set_entry("ColorCorrectionMatrixValueSelector", "Gain22"),
        #         ],
        #     ]
        # )

        factors_correction = idsp_ipl.ColorCorrectionFactors(*self.matrix_correction_hq.flatten())
        self.corrector_colors.SetColorCorrectionFactors(factors_correction)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def process_image(self, image):
        if self.enabled:
            self.corrector_colors.ProcessInPlace(image)
        return image
