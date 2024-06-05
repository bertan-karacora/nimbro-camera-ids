import ids_peak_afl.ids_peak_afl as idsp_afl


class ManagerAutoFeatures:
    def __init__(self, camera, auto_exposure="off", auto_gain="off", auto_white_balance="off"):
        self._auto_exposure = auto_exposure
        self._auto_gain = auto_gain
        self._auto_white_balance = auto_white_balance
        self.camera = camera
        self.controller_brightness = None
        self.controller_white_balance = None
        self.manager = None

        self._init()

    def __del__(self):
        idsp_afl.Library.Exit()

    def _init(self):
        idsp_afl.Library.Init()
        self.manager = idsp_afl.Manager(self.camera.nodemap)

        self.controller_brightness = self.manager.CreateController(idsp_afl.PEAK_AFL_CONTROLLER_TYPE_BRIGHTNESS)
        self.controller_brightness.SetSkipFrames(self.controller_brightness.GetSkipFramesRange().max)
        self.manager.AddController(self.controller_brightness)

        self.controller_white_balance = self.manager.CreateController(idsp_afl.PEAK_AFL_CONTROLLER_TYPE_WHITE_BALANCE)
        self.controller_white_balance.SetSkipFrames(self.controller_white_balance.GetSkipFramesRange().max)
        self.manager.AddController(self.controller_white_balance)

        self.auto_exposure = self._auto_exposure
        self.auto_gain = self._auto_gain
        self.auto_white_balance = self._auto_white_balance

    def _mode_to_afl(self, mode):
        name_mode = f"PEAK_AFL_CONTROLLER_AUTOMODE_{mode.upper()}"
        if hasattr(idsp_afl, name_mode):
            type_found = getattr(idsp_afl, name_mode)
            return type_found

        raise ValueError(f"Mode '{name_mode}' not found")

    @property
    def auto_exposure(self):
        return self._auto_exposure

    @auto_exposure.setter
    def auto_exposure(self, value):
        self._auto_exposure = value
        self.controller_brightness.BrightnessComponentSetMode(idsp_afl.PEAK_AFL_CONTROLLER_BRIGHTNESS_COMPONENT_EXPOSURE, self._mode_to_afl(value))

    @property
    def auto_gain(self):
        return self._auto_gain

    @auto_gain.setter
    def auto_gain(self, value):
        self._auto_gain = value
        self.controller_brightness.BrightnessComponentSetMode(idsp_afl.PEAK_AFL_CONTROLLER_BRIGHTNESS_COMPONENT_GAIN, self._mode_to_afl(value))

    @property
    def auto_white_balance(self):
        return self._auto_white_balance

    @auto_white_balance.setter
    def auto_white_balance(self, value):
        self._auto_white_balance = value
        self.controller_white_balance.SetMode(self._mode_to_afl(value))

    def process_image(self, image):
        try:
            self.manager.Process(image)
        except:
            pass

        return image
