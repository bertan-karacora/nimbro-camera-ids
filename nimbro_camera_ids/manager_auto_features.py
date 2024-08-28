import ids_peak_afl.ids_peak_afl as idsp_afl


class ManagerAutoFeatures:
    """Manager of automatic control features provided by the IDS peak API.
    Valid modes are: [off, once, continuous]."""

    def __init__(self, camera, mode_auto_exposure="off", mode_auto_gain="off", mode_auto_white_balance="off"):
        self._mode_auto_exposure = mode_auto_exposure
        self._mode_auto_gain = mode_auto_gain
        self._mode_auto_white_balance = mode_auto_white_balance
        self.camera = camera
        self.controller_brightness = None
        self.controller_white_balance = None
        self.manager_auto_features = None

        self._init()

    def __del__(self):
        idsp_afl.Library.Exit()

    def _init(self):
        idsp_afl.Library.Init()
        self.manager_auto_features = idsp_afl.Manager(self.camera.nodemap)

        self.controller_brightness = self.manager_auto_features.CreateController(idsp_afl.PEAK_AFL_CONTROLLER_TYPE_BRIGHTNESS)
        self.controller_brightness.SetSkipFrames(self.controller_brightness.GetSkipFramesRange().max)
        self.manager_auto_features.AddController(self.controller_brightness)

        self.controller_white_balance = self.manager_auto_features.CreateController(idsp_afl.PEAK_AFL_CONTROLLER_TYPE_WHITE_BALANCE)
        self.controller_white_balance.SetSkipFrames(self.controller_white_balance.GetSkipFramesRange().max)
        self.manager_auto_features.AddController(self.controller_white_balance)

        self.mode_auto_exposure = self._mode_auto_exposure
        self.mode_auto_gain = self._mode_auto_gain
        self.mode_auto_white_balance = self._mode_auto_white_balance

    def _mode_to_afl(self, mode):
        name_mode = f"PEAK_AFL_CONTROLLER_AUTOMODE_{mode.upper()}"
        if hasattr(idsp_afl, name_mode):
            type_found = getattr(idsp_afl, name_mode)
            return type_found

        raise ValueError(f"Mode '{name_mode}' not found")

    @property
    def mode_auto_exposure(self):
        return self._mode_auto_exposure

    @mode_auto_exposure.setter
    def mode_auto_exposure(self, value):
        self._mode_auto_exposure = value
        self.controller_brightness.BrightnessComponentSetMode(idsp_afl.PEAK_AFL_CONTROLLER_BRIGHTNESS_COMPONENT_EXPOSURE, self._mode_to_afl(value))

    @property
    def mode_auto_gain(self):
        return self._mode_auto_gain

    @mode_auto_gain.setter
    def mode_auto_gain(self, value):
        self._mode_auto_gain = value
        self.controller_brightness.BrightnessComponentSetMode(idsp_afl.PEAK_AFL_CONTROLLER_BRIGHTNESS_COMPONENT_GAIN, self._mode_to_afl(value))

    @property
    def mode_auto_white_balance(self):
        return self._mode_auto_white_balance

    @mode_auto_white_balance.setter
    def mode_auto_white_balance(self, value):
        self._mode_auto_white_balance = value
        self.controller_white_balance.SetMode(self._mode_to_afl(value))

    def process_image(self, image):
        try:
            self.manager_auto_features.Process(image)
        except:
            pass

        return image
