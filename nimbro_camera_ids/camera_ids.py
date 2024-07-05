import importlib.resources as resources
import sys
import threading

import ids_peak.ids_peak as idsp
import ids_peak_ipl.ids_peak_ipl as idsp_ipl
import ids_peak.ids_peak_ipl_extension as idsp_extension

from nimbro_camera_ids.manager_auto_features import ManagerAutoFeatures
from nimbro_camera_ids.corrector_colors import CorrectorColors


class CameraIDS:
    def __init__(self, id_device=0, name_pixelformat_target="PixelFormatName_RGB8"):
        self.is_acquiring = None
        self.capturing_thread = None
        self.corrector_colors = None
        self.converter_image = None
        self.datastream = None
        self.device = None
        self.device_manager = None
        self.id_device = id_device
        self.is_capturing = None
        self.killed = None
        self.manager_auto_features = None
        self.name_pixelformat_target = name_pixelformat_target
        self.nodemap = None
        self.pixelformat_target = None

        self._init()

    def __del__(self):
        self.close()

    def __repr__(self):
        r = f"{__package__}.{self.__class__.__name__}({self.id_device, self.name_pixelformat_target})"
        return r

    def __str__(self):
        name_model = self.device.ModelName()
        name_interface = self.device.ParentInterface().DisplayName()
        name_system = self.device.ParentInterface().ParentSystem().DisplayName()
        version_system = self.device.ParentInterface().ParentSystem().Version()

        s = f"{name_model} ({name_interface} ; {name_system} v.{version_system})"
        return s

    def get_attributes(self):
        names = [node.DisplayName() for node in self.nodemap.Nodes()]

        # TODO: get values too
        return names

    def get_value(self, name):
        value = self.nodemap.FindNode(name).Value()
        return value

    def get_min(self, name):
        min = self.nodemap.FindNode(name).Minimum()
        return min

    def get_max(self, name):
        max = self.nodemap.FindNode(name).Maximum()
        return max

    def get_entry(self, name):
        entry = self.nodemap.FindNode(name).CurrentEntry().Value()
        return entry

    def has_attribute(self, name):
        has_it = self.nodemap.HasNode(name)
        return has_it

    def set_value(self, name, value):
        self.nodemap.FindNode(name).SetValue(value)

    def set_entry(self, name, value):
        all_entries = self.nodemap.FindNode(name).Entries()
        available_entries = []
        for entry in all_entries:
            if entry.AccessStatus() != idsp.NodeAccessStatus_NotAvailable and entry.AccessStatus() != idsp.NodeAccessStatus_NotImplemented:
                available_entries.append(entry.SymbolicValue())
        if value in available_entries:
            self.nodemap.FindNode(name).SetCurrentEntry(value)

    def execute(self, command):
        self.nodemap.FindNode(command).Execute()
        self.nodemap.FindNode(command).WaitUntilDone()

    def load_config(self, name_config):
        path_config = resources.files(__package__) / "configs" / f"{name_config}.cset"
        self.nodemap.LoadFromFile(str(path_config))

    def save_config(self, name_config):
        path_config = resources.files(__package__) / "configs" / f"{name_config}.cset"
        self.nodemap.StoreToFile(str(path_config))

    def reset(self):
        self.execute("ResetToFactoryDefaults")

    def _open(self, id_device):
        self.device_manager.Update()
        device_descriptors = self.device_manager.Devices()

        if device_descriptors.empty() or len(device_descriptors) <= id_device:
            raise Exception("No device found")

        device_descriptor = device_descriptors[id_device]
        if device_descriptor.IsOpenable(idsp.DeviceAccessType_Control):
            device = device_descriptor.OpenDevice(idsp.DeviceAccessType_Control)
        else:
            raise Exception("Device found but not openable")

        return device

    def _setup_buffers(self):
        # PayloadSize depends on the image size and the source pixel format
        payload_size = self.get_value("PayloadSize")
        num_buffers = self.datastream.NumBuffersAnnouncedMinRequired()
        for _ in range(num_buffers):
            buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
            self.datastream.QueueBuffer(buffer)

    def _revoke_buffers(self):
        self.datastream.Flush(idsp.DataStreamFlushMode_DiscardAll)
        for buffer in self.datastream.AnnouncedBuffers():
            self.datastream.RevokeBuffer(buffer)

    def _preallocate_conversion(self):
        """Pre-allocate conversion buffers to speed up first image conversion while the acquisition is running."""
        image_width = self.get_value("Width")
        image_height = self.get_value("Height")
        input_pixelformat = idsp_ipl.PixelFormat(self.get_entry("PixelFormat"))

        self.converter_image.PreAllocateConversion(input_pixelformat, self.pixelformat_target, image_width, image_height)

    def _import_pixelformat(self, name_pixelformat):
        if hasattr(idsp_ipl, name_pixelformat):
            pixelformat = getattr(idsp_ipl, name_pixelformat)
            if isinstance(pixelformat, int):
                return pixelformat

        raise ImportError(f"Pixelformat '{name_pixelformat}' not found")

    def _init(self):
        idsp.Library.Initialize()

        self.pixelformat_target = self._import_pixelformat(self.name_pixelformat_target)
        self.device_manager = idsp.DeviceManager.Instance()
        self.device = self._open(self.id_device)

        self.is_acquiring = False
        self.killed = False
        self.is_capturing = False
        self.nodemap = self.device.RemoteDevice().NodeMaps()[0]
        self.datastream = self.device.DataStreams()[0].OpenDataStream()
        self.converter_image = idsp_ipl.ImageConverter()
        self.manager_auto_features = ManagerAutoFeatures(self, auto_exposure="off", auto_gain="continuous", auto_white_balance="continuous")
        self.corrector_colors = CorrectorColors(self)
        self.capturing_thread = threading.Thread(target=self.capture_threaded)

    def close(self):
        self.stop_capturing()
        self.stop_acquisition()
        idsp.Library.Close()

    def start_acquisition(self):
        if self.is_acquiring:
            return

        # Lock parameters that should not be accessed during acquisition
        self.set_value("TLParamsLocked", 1)

        self._setup_buffers()
        self._preallocate_conversion()
        self.datastream.StartAcquisition()
        self.execute("AcquisitionStart")

        self.is_acquiring = True

    def stop_acquisition(self):
        if not self.is_acquiring:
            return

        self.execute("AcquisitionStop")
        self.datastream.StopAcquisition(idsp.AcquisitionStopMode_Default)
        # NOTE: Re-create the image converter, so old conversion buffers get freed
        self.converter_image = idsp_ipl.ImageConverter()
        self._revoke_buffers()

        # Unlock parameters
        self.set_value("TLParamsLocked", 0)

        self.is_acquiring = False

    def capture(self):
        buffer = self.datastream.WaitForFinishedBuffer(5000)

        # NOTE: This still uses the buffer's underlying memory
        image = idsp_extension.BufferToImage(buffer)

        # This creates a copy the image, so the buffer is free to use again after queueing
        image = self.convert_image(image)

        self.datastream.QueueBuffer(buffer)

        image = self.process_image(image)

        return image

    def start_capturing(self, on_capture_callback=lambda *args: None):
        if not self.is_acquiring:
            raise ValueError("Camera is not in acquisition mode.")

        if self.is_capturing:
            return

        self.capturing_thread = threading.Thread(target=self.capture_threaded, kwargs={"on_capture_callback": on_capture_callback})
        self.capturing_thread.start()
        self.is_capturing = True

    def stop_capturing(self):
        if not self.is_capturing:
            return

        self.kill_thread()

        self.capturing_thread = None
        self.killed = False
        self.is_capturing = False

    def kill_thread(self):
        self.killed = True
        self.capturing_thread.join()

    def capture_threaded(self, on_capture_callback=lambda *args: None):
        while not self.killed:
            try:
                image = self.capture()
                on_capture_callback(image)
            except Exception as e:
                # TODO: Seems bad
                self.killed = True
                raise Exception("Thread died")

    def convert_image(self, image):
        # NOTE: Use `ImageConverter`, since the `ConvertTo` function re-allocates the conversion buffers on every call
        image = self.converter_image.Convert(image, self.pixelformat_target)
        return image

    def process_image(self, image):
        image = self.manager_auto_features.process_image(image)
        image = self.corrector_colors.process_image(image)

        return image
