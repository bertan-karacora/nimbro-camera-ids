import threading
import time

import ids_peak.ids_peak as idsp
import ids_peak_ipl.ids_peak_ipl as idsp_ipl
import ids_peak.ids_peak_ipl_extension as idsp_extension

TARGET_PIXELFORMAT = idsp_ipl.PixelFormatName_BGRa8


class CameraIDS:
    def __init__(self, device):
        self.device = device

        self.acquiring = None
        self.capturing_thread = None
        self.capturing_threaded = None
        self.datastream = None
        self.fps = None
        self.image = None
        self.image_converter = None
        self.killed = None
        self.nodemap = None

        self._initialize()

    def __del__(self):
        self.close()

    def __repr__(self):
        r = f"{__package__}.{self.__class__.__name__}({self.device})"
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

    def has_attribute(self, name):
        has_it = self.nodemap.HasNode(name)
        return has_it

    def set_value(self, name, value):
        self.nodemap.FindNode(name).SetValue(value)

    def get_entry(self, name):
        entry = self.nodemap.FindNode(name).CurrentEntry().Value()
        return entry

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

    def _initialize(self):
        idsp.Library.Initialize()

        self.acquiring = False
        self.killed = False
        self.capturing_threaded = False
        # TODO:
        self.fps = 1

        self.nodemap = self.device.RemoteDevice().NodeMaps()[0]
        self.datastream = self.device.DataStreams()[0]
        self.image_converter = idsp_ipl.ImageConverter()

        # TODO: Change from iterated capture calls to threading
        self.capturing_thread = threading.Thread(target=self.capture_threaded, args=())

    def _preallocate_conversion(self):
        image_width = self.get_value("Width")
        image_height = self.get_value("Height")
        input_pixelformat = idsp_ipl.PixelFormat(self.get_entry("PixelFormat"))

        # Pre-allocate conversion buffers to speed up first image conversion while the acquisition is running
        # NOTE: Re-create the image converter, so old conversion buffers get freed
        self.image_converter = idsp_ipl.ImageConverter()
        self.image_converter.PreAllocateConversion(input_pixelformat, TARGET_PIXELFORMAT, image_width, image_height)

    def setup_datastream(self):
        self.datastream.OpenDataStream()

        payload_size = self.get_value("PayloadSize")
        num_buffers = self.datastream.NumBuffersAnnouncedMinRequired()
        for _ in range(num_buffers):
            buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
            self.datastream.QueueBuffer(buffer)

    def close_datastream(self):
        for buffer in self.datastream.AnnouncedBuffers():
            self.datastream.RevokeBuffer(buffer)

        self.datastream.KillWait()
        self.datastream.StopAcquisition(idsp.AcquisitionStopMode_Default)
        self.datastream.Flush(idsp.DataStreamFlushMode_DiscardAll)

    def start_acquisition(self):
        if self.acquiring:
            return

        self.datastream = self.setup_datastream()

        # TODO: Is this desired? From where control this?
        max_framerate = self.get_max("AcquisitionFrameRate")
        self.set_value("AcquisitionFrameRate", max_framerate)

        # Lock parameters that should not be accessed during acquisition
        self.set_value("TLParamsLocked", 1)

        self._preallocate_conversion()

        self.datastream.StartAcquisition()
        self.execute("AcquisitionStart")

        self.acquiring = True

    def stop_acquisition(self):
        if not self.acquiring:
            return

        self.nodemap.FindNode("AcquisitionStop").Execute()
        self.close_datastream()

        # Unlock parameters
        self.nodemap.FindNode("TLParamsLocked").SetValue(0)

        self.acquiring = False

    def close(self):
        self.stop_acquisition()
        self.execute("ResetToFactoryDefaults")
        idsp.Library.Close()

    def capture(self):
        buffer = self.datastream.WaitForFinishedBuffer(5000)

        # NOTE: This still uses the buffer's underlying memory
        ipl_image = idsp_extension.BufferToImage(buffer)

        # This creates a copy the image, so the buffer is free to use again after queuing
        # NOTE: Use `ImageConverter`, since the `ConvertTo` function re-allocates the conversion buffers on every call
        image_converted = self.image_converter.Convert(ipl_image, TARGET_PIXELFORMAT)

        self.datastream.QueueBuffer(buffer)

        return image_converted

    def start_capturing(self):
        self.capturing_threaded = True
        self.capturing_thread.start()

    def stop_capturing(self):
        self.killed = True
        self.capturing_thread.join()
        self.killed = False
        self.capturing_threaded = False

    def capture_threaded(self):
        while not self.killed:
            self.image = self.capture()
            # TODO: Control framerate
            time.sleep(1 / self.fps)

    def load_settings(self, path):
        self.nodemap.LoadFromFile(str(path))

    def save_settings(self, path):
        self.nodemap.StoreToFile(str(path))
