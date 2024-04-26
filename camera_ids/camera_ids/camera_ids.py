import threading

import ids_peak.ids_peak as idsp
import ids_peak_ipl.ids_peak_ipl as idsp_ipl
import ids_peak.ids_peak_ipl_extension as idsp_extension

from sensor_msgs.msg import Image

# BayerGR8, BayerGR10g40IDS
TARGET_PIXELFORMAT = idsp_ipl.PixelFormatName_BGRa8


class CameraIDS:
    def __init__(self, device):
        self._device = device

        self._acquiring = None
        self._capturing_thread = None
        self._capturing_threaded = None
        self._datastream = None
        self._image_converter = None
        self._killed = None
        self._nodemap = None

        self._initialize()

    def __del__(self):
        self.close()

    def __repr__(self):
        r = f"cameraIDS.CameraIDS({self._device})"
        return r

    def __str__(self):
        name_model = self._device.ModelName()
        name_interface = self._device.ParentInterface().DisplayName()
        name_system = self._device.ParentInterface().ParentSystem().DisplayName()
        version_system = self._device.ParentInterface().ParentSystem().Version()

        s = f"{name_model} ({name_interface} ; {name_system} v.{version_system})"
        return s

    def _initialize(self):
        self._acquiring = False
        self._capturing_thread = threading.Thread(target=self.capture_threaded, args=())
        self._capturing_threaded = False
        self._datastream = self._device.DataStreams()[0].OpenDataStream()
        self._image_converter = idsp_ipl.ImageConverter()
        self._killed = False
        self._nodemap = self._device.RemoteDevice().NodeMaps()[0]

        self._setup_device_and_datastream()

    def _setup_device_and_datastream(self):
        payload_size = self._nodemap.FindNode("PayloadSize").Value()

        num_buffers = self._datastream.NumBuffersAnnouncedMinRequired()
        for _ in range(num_buffers):
            buffer = self._datastream.AllocAndAnnounceBuffer(payload_size)
            self._datastream.QueueBuffer(buffer)

    def _close_buffers(self):
        if self._datastream is not None:
            for buffer in self._datastream.AnnouncedBuffers():
                self._datastream.RevokeBuffer(buffer)

    def _preallocate_buffers(self):
        image_width = self.get_value("Width")
        image_height = self.get_value("Height")
        input_pixelformat = idsp_ipl.PixelFormat(self.get_entry("PixelFormat"))

        # Pre-allocate conversion buffers to speed up first image conversion while the acquisition is running
        # NOTE: Re-create the image converter, so old conversion buffers get freed
        self._image_converter = idsp_ipl.ImageConverter()
        self._image_converter.PreAllocateConversion(input_pixelformat, TARGET_PIXELFORMAT, image_width, image_height)

    def start_acquisition(self):
        if self._acquiring:
            return

        max_framerate = self.get_max("AcquisitionFrameRate")
        self.set_value("AcquisitionFrameRate", max_framerate)

        # Lock parameters that should not be accessed during acquisition
        self.set_value("TLParamsLocked", 1)

        self._preallocate_buffers()

        self._datastream.StartAcquisition()
        self._nodemap.FindNode("AcquisitionStart").Execute()
        self._nodemap.FindNode("AcquisitionStart").WaitUntilDone()

        self._acquiring = True

    def stop_acquisition(self):
        if not self._acquiring:
            return

        self._nodemap.FindNode("AcquisitionStop").Execute()
        self._datastream.KillWait()
        self._datastream.StopAcquisition(idsp.AcquisitionStopMode_Default)
        self._datastream.Flush(idsp.DataStreamFlushMode_DiscardAll)

        # Unlock parameters
        self._nodemap.FindNode("TLParamsLocked").SetValue(0)

        self._acquiring = False

    def close(self):
        self.stop_acquisition()
        self._close_buffers()

    def get_attributes(self):
        names = []

        for node in self._nodemap.Nodes():
            names += [node.DisplayName()]

        # TODO: get values too
        return names

    def get_value(self, name):
        value = self._nodemap.FindNode(name).Value()
        return value

    def get_min(self, name):
        min = self._nodemap.FindNode(name).Minimum()
        return min

    def get_max(self, name):
        max = self._nodemap.FindNode(name).Maximum()
        return max

    def has_attribute(self, name):
        has_it = self._nodemap.HasNode(name)
        return has_it

    def set_value(self, name, value):
        self._nodemap.FindNode(name).SetValue(value)

    def get_entry(self, name):
        entry = self._nodemap.FindNode(name).CurrentEntry().Value()
        return entry

    def set_entry(self, name, value):
        all_entries = self._nodemap.FindNode(name).Entries()
        available_entries = []
        for entry in all_entries:
            if entry.AccessStatus() != idsp.NodeAccessStatus_NotAvailable and entry.AccessStatus() != idsp.NodeAccessStatus_NotImplemented:
                available_entries.append(entry.SymbolicValue())
        if value in available_entries:
            self._nodemap.FindNode(name).SetCurrentEntry(value)

    def load_settings(self, path):
        self._nodemap.LoadFromFile(str(path))

    def save_settings(self, path):
        self._nodemap.StoreToFile(str(path))

    def capture(self):
        # TODO: Timestamp: https://www.1stvision.com/cameras/IDS/IDS-manuals/uEye_Manual/is_getimageinfo.html
        # Publish over ros2

        # TODO: Check this
        buffer = self._datastream.WaitForFinishedBuffer(5000)

        # NOTE: This still uses the buffer's underlying memory
        ipl_image = idsp_extension.BufferToImage(buffer)

        # This creates a copy the image, so the buffer is free to use again after queuing
        # NOTE: Use `ImageConverter`, since the `ConvertTo` function re-allocates the conversion buffers on every call
        image_converted = self._image_converter.Convert(ipl_image, TARGET_PIXELFORMAT)

        self._datastream.QueueBuffer(buffer)

        return image_converted

    def start_capturing(self):
        self._capturing_threaded = True
        self._capturing_thread.start()

    def stop_capturing(self):
        self._killed = True
        self._capturing_thread.join()
        self._killed = False
        self._capturing_threaded = False

    def capture_threaded(self):
        while not self._killed:
            image = self.capture()
            return image

    def is_capturing(self):
        is_it = self._capturing_threaded
        return is_it

    def is_acquiring(self):
        is_it = self._acquiring
        return is_it

    # def record(self, t_seconds: int):
    #     # Create video writing object
    #     video = idsp_ipl.VideoWriter()
    #     cwd = os.getcwd()

    #     dropped_before = 0
    #     lost_before = 0

    #     try:
    #         # Create a new file the video will be saved in.
    #         video.Open(self._valid_name(cwd + "/" + "video", ".avi"))

    #         # Set target frame rate and gain
    #         self.set_remote_device_value("AcquisitionFrameRate", self.target_fps)
    #         self.set_remote_device_value("Gain", self.target_gain)

    #         video.Container().SetFramerate(self.target_fps)

    #         print("Recording with: ")
    #         var_name = "AcquisitionFrameRate"
    #         print(f"  Framerate: {self._nodemap.FindNode(var_name).Value():.2f}")
    #         var_name = "Gain"
    #         print(f"  Gain: {self._nodemap.FindNode(var_name).Value():.2f}")
    #         data_stream_nodemap = self._datastream.NodeMaps()[0]
    #         dropped_before = data_stream_nodemap.FindNode("StreamDroppedFrameCount").Value()
    #         lost_before = data_stream_nodemap.FindNode("StreamLostFrameCount").Value()

    #     except Exception as e:
    #         print(str(e))
    #         raise

    #     limit = timer + time.time()
    #     while (limit - time.time()) > 0 and not self.killed:
    #         buffer = self._datastream.WaitForFinishedBuffer(5000)
    #         ipl_image = idsp_extension.BufferToImage(buffer)
    #         converted_ipl_image = self._image_converter.Convert(ipl_image, TARGET_PIXEL_FORMAT)

    #         # Append image to video
    #         video.Append(converted_ipl_image)
    #         # Give buffer back into the queue so it can be used again
    #         self._datastream.QueueBuffer(buffer)

    #     if self.killed:
    #         return

    #     # See if the acquisition was lossless. Note that between the last
    #     # acquisition and the next acquisition some frames will be lost
    #     # (seen after the second recording).
    #     data_stream_nodemap = self._datastream.NodeMaps()[0]
    #     dropped_stream_frames = data_stream_nodemap.FindNode("StreamDroppedFrameCount").Value() - dropped_before
    #     lost_stream_frames = data_stream_nodemap.FindNode("StreamLostFrameCount").Value() - lost_before

    #     frames_encoded = video.NumFramesEncoded()
    #     frames_video_dropped = video.NumFramesDropped()

    #     fps = self.frames_encoded / self.duration

    #     # AVI framerate sets the playback speed.
    #     # You can calculate that with the amount of frames captured in the
    #     # time duration the video was recorded
    #     video.Container().SetFramerate(fps)
    #     # Wait until all frames are written to the file
    #     video.WaitUntilFrameDone(10000)
    #     video.Close()
