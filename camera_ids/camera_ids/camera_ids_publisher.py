import cv2
import ids_peak.ids_peak as idsp
from rclpy.node import Node
from sensor_msgs.msg import Image

from camera_ids.camera_ids import CameraIDS


class CameraIDSPublisher(Node):
    def __init__(self):
        super().__init__("camera_ids_publisher")

        self.camera_ids = None
        self.counter = None
        self.device = None
        self.device_manager = None
        self.publisher = None
        self.timer = None

        self._initialize()

    def _initialize(self):
        idsp.Library.Initialize()

        self.device_manager = idsp.DeviceManager.Instance()
        self.device = self.open_device()
        self.camera = CameraIDS(self.device)
        self.camera.start_acquisition()
        self.camera.start_capturing()

        self.counter = 0
        self.timer = self.create_timer(1 / 30, self.timer_callback)
        self.publisher = self.create_publisher(Image, "camera_ids", 10)

    def print_device_info(device_descriptor):
        name_model = device_descriptor.ModelName()
        name_interface = device_descriptor.ParentInterface().DisplayName()
        name_system = device_descriptor.ParentInterface().ParentSystem().DisplayName()
        version_system = device_descriptor.ParentInterface().ParentSystem().Version()

        print(name_model)
        print(name_interface)
        print(name_system)
        print(version_system)

    def open_device(self):
        self.device_manager.Update()
        device_descriptors = self.device_manager.Devices()

        if device_descriptors.empty():
            print("No device found. Exiting Program.")
            return

        device = device_descriptors[0].OpenDevice(idsp.DeviceAccessType_Control)
        return device

    def timer_callback(self):
        img = self.camera.capture()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = ""
        # msg.height = np.shape(frame)[0]
        # msg.width = np.shape(frame)[1]
        # msg.encoding = "bgr8"
        # msg.is_bigendian = False
        msg.data = img.get_numpy_3D().tobytes()

        self.publisher.publish(msg)
        self.counter += 1
        self.get_logger().info(f"Images published: {self.counter}")
