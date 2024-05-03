import importlib
import importlib.resources
import sys

from cv_bridge import CvBridge
import ids_peak.ids_peak as idsp
from rclpy.node import Node
from sensor_msgs.msg import Image

from camera_ids.camera_ids import CameraIDS


class NodeCameraIDS(Node):
    def __init__(self, config_camera="default"):
        super().__init__("camera_ids_publisher")
        self.bridge = None
        self.camera_ids = None
        self.config_camera = None
        self.counter = None
        self.device = None
        self.device_manager = None
        self.publisher = None
        self.timer = None

        self._initialize(config_camera)

    def _initialize(self, config_camera):
        idsp.Library.Initialize()

        self.device_manager = idsp.DeviceManager.Instance()
        self.device = self.open_device()
        self.camera = CameraIDS(self.device)

        self.config_camera = config_camera
        path_config = importlib.resources.files(__package__) / "configs" / f"{self.config_camera}.cset"
        self.camera.load_settings(path_config)

        self.camera.start_acquisition()

        self.bridge = CvBridge()
        self.counter = 0
        fps = 30
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
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

    def open_device(self, id_device=0):
        self.device_manager.Update()
        device_descriptors = self.device_manager.Devices()

        if device_descriptors.empty() or len(device_descriptors) <= id_device:
            print("No device found.")
            sys.exit()

        device = device_descriptors[id_device].OpenDevice(idsp.DeviceAccessType_Control)
        return device

    def timer_callback(self):
        img_ipl = self.camera.capture()

        img = img_ipl.get_numpy_3D()
        msg = self.bridge.cv2_to_imgmsg(img, "bgra8")

        self.publisher.publish(msg)
        self.counter += 1
        self.get_logger().info(f"Images published: {self.counter}")
