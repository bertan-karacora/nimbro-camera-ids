import importlib
import importlib.resources
import sys

from cv_bridge import CvBridge
import ids_peak.ids_peak as idsp
from rclpy.node import Node

# from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange, SetParametersResult
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

        self.camera.start_capturing()

        self.bridge = CvBridge()
        self.counter = 0

        # TODO: Node parameter interface for fps
        fps = 30
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.publisher = self.create_publisher(Image, "camera_ids", 10)

        # self.add_on_set_parameters_callback(self.parameter_callback)
        # descriptor = ParameterDescriptor()
        # descriptor.name = "my_param_name"
        # descriptor.type = ParameterType.PARAMETER_INTEGER
        # descriptor.description = "Size threshold of bounding box in pixels below which face detections are discarded"
        # descriptor.read_only = False
        # int_range = IntegerRange()
        # int_range.from_value = 0
        # int_range.to_value = 250000
        # int_range.step = 1
        # descriptor.integer_range.append(int_range)
        # self.parameter_descriptors.append(descriptor)
        # self.declare_parameter(descriptor.name, 30, descriptor)

    def print_device_info(self, device_descriptor):
        name_model = device_descriptor.ModelName()
        name_interface = device_descriptor.ParentInterface().DisplayName()
        name_system = device_descriptor.ParentInterface().ParentSystem().DisplayName()
        version_system = device_descriptor.ParentInterface().ParentSystem().Version()

        self.get_logger().info(name_model)
        self.get_logger().info(name_interface)
        self.get_logger().info(name_system)
        self.get_logger().info(version_system)

    def open_device(self, id_device=0):
        self.device_manager.Update()
        device_descriptors = self.device_manager.Devices()

        if device_descriptors.empty() or len(device_descriptors) <= id_device:
            self.get_logger().info("No device found.")
            sys.exit()

        device = device_descriptors[id_device].OpenDevice(idsp.DeviceAccessType_Control)
        return device

    def timer_callback(self):
        img = self.camera.image.get_numpy_3D()
        msg = self.bridge.cv2_to_imgmsg(img, "bgra8")

        self.publisher.publish(msg)
        self.counter += 1

        # TODO: Add more message params? Use timestamp instead of counter. What other logging is desired?
        self.get_logger().info(f"Images published: {self.counter}")

    # def parameter_callback(self, parameters):
    #     result = SetParametersResult()
    #     result.successful = True
    #     result.reason = ""

    #     for i in range(len(parameters)):
    #         if parameters[i].name == "my_param_name":
    #             self.my_param_name = parameters[i].value

    #     return result
