from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, SetParametersResult
from sensor_msgs.msg import Image

from nimbro_camera_ids.camera_ids import CameraIDS
from nimbro_utils.parameter_handler import ParameterHandler


class NodeCameraIDS(Node):
    def __init__(self, config_camera="default", framerate=30.0):
        super().__init__("camera_ids_publisher")

        self.bridge_cv = None
        self.camera_ids = None
        self.config_camera = config_camera
        self.counter = None
        self.framerate = framerate
        self.parameter_handler = None
        self.publisher = None
        self.timer = None

        self._initialize()

    def _initialize(self):
        self.camera = CameraIDS()
        self.get_logger().info(f"Opened device {self.camera}")

        # self.camera.load_config(self.config_camera)
        # self.get_logger().info(f"Loaded config {self.config_camera}")
        self.camera.save_config("original")
        self.camera.save_config("test")

        self.camera.start_acquisition()
        self.get_logger().info(f"Started acquisition")

        self.camera.start_capturing(on_capture_callback=self.on_capture_callback)
        self.get_logger().info(f"Started capturing")

        self.bridge_cv = CvBridge()
        self.publisher = self.create_publisher(Image, "camera_ids", 10)

        self.parameter_handler = ParameterHandler(self, verbose=False)

        self._setup_parameters()

    def _setup_parameters(self):
        self.add_on_set_parameters_callback(self.parameter_handler.parameter_callback)

        self._setup_parameter_config()
        self._setup_parameter_framerate()

        self.parameter_handler.all_declared()

    def _setup_parameter_config(self):
        descriptor = ParameterDescriptor(
            name="config",
            type=ParameterType.PARAMETER_STRING,
            description="Config of IDS camera",
            read_only=False,
            floating_point_range=FloatingPointRange(
                from_value=self.camera.get_min("AcquisitionFrameRate"),
                to_value=self.camera.get_max("AcquisitionFrameRate"),
                step=0.0,
            ),
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, "default", descriptor)

    def _setup_parameter_framerate(self):
        descriptor = ParameterDescriptor(
            name="framerate",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Acquisition framerate of IDS camera",
            read_only=False,
            floating_point_range=FloatingPointRange(
                from_value=self.camera.get_min("AcquisitionFrameRate"),
                to_value=self.camera.get_max("AcquisitionFrameRate"),
                step=0.0,
            ),
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, self.camera.get_max("AcquisitionFrameRate"), descriptor)

    def on_set_parameters_callback(self, parameters):
        result = SetParametersResult(successful=True)

        for i in range(len(parameters)):
            name = parameters[i].name
            value = parameters[i].value
            match name:
                case "config":
                    return "Not found"
                case "framerate":
                    self.camera.set_value("AcquisitionFrameRate", value)
                case _:
                    self.get_logger().info(f"Published image")
                    raise ValueError(f"Parameter {name} does not exist")

        return result

    def on_capture_callback(self, image):
        image = image.get_numpy_3D()

        message = self.bridge_cv.cv2_to_imgmsg(image, "bgra8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_ids"
        message.height = np.shape(image)[0]
        message.width = np.shape(image)[1]
        message.encoding = "bgr8"

        self.publisher.publish(message)
        self.get_logger().info(f"Published image")


# max_framerate = self.camera.get_max("AcquisitionFrameRate")
# self.camera.set_value("AcquisitionFrameRate", max_framerate)
# framerate = self.camera.get_value("AcquisitionFrameRate")

# gain = self.camera.get_value("Gain")
# self.get_logger().info(f"Gain set to {gain}")
# nodemap_datastream = self.camera.datastream.NodeMaps()[0]

# count_frames_dropped = nodemap_datastream.FindNode("StreamDroppedFrameCount").Value()
# count_frames_dropped = nodemap_datastream.FindNode("StreamDeliveredFrameCount").Value()
# lost_before = nodemap_datastream.FindNode("StreamLostFrameCount").Value()
# value = nodeMapSystem.FindNode("RegisteredReconnectEventsCount").Value()
