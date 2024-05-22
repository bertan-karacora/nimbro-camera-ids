import importlib.resources as resources
import inspect

from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, SetParametersResult
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from std_msgs.msg import Header

from nimbro_camera_ids.camera_ids import CameraIDS
from nimbro_utils.parameter_handler import ParameterHandler


class NodeCameraIDS(Node):
    def __init__(self):
        super().__init__("node_camera_ids")

        self.bridge_cv = None
        self.camera_ids = None
        self.counter = None
        self.parameter_handler = None
        self.publisher_info = None
        self.publisher_image = None
        self.timer = None

        self._initialize()

    def _initialize(self):
        self.camera = CameraIDS(name_pixelformat_target="PixelFormatName_RGB8")
        self.get_logger().info(f"Device {self.camera} opened")

        self.bridge_cv = CvBridge()
        self.publisher_info = self.create_publisher(CameraInfo, "camera_ids/camera_info", 10)
        self.publisher_image = self.create_publisher(Image, "camera_ids/image_color", 10)
        self.parameter_handler = ParameterHandler(self, verbose=False)

        self._setup_parameters()

        self.camera.start_acquisition()
        self.get_logger().info(f"Acquisition started")

        self.calibrate()

        self.camera.start_capturing(on_capture_callback=self.on_capture_callback)
        self.get_logger().info(f"Capturing started")

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
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, "default", descriptor)

    def _setup_parameter_framerate(self):
        descriptor = ParameterDescriptor(
            name="framerate",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Acquisition framerate of IDS camera",
            read_only=False,
            floating_point_range=(
                FloatingPointRange(
                    from_value=self.camera.get_min("AcquisitionFrameRate"),
                    to_value=self.camera.get_max("AcquisitionFrameRate"),
                    step=0.0,
                ),
            ),
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, self.camera.get_max("AcquisitionFrameRate"), descriptor)

    def parameter_changed(self, parameter):
        func_name = f"update_{parameter.name}"

        if not hasattr(NodeCameraIDS, func_name):
            raise ValueError(f"Parameter {parameter.name} does not exist")

        func_update = getattr(NodeCameraIDS, func_name)

        if not inspect.isfunction(func_update):
            raise ValueError(f"{func_update} is not a function")

        success, reason = func_update(self, parameter.value)

        return success, reason

    def update_config(self, config_camera):
        self.camera.load_config(config_camera)
        self.get_logger().info(f"Loaded config {config_camera}")

        success = True
        reason = ""

        return success, reason

    def update_framerate(self, framerate):
        self.camera.set_value("AcquisitionFrameRate", framerate)
        framerate_set = self.camera.get_value("AcquisitionFrameRate")
        self.get_logger().info(f"Framerate set to {framerate_set}")

        success = True
        reason = ""

        return success, reason

    def calibrate(self):
        # TODO
        # model_distortion: fisheye
        # values_vector_distortion: [0.0, 0.0, 0.0, 0.0]
        # values_matrix_intrinsic: [1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0]
        # values_matrix_rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # values_matrix_projection: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.height = self.camera.get_value("Height")
        self.width = self.camera.get_value("Width")

        self.distortion_model = "fisheye"
        self.values_vector_distortion = [0.0, 0.0, 0.0, 0.0]
        self.values_matrix_intrinsic = [1.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0]
        self.values_matrix_rectification = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        self.values_matrix_projection = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.binning = (self.camera.get_value("BinningHorizontal"), self.camera.get_value("BinningVertical"))
        self.roi = RegionOfInterest(
            x_offset=self.camera.get_value("OffsetX"),
            y_offset=self.camera.get_value("OffsetY"),
            height=self.camera.get_value("Height"),
            width=self.camera.get_value("Width"),
            do_rectify=self.camera.get_value("OffsetX") != 0 or self.camera.get_value("OffsetY") != 0,
        )

    def publish_info(self):
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_ids")
        message = CameraInfo(
            header=header,
            height=self.height,
            width=self.width,
            distortion_model=self.distortion_model,
            d=self.values_vector_distortion,
            k=self.values_matrix_intrinsic,
            r=self.values_matrix_rectification,
            p=self.values_matrix_projection,
            binning_x=self.binning[0],
            binning_y=self.binning[1],
            roi=self.roi,
        )

        self.publisher_info.publish(message)
        self.get_logger().info(f"Published info")

    def publish_image(self, image):
        image = image.get_numpy_3D()

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="camera_ids")
        message = self.bridge_cv.cv2_to_imgmsg(image, encoding="rgb8", header=header)

        self.publisher_image.publish(message)
        self.get_logger().info(f"Published image")

    def on_capture_callback(self, image):
        self.publish_info()
        self.publish_image(image)

    def list_available_configs():
        path_configs = resources.files(__package__) / "configs"
        files = sorted(path_configs.glob("**/*.cset"))
        configs_available = [str(f.parent.relative_to(path_configs) / f.stem) for f in files]
        return configs_available


# count_frames_dropped = nodemap_datastream.FindNode("StreamDroppedFrameCount").Value()
# count_frames_dropped = nodemap_datastream.FindNode("StreamDeliveredFrameCount").Value()
# lost_before = nodemap_datastream.FindNode("StreamLostFrameCount").Value()
# value = nodeMapSystem.FindNode("RegisteredReconnectEventsCount").Value()
