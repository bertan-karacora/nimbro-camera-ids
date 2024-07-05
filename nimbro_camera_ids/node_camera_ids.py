import importlib.resources as resources

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from sensor_msgs.msg import CameraInfo, RegionOfInterest, Image
from std_msgs.msg import Header
import yaml

from nimbro_camera_ids.camera_ids import CameraIDS
from nimbro_utils.parameter_handler import ParameterHandler


class NodeCameraIDS(Node):
    def __init__(self):
        super().__init__(node_name="camera_ids")

        self.bridge_cv = None
        self.camera = None
        self.counter = None
        self.info = {}
        self.handler_parameters = None
        self.publisher_info = None
        self.publisher_image = None

        self._init()

    def _init(self):
        self.bridge_cv = CvBridge()
        self.handler_parameters = ParameterHandler(self, verbose=False)

        self._init_publishers()
        self._init_camera()
        self._init_info()
        self._init_parameters()

        # TODO: Seems bad.
        self.timer_device = self.create_timer(1.0, self.check_capturing_thread, callback_group=ReentrantCallbackGroup())

    def check_capturing_thread(self):
        if self.camera.is_capturing and self.camera.killed:
            raise Exception("Thread died")

    def _init_publishers(self):
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=3)
        self.publisher_info = self.create_publisher(CameraInfo, "camera_ids/camera_info", qos_profile=qos_profile, callback_group=ReentrantCallbackGroup())
        self.publisher_image = self.create_publisher(Image, "camera_ids/image_color", qos_profile=qos_profile, callback_group=ReentrantCallbackGroup())

    def _init_info(self):
        path_intrinsics = resources.files(__package__) / "resources" / "intrinsics.yaml"
        intrinsics = self._read_yaml(path_intrinsics)

        self.info["height"] = intrinsics["height"]
        self.info["width"] = intrinsics["width"]
        self.info["binning_x"] = intrinsics["binning_x"]
        self.info["binning_y"] = intrinsics["binning_y"]
        self.info["roi"] = RegionOfInterest(
            x_offset=intrinsics["offset_x"],
            y_offset=intrinsics["offset_y"],
            height=intrinsics["height"],
            width=intrinsics["width"],
            do_rectify=intrinsics["offset_x"] != 0 or intrinsics["offset_y"] != 0,
        )
        self.info["distortion_model"] = intrinsics["distortion_model"]
        self.info["d"] = [intrinsics["xi"], intrinsics["alpha"]]
        self.info["k"] = [intrinsics["fx"], 0.0, intrinsics["cx"]] + [0.0, intrinsics["fy"], intrinsics["cy"]] + [0.0, 0.0, 1.0]
        self.info["r"] = [1.0, 0.0, 0.0] + [0.0, 1.0, 0.0] + [0.0, 0.0, 1.0]
        self.info["p"] = [intrinsics["fx"], 0.0, intrinsics["cx"], 0.0] + [0.0, intrinsics["fy"], intrinsics["cy"], 0.0] + [0.0, 0.0, 1.0, 0.0]

    def _read_yaml(self, path):
        with open(path, "r") as stream:
            data = yaml.safe_load(stream)

        return data

    def _init_camera(self):
        self.camera = CameraIDS(name_pixelformat_target="PixelFormatName_RGB8")
        self.get_logger().info(f"Device {self.camera} opened")

        self.camera.start_acquisition()
        self.get_logger().info(f"Acquisition started")

        self.camera.start_capturing(on_capture_callback=self.on_capture_callback)
        self.get_logger().info(f"Capturing started")

    def _init_parameters(self):
        self.add_on_set_parameters_callback(self.handler_parameters.parameter_callback)

        self._init_parameter_config()
        self._init_parameter_framerate()
        self._init_parameter_exposure_time()
        self._init_parameter_auto_exposure()
        self._init_parameter_auto_gain()
        self._init_parameter_auto_white_balance()

        self.handler_parameters.all_declared()

    def _init_parameter_config(self, value="default"):
        descriptor = ParameterDescriptor(
            name="config",
            type=ParameterType.PARAMETER_STRING,
            description="Config of IDS camera",
            read_only=False,
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, value, descriptor)

    def _init_parameter_framerate(self, value=10.0):
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
        self.declare_parameter(descriptor.name, value, descriptor)

    def _init_parameter_exposure_time(self, value=None):
        descriptor = ParameterDescriptor(
            name="exposure_time",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Exposure time of IDS camera in microseconds",
            read_only=False,
            floating_point_range=(
                FloatingPointRange(
                    from_value=self.camera.get_min("ExposureTime"),
                    to_value=self.camera.get_max("ExposureTime"),
                    step=0.0,
                ),
            ),
        )
        self.parameter_descriptors.append(descriptor)
        if value is None:
            exposure_time_set = self.camera.get_value("ExposureTime")
            self.declare_parameter(descriptor.name, exposure_time_set, descriptor)
        else:
            self.declare_parameter(descriptor.name, value, descriptor)

    def _init_parameter_auto_exposure(self, value="off"):
        descriptor = ParameterDescriptor(
            name="auto_exposure",
            type=ParameterType.PARAMETER_STRING,
            description="Auto-exposure feature",
            read_only=False,
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, value, descriptor)

    def _init_parameter_auto_gain(self, value="continuous"):
        descriptor = ParameterDescriptor(
            name="auto_gain",
            type=ParameterType.PARAMETER_STRING,
            description="Auto-gain feature",
            read_only=False,
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, value, descriptor)

    def _init_parameter_auto_white_balance(self, value="continuous"):
        descriptor = ParameterDescriptor(
            name="auto_white_balance",
            type=ParameterType.PARAMETER_STRING,
            description="Auto-white-balance feature",
            read_only=False,
        )
        self.parameter_descriptors.append(descriptor)
        self.declare_parameter(descriptor.name, value, descriptor)

    def parameter_changed(self, parameter):
        try:
            func_update = getattr(NodeCameraIDS, f"update_{parameter.name}")
            success, reason = func_update(self, parameter.value)
        except Exception as e:
            self.get_logger().info(f"Error: {e}")

        return success, reason

    def update_config(self, config_camera):
        # Some config parameters cannot be changed when acquisition is running
        was_acquiring = self.camera.is_acquiring
        was_capturing = self.camera.is_capturing

        if was_capturing:
            self.camera.stop_capturing()
            self.get_logger().info(f"Capturing stopped")

        if was_acquiring:
            self.camera.stop_acquisition()
            self.get_logger().info(f"Acquisition stopped")

        self.camera.load_config(config_camera)
        self.get_logger().info(f"Config '{config_camera}' loaded")

        if was_acquiring:
            self.camera.start_acquisition()
            self.get_logger().info(f"Acquisition started")

        if was_capturing:
            self.camera.start_capturing(on_capture_callback=self.on_capture_callback)
            self.get_logger().info(f"Capturing started")

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

    def update_exposure_time(self, exposure_time):
        self.camera.set_value("ExposureTime", exposure_time)

        exposure_time_set = self.camera.get_value("ExposureTime")
        self.get_logger().info(f"Exposure time set to {exposure_time_set}")

        success = True
        reason = ""

        return success, reason

    def update_auto_exposure(self, auto_exposure):
        self.camera.manager_auto_features.auto_exposure = auto_exposure
        self.get_logger().info(f"Auto-exposure set to {self.camera.manager_auto_features.auto_exposure}")

        success = True
        reason = ""

        return success, reason

    def update_auto_gain(self, auto_gain):
        self.camera.manager_auto_features.auto_gain = auto_gain
        self.get_logger().info(f"Auto-gain set to {self.camera.manager_auto_features.auto_gain}")

        success = True
        reason = ""

        return success, reason

    def update_auto_white_balance(self, auto_white_balance):
        self.camera.manager_auto_features.auto_white_balance = auto_white_balance
        self.get_logger().info(f"Auto-white-balance set to {self.camera.manager_auto_features.auto_white_balance}")

        success = True
        reason = ""

        return success, reason

    def publish_info(self, time_ros2):
        header = Header(stamp=time_ros2.to_msg(), frame_id="camera_ids_link")
        message = CameraInfo(header=header, **self.info)

        self.publisher_info.publish(message)

    def publish_image(self, image, time_ros2):
        header = Header(stamp=time_ros2.to_msg(), frame_id="camera_ids_link")
        message = self.bridge_cv.cv2_to_imgmsg(image.get_numpy_3D(), header=header, encoding="rgb8")

        self.publisher_image.publish(message)

    def on_capture_callback(self, image):
        time_ros2 = self.get_clock().now()
        self.publish_image(image, time_ros2=time_ros2)
        self.publish_info(time_ros2=time_ros2)

    def list_available_configs():
        path_configs = resources.files(__package__) / "configs"
        files = sorted(path_configs.glob("**/*.cset"))
        configs_available = [str(f.parent.relative_to(path_configs) / f.stem) for f in files]
        return configs_available

    def run_benchmark():
        ...
        # TODO: Benchmarking
        # count_frames_dropped = nodemap_datastream.FindNode("StreamDroppedFrameCount").Value()
        # count_frames_dropped = nodemap_datastream.FindNode("StreamDeliveredFrameCount").Value()
        # lost_before = nodemap_datastream.FindNode("StreamLostFrameCount").Value()
        # value = nodeMapSystem.FindNode("RegisteredReconnectEventsCount").Value()
