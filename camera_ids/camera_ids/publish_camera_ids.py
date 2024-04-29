import rclpy

from camera_ids.camera_ids_publisher import CameraIDSPublisher


def main(args=None):
    rclpy.init(args=args)

    publisher = CameraIDSPublisher()
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
