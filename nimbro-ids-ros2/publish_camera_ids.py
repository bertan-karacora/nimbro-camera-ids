import rclpy

from camera_ids_publisher import CameraIDSPublisher


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraIDSPublisher()
    rclpy.spin(minimal_publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
