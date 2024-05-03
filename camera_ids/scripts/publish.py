import argparse
import rclpy

from camera_ids.node_camera_ids import NodeCameraIDS


def parse_args():
    parser = argparse.ArgumentParser(description="Start camera node for ROS 2.")
    parser.add_argument("-c", "--config", help="Camera config name", default="default")

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    rclpy.init()

    publisher = NodeCameraIDS(config_camera=args.config)
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
