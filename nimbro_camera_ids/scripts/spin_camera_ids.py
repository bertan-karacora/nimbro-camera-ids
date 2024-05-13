from nimbro_camera_ids.node_camera_ids import NodeCameraIDS

import nimbro_utils.node as utils_node


def main():
    utils_node.start_and_spin_node(NodeCameraIDS)


if __name__ == "__main__":
    main()
