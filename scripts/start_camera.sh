#!/usr/bin/env bash

set -eo pipefail

readonly path_repo="$(dirname "$(dirname "$(realpath "$BASH_SOURCE")")")"
source "$path_repo/libs/nimbro_config/config.sh"

source /etc/profile.d/idsGigETL_64bit.sh
source "/opt/ros/$DISTRIBUTION_ROS/setup.bash"
source "$HOME/colcon_ws/install/setup.bash"

set -u

start_camera() {
    ros2 run nimbro_camera_ids spin "$@"
}

main() {
    start_camera "$@"
}

main "$@"
