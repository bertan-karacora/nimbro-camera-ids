#!/usr/bin/env bash

set -eo pipefail

readonly path_repo="$(dirname "$(dirname "$(realpath "$BASH_SOURCE")")")"
source "$path_repo/libs/nimbro_config/config.sh"

source /etc/profile.d/idsGigETL_64bit.sh
source "/opt/ros/$DISTRIBUTION_ROS/setup.bash"
source "$HOME/colcon_ws/install/setup.bash"

set -u

main() {
    "$path_repo/libs/nimbro_config/scripts/setup_rmw.sh"
    "$path_repo/scripts/start_camera_tmux.sh" "$@"
    "$path_repo/scripts/start_watchdog_tmux.sh"
    tmux a -t "watchdog_$NAME_CONTAINER_NIMBRO_CAMERA_IDS"
}

main "$@"
