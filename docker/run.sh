#!/usr/bin/env bash

set -euo pipefail

readonly path_repo="$(dirname "$(dirname "$(realpath "$BASH_SOURCE")")")"
source "$path_repo/libs/nimbro_config/config.sh"

is_found_overlay_nimbro_config=""
is_found_overlay_nimbro_utils=""

show_help() {
    echo "Usage:"
    echo "  ./run.sh"
    echo
    echo "Run Docker container."
    echo
}

get_usb_bus() {
    local bus_usb="$(lsusb | grep 'IDS' | awk '{print $2}')"

    echo "$bus_usb"
}

check_overlays() {
    if [ -d "$PATH_NIMBRO_CONFIG" ]; then
        is_found_overlay_nimbro_config=0
    fi
    if [ -d "$PATH_NIMBRO_UTILS" ]; then
        is_found_overlay_nimbro_utils=0
    fi
}

run_docker() {
    local arch="$(arch)"
    local name_repo="$(basename "$path_repo")"
    local bus_usb="$(get_usb_bus)"

    docker run \
        --name "$NAME_CONTAINER_NIMBRO_CAMERA_IDS" \
        --shm-size 12G \
        --privileged \
        --interactive \
        --tty \
        --net host \
        --detach \
        --restart unless-stopped \
        --env DISPLAY \
        --device "/dev/bus/usb/$bus_usb" \
        --volume /etc/localtime:/etc/localtime:ro \
        --volume /tmp/.X11-unix/:/tmp/.X11-unix/ \
        --volume "$HOME/.Xauthority:/root/.Xauthority" \
        --volume "/dev/bus/usb/$bus_usb:/dev/bus/usb/$bus_usb" \
        --volume "$HOME/.ros/:/root/.ros/" \
        --volume "$path_repo:/root/colcon_ws/src/$name_repo" \
        ${is_found_overlay_nimbro_config:+--volume "$PATH_NIMBRO_CONFIG:/root/colcon_ws/src/$name_repo/libs/nimbro_config"} \
        ${is_found_overlay_nimbro_utils:+--volume "$PATH_NIMBRO_UTILS:/root/colcon_ws/src/nimbro_utils"} \
        "$NAME_CONTAINER_NIMBRO_CAMERA_IDS:$arch" "$@"
}

main() {
    check_overlays
    run_docker "$@"
}

main "$@"
